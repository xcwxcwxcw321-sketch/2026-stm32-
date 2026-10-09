#!/usr/bin/env python3
"""Vibration-tolerant hybrid steel-ball detector for the D36A rig.

The primary detector uses local-contrast contours.  A row-normalized 1-D
projection provides a fallback when reflection, motion blur, or an image edge
breaks the circular contour.  A constant-velocity Kalman filter bridges short
detection gaps.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import threading
import time
import urllib.request
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import cv2
import numpy as np


DEFAULT_DEVICE = (
    "/dev/v4l/by-id/"
    "usb-Web_Camera_Web_Camera_202604081837-video-index0"
)

DEFAULT_SETTINGS: dict[str, int | float] = {
    "crop_top": 112,
    "crop_bottom": 218,
    "pipe_top_px": 16.0,
    "pipe_bottom_px": 82.0,
    "pipe_left_x_step0_px": 40.0,
    "pipe_left_x_balance_px": 14.0,
    "pipe_left_x_max_px": 0.0,
    "pipe_min_steps": 0,
    "pipe_balance_steps": 235,
    "pipe_max_steps": 681,
    "pipe_length_cm": 25.0,
    "detect_min_cm": -8.0,
    "detect_max_cm": 8.0,
    "detect_top_px": 16.0,
    "detect_bottom_px": 82.0,
    "exposure": 280,
    "illumination_normalization_enabled": 1,
    "contrast_drop": 12.0,
    "background_kernel": 41,
    "min_area": 45.0,
    "max_area": 1200.0,
    "pipe_roi_erode_px": 9,
    "tracker_gate_px": 80.0,
    "track_hold_frames": 8,
    "kalman_process_noise": 8.0,
    "kalman_measurement_noise": 12.0,
    "projection_enabled": 1,
    "projection_window_px": 25,
    "projection_threshold": 650.0,
    "projection_row_percentile": 60.0,
}

INTEGER_KEYS = {
    "crop_top",
    "crop_bottom",
    "exposure",
    "background_kernel",
    "pipe_roi_erode_px",
    "track_hold_frames",
    "projection_enabled",
    "illumination_normalization_enabled",
    "projection_window_px",
    "pipe_min_steps",
    "pipe_balance_steps",
    "pipe_max_steps",
}

VIEWER_HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>防抖钢球识别</title>
  <style>
    :root { color-scheme:dark; font-family:"Microsoft YaHei",sans-serif; }
    body { margin:0; background:#0d1117; color:#edf3f8; }
    main { width:min(98vw,1280px); margin:auto; padding:14px; }
    h1 { margin:0 0 10px; font-size:22px; }
    img { display:block; width:100%; border:2px solid #34404a;
          border-radius:10px; background:#000; }
    p { color:#9ba7b4; }
  </style>
</head>
<body><main>
  <h1>25 cm 水管 · 防抖灰度识别</h1>
  <img src="/stream.mjpg" alt="防抖钢球识别画面">
  <p>稳定水管坐标系 · Kalman 短时预测 · 参数请在 8081 综合调参页设置</p>
</main></body>
</html>
"""


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def atomic_write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


class SettingsStore:
    def __init__(
        self,
        path: Path,
        frame_width: int,
        frame_height: int,
    ) -> None:
        self.path = path
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.lock = threading.RLock()
        self.values = dict(DEFAULT_SETTINGS)
        self.revision = 0
        self._load()

    def _convert(
        self,
        payload: dict[str, object],
    ) -> dict[str, int | float]:
        converted: dict[str, int | float] = {}
        for key, default in DEFAULT_SETTINGS.items():
            value = payload.get(key, default)
            converted[key] = int(value) if key in INTEGER_KEYS else float(value)
        return converted

    def _validate(self, values: dict[str, int | float]) -> None:
        crop_top = int(values["crop_top"])
        crop_bottom = int(values["crop_bottom"])
        if not 0 <= crop_top < crop_bottom <= self.frame_height:
            raise ValueError("裁剪上下边界无效")
        if crop_bottom - crop_top < 70:
            raise ValueError("裁剪高度不能小于70像素")

        checks = (
            ("pipe_top_px", values["pipe_top_px"], 0, crop_bottom - crop_top - 2),
            ("pipe_bottom_px", values["pipe_bottom_px"], 1, crop_bottom - crop_top - 1),
            ("pipe_left_x_step0_px", values["pipe_left_x_step0_px"], 0, self.frame_width - 2),
            ("pipe_left_x_balance_px", values["pipe_left_x_balance_px"], 0, self.frame_width - 2),
            ("pipe_left_x_max_px", values["pipe_left_x_max_px"], 0, self.frame_width - 2),
            ("pipe_min_steps", values["pipe_min_steps"], 0, 680),
            ("pipe_balance_steps", values["pipe_balance_steps"], 1, 680),
            ("pipe_max_steps", values["pipe_max_steps"], 2, 2000),
            ("pipe_length_cm", values["pipe_length_cm"], 1, 100),
            ("detect_min_cm", values["detect_min_cm"], -50, 50),
            ("detect_max_cm", values["detect_max_cm"], -50, 50),
            ("detect_top_px", values["detect_top_px"], 0, crop_bottom - crop_top - 2),
            ("detect_bottom_px", values["detect_bottom_px"], 1, crop_bottom - crop_top - 1),
            ("exposure", values["exposure"], 1, 5000),
            ("contrast_drop", values["contrast_drop"], 5, 120),
            ("background_kernel", values["background_kernel"], 15, 101),
            ("min_area", values["min_area"], 5, 5000),
            ("max_area", values["max_area"], 10, 10000),
            ("pipe_roi_erode_px", values["pipe_roi_erode_px"], 1, 31),
            ("tracker_gate_px", values["tracker_gate_px"], 10, 250),
            ("track_hold_frames", values["track_hold_frames"], 0, 30),
            ("kalman_process_noise", values["kalman_process_noise"], 0.01, 200),
            ("kalman_measurement_noise", values["kalman_measurement_noise"], 0.01, 200),
            ("projection_window_px", values["projection_window_px"], 9, 61),
            ("projection_threshold", values["projection_threshold"], 50, 2000),
            ("projection_row_percentile", values["projection_row_percentile"], 30, 90),
        )
        for name, value, lower, upper in checks:
            if not lower <= float(value) <= upper:
                raise ValueError(f"{name}必须在{lower:g}到{upper:g}之间")

        if not (
            int(values["pipe_min_steps"])
            < int(values["pipe_balance_steps"])
            < int(values["pipe_max_steps"])
        ):
            raise ValueError("步数标定必须满足最低点 < 平衡点 < 最高点")
        if float(values["pipe_top_px"]) >= float(values["pipe_bottom_px"]):
            raise ValueError("水管上边必须小于下边")
        if float(values["detect_top_px"]) >= float(values["detect_bottom_px"]):
            raise ValueError("紫色识别框上边必须小于下边")
        if float(values["min_area"]) >= float(values["max_area"]):
            raise ValueError("最小面积必须小于最大面积")
        if int(values["projection_enabled"]) not in (0, 1):
            raise ValueError("projection_enabled只能为0或1")
        if int(values["illumination_normalization_enabled"]) not in (0, 1):
            raise ValueError("illumination_normalization_enabled只能为0或1")
        if int(values["projection_window_px"]) % 2 == 0:
            raise ValueError("projection_window_px必须为奇数")
        half_length = float(values["pipe_length_cm"]) * 0.5
        if not (
            -half_length
            <= float(values["detect_min_cm"])
            < float(values["detect_max_cm"])
            <= half_length
        ):
            raise ValueError("检测范围必须位于水管坐标范围内且最小值小于最大值")

    def _load(self) -> None:
        if not self.path.exists():
            return
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict):
                raise TypeError("settings root must be an object")
            converted = self._convert(payload)
            self._validate(converted)
            self.values.update(converted)
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
            print(f"Ignoring invalid vision settings: {error}", flush=True)

    def snapshot(self) -> tuple[dict[str, int | float], int]:
        with self.lock:
            return dict(self.values), self.revision

    def update(
        self,
        payload: dict[str, object],
        persist: bool,
    ) -> dict[str, int | float]:
        with self.lock:
            merged = dict(self.values)
            for key in DEFAULT_SETTINGS:
                if key in payload:
                    merged[key] = payload[key]
            converted = self._convert(merged)
            self._validate(converted)
            self.values.update(converted)
            self.revision += 1
            result = dict(self.values)
            if persist:
                atomic_write_json(self.path, result)
            return result


class CameraControls:
    def __init__(self, device: str) -> None:
        self.device = device
        self.lock = threading.Lock()

    def apply(self, settings: dict[str, int | float]) -> None:
        exposure = int(settings["exposure"])
        command = [
            "/usr/bin/v4l2-ctl",
            "-d",
            self.device,
            "--set-ctrl",
            f"auto_exposure=1,exposure_time_absolute={exposure}",
        ]
        with self.lock:
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=3,
            )
        if result.returncode != 0:
            message = result.stderr.strip() or result.stdout.strip()
            raise RuntimeError(f"摄像头曝光设置失败: {message}")


class FrameHub:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.jpeg: bytes | None = None
        self.frame_id = 0
        self.status: dict[str, object] = {}
        self.running = True
        self.clients = 0

    def update(self, jpeg: bytes, status: dict[str, object]) -> None:
        with self.condition:
            self.jpeg = jpeg
            self.frame_id += 1
            self.status = status
            self.condition.notify_all()

    def latest(self) -> tuple[int, bytes | None, dict[str, object], int]:
        with self.condition:
            return self.frame_id, self.jpeg, dict(self.status), self.clients

    def wait_after(
        self,
        previous_frame_id: int,
        timeout: float = 2.0,
    ) -> tuple[int, bytes | None, bool]:
        with self.condition:
            self.condition.wait_for(
                lambda: self.frame_id > previous_frame_id or not self.running,
                timeout=timeout,
            )
            return self.frame_id, self.jpeg, self.running

    def add_client(self) -> None:
        with self.condition:
            self.clients += 1

    def remove_client(self) -> None:
        with self.condition:
            self.clients = max(0, self.clients - 1)

    def stop(self) -> None:
        with self.condition:
            self.running = False
            self.condition.notify_all()


class ViewerServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address: tuple[str, int],
        hub: FrameHub,
        settings: SettingsStore,
        camera_controls: CameraControls,
    ) -> None:
        self.hub = hub
        self.settings = settings
        self.camera_controls = camera_controls
        super().__init__(address, ViewerHandler)


class ViewerHandler(BaseHTTPRequestHandler):
    server: ViewerServer

    def log_message(self, _format: str, *_args: object) -> None:
        return

    def send_bytes(
        self,
        payload: bytes,
        content_type: str,
        status: HTTPStatus = HTTPStatus.OK,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Pragma", "no-cache")
        self.end_headers()
        self.wfile.write(payload)

    def send_json(
        self,
        payload: dict[str, object],
        status: HTTPStatus = HTTPStatus.OK,
    ) -> None:
        self.send_bytes(
            json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            "application/json; charset=utf-8",
            status,
        )

    def do_OPTIONS(self) -> None:
        self.send_response(HTTPStatus.NO_CONTENT)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.end_headers()

    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if path == "/":
            self.send_bytes(
                VIEWER_HTML.encode("utf-8"),
                "text/html; charset=utf-8",
            )
        elif path == "/snapshot.jpg":
            _frame_id, jpeg, _status, _clients = self.server.hub.latest()
            if jpeg is None:
                self.send_bytes(
                    b"Camera is starting",
                    "text/plain; charset=utf-8",
                    HTTPStatus.SERVICE_UNAVAILABLE,
                )
            else:
                self.send_bytes(jpeg, "image/jpeg")
        elif path == "/status.json":
            frame_id, _jpeg, status, clients = self.server.hub.latest()
            self.send_json(
                {
                    "frame_id": frame_id,
                    "clients": clients,
                    **status,
                }
            )
        elif path == "/settings.json":
            values, revision = self.server.settings.snapshot()
            self.send_json({"settings": values, "revision": revision})
        elif path == "/stream.mjpg":
            self.stream_mjpeg()
        else:
            self.send_bytes(
                b"Not found",
                "text/plain; charset=utf-8",
                HTTPStatus.NOT_FOUND,
            )

    def do_POST(self) -> None:
        if urlsplit(self.path).path != "/settings":
            self.send_bytes(
                b"Not found",
                "text/plain; charset=utf-8",
                HTTPStatus.NOT_FOUND,
            )
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 16384:
                raise ValueError("请求大小无效")
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict):
                raise ValueError("JSON必须是对象")
            values = payload.get("settings")
            if not isinstance(values, dict):
                raise ValueError("settings必须是对象")
            updated = self.server.settings.update(
                values,
                bool(payload.get("persist", False)),
            )
            self.server.camera_controls.apply(updated)
            self.send_json(
                {
                    "ok": True,
                    "message": (
                        "视觉参数已保存"
                        if payload.get("persist")
                        else "视觉参数已应用"
                    ),
                    "settings": updated,
                }
            )
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
        except (OSError, RuntimeError, subprocess.SubprocessError) as error:
            self.send_json({"error": str(error)}, HTTPStatus.CONFLICT)

    def stream_mjpeg(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=frame",
        )
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()
        self.connection.settimeout(2.0)
        frame_id = -1
        self.server.hub.add_client()
        try:
            while True:
                frame_id, jpeg, running = self.server.hub.wait_after(frame_id)
                if not running:
                    break
                if jpeg is None:
                    continue
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(
                    f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii")
                )
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            self.server.hub.remove_client()


def open_camera(args: argparse.Namespace) -> cv2.VideoCapture:
    pipeline = (
        f"v4l2src device={args.device} io-mode=2 ! "
        f"image/jpeg,width={args.width},height={args.height},"
        f"framerate={args.camera_fps}/1 ! "
        "jpegdec ! videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )
    camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"Cannot open USB camera: {args.device}")
    return camera


def order_box(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32).reshape(4, 2)
    sums = points.sum(axis=1)
    differences = np.diff(points, axis=1).reshape(-1)
    return np.array(
        [
            points[np.argmin(sums)],
            points[np.argmin(differences)],
            points[np.argmax(sums)],
            points[np.argmax(differences)],
        ],
        dtype=np.float32,
    )


class AxisPositionSource:
    """Read the D36A software step count without blocking the camera loop."""

    def __init__(self, status_url: str) -> None:
        self.status_url = status_url
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.current_steps = 0
        self.updated_at: float | None = None
        self.thread = threading.Thread(
            target=self._run,
            name="axis-position-source",
            daemon=True,
        )

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=1)

    def snapshot(self) -> tuple[int, float | None]:
        with self.lock:
            age = (
                time.monotonic() - self.updated_at
                if self.updated_at is not None
                else None
            )
            return self.current_steps, age

    def _run(self) -> None:
        while not self.stop_event.is_set():
            try:
                with urllib.request.urlopen(
                    self.status_url,
                    timeout=0.20,
                ) as response:
                    payload = json.load(response)
                steps = int(payload["current_steps"])
                with self.lock:
                    self.current_steps = steps
                    self.updated_at = time.monotonic()
            except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
                pass
            self.stop_event.wait(0.04)


def pipe_left_x_from_steps(
    current_steps: int,
    settings: dict[str, int | float],
) -> float:
    minimum = int(settings["pipe_min_steps"])
    balance = int(settings["pipe_balance_steps"])
    maximum = int(settings["pipe_max_steps"])
    step = int(clamp(current_steps, minimum, maximum))

    if step <= balance:
        span = max(1, balance - minimum)
        fraction = (step - minimum) / span
        start_x = float(settings["pipe_left_x_step0_px"])
        balance_x = float(settings["pipe_left_x_balance_px"])
        return start_x + (balance_x - start_x) * fraction

    span = max(1, maximum - balance)
    fraction = (step - balance) / span
    balance_x = float(settings["pipe_left_x_balance_px"])
    maximum_x = float(settings["pipe_left_x_max_px"])
    return balance_x + (maximum_x - balance_x) * fraction


def step_interpolated_pipe_box(
    shape: tuple[int, int],
    current_steps: int,
    settings: dict[str, int | float],
) -> tuple[np.ndarray, float]:
    height, width = shape
    left_x = pipe_left_x_from_steps(current_steps, settings)
    right_x = float(width - 1)
    top = float(settings["pipe_top_px"])
    bottom = float(settings["pipe_bottom_px"])
    box = np.array(
        [
            [left_x, top],
            [right_x, top],
            [right_x, bottom],
            [left_x, bottom],
        ],
        dtype=np.float32,
    )
    box[:, 0] = np.clip(box[:, 0], 0, width - 1)
    box[:, 1] = np.clip(box[:, 1], 0, height - 1)
    return box, left_x


def pipe_axis(pipe_box: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    top_left, top_right, bottom_right, bottom_left = pipe_box
    left = (top_left + bottom_left) * 0.5
    right = (top_right + bottom_right) * 0.5
    return left, right


def restrict_pipe_box(
    pipe_box: np.ndarray,
    minimum_cm: float,
    maximum_cm: float,
    length_cm: float,
    top_px: float,
    bottom_px: float,
) -> np.ndarray:
    """Return the part of the pipe used by this task's ball detector."""
    top_left, top_right, bottom_right, bottom_left = pipe_box
    start = clamp(minimum_cm / length_cm + 0.5, 0.0, 1.0)
    end = clamp(maximum_cm / length_cm + 0.5, 0.0, 1.0)
    restricted = np.array(
        [
            top_left + (top_right - top_left) * start,
            top_left + (top_right - top_left) * end,
            bottom_left + (bottom_right - bottom_left) * end,
            bottom_left + (bottom_right - bottom_left) * start,
        ],
        dtype=np.float32,
    )
    restricted[0:2, 1] = top_px
    restricted[2:4, 1] = bottom_px
    return restricted


def point_to_cm(
    point: tuple[float, float],
    pipe_box: np.ndarray,
    pipe_length_cm: float,
) -> float:
    left, right = pipe_axis(pipe_box)
    axis = right - left
    denominator = float(np.dot(axis, axis))
    if denominator < 1.0:
        return 0.0
    fraction = float(np.dot(np.asarray(point) - left, axis) / denominator)
    return clamp(
        (fraction - 0.5) * pipe_length_cm,
        -pipe_length_cm / 2.0,
        pipe_length_cm / 2.0,
    )


class BallTracker:
    def __init__(self) -> None:
        self.filter = cv2.KalmanFilter(4, 2, 0, cv2.CV_32F)
        self.filter.measurementMatrix = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 0]],
            dtype=np.float32,
        )
        self.filter.errorCovPost = np.eye(4, dtype=np.float32) * 10.0
        self.initialized = False
        self.last_time: float | None = None
        self.predicted: tuple[float, float] | None = None
        self.missed = 0
        self.last_score = 0.0

    def reset(self) -> None:
        self.initialized = False
        self.last_time = None
        self.predicted = None
        self.missed = 0
        self.last_score = 0.0

    def predict(
        self,
        now: float,
        settings: dict[str, int | float],
    ) -> tuple[float, float] | None:
        if not self.initialized:
            self.last_time = now
            self.predicted = None
            return None
        dt = clamp(
            now - self.last_time if self.last_time is not None else 1.0 / 30.0,
            0.005,
            0.15,
        )
        self.last_time = now
        self.filter.transitionMatrix = np.array(
            [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]],
            dtype=np.float32,
        )
        q = float(settings["kalman_process_noise"])
        self.filter.processNoiseCov = np.diag(
            [q * dt * dt, q * dt * dt, q, q]
        ).astype(np.float32)
        r = float(settings["kalman_measurement_noise"])
        self.filter.measurementNoiseCov = np.eye(2, dtype=np.float32) * r
        prediction = self.filter.predict()
        self.predicted = (float(prediction[0, 0]), float(prediction[1, 0]))
        return self.predicted

    def update(
        self,
        measurement: tuple[float, float] | None,
        score: float,
        settings: dict[str, int | float],
    ) -> tuple[tuple[float, float] | None, str, float]:
        hold_frames = int(settings["track_hold_frames"])
        if measurement is not None:
            vector = np.array(
                [[measurement[0]], [measurement[1]]],
                dtype=np.float32,
            )
            if not self.initialized:
                self.filter.statePost = np.array(
                    [[measurement[0]], [measurement[1]], [0], [0]],
                    dtype=np.float32,
                )
                self.filter.statePre = self.filter.statePost.copy()
                corrected = self.filter.statePost
                self.initialized = True
                self.last_time = time.monotonic()
            else:
                corrected = self.filter.correct(vector)
                # A valid contour has already passed ROI, geometry and motion
                # gates.  Keep Kalman velocity estimation for missed frames,
                # but never let its display/control position trail a real
                # measurement by several ball diameters.
                self.filter.statePost[0, 0] = measurement[0]
                self.filter.statePost[1, 0] = measurement[1]
            self.missed = 0
            self.last_score = score
            point = (float(measurement[0]), float(measurement[1]))
            self.predicted = point
            return point, "measured", score

        if not self.initialized:
            return None, "lost", 0.0
        self.missed += 1
        if self.missed <= hold_frames and self.predicted is not None:
            confidence = self.last_score * (
                1.0 - self.missed / float(hold_frames + 1)
            )
            return self.predicted, "predicted", max(0.0, confidence)
        if self.missed > max(hold_frames * 3, hold_frames + 8):
            self.reset()
        return None, "lost", 0.0

    def gate_prediction(
        self,
        settings: dict[str, int | float],
    ) -> tuple[float, float] | None:
        if self.missed > int(settings["track_hold_frames"]):
            return None
        return self.predicted


def detect_projection_candidate(
    local_contrast: np.ndarray,
    pipe_roi: np.ndarray,
    predicted: tuple[float, float] | None,
    settings: dict[str, int | float],
) -> tuple[
    tuple[int, int, int, int, float, float, float] | None,
    float,
]:
    """Find a dark ball as a strong 1-D energy peak along the pipe.

    Subtracting the per-row contrast percentile suppresses long horizontal pipe
    edges while preserving the compact vertical energy of the ball.  This is
    deliberately insensitive to a broken circle and to vertical camera shake.
    """

    roi = pipe_roi > 0
    valid_rows = np.flatnonzero(np.any(roi, axis=1))
    valid_columns = np.flatnonzero(np.any(roi, axis=0))
    if valid_rows.size == 0 or valid_columns.size == 0:
        return None, 0.0

    contrast = local_contrast.astype(np.float32)
    percentile = float(settings["projection_row_percentile"])
    first_row = int(valid_rows[0])
    last_row = int(valid_rows[-1])
    first_column = int(valid_columns[0])
    last_column = int(valid_columns[-1])
    row_region = contrast[
        first_row : last_row + 1,
        first_column : last_column + 1,
    ]
    row_baseline = np.zeros(contrast.shape[0], dtype=np.float32)
    row_baseline[first_row : last_row + 1] = np.percentile(
        row_region,
        percentile,
        axis=1,
    ).astype(np.float32)

    residual = np.maximum(contrast - row_baseline[:, None] - 5.0, 0.0)
    residual = np.minimum(residual, 60.0)
    residual *= roi
    column_energy = np.sum(residual, axis=0)

    window = int(settings["projection_window_px"])
    profile = np.convolve(
        column_energy,
        np.ones(window, dtype=np.float32) / float(window),
        mode="same",
    )
    profile[:first_column] = 0.0
    profile[last_column + 1 :] = 0.0

    global_x = int(np.argmax(profile))
    global_peak = float(profile[global_x])
    threshold = float(settings["projection_threshold"])
    selected_x = global_x
    selected_peak = global_peak

    if predicted is not None:
        gate = int(round(float(settings["tracker_gate_px"])))
        left = max(first_column, int(round(predicted[0])) - gate)
        right = min(last_column, int(round(predicted[0])) + gate)
        if left <= right:
            local_x = left + int(np.argmax(profile[left : right + 1]))
            local_peak = float(profile[local_x])
            if local_peak >= threshold:
                selected_x = local_x
                selected_peak = local_peak
            elif global_peak < threshold * 1.35:
                return None, global_peak

    if selected_peak < threshold:
        return None, global_peak

    half_window = window // 2
    x0 = max(first_column, selected_x - half_window)
    x1 = min(last_column + 1, selected_x + half_window + 1)
    patch = residual[:, x0:x1]
    total = float(np.sum(patch))
    if total > 0.0:
        rows = np.arange(residual.shape[0], dtype=np.float32)[:, None]
        center_y = float(np.sum(patch * rows) / total)
    else:
        center_y = float(np.mean(valid_rows))

    box_width = min(window, last_column - first_column + 1)
    box_height = window
    box_x = int(round(selected_x - box_width / 2.0))
    box_y = int(round(center_y - box_height / 2.0))
    box_x = max(0, min(local_contrast.shape[1] - box_width, box_x))
    box_y = max(0, min(local_contrast.shape[0] - box_height, box_y))
    confidence = clamp(
        0.55 + 0.35 * (selected_peak / max(threshold, 1.0) - 1.0),
        0.55,
        0.96,
    )
    return (
        box_x,
        box_y,
        box_width,
        box_height,
        float(selected_x),
        center_y,
        confidence,
    ), global_peak


def build_local_contrast(
    gray: np.ndarray,
    settings: dict[str, int | float],
) -> np.ndarray:
    """Build a dark-object response that is stable across exposure changes.

    Absolute background subtraction works well in bright scenes but weakens
    almost linearly when the whole image becomes darker.  The relative term
    divides that difference by local brightness.  Taking the maximum of the
    absolute and relative responses preserves the old behavior in normal
    light and recovers the ball in dim or uneven light.
    """
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    background_kernel = int(settings["background_kernel"])
    if background_kernel % 2 == 0:
        background_kernel += 1
    local_background = cv2.GaussianBlur(
        gray,
        (background_kernel, background_kernel),
        0,
    )
    absolute = cv2.subtract(local_background, blurred)
    if not int(settings["illumination_normalization_enabled"]):
        return absolute

    difference = absolute.astype(np.float32)
    relative = difference * 255.0 / (
        local_background.astype(np.float32) + 24.0
    )

    combined = np.maximum(difference, relative)
    return np.clip(combined, 0.0, 255.0).astype(np.uint8)


def detect_candidates(
    gray: np.ndarray,
    pipe_box: np.ndarray,
    predicted: tuple[float, float] | None,
    settings: dict[str, int | float],
) -> tuple[
    tuple[int, int, int, int, float, float, float] | None,
    np.ndarray,
    int,
    str,
    float,
]:
    pipe_roi = np.zeros(gray.shape, dtype=np.uint8)
    cv2.fillConvexPoly(
        pipe_roi,
        np.rint(pipe_box).astype(np.int32),
        255,
    )
    erode_size = int(settings["pipe_roi_erode_px"])
    if erode_size % 2 == 0:
        erode_size += 1
    pipe_roi = cv2.erode(
        pipe_roi,
        cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (erode_size, erode_size),
        ),
    )
    local_contrast = build_local_contrast(gray, settings)
    threshold = int(round(float(settings["contrast_drop"])))
    mask = cv2.inRange(local_contrast, threshold, 255)
    mask = cv2.bitwise_and(mask, pipe_roi)
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)),
    )

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    best = None
    best_score = -1.0
    gate = float(settings["tracker_gate_px"])
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if not float(settings["min_area"]) <= area <= float(settings["max_area"]):
            continue
        x, y, width, height = cv2.boundingRect(contour)
        # A dark pipe reflection can attach a short horizontal tail to the
        # ball.  The old 55 px cutoff rejected the real ball at 56 px, so keep
        # geometry permissive here and let projection/temporal agreement rank
        # merged contours instead of dropping them at a one-pixel boundary.
        if not 10 <= width <= 80 or not 10 <= height <= 65:
            continue
        # The physical ball can touch either image edge at a pipe endpoint.
        # Rejecting left/right-border contours caused a permanent loss at +12.5 cm.
        if y <= 1:
            continue
        aspect = width / float(height)
        if not 0.35 <= aspect <= 2.8:
            continue
        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 0:
            continue
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
        solidity = area / hull_area if hull_area > 0 else 0.0
        if circularity < 0.18 or solidity < 0.55:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        center_x = moments["m10"] / moments["m00"]
        center_y = moments["m01"] / moments["m00"]
        area_prior = math.exp(-abs(math.log(max(area, 1.0) / 300.0)))
        shape_score = (
            0.45 * min(1.0, circularity)
            + 0.25 * min(1.0, solidity)
            + 0.30 * area_prior
        )
        if predicted is not None:
            distance = math.hypot(
                center_x - predicted[0],
                center_y - predicted[1],
            )
            if distance > gate:
                continue
            proximity = math.exp(-0.5 * (distance / max(gate * 0.45, 1.0)) ** 2)
            score = 0.70 * shape_score + 0.30 * proximity
        else:
            score = shape_score
        if score > best_score:
            best_score = score
            best = (
                x,
                y,
                width,
                height,
                center_x,
                center_y,
                score,
            )
    projection_candidate = None
    projection_peak = 0.0
    if int(settings["projection_enabled"]):
        projection_candidate, projection_peak = detect_projection_candidate(
            local_contrast,
            pipe_roi,
            predicted,
            settings,
        )
    if best is not None and projection_candidate is not None:
        separation = math.hypot(
            best[4] - projection_candidate[4],
            best[5] - projection_candidate[5],
        )
        agreement_gate = max(
            30.0,
            float(settings["projection_window_px"]) * 1.5,
        )
        if separation <= agreement_gate:
            center_y = 0.5 * (best[5] + projection_candidate[5])
            box_height = projection_candidate[3]
            box_y = int(round(center_y - box_height * 0.5))
            fused = (
                projection_candidate[0],
                max(0, box_y),
                projection_candidate[2],
                box_height,
                projection_candidate[4],
                center_y,
                max(best[6], projection_candidate[6]),
            )
            return fused, mask, threshold, "fused", projection_peak
        if predicted is not None:
            contour_distance = math.hypot(
                best[4] - predicted[0],
                best[5] - predicted[1],
            )
            projection_distance = math.hypot(
                projection_candidate[4] - predicted[0],
                projection_candidate[5] - predicted[1],
            )
            if projection_distance < contour_distance:
                return (
                    projection_candidate,
                    mask,
                    threshold,
                    "projection",
                    projection_peak,
                )
        elif projection_peak >= float(settings["projection_threshold"]):
            return (
                projection_candidate,
                mask,
                threshold,
                "projection",
                projection_peak,
            )
    if best is not None and (
        predicted is not None or not int(settings["projection_enabled"])
    ):
        return best, mask, threshold, "contour", projection_peak
    if projection_candidate is not None:
        return projection_candidate, mask, threshold, "projection", projection_peak
    return None, mask, threshold, "none", projection_peak


def put_outlined_text(
    frame: np.ndarray,
    text: str,
    origin: tuple[int, int],
    scale: float,
    color: tuple[int, int, int],
) -> None:
    cv2.putText(
        frame,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (0, 0, 0),
        3,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        1,
        cv2.LINE_AA,
    )


def axis_point(
    left: np.ndarray,
    right: np.ndarray,
    position_cm: float,
    length_cm: float,
) -> np.ndarray:
    fraction = position_cm / length_cm + 0.5
    return left + (right - left) * fraction


def draw_overlay(
    gray: np.ndarray,
    measured_ball: tuple[int, int, int, int, float, float, float] | None,
    tracked_point: tuple[float, float] | None,
    track_source: str,
    measurement_source: str,
    pipe_box: np.ndarray,
    detection_box: np.ndarray,
    position: float | None,
    fps: float,
    processing_ms: float,
    frame_count: int,
    current_steps: int,
    pipe_left_x_px: float,
    confidence: float,
    missed_frames: int,
    settings: dict[str, int | float],
) -> np.ndarray:
    display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    height, width = gray.shape
    stable_box = np.rint(pipe_box).astype(np.int32)
    cv2.polylines(display, [stable_box], True, (255, 255, 0), 2)
    active_box = np.rint(detection_box).astype(np.int32)
    cv2.polylines(display, [active_box], True, (255, 0, 255), 1)

    if measured_ball is not None:
        x, y, box_width, box_height, _cx, _cy, _score = measured_ball
        cv2.rectangle(
            display,
            (x, y),
            (x + box_width - 1, y + box_height - 1),
            (0, 255, 0),
            2,
        )
    if tracked_point is not None:
        marker_color = (0, 0, 255) if track_source == "measured" else (0, 215, 255)
        cv2.drawMarker(
            display,
            (int(round(tracked_point[0])), int(round(tracked_point[1]))),
            marker_color,
            cv2.MARKER_CROSS,
            13,
            2,
        )

    position_text = (
        f"POS {position:+.2f} cm" if position is not None else "POS N/A"
    )
    measured_label = {
        "projection": "MEAS-P",
        "fused": "MEAS-F",
        "contour": "MEAS-C",
    }.get(measurement_source, "MEAS")
    source_label = {
        "measured": measured_label,
        "predicted": "PRED",
        "lost": "LOST",
    }[track_source]
    put_outlined_text(
        display,
        position_text,
        (8, 18),
        0.52,
        (0, 255, 0) if position is not None else (0, 215, 255),
    )
    put_outlined_text(
        display,
        f"{source_label} c={confidence:.2f} miss={missed_frames}",
        (210, 18),
        0.38,
        (180, 255, 180) if track_source == "measured" else (0, 215, 255),
    )
    put_outlined_text(
        display,
        f"FPS {fps:.1f}  S={current_steps}  Lx={pipe_left_x_px:.1f}",
        (405, 18),
        0.34,
        (180, 255, 180),
    )

    left, right = pipe_axis(pipe_box)
    scale_y = height - 13
    line_left = np.array([left[0], scale_y])
    line_right = np.array([right[0], scale_y])
    cv2.line(
        display,
        tuple(np.rint(line_left).astype(int)),
        tuple(np.rint(line_right).astype(int)),
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    length_cm = float(settings["pipe_length_cm"])
    for tick in (-12.5, -5.0, 0.0, 5.0, 12.5):
        point = axis_point(line_left, line_right, tick, length_cm)
        x = int(round(point[0]))
        color = (0, 215, 255) if abs(tick) == 5.0 else (255, 255, 255)
        cv2.line(display, (x, scale_y - 5), (x, scale_y + 5), color, 1)
        label = f"{tick:+g}"
        text_width = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.30,
            1,
        )[0][0]
        put_outlined_text(
            display,
            label,
            (max(0, min(width - text_width, x - text_width // 2)), height - 1),
            0.30,
            color,
        )
    return display


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--jpeg-quality", type=int, default=78)
    parser.add_argument(
        "--axis-status-url",
        default="http://127.0.0.1:8081/status.json",
    )
    parser.add_argument(
        "--settings",
        type=Path,
        default=Path(
            "/home/arick/steel_ball/gray_crop/stabilized_vision_settings.json"
        ),
    )
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--test-frames", type=int, default=120)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cv2.setNumThreads(4)
    settings_store = SettingsStore(args.settings, args.width, args.height)
    camera_controls = CameraControls(args.device)
    camera = open_camera(args)
    initial_settings, _revision = settings_store.snapshot()
    camera_controls.apply(initial_settings)

    hub = FrameHub()
    server = ViewerServer(
        (args.bind_host, args.port),
        hub,
        settings_store,
        camera_controls,
    )
    server_thread = threading.Thread(
        target=server.serve_forever,
        name="stabilized-viewer",
        daemon=True,
    )
    server_thread.start()

    axis_source = AxisPositionSource(args.axis_status_url)
    axis_source.start()
    ball_tracker = BallTracker()
    observed_revision = -1
    frame_count = 0
    detections = 0
    fps_times: deque[float] = deque()
    encode_options = [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality]

    print("Detector: hybrid contour + 1-D projection + Kalman tracking", flush=True)
    print(
        f"Camera: manual exposure={int(initial_settings['exposure'])}",
        flush=True,
    )

    try:
        for _index in range(12):
            camera.read()
        while True:
            ok, full_frame = camera.read()
            if not ok or full_frame is None:
                raise RuntimeError("USB camera frame read failed")

            settings, revision = settings_store.snapshot()
            if revision != observed_revision:
                ball_tracker.reset()
                observed_revision = revision
                print(
                    "Vision settings applied: "
                    f"crop={settings['crop_top']}:{settings['crop_bottom']} "
                    f"exposure={settings['exposure']}",
                    flush=True,
                )

            crop_top = int(settings["crop_top"])
            crop_bottom = int(settings["crop_bottom"])
            frame = full_frame[crop_top:crop_bottom, :]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            processing_start = time.perf_counter()
            current_steps, axis_age_s = axis_source.snapshot()
            pipe_box, pipe_left_x_px = step_interpolated_pipe_box(
                gray.shape,
                current_steps,
                settings,
            )
            axis_synchronized = axis_age_s is not None and axis_age_s <= 0.50
            detection_box = restrict_pipe_box(
                pipe_box,
                float(settings["detect_min_cm"]),
                float(settings["detect_max_cm"]),
                float(settings["pipe_length_cm"]),
                float(settings["detect_top_px"]),
                float(settings["detect_bottom_px"]),
            )
            coordinate_box = pipe_box

            now_monotonic = time.monotonic()
            ball_tracker.predict(now_monotonic, settings)
            gate_prediction = ball_tracker.gate_prediction(settings)
            (
                measured_ball,
                _mask,
                threshold,
                measurement_source,
                projection_peak,
            ) = detect_candidates(
                gray,
                detection_box,
                gate_prediction,
                settings,
            )
            if measured_ball is None:
                measurement = None
                measured_score = 0.0
            else:
                measurement = (measured_ball[4], measured_ball[5])
                measured_score = measured_ball[6]
                detections += 1

            tracked_point, track_source, tracking_confidence = ball_tracker.update(
                measurement,
                measured_score,
                settings,
            )
            position = (
                point_to_cm(
                    tracked_point,
                    coordinate_box,
                    float(settings["pipe_length_cm"]),
                )
                if tracked_point is not None
                else None
            )
            measured_position = (
                point_to_cm(
                    measurement,
                    coordinate_box,
                    float(settings["pipe_length_cm"]),
                )
                if measurement is not None
                else None
            )
            processing_ms = (time.perf_counter() - processing_start) * 1000.0

            frame_count += 1
            now = time.perf_counter()
            fps_times.append(now)
            while fps_times and now - fps_times[0] > 1.0:
                fps_times.popleft()
            fps = (
                (len(fps_times) - 1) / (fps_times[-1] - fps_times[0])
                if len(fps_times) >= 2
                else 0.0
            )

            display = draw_overlay(
                gray,
                measured_ball,
                tracked_point,
                track_source,
                measurement_source,
                pipe_box,
                detection_box,
                position,
                fps,
                processing_ms,
                frame_count,
                current_steps,
                pipe_left_x_px,
                tracking_confidence,
                ball_tracker.missed,
                settings,
            )
            encoded, jpeg = cv2.imencode(".jpg", display, encode_options)
            if not encoded:
                raise RuntimeError("JPEG encoding failed")

            hub.update(
                jpeg.tobytes(),
                {
                    "detector": "illumination-normalized-fusion-v4",
                    "measurement_source": measurement_source,
                    "projection_peak": round(projection_peak, 2),
                    "position_cm": position,
                    "measured_position_cm": measured_position,
                    "track_source": track_source,
                    "tracking_confidence": round(tracking_confidence, 4),
                    "missed_frames": ball_tracker.missed,
                    "shape_score": (
                        float(measured_ball[6])
                        if measured_ball is not None
                        else None
                    ),
                    "pipe_source": "step-interpolated",
                    "pipe_confidence": 1.0 if axis_synchronized else 0.0,
                    "pipe_measured": False,
                    "pipe_box_px": np.rint(pipe_box).astype(int).tolist(),
                    "detection_box_px": np.rint(detection_box).astype(int).tolist(),
                    "detect_range_cm": [
                        float(settings["detect_min_cm"]),
                        float(settings["detect_max_cm"]),
                    ],
                    "pipe_left_x_px": round(pipe_left_x_px, 3),
                    "pipe_right_x_px": int(gray.shape[1] - 1),
                    "axis_steps": current_steps,
                    "axis_status_age_s": (
                        round(axis_age_s, 4)
                        if axis_age_s is not None
                        else None
                    ),
                    "axis_synchronized": axis_synchronized,
                    "fps": round(fps, 2),
                    "processing_ms": round(processing_ms, 2),
                    "threshold": threshold,
                    "crop": [crop_top, crop_bottom],
                    "exposure": int(settings["exposure"]),
                    "illumination_normalized": bool(
                        int(settings["illumination_normalization_enabled"])
                    ),
                    "pipe_length_cm": float(settings["pipe_length_cm"]),
                    "timestamp": round(time.time(), 3),
                    "settings_revision": revision,
                },
            )

            if args.self_test and frame_count >= args.test_frames:
                print(
                    f"SELF_TEST_OK frames={frame_count} fps={fps:.2f} "
                    f"detections={detections} source={track_source} "
                    f"position_cm={position if position is not None else 'N/A'}",
                    flush=True,
                )
                break
    except KeyboardInterrupt:
        print("Stopped.", flush=True)
    finally:
        axis_source.stop()
        hub.stop()
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=2)
        camera.release()


if __name__ == "__main__":
    main()
