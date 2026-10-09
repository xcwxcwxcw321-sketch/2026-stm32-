from __future__ import annotations

import json
import os
import sys
import urllib.request
from pathlib import Path

import paramiko


HOST = "192.168.252.136"
USER = "arick"


def main() -> int:
    password = os.environ["D36A_PI_PASSWORD"]
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        HOST,
        username=USER,
        password=password,
        timeout=8,
        auth_timeout=8,
        banner_timeout=8,
    )
    command = """python3 - <<'PY'
import json
import urllib.request

result = {}
for name, url in (
    ("control", "http://127.0.0.1:8081/status.json"),
    ("vision", "http://127.0.0.1:8080/status.json"),
    ("vision_settings", "http://127.0.0.1:8080/settings.json"),
):
    try:
        with urllib.request.urlopen(url, timeout=1) as response:
            result[name] = json.load(response)
    except Exception as error:
        result[name] = {"error": str(error)}
try:
    import importlib.util
    detector_path = "/home/arick/steel_ball/gray_crop/detect_ball_stabilized.py"
    spec = importlib.util.spec_from_file_location("deployed_detector", detector_path)
    detector = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(detector)
    settings = result["vision_settings"]["settings"]
    result["interpolation"] = {
        str(step): round(detector.pipe_left_x_from_steps(step, settings), 3)
        for step in (0, 117, 235, 458, 681)
    }
except Exception as error:
    result["interpolation"] = {"error": str(error)}
try:
    with urllib.request.urlopen("http://127.0.0.1:8081/", timeout=1) as response:
        page = response.read().decode("utf-8", "replace")
    result["ui"] = {
        "step_interpolation_text": "40→14→0 px" in page,
        "pipe_left_x_field": "pipe_left_x_step0_px" in page,
        "pipe_left_x_status": 'id="pipeLeftX"' in page,
    }
except Exception as error:
    result["ui"] = {"error": str(error)}
print(json.dumps(result, ensure_ascii=False))
PY"""
    _stdin, stdout, stderr = client.exec_command(command, timeout=12)
    output = stdout.read().decode("utf-8", "replace").strip()
    error = stderr.read().decode("utf-8", "replace").strip()
    status = stdout.channel.recv_exit_status()
    client.close()
    if error:
        print(error, file=sys.stderr)
    if status:
        return status
    payload = json.loads(output)
    control = payload.get("control", {})
    vision = payload.get("vision", {})
    vision_settings = payload.get("vision_settings", {}).get("settings", {})
    summary = {
        "control": {
            key: control.get(key)
            for key in (
                "current_steps",
                "axis_target_steps",
                "axis_referenced",
                "driver_enabled",
                "control_enabled",
                "safety_reason",
            )
        },
        "vision": {
            key: vision.get(key)
            for key in (
                "position_cm",
                "track_source",
                "tracking_confidence",
                "pipe_source",
                "pipe_box_px",
                "pipe_left_x_px",
                "pipe_right_x_px",
                "axis_steps",
                "axis_status_age_s",
                "axis_synchronized",
                "fps",
                "exposure",
            )
        },
        "vision_settings": vision_settings,
        "interpolation": payload.get("interpolation"),
        "ui": payload.get("ui"),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    snapshot = (
        Path(__file__).resolve().parents[1]
        / "work"
        / "vision_rework"
        / "step_interpolated_live.jpg"
    )
    with urllib.request.urlopen(
        f"http://{HOST}:8080/snapshot.jpg",
        timeout=5,
    ) as response:
        snapshot.write_bytes(response.read())
    print(f"snapshot: {snapshot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
