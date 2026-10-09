#!/usr/bin/env python3
"""Vision-to-stepper balance controller and local tuning web UI.

The grayscale detector remains the camera owner on port 8080.  This process
reads its JSON position, calculates a target around the calibrated horizontal
step, and sends small, bounded moves to the existing ``d36a-cal`` tmux
session.  The calibration process remains the only GPIO owner and enforces the
absolute 0..max_steps software limits.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import threading
import time
import urllib.request
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from d36a_stepper_web import StepperBridge


DEFAULT_SETTINGS = {
    "target_cm": 0.0,
    "kp_steps_per_cm": 6.0,
    "ki_steps_per_cm_s": 0.0,
    "kd_steps_per_cm_s": 1.5,
    "integral_limit": 20.0,
    "deadband_cm": 0.15,
    "max_offset_steps": 60,
    "max_command_steps": 5,
    "control_direction": 1,
    "loop_hz": 12.0,
    "velocity_alpha": 0.35,
    "vision_timeout_s": 0.30,
}


HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>小球步进平衡控制</title>
  <style>
    :root { color-scheme:dark; font-family:"Microsoft YaHei",sans-serif; }
    * { box-sizing:border-box; }
    body { margin:0; background:#0d1117; color:#edf3f8; }
    main { width:min(98vw,1320px); margin:auto; padding:12px; }
    h1 { margin:0 0 4px; font-size:22px; }
    h2 { margin:0 0 8px; font-size:16px; color:#58a6ff; }
    .subtitle { color:#9ba7b4; font-size:13px; margin-bottom:9px; }
    .viewer { overflow:hidden; border:2px solid #34404a; border-radius:10px; background:#000; }
    .viewer img { display:block; width:100%; min-height:120px; object-fit:contain; }
    .cards { display:grid; grid-template-columns:repeat(6,minmax(105px,1fr)); gap:7px; margin:9px 0; }
    .card,.panel { background:#161b22; border:1px solid #30363d; border-radius:9px; padding:10px; }
    .panel { margin-top:9px; }
    .label { color:#9ba7b4; font-size:12px; }
    .value { margin-top:3px; font:700 18px Consolas,monospace; }
    .row { display:flex; flex-wrap:wrap; gap:8px; align-items:center; margin-top:8px; }
    .grid { display:grid; grid-template-columns:repeat(4,minmax(145px,1fr)); gap:8px; }
    label.field { color:#9ba7b4; font-size:12px; }
    input,select { width:100%; margin-top:4px; background:#0d1117; color:#fff; border:1px solid #59636e; border-radius:6px; padding:8px; font-size:15px; }
    input.dirty,select.dirty { border-color:#d29922; box-shadow:0 0 0 1px #d29922; }
    button { border:0; border-radius:7px; padding:10px 15px; color:#fff; background:#238636; font-size:15px; cursor:pointer; }
    button.down { background:#1f6feb; }
    button.secondary { background:#30363d; }
    button.warn { background:#9e6a03; }
    button.stop { background:#da3633; }
    #notice { min-height:22px; margin-top:8px; }
    .ok { color:#3fb950; } .bad { color:#f85149; } .warn-text { color:#d29922; }
    .hint { color:#9ba7b4; font-size:13px; line-height:1.5; margin-top:7px; }
    @media(max-width:900px) {
      .cards { grid-template-columns:repeat(3,1fr); }
      .grid { grid-template-columns:repeat(2,1fr); }
    }
  </style>
</head>
<body>
<main>
  <h1>25 cm水管 · 视觉步进平衡控制</h1>
  <div class="subtitle">坐标：水管框左端 -12.5 cm，中间 0，右端 +12.5 cm</div>
  <div class="viewer"><img id="vision" alt="实时二值化画面"></div>

  <div class="cards">
    <div class="card"><div class="label">控制状态</div><div class="value" id="mode">--</div></div>
    <div class="card"><div class="label">小球位置</div><div class="value" id="ball">N/A</div></div>
    <div class="card"><div class="label">目标位置</div><div class="value" id="target">0.00</div></div>
    <div class="card"><div class="label">位置误差</div><div class="value" id="error">N/A</div></div>
    <div class="card"><div class="label">小球速度</div><div class="value" id="velocity">N/A</div></div>
    <div class="card"><div class="label">PID输出</div><div class="value" id="output">0</div></div>
    <div class="card"><div class="label">当前步数</div><div class="value" id="position">--</div></div>
    <div class="card"><div class="label">目标步数</div><div class="value" id="desired">--</div></div>
    <div class="card"><div class="label">最低限位</div><div class="value">0</div></div>
    <div class="card"><div class="label">水平位置</div><div class="value" id="balance">235</div></div>
    <div class="card"><div class="label">最高限位</div><div class="value" id="maximum">681</div></div>
    <div class="card"><div class="label">视觉帧率</div><div class="value" id="fps">--</div></div>
  </div>

  <div class="panel">
    <h2>闭环控制</h2>
    <div class="row">
      <button onclick="action('enable')">开启视觉闭环</button>
      <button class="warn" onclick="action('disable')">停止闭环并保持</button>
      <button class="secondary" onclick="action('return_balance')">缓慢回水平235</button>
      <button class="stop" onclick="emergencyStop()">紧急停止并释放</button>
    </div>
    <div id="notice" class="warn-text">控制默认关闭，正在读取状态……</div>
  </div>

  <div class="panel">
    <h2>PID与安全参数</h2>
    <div class="grid">
      <label class="field">目标位置 cm
        <input data-key="target_cm" type="number" min="-12.5" max="12.5" step="0.1">
      </label>
      <label class="field">Kp 步/cm
        <input data-key="kp_steps_per_cm" type="number" min="0" max="100" step="0.1">
      </label>
      <label class="field">Ki 步/(cm·s)
        <input data-key="ki_steps_per_cm_s" type="number" min="0" max="50" step="0.05">
      </label>
      <label class="field">Kd 步/(cm/s)
        <input data-key="kd_steps_per_cm_s" type="number" min="0" max="50" step="0.1">
      </label>
      <label class="field">积分限幅
        <input data-key="integral_limit" type="number" min="0" max="100" step="1">
      </label>
      <label class="field">位置死区 cm
        <input data-key="deadband_cm" type="number" min="0" max="2" step="0.05">
      </label>
      <label class="field">最大偏离水平步数
        <input data-key="max_offset_steps" type="number" min="1" max="235" step="1">
      </label>
      <label class="field">每次命令最大步数
        <input data-key="max_command_steps" type="number" min="1" max="20" step="1">
      </label>
      <label class="field">控制方向
        <select data-key="control_direction">
          <option value="1">+1</option>
          <option value="-1">-1</option>
        </select>
      </label>
      <label class="field">控制频率 Hz
        <input data-key="loop_hz" type="number" min="2" max="25" step="1">
      </label>
      <label class="field">速度滤波系数
        <input data-key="velocity_alpha" type="number" min="0.02" max="1" step="0.05">
      </label>
      <label class="field">视觉丢失超时 s
        <input data-key="vision_timeout_s" type="number" min="0.1" max="2" step="0.05">
      </label>
    </div>
    <div class="row">
      <button onclick="applySettings(false)">应用参数</button>
      <button class="secondary" onclick="applySettings(true)">保存参数</button>
    </div>
    <div class="hint">
      初始建议：Kp=6，Ki=0，Kd=1.5，最大偏移60步。若小球越跑越远，先停止闭环，
      把“控制方向”从+1改为-1。视觉丢失后会停止闭环并分段回到235。
    </div>
  </div>

  <div class="panel">
    <h2>手动步进（闭环关闭时使用）</h2>
    <div class="row">
      <button class="down" onclick="move(-10)">-10</button>
      <button class="down" onclick="move(-5)">-5</button>
      <button class="down" onclick="move(-1)">-1</button>
      <button onclick="move(1)">+1</button>
      <button onclick="move(5)">+5</button>
      <button onclick="move(10)">+10</button>
      <input id="gotoValue" type="number" min="0" max="681" step="1" value="235" style="width:110px">
      <button class="secondary" onclick="gotoPosition()">移动到绝对步数</button>
    </div>
  </div>
</main>
<script>
document.getElementById("vision").src =
  location.protocol + "//" + location.hostname + ":8080/stream.mjpg";
const fields = [...document.querySelectorAll("[data-key]")];
let initialized = false;
fields.forEach(item => item.addEventListener("input", () => item.classList.add("dirty")));

function format(value, digits=2, suffix="") {
  return value === null || value === undefined ? "N/A" : Number(value).toFixed(digits) + suffix;
}
function notice(message, kind="ok") {
  const item = document.getElementById("notice");
  item.textContent = message;
  item.className = kind;
}
async function post(payload) {
  const response = await fetch("/control", {
    method:"POST", headers:{"Content-Type":"application/json"},
    body:JSON.stringify(payload)
  });
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || "操作失败");
  notice(data.message || "操作完成");
  setTimeout(refresh, 150);
}
function action(name) {
  post({action:name}).catch(error => notice(error.message,"bad"));
}
function move(steps) {
  post({action:"move",steps}).catch(error => notice(error.message,"bad"));
}
function gotoPosition() {
  const position = Number(document.getElementById("gotoValue").value);
  post({action:"goto",position}).catch(error => notice(error.message,"bad"));
}
function emergencyStop() {
  if (confirm("立即停止并释放电机？之后必须重新从最低点ZERO。"))
    action("emergency_stop");
}
function applySettings(persist) {
  const settings = {};
  fields.forEach(item => settings[item.dataset.key] =
    item.tagName === "SELECT" ? Number(item.value) : Number(item.value));
  post({action:"settings",settings,persist})
    .then(() => fields.forEach(item => item.classList.remove("dirty")))
    .catch(error => notice(error.message,"bad"));
}
async function refresh() {
  try {
    const response = await fetch("/status.json", {cache:"no-store"});
    const data = await response.json();
    document.getElementById("mode").textContent =
      data.control_enabled ? "闭环运行" : (data.returning_to_balance ? "回水平" : "已停止");
    document.getElementById("ball").textContent = format(data.ball_position_cm,2," cm");
    document.getElementById("target").textContent = format(data.target_cm,2," cm");
    document.getElementById("error").textContent = format(data.position_error_cm,2," cm");
    document.getElementById("velocity").textContent = format(data.velocity_cm_s,2," cm/s");
    document.getElementById("output").textContent = format(data.control_output_steps,1," 步");
    document.getElementById("position").textContent = data.position_steps ?? "--";
    document.getElementById("desired").textContent = data.desired_steps ?? "--";
    document.getElementById("balance").textContent = data.balance_steps ?? "--";
    document.getElementById("maximum").textContent = data.max_steps ?? "--";
    document.getElementById("fps").textContent = format(data.vision_fps,1);
    if (!initialized) {
      fields.forEach(item => {
        if (data[item.dataset.key] !== undefined) item.value = data[item.dataset.key];
      });
      initialized = true;
    }
    if (data.safety_reason) notice(data.safety_reason,"warn-text");
    if (!data.axis_running) notice("步进轴会话未运行","bad");
  } catch (error) {
    notice("状态读取失败："+error.message,"bad");
  }
}
refresh();
setInterval(refresh, 500);
</script>
</body>
</html>
"""


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


class BalanceState:
    INTEGER_KEYS = {
        "max_offset_steps",
        "max_command_steps",
        "control_direction",
    }

    def __init__(self, settings_path: Path) -> None:
        self.lock = threading.Lock()
        self.settings_path = settings_path
        self.settings = dict(DEFAULT_SETTINGS)
        self.control_enabled = False
        self.returning_to_balance = False
        self.safety_reason = ""
        self.runtime: dict[str, object] = {
            "axis_running": False,
            "axis_ready": False,
            "position_steps": None,
            "desired_steps": None,
            "min_steps": 0,
            "balance_steps": None,
            "max_steps": None,
            "ball_position_cm": None,
            "position_error_cm": None,
            "velocity_cm_s": None,
            "control_output_steps": 0.0,
            "vision_fps": None,
            "vision_age_s": None,
            "integral": 0.0,
        }
        self._load()

    def _convert(self, values: dict[str, object]) -> dict[str, int | float]:
        converted: dict[str, int | float] = {}
        for key, default in DEFAULT_SETTINGS.items():
            value = values.get(key, default)
            converted[key] = (
                int(value) if key in self.INTEGER_KEYS else float(value)
            )
        return converted

    def _validate(self, values: dict[str, int | float]) -> None:
        checks = (
            ("target_cm", float(values["target_cm"]), -12.5, 12.5),
            ("kp_steps_per_cm", float(values["kp_steps_per_cm"]), 0.0, 100.0),
            ("ki_steps_per_cm_s", float(values["ki_steps_per_cm_s"]), 0.0, 50.0),
            ("kd_steps_per_cm_s", float(values["kd_steps_per_cm_s"]), 0.0, 50.0),
            ("integral_limit", float(values["integral_limit"]), 0.0, 100.0),
            ("deadband_cm", float(values["deadband_cm"]), 0.0, 2.0),
            ("max_offset_steps", float(values["max_offset_steps"]), 1.0, 235.0),
            ("max_command_steps", float(values["max_command_steps"]), 1.0, 20.0),
            ("loop_hz", float(values["loop_hz"]), 2.0, 25.0),
            ("velocity_alpha", float(values["velocity_alpha"]), 0.02, 1.0),
            ("vision_timeout_s", float(values["vision_timeout_s"]), 0.1, 2.0),
        )
        for name, value, lower, upper in checks:
            if not lower <= value <= upper:
                raise ValueError(f"{name}必须在{lower:g}～{upper:g}范围内")
        if int(values["control_direction"]) not in (-1, 1):
            raise ValueError("control_direction只能为-1或+1")

    def _load(self) -> None:
        if not self.settings_path.exists():
            return
        try:
            payload = json.loads(self.settings_path.read_text(encoding="utf-8"))
            values = self._convert(payload)
            self._validate(values)
            self.settings.update(values)
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
            print(f"Ignoring invalid controller settings: {error}", flush=True)

    def update_settings(
        self,
        values: dict[str, object],
        persist: bool,
    ) -> None:
        with self.lock:
            merged = dict(self.settings)
            for key in DEFAULT_SETTINGS:
                if key in values:
                    merged[key] = values[key]
            converted = self._convert(merged)
            self._validate(converted)
            self.settings.update(converted)
            self.safety_reason = ""
            if persist:
                self.settings_path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self.settings_path.with_suffix(
                    self.settings_path.suffix + ".tmp"
                )
                temporary.write_text(
                    json.dumps(self.settings, ensure_ascii=False, indent=2)
                    + "\n",
                    encoding="utf-8",
                )
                temporary.replace(self.settings_path)

    def settings_snapshot(self) -> dict[str, int | float]:
        with self.lock:
            return dict(self.settings)

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            return {
                **self.settings,
                **self.runtime,
                "control_enabled": self.control_enabled,
                "returning_to_balance": self.returning_to_balance,
                "safety_reason": self.safety_reason,
            }

    def set_runtime(self, values: dict[str, object]) -> None:
        with self.lock:
            self.runtime.update(values)

    def enable(self) -> None:
        with self.lock:
            self.control_enabled = True
            self.returning_to_balance = False
            self.safety_reason = ""

    def disable(self, reason: str = "") -> None:
        with self.lock:
            self.control_enabled = False
            self.returning_to_balance = False
            self.safety_reason = reason

    def begin_return(self, reason: str = "") -> None:
        with self.lock:
            self.control_enabled = False
            self.returning_to_balance = True
            self.safety_reason = reason

    def finish_return(self) -> None:
        with self.lock:
            self.returning_to_balance = False
            self.safety_reason = "已回到水平位置235"


class BalanceController:
    PROMPT_PATTERN = re.compile(r"step\[(\d+)\]>\s*$")

    def __init__(
        self,
        bridge: StepperBridge,
        state: BalanceState,
        vision_status_url: str,
        calibration_path: Path,
    ) -> None:
        self.bridge = bridge
        self.state = state
        self.vision_status_url = vision_status_url
        self.calibration_path = calibration_path
        self.command_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run,
            name="stepper-balance-loop",
            daemon=True,
        )

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=2)

    def _vision(self) -> tuple[float | None, float | None, float | None]:
        try:
            with urllib.request.urlopen(
                self.vision_status_url,
                timeout=0.35,
            ) as response:
                payload = json.load(response)
            position = (
                float(payload["position_cm"])
                if payload.get("position_cm") is not None
                else None
            )
            fps = (
                float(payload["fps"])
                if payload.get("fps") is not None
                else None
            )
            timestamp = (
                float(payload["timestamp"])
                if payload.get("timestamp") is not None
                else None
            )
            return position, fps, timestamp
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return None, None, None

    def _axis(self) -> tuple[bool, bool, int | None, int | None, int | None]:
        if not self.bridge.session_running():
            return False, False, None, None, None
        pane = self.bridge.capture()
        match = self.PROMPT_PATTERN.search(pane)
        position = int(match.group(1)) if match else None
        ready = match is not None
        maximum = None
        balance = None
        try:
            payload = json.loads(
                self.calibration_path.read_text(encoding="utf-8")
            )
            if payload.get("max_steps") is not None:
                maximum = int(payload["max_steps"])
            if payload.get("balance_steps") is not None:
                balance = int(payload["balance_steps"])
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            pass
        return True, ready, position, balance, maximum

    def _send(self, command: str) -> None:
        with self.command_lock:
            self.bridge.send_literal(command)

    def manual(self, action: str, value: int | None = None) -> str:
        snapshot = self.state.snapshot()
        if snapshot["control_enabled"] or snapshot["returning_to_balance"]:
            raise RuntimeError("请先停止闭环和回水平动作")
        axis_running, ready, _position, _balance, _maximum = self._axis()
        if not axis_running:
            raise RuntimeError("步进标定会话未运行")
        if not ready:
            raise RuntimeError("电机正在执行上一条命令")
        if action == "move" and value is not None:
            if value == 0 or abs(value) > 20:
                raise ValueError("单次移动必须在-20～+20步且不能为0")
            command = f"+ {value}" if value > 0 else f"- {abs(value)}"
            self._send(command)
            return f"已发送相对移动{value:+d}步"
        if action == "goto" and value is not None:
            if not 0 <= value <= 681:
                raise ValueError("绝对位置必须在0～681步")
            self._send(f"goto {value}")
            return f"已发送绝对位置{value}"
        raise ValueError("无效手动命令")

    def enable(self) -> None:
        running, ready, position, balance, maximum = self._axis()
        ball, _fps, _timestamp = self._vision()
        if not running or not ready or position is None:
            raise RuntimeError("步进轴未就绪")
        if balance is None or maximum is None:
            raise RuntimeError("必须先标定水平点和最高限位")
        if ball is None:
            raise RuntimeError("当前没有识别到小球")
        self.state.enable()

    def emergency_stop(self) -> None:
        self.state.disable("紧急停止：必须重新从最低点ZERO")
        with self.command_lock:
            if self.bridge.session_running():
                result = subprocess.run(
                    ["tmux", "send-keys", "-t", self.bridge.session, "C-c"],
                    check=False,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=2,
                )
                if result.returncode != 0:
                    raise RuntimeError("发送紧急停止失败")

    def _run(self) -> None:
        last_time = time.monotonic()
        last_ball: float | None = None
        last_seen = last_time
        filtered_velocity = 0.0
        integral = 0.0

        while not self.stop_event.is_set():
            cycle_start = time.monotonic()
            settings = self.state.settings_snapshot()
            dt = max(0.001, cycle_start - last_time)
            last_time = cycle_start

            ball, vision_fps, vision_timestamp = self._vision()
            if ball is not None:
                if last_ball is None:
                    raw_velocity = 0.0
                else:
                    raw_velocity = clamp(
                        (ball - last_ball) / dt,
                        -80.0,
                        80.0,
                    )
                alpha = float(settings["velocity_alpha"])
                filtered_velocity = (
                    (1.0 - alpha) * filtered_velocity
                    + alpha * raw_velocity
                )
                last_ball = ball
                last_seen = cycle_start
            else:
                filtered_velocity *= 0.85

            axis_running, axis_ready, position, balance, maximum = self._axis()
            current_state = self.state.snapshot()
            enabled = bool(current_state["control_enabled"])
            returning = bool(current_state["returning_to_balance"])
            desired_steps = position
            error: float | None = (
                float(settings["target_cm"]) - ball
                if ball is not None
                else None
            )
            output = 0.0

            if enabled:
                if not axis_running or balance is None or maximum is None:
                    self.state.disable("轴或标定参数丢失，闭环已停止")
                    enabled = False
                elif cycle_start - last_seen > float(
                    settings["vision_timeout_s"]
                ):
                    self.state.begin_return(
                        "视觉丢失，闭环已停止并回水平"
                    )
                    enabled = False
                    returning = True
                    integral = 0.0
                elif ball is not None:
                    effective_error = (
                        0.0
                        if abs(error) <= float(settings["deadband_cm"])
                        else error
                    )
                    if effective_error == 0.0:
                        integral *= 0.92
                    else:
                        integral = clamp(
                            integral + effective_error * dt,
                            -float(settings["integral_limit"]),
                            float(settings["integral_limit"]),
                        )
                    raw_output = (
                        float(settings["kp_steps_per_cm"])
                        * effective_error
                        + float(settings["ki_steps_per_cm_s"]) * integral
                        - float(settings["kd_steps_per_cm_s"])
                        * filtered_velocity
                    )
                    output = (
                        int(settings["control_direction"])
                        * clamp(
                            raw_output,
                            -float(settings["max_offset_steps"]),
                            float(settings["max_offset_steps"]),
                        )
                    )
                    desired_steps = int(
                        round(
                            clamp(
                                balance + output,
                                0,
                                maximum,
                            )
                        )
                    )

            current_state = self.state.snapshot()
            returning = bool(current_state["returning_to_balance"])
            if returning and balance is not None:
                desired_steps = balance
                output = float(balance - (position or 0))
                if position == balance:
                    self.state.finish_return()
                    returning = False

            if (
                axis_ready
                and position is not None
                and desired_steps is not None
                and (enabled or returning)
                and position != desired_steps
            ):
                max_command = int(settings["max_command_steps"])
                delta = int(
                    clamp(
                        desired_steps - position,
                        -max_command,
                        max_command,
                    )
                )
                command_target = position + delta
                try:
                    self._send(f"goto {command_target}")
                except (RuntimeError, OSError, subprocess.SubprocessError) as error:
                    self.state.disable(f"步进命令失败：{error}")

            vision_age = (
                max(0.0, time.time() - vision_timestamp)
                if vision_timestamp is not None
                else None
            )
            self.state.set_runtime(
                {
                    "axis_running": axis_running,
                    "axis_ready": axis_ready,
                    "position_steps": position,
                    "desired_steps": desired_steps,
                    "min_steps": 0,
                    "balance_steps": balance,
                    "max_steps": maximum,
                    "ball_position_cm": ball,
                    "position_error_cm": error,
                    "velocity_cm_s": (
                        filtered_velocity if ball is not None else None
                    ),
                    "control_output_steps": round(output, 2),
                    "vision_fps": vision_fps,
                    "vision_age_s": vision_age,
                    "integral": round(integral, 4),
                }
            )

            interval = 1.0 / float(settings["loop_hz"])
            self.stop_event.wait(max(0.005, interval - (time.monotonic() - cycle_start)))


class BalanceServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address: tuple[str, int],
        state: BalanceState,
        controller: BalanceController,
    ) -> None:
        self.state = state
        self.controller = controller
        super().__init__(address, BalanceHandler)


class BalanceHandler(BaseHTTPRequestHandler):
    server: BalanceServer

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
        self.send_header("Cache-Control", "no-store")
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

    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if path == "/":
            self.send_bytes(HTML.encode("utf-8"), "text/html; charset=utf-8")
        elif path == "/status.json":
            self.send_json(self.server.state.snapshot())
        else:
            self.send_bytes(
                b"Not found", "text/plain", HTTPStatus.NOT_FOUND
            )

    def do_POST(self) -> None:
        if urlsplit(self.path).path != "/control":
            self.send_bytes(
                b"Not found", "text/plain", HTTPStatus.NOT_FOUND
            )
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 8192:
                raise ValueError("请求大小无效")
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict):
                raise ValueError("JSON必须是对象")
            action = payload.get("action")
            if action == "enable":
                self.server.controller.enable()
                message = "视觉闭环已开启"
            elif action == "disable":
                self.server.state.disable("")
                message = "闭环已停止，保持当前位置"
            elif action == "return_balance":
                self.server.state.begin_return("正在分段回水平235")
                message = "开始分段回水平位置"
            elif action == "emergency_stop":
                self.server.controller.emergency_stop()
                message = "已紧急停止并释放"
            elif action == "move":
                value = payload.get("steps")
                if isinstance(value, bool) or not isinstance(value, int):
                    raise ValueError("steps必须是整数")
                message = self.server.controller.manual("move", value)
            elif action == "goto":
                value = payload.get("position")
                if isinstance(value, bool) or not isinstance(value, int):
                    raise ValueError("position必须是整数")
                message = self.server.controller.manual("goto", value)
            elif action == "settings":
                settings = payload.get("settings")
                if not isinstance(settings, dict):
                    raise ValueError("settings必须是对象")
                self.server.state.update_settings(
                    settings,
                    bool(payload.get("persist", False)),
                )
                message = (
                    "参数已保存" if payload.get("persist") else "参数已应用"
                )
            else:
                raise ValueError("不支持的控制命令")
            self.send_json({"ok": True, "message": message})
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
        except (RuntimeError, OSError, subprocess.SubprocessError) as error:
            self.send_json({"error": str(error)}, HTTPStatus.CONFLICT)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--session", default="d36a-cal")
    parser.add_argument(
        "--calibration",
        type=Path,
        default=Path("/home/arick/d36a_test/axis_calibration.json"),
    )
    parser.add_argument(
        "--settings",
        type=Path,
        default=Path(
            "/home/arick/d36a_test/stepper_controller_settings.json"
        ),
    )
    parser.add_argument(
        "--vision-status-url",
        default="http://127.0.0.1:8080/status.json",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    bridge = StepperBridge(
        args.session,
        args.calibration,
        args.vision_status_url,
    )
    state = BalanceState(args.settings)
    controller = BalanceController(
        bridge,
        state,
        args.vision_status_url,
        args.calibration,
    )
    server = BalanceServer((args.host, args.port), state, controller)
    controller.start()
    print(
        f"Balance tuning panel: http://{args.host}:{args.port}/ "
        "(closed loop starts disabled)",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        state.disable("控制程序已停止")
        controller.stop()
        server.server_close()


if __name__ == "__main__":
    main()
