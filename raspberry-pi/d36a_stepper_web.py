#!/usr/bin/env python3
"""Local web panel for the D36A tmux calibration session.

The panel does not claim any GPIO.  It sends whitelisted commands to the
already-running ``d36a-cal`` tmux session, so the calibration program remains
the single owner of EN1, DIR1, and STEP1 and continues to enforce its software
position limits.
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


HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>D36A步进位置调试</title>
  <style>
    :root { color-scheme:dark; font-family:"Microsoft YaHei",sans-serif; }
    * { box-sizing:border-box; }
    body { margin:0; background:#0d1117; color:#edf3f8; }
    main { width:min(98vw,1280px); margin:auto; padding:12px; }
    h1 { margin:0 0 8px; font-size:22px; }
    .viewer { overflow:hidden; border:2px solid #34404a; border-radius:10px; background:#000; }
    .viewer img { display:block; width:100%; min-height:120px; object-fit:contain; }
    .cards { display:grid; grid-template-columns:repeat(4,minmax(120px,1fr)); gap:8px; margin:10px 0; }
    .card,.panel { background:#161b22; border:1px solid #30363d; border-radius:9px; padding:10px; }
    .label { color:#9ba7b4; font-size:12px; }
    .value { margin-top:3px; font:700 21px Consolas,monospace; }
    .row { display:flex; flex-wrap:wrap; gap:8px; align-items:center; margin-top:9px; }
    button { border:0; border-radius:7px; padding:11px 17px; color:#fff; background:#238636; font-size:16px; cursor:pointer; }
    button.down { background:#1f6feb; }
    button.secondary { background:#30363d; }
    button.warn { background:#9e6a03; }
    button.stop { background:#da3633; }
    input { width:120px; background:#0d1117; color:#fff; border:1px solid #59636e; border-radius:6px; padding:10px; font-size:16px; }
    #notice { min-height:23px; margin-top:8px; }
    .ok { color:#3fb950; } .bad { color:#f85149; } .warn-text { color:#d29922; }
    .hint { color:#9ba7b4; font-size:13px; line-height:1.55; margin-top:8px; }
    @media(max-width:720px) { .cards { grid-template-columns:repeat(2,1fr); } }
  </style>
</head>
<body>
<main>
  <h1>D36A步进位置与视觉调试</h1>
  <div class="viewer"><img id="vision" alt="实时二值化画面"></div>
  <div class="cards">
    <div class="card"><div class="label">当前步数</div><div class="value" id="position">--</div></div>
    <div class="card"><div class="label">最高限位</div><div class="value" id="maximum">未标定</div></div>
    <div class="card"><div class="label">水平位置</div><div class="value" id="balance">未标定</div></div>
    <div class="card"><div class="label">小球位置</div><div class="value" id="ball">N/A</div></div>
  </div>
  <div class="panel">
    <div class="label">相对移动</div>
    <div class="row">
      <button class="down" onclick="move(-10)">-10</button>
      <button class="down" onclick="move(-5)">-5</button>
      <button class="down" onclick="move(-1)">-1</button>
      <button onclick="move(1)">+1</button>
      <button onclick="move(5)">+5</button>
      <button onclick="move(10)">+10</button>
    </div>
    <div class="row">
      <input id="gotoValue" type="number" min="0" max="2000" step="1" value="0">
      <button class="secondary" onclick="gotoPosition()">移动到绝对步数</button>
      <button class="warn" onclick="setMaximum()">记录当前为最高点</button>
      <button class="warn" onclick="setBalance()">记录当前为水平点</button>
      <button class="secondary" onclick="command('center')">回水平点</button>
      <button class="stop" onclick="emergencyStop()">紧急停止并释放</button>
    </div>
    <div id="notice" class="warn-text">正在读取状态……</div>
    <div class="hint">
      最低点固定为0。未记录最高点前，临时上限为标定程序的probe ceiling；
      记录最高点后，底层程序会拒绝超出0～最高点的命令。
    </div>
  </div>
</main>
<script>
const vision = document.getElementById("vision");
vision.src = location.protocol + "//" + location.hostname + ":8080/stream.mjpg";

function showNotice(message, kind="ok") {
  const item = document.getElementById("notice");
  item.textContent = message;
  item.className = kind;
}

async function post(payload) {
  const response = await fetch("/control", {
    method:"POST",
    headers:{"Content-Type":"application/json"},
    body:JSON.stringify(payload)
  });
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || "操作失败");
  showNotice(data.message || "命令已发送");
  setTimeout(refresh, 180);
}

function move(steps) {
  post({action:"move",steps}).catch(error => showNotice(error.message,"bad"));
}

function gotoPosition() {
  const position = Number(document.getElementById("gotoValue").value);
  post({action:"goto",position}).catch(error => showNotice(error.message,"bad"));
}

function command(action) {
  post({action}).catch(error => showNotice(error.message,"bad"));
}

function setMaximum() {
  if (confirm("把当前位置保存为安全最高点？之后程序将拒绝越过这里。")) command("set_maximum");
}

function setBalance() {
  if (confirm("确认水管现在处于水平位置？")) command("set_balance");
}

function emergencyStop() {
  if (confirm("立即停止并结束标定会话？之后必须重新从最低点ZERO。")) command("emergency_stop");
}

async function refresh() {
  try {
    const response = await fetch("/status.json", {cache:"no-store"});
    const data = await response.json();
    document.getElementById("position").textContent =
      data.position_steps === null ? "--" : data.position_steps;
    document.getElementById("maximum").textContent =
      data.max_steps === null ? "未标定" : data.max_steps;
    document.getElementById("balance").textContent =
      data.balance_steps === null ? "未标定" : data.balance_steps;
    document.getElementById("ball").textContent =
      data.ball_position_cm === null ? "N/A" : data.ball_position_cm.toFixed(2) + " cm";
    if (!data.axis_running) showNotice("步进标定会话未运行","bad");
  } catch (error) {
    showNotice("状态读取失败：" + error.message,"bad");
  }
}

refresh();
setInterval(refresh, 800);
</script>
</body>
</html>
"""


class StepperBridge:
    POSITION_PATTERN = re.compile(r"step\[(\d+)\]>")

    def __init__(
        self,
        session: str,
        config_path: Path,
        vision_status_url: str,
    ) -> None:
        self.session = session
        self.config_path = config_path
        self.vision_status_url = vision_status_url
        self.lock = threading.Lock()

    def _run(self, arguments: list[str]) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            arguments,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=2.0,
        )

    def session_running(self) -> bool:
        return (
            self._run(["tmux", "has-session", "-t", self.session]).returncode
            == 0
        )

    def capture(self) -> str:
        result = self._run(
            ["tmux", "capture-pane", "-p", "-t", self.session, "-S", "-80"]
        )
        return result.stdout if result.returncode == 0 else ""

    def send_literal(self, command: str) -> None:
        if not self.session_running():
            raise RuntimeError("步进标定会话未运行")
        typed = self._run(
            ["tmux", "send-keys", "-t", self.session, "-l", command]
        )
        if typed.returncode != 0:
            raise RuntimeError(typed.stderr.strip() or "发送命令失败")
        entered = self._run(
            ["tmux", "send-keys", "-t", self.session, "Enter"]
        )
        if entered.returncode != 0:
            raise RuntimeError(entered.stderr.strip() or "发送回车失败")

    def status(self) -> dict[str, object]:
        running = self.session_running()
        pane = self.capture() if running else ""
        matches = self.POSITION_PATTERN.findall(pane)
        position = int(matches[-1]) if matches else None
        maximum = None
        balance = None
        try:
            config = json.loads(self.config_path.read_text(encoding="utf-8"))
            if config.get("max_steps") is not None:
                maximum = int(config["max_steps"])
            if config.get("balance_steps") is not None:
                balance = int(config["balance_steps"])
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            pass

        ball_position = None
        vision_fps = None
        try:
            with urllib.request.urlopen(
                self.vision_status_url, timeout=0.35
            ) as response:
                vision = json.load(response)
            if vision.get("position_cm") is not None:
                ball_position = float(vision["position_cm"])
            if vision.get("fps") is not None:
                vision_fps = float(vision["fps"])
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            pass

        return {
            "axis_running": running,
            "position_steps": position,
            "max_steps": maximum,
            "balance_steps": balance,
            "ball_position_cm": ball_position,
            "vision_fps": vision_fps,
            "timestamp": round(time.time(), 3),
        }

    def apply(self, payload: dict[str, object]) -> dict[str, object]:
        action = payload.get("action")
        with self.lock:
            if action == "move":
                value = payload.get("steps")
                if isinstance(value, bool) or not isinstance(value, int):
                    raise ValueError("steps必须是整数")
                if value == 0 or abs(value) > 20:
                    raise ValueError("单次相对移动必须在-20～+20步且不能为0")
                command = f"+ {value}" if value > 0 else f"- {abs(value)}"
                message = f"已发送相对移动 {value:+d} 步"
                self.send_literal(command)
            elif action == "goto":
                value = payload.get("position")
                if isinstance(value, bool) or not isinstance(value, int):
                    raise ValueError("position必须是整数")
                if not 0 <= value <= 2000:
                    raise ValueError("绝对位置必须在0～2000步")
                self.send_literal(f"goto {value}")
                message = f"已发送绝对位置 {value}"
            elif action == "set_maximum":
                self.send_literal("max")
                message = "已请求记录当前为最高点"
            elif action == "set_balance":
                self.send_literal("balance")
                message = "已请求记录当前为水平点"
            elif action == "center":
                self.send_literal("center")
                message = "已请求回到水平点"
            elif action == "emergency_stop":
                if not self.session_running():
                    raise RuntimeError("步进标定会话未运行")
                result = self._run(
                    ["tmux", "send-keys", "-t", self.session, "C-c"]
                )
                if result.returncode != 0:
                    raise RuntimeError(result.stderr.strip() or "停止失败")
                message = "已发送紧急停止，必须重新从最低点ZERO"
            else:
                raise ValueError("不支持的控制命令")

        return {"ok": True, "message": message}


class StepperWebServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address: tuple[str, int],
        bridge: StepperBridge,
    ) -> None:
        self.bridge = bridge
        super().__init__(address, StepperHandler)


class StepperHandler(BaseHTTPRequestHandler):
    server: StepperWebServer

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
            self.send_json(self.server.bridge.status())
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
            if not 0 < length <= 2048:
                raise ValueError("请求大小无效")
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict):
                raise ValueError("JSON必须是对象")
            self.send_json(self.server.bridge.apply(payload))
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            self.send_json(
                {"error": str(error)}, HTTPStatus.BAD_REQUEST
            )
        except (RuntimeError, OSError, subprocess.SubprocessError) as error:
            self.send_json(
                {"error": str(error)}, HTTPStatus.CONFLICT
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--session", default="d36a-cal")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("/home/arick/d36a_test/axis_calibration.json"),
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
        args.config,
        args.vision_status_url,
    )
    server = StepperWebServer((args.host, args.port), bridge)
    print(
        f"D36A stepper web panel: http://{args.host}:{args.port}/ "
        f"(tmux session={args.session})",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
