from __future__ import annotations

import json
import os
import shlex
import sys
import time
import urllib.request
from pathlib import Path

import paramiko


HOST = "192.168.252.136"
USER = "arick"
ROOT = Path(__file__).resolve().parents[1]

LOCAL_DETECTOR = ROOT / "outputs" / "detect_ball_stabilized.py"
LOCAL_CONTROLLER = ROOT / "outputs" / "d36a_cascade_direct_web.py"
LOCAL_SETTINGS = ROOT / "outputs" / "stabilized_vision_settings.json"
LOCAL_SNAPSHOT = ROOT / "work" / "vision_rework" / "step_interpolated_live.jpg"

REMOTE_DETECTOR = "/home/arick/steel_ball/gray_crop/detect_ball_stabilized.py"
REMOTE_CONTROLLER = "/home/arick/d36a_test/d36a_cascade_direct_web.py"
REMOTE_SETTINGS = (
    "/home/arick/steel_ball/gray_crop/stabilized_vision_settings.json"
)


def run(
    client: paramiko.SSHClient,
    command: str,
    *,
    password: str | None = None,
    timeout: int = 30,
) -> str:
    stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
    if password is not None:
        stdin.write(password + "\n")
        stdin.flush()
    output = stdout.read().decode("utf-8", "replace")
    error = stderr.read().decode("utf-8", "replace")
    status = stdout.channel.recv_exit_status()
    if status:
        raise RuntimeError(
            f"Remote command failed ({status}): {command}\n{error or output}"
        )
    return output.strip()


def read_json_url(client: paramiko.SSHClient, url: str) -> dict[str, object]:
    code = (
        "import json,urllib.request;"
        f"r=urllib.request.urlopen({url!r},timeout=2);"
        "print(json.dumps(json.load(r),ensure_ascii=False))"
    )
    return json.loads(run(client, f"python3 -c {shlex.quote(code)}"))


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

    control = read_json_url(
        client,
        "http://127.0.0.1:8081/status.json",
    )
    safe = (
        control.get("current_steps") == 0
        and not control.get("axis_referenced")
        and not control.get("driver_enabled")
        and not control.get("control_enabled")
    )
    if not safe:
        print(
            "Axis is no longer in the safe unreferenced/released zero state; "
            "deployment was not started.",
            file=sys.stderr,
        )
        print(json.dumps(control, ensure_ascii=False, indent=2))
        client.close()
        return 3

    sftp = client.open_sftp()
    with sftp.open(REMOTE_SETTINGS, "r") as handle:
        remote_settings = json.loads(
            handle.read().decode("utf-8", "replace")
        )

    top = float(remote_settings.get("fallback_top_px", 16.0))
    bottom = float(remote_settings.get("fallback_bottom_px", 82.0))
    for obsolete in (
        "fallback_left_px",
        "fallback_right_px",
        "fallback_top_px",
        "fallback_bottom_px",
        "bright_threshold",
        "pipe_smoothing",
    ):
        remote_settings.pop(obsolete, None)
    remote_settings.update(
        {
            "pipe_top_px": top,
            "pipe_bottom_px": bottom,
            "pipe_left_x_step0_px": 40.0,
            "pipe_left_x_balance_px": 14.0,
            "pipe_left_x_max_px": 0.0,
            "pipe_min_steps": 0,
            "pipe_balance_steps": 235,
            "pipe_max_steps": 681,
        }
    )

    temporary_detector = REMOTE_DETECTOR + ".codex-new"
    temporary_controller = REMOTE_CONTROLLER + ".codex-new"
    temporary_settings = REMOTE_SETTINGS + ".codex-new"
    sftp.put(str(LOCAL_DETECTOR), temporary_detector)
    sftp.put(str(LOCAL_CONTROLLER), temporary_controller)
    with sftp.open(temporary_settings, "w") as handle:
        handle.write(
            (
                json.dumps(remote_settings, ensure_ascii=False, indent=2)
                + "\n"
            ).encode("utf-8")
        )
    sftp.close()

    validate = (
        "python3 -m py_compile "
        f"{shlex.quote(temporary_detector)} "
        f"{shlex.quote(temporary_controller)}"
        " && python3 -c "
        + shlex.quote(
            "import json;"
            f"json.load(open({temporary_settings!r},encoding='utf-8'));"
            "print('remote validation ok')"
        )
    )
    print(run(client, validate))

    install = f"""
set -eu
stamp=$(date +%Y%m%d-%H%M%S)
cp {shlex.quote(REMOTE_DETECTOR)} {shlex.quote(REMOTE_DETECTOR)}.bak-$stamp
cp {shlex.quote(REMOTE_CONTROLLER)} {shlex.quote(REMOTE_CONTROLLER)}.bak-$stamp
cp {shlex.quote(REMOTE_SETTINGS)} {shlex.quote(REMOTE_SETTINGS)}.bak-$stamp
mv {shlex.quote(temporary_detector)} {shlex.quote(REMOTE_DETECTOR)}
mv {shlex.quote(temporary_controller)} {shlex.quote(REMOTE_CONTROLLER)}
mv {shlex.quote(temporary_settings)} {shlex.quote(REMOTE_SETTINGS)}
chmod 755 {shlex.quote(REMOTE_DETECTOR)} {shlex.quote(REMOTE_CONTROLLER)}
echo "$stamp"
"""
    stamp = run(client, install)
    print(f"backup stamp: {stamp}")

    run(
        client,
        "sudo -S -p '' systemctl restart hball-gray d36a-stepper-control",
        password=password,
    )
    time.sleep(2)
    active = run(
        client,
        "systemctl is-active hball-gray d36a-stepper-control",
    )
    print(active)

    vision = read_json_url(
        client,
        "http://127.0.0.1:8080/status.json",
    )
    updated_control = read_json_url(
        client,
        "http://127.0.0.1:8081/status.json",
    )
    result = {
        "vision": {
            key: vision.get(key)
            for key in (
                "pipe_source",
                "pipe_box_px",
                "pipe_left_x_px",
                "pipe_right_x_px",
                "axis_steps",
                "axis_synchronized",
                "position_cm",
                "track_source",
                "tracking_confidence",
                "fps",
            )
        },
        "control": {
            key: updated_control.get(key)
            for key in (
                "current_steps",
                "axis_referenced",
                "driver_enabled",
                "control_enabled",
            )
        },
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    client.close()

    with urllib.request.urlopen(
        f"http://{HOST}:8080/snapshot.jpg",
        timeout=5,
    ) as response:
        LOCAL_SNAPSHOT.parent.mkdir(parents=True, exist_ok=True)
        LOCAL_SNAPSHOT.write_bytes(response.read())
    print(f"snapshot: {LOCAL_SNAPSHOT}")

    LOCAL_SETTINGS.write_text(
        json.dumps(remote_settings, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
