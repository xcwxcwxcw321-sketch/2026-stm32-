from __future__ import annotations

import json
import os
import shlex
import time

import paramiko


HOST = "192.168.252.136"
USER = "arick"


def run(
    client: paramiko.SSHClient,
    command: str,
    *,
    password: str | None = None,
) -> str:
    stdin, stdout, stderr = client.exec_command(command, timeout=30)
    if password is not None:
        stdin.write(password + "\n")
        stdin.flush()
    output = stdout.read().decode("utf-8", "replace")
    error = stderr.read().decode("utf-8", "replace")
    status = stdout.channel.recv_exit_status()
    if status:
        raise RuntimeError(error or output)
    return output.strip()


def get_status(client: paramiko.SSHClient) -> dict[str, object]:
    code = (
        "import json,urllib.request;"
        "r=urllib.request.urlopen("
        "'http://127.0.0.1:8081/status.json',timeout=2);"
        "print(json.dumps(json.load(r),ensure_ascii=False))"
    )
    return json.loads(run(client, f"python3 -c {shlex.quote(code)}"))


def post_action(
    client: paramiko.SSHClient,
    action: str,
) -> dict[str, object]:
    code = (
        "import json,urllib.request;"
        f"d=json.dumps({{'action':{action!r}}}).encode();"
        "q=urllib.request.Request("
        "'http://127.0.0.1:8081/control',data=d,"
        "headers={'Content-Type':'application/json'},method='POST');"
        "r=urllib.request.urlopen(q,timeout=2);"
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
    before = get_status(client)
    if before.get("driver_enabled") or before.get("control_enabled"):
        raise RuntimeError("轴不在急停释放/闭环停止状态，已取消重启")

    run(
        client,
        "sudo -S -p '' systemctl restart d36a-stepper-control",
        password=password,
    )
    after_restart = None
    for _attempt in range(20):
        try:
            after_restart = get_status(client)
            break
        except Exception:
            time.sleep(0.2)
    if after_restart is None:
        raise RuntimeError("控制服务重启后没有开始监听8081")
    required = {
        "control_center_steps",
        "control_max_steps",
        "escape_enabled",
        "escape_direction",
        "escape_push_steps",
        "escape_recovery_steps",
        "escape_release_velocity_cm_s",
        "escape_recovery_velocity_cm_s",
    }
    missing = sorted(required.difference(after_restart))
    if missing:
        raise RuntimeError(f"新版本参数没有加载：{missing}")
    if (
        after_restart.get("axis_referenced")
        or after_restart.get("driver_enabled")
        or after_restart.get("current_steps") != 0
    ):
        raise RuntimeError("重启后的轴初始状态异常，未执行零点确认")

    reference_result = post_action(client, "reference_zero")
    final = get_status(client)
    page_code = (
        "import urllib.request;"
        "p=urllib.request.urlopen('http://127.0.0.1:8081/',timeout=2)"
        ".read().decode();"
        "print(int('小球静止一次性大角度脱困' in p),"
        "int('control_max_steps' in p),int('test_escape' in p))"
    )
    page_check = run(client, f"python3 -c {shlex.quote(page_code)}")
    active = run(
        client,
        "systemctl is-active d36a-stepper-control",
    )
    client.close()

    result = {
        "service": active,
        "reference_action": reference_result,
        "page_check": page_check,
        "axis": {
            key: final.get(key)
            for key in (
                "current_steps",
                "axis_target_steps",
                "axis_referenced",
                "driver_enabled",
                "control_enabled",
            )
        },
        "limits": {
            key: final.get(key)
            for key in (
                "control_center_steps",
                "control_min_steps",
                "control_max_steps",
                "max_offset_steps",
            )
        },
        "escape": {
            key: final.get(key)
            for key in (
                "escape_enabled",
                "escape_velocity_threshold_cm_s",
                "escape_error_threshold_cm",
                "escape_stationary_s",
                "escape_direction",
                "escape_push_steps",
                "escape_release_velocity_cm_s",
                "escape_push_max_s",
                "escape_recovery_steps",
                "escape_recovery_velocity_cm_s",
                "escape_recovery_min_s",
                "escape_recovery_max_s",
                "escape_cooldown_s",
                "escape_active",
                "escape_phase",
                "escape_count",
            )
        },
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
