from __future__ import annotations

import json
import os
import shlex
from pathlib import Path

import paramiko


HOST = "192.168.252.136"
USER = "arick"
ROOT = Path(__file__).resolve().parents[1]
LOCAL_CONTROLLER = ROOT / "outputs" / "d36a_cascade_direct_web.py"
LOCAL_SETTINGS_COPY = (
    ROOT / "work" / "pi_snapshot" / "cascade_controller_settings.json"
)
REMOTE_CONTROLLER = "/home/arick/d36a_test/d36a_cascade_direct_web.py"
REMOTE_SETTINGS = "/home/arick/d36a_test/cascade_stepper_settings.json"

NEW_SETTINGS = {
    "control_center_steps": 235,
    "control_min_steps": 0,
    "control_max_steps": 681,
    "escape_enabled": 1,
    "escape_velocity_threshold_cm_s": 0.20,
    "escape_error_threshold_cm": 0.80,
    "escape_stationary_s": 0.70,
    "escape_direction": -1,
    "escape_push_steps": 160,
    "escape_release_velocity_cm_s": 0.80,
    "escape_push_max_s": 1.20,
    "escape_recovery_steps": 100,
    "escape_recovery_velocity_cm_s": 0.20,
    "escape_recovery_min_s": 0.25,
    "escape_recovery_max_s": 1.20,
    "escape_cooldown_s": 1.50,
}


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
        raise RuntimeError(
            f"Remote command failed ({status}): {command}\n{error or output}"
        )
    return output.strip()


def remote_json(client: paramiko.SSHClient, url: str) -> dict[str, object]:
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
    before = remote_json(client, "http://127.0.0.1:8081/status.json")

    sftp = client.open_sftp()
    with sftp.open(REMOTE_SETTINGS, "r") as handle:
        settings = json.loads(handle.read().decode("utf-8", "replace"))
    for obsolete in (
        "shake_enabled",
        "shake_velocity_threshold_cm_s",
        "shake_error_threshold_cm",
        "shake_stationary_s",
        "shake_amplitude_steps",
        "shake_frequency_hz",
        "shake_duration_s",
        "shake_cooldown_s",
    ):
        settings.pop(obsolete, None)
    for key, value in NEW_SETTINGS.items():
        settings.setdefault(key, value)

    temporary_controller = REMOTE_CONTROLLER + ".codex-new"
    temporary_settings = REMOTE_SETTINGS + ".codex-new"
    sftp.put(str(LOCAL_CONTROLLER), temporary_controller)
    with sftp.open(temporary_settings, "w") as handle:
        handle.write(
            (
                json.dumps(settings, ensure_ascii=False, indent=2) + "\n"
            ).encode("utf-8")
        )
    sftp.close()

    validate_code = (
        "import json;"
        f"json.load(open({temporary_settings!r},encoding='utf-8'));"
        "print('settings ok')"
    )
    print(
        run(
            client,
            "python3 -m py_compile "
            f"{shlex.quote(temporary_controller)}"
            " && python3 -c "
            + shlex.quote(validate_code),
        )
    )

    install = f"""
set -eu
stamp=$(date +%Y%m%d-%H%M%S)
cp {shlex.quote(REMOTE_CONTROLLER)} {shlex.quote(REMOTE_CONTROLLER)}.bak-$stamp
cp {shlex.quote(REMOTE_SETTINGS)} {shlex.quote(REMOTE_SETTINGS)}.bak-$stamp
mv {shlex.quote(temporary_controller)} {shlex.quote(REMOTE_CONTROLLER)}
mv {shlex.quote(temporary_settings)} {shlex.quote(REMOTE_SETTINGS)}
chmod 755 {shlex.quote(REMOTE_CONTROLLER)}
echo "$stamp"
"""
    stamp = run(client, install)

    safe_to_restart = (
        before.get("current_steps") == 0
        and not before.get("axis_referenced")
        and not before.get("driver_enabled")
        and not before.get("control_enabled")
    )
    restarted = False
    if safe_to_restart:
        run(
            client,
            "sudo -S -p '' systemctl restart d36a-stepper-control",
            password=password,
        )
        restarted = True

    client.close()
    LOCAL_SETTINGS_COPY.write_text(
        json.dumps(settings, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "backup_stamp": stamp,
                "deployed": True,
                "service_restarted": restarted,
                "activation_pending_safe_restart": not restarted,
                "axis_before": {
                    key: before.get(key)
                    for key in (
                        "current_steps",
                        "axis_referenced",
                        "driver_enabled",
                        "control_enabled",
                        "safety_reason",
                    )
                },
                "escape_profile": NEW_SETTINGS,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
