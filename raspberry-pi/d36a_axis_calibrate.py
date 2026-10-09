#!/usr/bin/env python3
"""Interactively calibrate a manually referenced D36A stepper axis.

There is no limit switch and no encoder.  Before every launch, the operator
must place the mechanism at its safe minimum position.  After the operator
confirms ZERO, the program treats that position as step 0 and refuses to move
below it.  Once a safe maximum is recorded, it also refuses to move above the
saved maximum.

This is an open-loop software limit.  It is only valid while the motor has not
lost steps and the mechanism has not been moved by hand.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path


EN1 = 22
DIR1 = 23
STEP1 = 18
DEFAULT_CONFIG = Path("/home/arick/d36a_test/axis_calibration.json")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="D36A manual-zero axis calibration"
    )
    parser.add_argument("--hz", type=float, default=100.0)
    parser.add_argument(
        "--up-dir-level",
        type=int,
        choices=(0, 1),
        default=1,
        help="DIR level used by the '+' command",
    )
    parser.add_argument(
        "--probe-ceiling",
        type=int,
        default=2000,
        help="absolute safety ceiling before max_steps is calibrated",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print simulated moves without claiming GPIO",
    )
    return parser.parse_args()


def vision_service_active() -> bool:
    result = subprocess.run(
        ["systemctl", "is-active", "--quiet", "hball-control.service"],
        check=False,
    )
    return result.returncode == 0


def load_config(path: Path) -> dict[str, int | float | None]:
    default: dict[str, int | float | None] = {
        "max_steps": None,
        "balance_steps": None,
    }
    if not path.exists():
        return default
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        for key in default:
            if key in payload and payload[key] is not None:
                default[key] = int(payload[key])
        return default
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
        print(f"Ignoring invalid calibration file: {error}")
        return default


def save_config(
    path: Path,
    *,
    maximum: int | None,
    balance: int | None,
    frequency: float,
    up_dir_level: int,
) -> None:
    payload = {
        "minimum_steps": 0,
        "max_steps": maximum,
        "balance_steps": balance,
        "frequency_hz": frequency,
        "up_dir_level": up_dir_level,
        "updated_at": round(time.time(), 3),
        "reference_method": "manual minimum at every startup",
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


class D36AAxis:
    def __init__(
        self,
        *,
        frequency: float,
        up_dir_level: int,
        dry_run: bool,
    ) -> None:
        self.frequency = frequency
        self.up_dir_level = up_dir_level
        self.dry_run = dry_run
        self.lgpio = None
        self.handle = None

        if dry_run:
            return

        import lgpio

        self.lgpio = lgpio
        self.handle = lgpio.gpiochip_open(0)
        try:
            lgpio.gpio_claim_output(self.handle, EN1, 0)
            lgpio.gpio_claim_output(self.handle, DIR1, 0)
            lgpio.gpio_claim_output(self.handle, STEP1, 0)
        except Exception:
            lgpio.gpiochip_close(self.handle)
            self.handle = None
            raise

    def move(self, signed_steps: int) -> None:
        if signed_steps == 0:
            return
        direction = (
            self.up_dir_level
            if signed_steps > 0
            else 1 - self.up_dir_level
        )
        cycles = abs(signed_steps)
        print(
            f"MOVE {'UP' if signed_steps > 0 else 'DOWN'} "
            f"{cycles} step(s), DIR={direction}, {self.frequency:g} Hz"
        )
        if self.dry_run:
            return

        assert self.lgpio is not None and self.handle is not None
        lgpio = self.lgpio
        lgpio.gpio_write(self.handle, STEP1, 0)
        lgpio.gpio_write(self.handle, DIR1, direction)
        time.sleep(0.00001)
        lgpio.gpio_write(self.handle, EN1, 1)
        time.sleep(0.002)
        lgpio.tx_pwm(
            self.handle,
            STEP1,
            self.frequency,
            50.0,
            pulse_offset=0,
            pulse_cycles=cycles,
        )
        while lgpio.tx_busy(self.handle, STEP1, lgpio.TX_PWM):
            time.sleep(0.002)
        lgpio.gpio_write(self.handle, STEP1, 0)

    def release(self) -> None:
        if self.dry_run or self.lgpio is None or self.handle is None:
            return
        lgpio = self.lgpio
        try:
            if lgpio.tx_busy(self.handle, STEP1, lgpio.TX_PWM):
                lgpio.gpio_free(self.handle, STEP1)
                lgpio.gpio_claim_output(self.handle, STEP1, 0)
            else:
                lgpio.gpio_write(self.handle, STEP1, 0)
            lgpio.gpio_write(self.handle, EN1, 0)
        finally:
            lgpio.gpiochip_close(self.handle)
            self.handle = None


def print_help() -> None:
    print(
        """
Commands:
  + [N]       move upward N steps (default 1)
  - [N]       move downward N steps (default 1)
  goto N      move to absolute software position N
  max         save current position as the safe maximum
  clearmax    forget the saved maximum and re-calibrate it
  balance     save current position as the horizontal/balance position
  center      move to the saved balance position
  status      show current position and saved limits
  release     set EN1 low; the axis may no longer hold its position
  help        show this help
  quit        release the motor and exit
"""
    )


def main() -> int:
    args = parse_args()
    if not 1.0 <= args.hz <= 500.0:
        raise SystemExit("--hz must be between 1 and 500")
    if not 1 <= args.probe_ceiling <= 100_000:
        raise SystemExit("--probe-ceiling must be between 1 and 100000")
    if not args.dry_run and vision_service_active():
        raise SystemExit(
            "hball-control.service is using GPIO18. Stop it first with:\n"
            "  sudo systemctl stop hball-control.service"
        )

    saved = load_config(args.config)
    maximum = (
        int(saved["max_steps"])
        if saved["max_steps"] is not None
        else None
    )
    balance = (
        int(saved["balance_steps"])
        if saved["balance_steps"] is not None
        else None
    )

    print("D36A manual-reference calibration")
    print(f"Pins: EN1=GPIO{EN1}, DIR1=GPIO{DIR1}, STEP1=GPIO{STEP1}")
    print(f"Frequency: {args.hz:g} Hz; '+' DIR level={args.up_dir_level}")
    print(f"Saved max_steps={maximum}, balance_steps={balance}")
    print("Place the pipe mechanism at its SAFE MINIMUM position.")
    confirmation = input("Type ZERO to make this position step 0: ").strip()
    if confirmation != "ZERO":
        print("Cancelled; GPIO was not opened.")
        return 1

    position = 0
    axis = D36AAxis(
        frequency=args.hz,
        up_dir_level=args.up_dir_level,
        dry_run=args.dry_run,
    )
    print_help()
    try:
        while True:
            raw = input(f"step[{position}]> ").strip().lower()
            if not raw:
                continue
            parts = raw.split()
            command = parts[0]

            if command in {"+", "-"}:
                amount = int(parts[1]) if len(parts) > 1 else 1
                if amount <= 0:
                    print("N must be greater than 0.")
                    continue
                target = position + amount if command == "+" else position - amount
            elif command == "goto" and len(parts) == 2:
                target = int(parts[1])
            elif command == "max":
                maximum = position
                if balance is not None and balance > maximum:
                    balance = None
                save_config(
                    args.config,
                    maximum=maximum,
                    balance=balance,
                    frequency=args.hz,
                    up_dir_level=args.up_dir_level,
                )
                print(f"Saved safe maximum: {maximum} steps")
                continue
            elif command == "clearmax":
                maximum = None
                print(
                    "Maximum cleared for this session. "
                    f"Probe ceiling remains {args.probe_ceiling} steps."
                )
                continue
            elif command == "balance":
                balance = position
                save_config(
                    args.config,
                    maximum=maximum,
                    balance=balance,
                    frequency=args.hz,
                    up_dir_level=args.up_dir_level,
                )
                print(f"Saved balance position: {balance} steps")
                continue
            elif command == "center":
                if balance is None:
                    print("No balance position has been saved.")
                    continue
                target = balance
            elif command == "status":
                print(
                    f"position={position}, minimum=0, "
                    f"maximum={maximum}, balance={balance}, "
                    f"probe_ceiling={args.probe_ceiling}"
                )
                continue
            elif command == "release":
                axis.release()
                print("Motor released. Exit and restart to move again.")
                return 0
            elif command == "help":
                print_help()
                continue
            elif command in {"quit", "exit", "q"}:
                return 0
            else:
                print("Unknown command. Type help.")
                continue

            upper_bound = (
                maximum if maximum is not None else args.probe_ceiling
            )
            if not 0 <= target <= upper_bound:
                print(
                    f"REFUSED: target {target} is outside "
                    f"0..{upper_bound}."
                )
                continue
            axis.move(target - position)
            position = target
    except (EOFError, KeyboardInterrupt):
        print("\nStopped.")
    finally:
        axis.release()
        print("STEP stopped and EN1 set low.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
