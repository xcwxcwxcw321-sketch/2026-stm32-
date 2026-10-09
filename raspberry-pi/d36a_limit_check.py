#!/usr/bin/env python3
"""Read two D36A-axis limit switches without moving the motor.

Recommended fail-safe wiring for each microswitch:

    Raspberry Pi GPIO ---- COM
    Raspberry Pi GND  ---- NC

The GPIO uses a pull-up.  Therefore:

    raw 0 = circuit closed, normal travel
    raw 1 = switch pressed OR cable disconnected, stop/fault

This script only claims GPIO5 and GPIO6 as inputs.  It never claims EN, DIR,
or STEP and cannot make the stepper motor move.
"""

from __future__ import annotations

import argparse
import signal
import time

import lgpio


MIN_LIMIT_GPIO = 5
MAX_LIMIT_GPIO = 6


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="D36A axis fail-safe limit-switch checker"
    )
    parser.add_argument("--chip", type=int, default=0)
    parser.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="stop after this many seconds; 0 means run until Ctrl+C",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.05,
        help="poll interval in seconds",
    )
    return parser.parse_args()


def state_text(raw_level: int) -> str:
    if raw_level:
        return "ACTIVE/FAULT (pressed, open circuit, or disconnected)"
    return "SAFE (NC circuit closed)"


def main() -> int:
    args = parse_args()
    if args.seconds < 0:
        raise SystemExit("--seconds must be >= 0")
    if not 0.005 <= args.interval <= 1.0:
        raise SystemExit("--interval must be between 0.005 and 1.0")

    stopped = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    handle = lgpio.gpiochip_open(args.chip)
    try:
        lgpio.gpio_claim_input(
            handle, MIN_LIMIT_GPIO, lgpio.SET_PULL_UP
        )
        lgpio.gpio_claim_input(
            handle, MAX_LIMIT_GPIO, lgpio.SET_PULL_UP
        )
        print("D36A limit input check (motor outputs are untouched)")
        print(
            f"MIN: BCM{MIN_LIMIT_GPIO}, physical pin 29; "
            f"MAX: BCM{MAX_LIMIT_GPIO}, physical pin 31"
        )
        print("Wiring: GPIO -> COM, Pi GND -> NC")
        print("Expected at rest: MIN=0 and MAX=0")

        previous: tuple[int, int] | None = None
        started = time.monotonic()
        while not stopped:
            minimum = int(lgpio.gpio_read(handle, MIN_LIMIT_GPIO))
            maximum = int(lgpio.gpio_read(handle, MAX_LIMIT_GPIO))
            current = (minimum, maximum)
            if current != previous:
                print(
                    f"MIN raw={minimum}: {state_text(minimum)} | "
                    f"MAX raw={maximum}: {state_text(maximum)}",
                    flush=True,
                )
                previous = current
            if minimum and maximum:
                print(
                    "FAULT: both inputs are open/active. "
                    "Do not home or enable the motor.",
                    flush=True,
                )
                return 2
            if args.seconds and time.monotonic() - started >= args.seconds:
                break
            time.sleep(args.interval)
    finally:
        lgpio.gpiochip_close(handle)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
