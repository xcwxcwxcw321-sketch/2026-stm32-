#!/usr/bin/env python3
"""Small, conservative D36A motor-1 test for Raspberry Pi."""

import argparse
import time

import lgpio


# BCM GPIO numbering.
EN1 = 22
DIR1 = 23
STEP1 = 18


def parse_args():
    parser = argparse.ArgumentParser(description="D36A motor-1 smoke test")
    parser.add_argument(
        "--steps",
        type=int,
        default=200,
        help="number of STEP rising edges (default: 200)",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=200.0,
        help="STEP frequency in Hz (default: 200)",
    )
    parser.add_argument(
        "--reverse",
        action="store_true",
        help="reverse the DIR level",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="skip the RUN confirmation",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if not 1 <= args.steps <= 10_000:
        raise SystemExit("--steps must be between 1 and 10000")
    if not 1 <= args.hz <= 1_000:
        raise SystemExit("--hz must be between 1 and 1000 for this smoke test")

    direction = 0 if args.reverse else 1
    duration = args.steps / args.hz

    print("D36A motor-1 test")
    print(f"BCM pins: EN1={EN1}, DIR1={DIR1}, ST1={STEP1}")
    print(
        f"DIR={direction}, steps={args.steps}, "
        f"frequency={args.hz:g} Hz, duration={duration:.2f} s"
    )
    print("Confirm D36A uses its own 12V supply and shares GND with the Pi.")

    if not args.yes:
        answer = input("Type RUN to start: ").strip()
        if answer != "RUN":
            print("Cancelled; GPIO was not opened.")
            return 0

    handle = lgpio.gpiochip_open(0)
    try:
        lgpio.gpio_claim_output(handle, EN1, 0)
        lgpio.gpio_claim_output(handle, DIR1, direction)
        lgpio.gpio_claim_output(handle, STEP1, 0)

        lgpio.gpio_write(handle, DIR1, direction)
        time.sleep(0.00001)

        # D36A EN is connected to the ATD5984 nSLEEP input:
        # low = sleep, high = enabled. Allow more than its 1 ms wake time.
        lgpio.gpio_write(handle, EN1, 1)
        time.sleep(0.002)

        # The ATD5984 advances once per rising edge. 50% duty cycle at
        # 1 kHz or below is far above the 1 us minimum high/low pulse width.
        lgpio.tx_pwm(
            handle,
            STEP1,
            args.hz,
            50.0,
            pulse_offset=0,
            pulse_cycles=args.steps,
        )
        while lgpio.tx_busy(handle, STEP1, lgpio.TX_PWM):
            time.sleep(0.01)

        print("Test complete.")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        try:
            # A finite tx_pwm stops by itself. Calling tx_pwm(..., 0, 0)
            # after it has finished raises "bad PWM micros" on the lgpio
            # version packaged with this Pi. If interrupted while still busy,
            # freeing and reclaiming the line cancels the transmission.
            if lgpio.tx_busy(handle, STEP1, lgpio.TX_PWM):
                lgpio.gpio_free(handle, STEP1)
                lgpio.gpio_claim_output(handle, STEP1, 0)
            else:
                lgpio.gpio_write(handle, STEP1, 0)
            lgpio.gpio_write(handle, EN1, 0)
        finally:
            lgpio.gpiochip_close(handle)
        print("Motor output released.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
