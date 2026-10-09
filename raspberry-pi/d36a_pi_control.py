#!/usr/bin/env python3
"""
Raspberry Pi -> WHEELTEC D36A dual stepper-motor controller.

Default BCM GPIO wiring:

    D36A EN1  -> GPIO22 (physical pin 15)
    D36A DIR1 -> GPIO23 (physical pin 16)
    D36A ST1  -> GPIO18 (physical pin 12)
    D36A EN2  -> GPIO24 (physical pin 18)
    D36A DIR2 -> GPIO25 (physical pin 22)
    D36A ST2  -> GPIO19 (physical pin 35)
    D36A GND  -> Raspberry Pi GND (for example physical pin 6)

Power the Raspberry Pi and D36A separately. Do not connect D36A 5V to a
Raspberry Pi GPIO. The grounds must be connected together.

Install on Raspberry Pi OS:

    sudo apt update
    sudo apt install -y python3-lgpio

Examples:

    python3 d36a_pi_control.py
    python3 d36a_pi_control.py move 1 90 --rpm 60 --microstep 16
    python3 d36a_pi_control.py move 1 -360 --rpm 30 --microstep 8
    python3 d36a_pi_control.py steps 2 200 --hz 800
    python3 d36a_pi_control.py spin 1 2 --rpm -60 --microstep 16

D36A is an open-loop stepper driver. The requested angle is only reliable if
the motor does not lose steps. Use a homing switch or encoder when absolute
position must be recovered.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass


VALID_MICROSTEPS = (1, 2, 4, 8, 16, 32)
MIN_FREQUENCY_HZ = 1.0
MAX_FREQUENCY_HZ = 10_000.0


@dataclass(frozen=True)
class MotorPins:
    enable: int
    direction: int
    step: int


MOTOR_PINS = {
    1: MotorPins(enable=22, direction=23, step=18),
    2: MotorPins(enable=24, direction=25, step=19),
}


def steps_per_revolution(step_angle: float, microstep: int) -> int:
    if step_angle <= 0:
        raise ValueError("电机步距角必须大于 0")
    if microstep not in VALID_MICROSTEPS:
        raise ValueError(f"细分只能是 {VALID_MICROSTEPS}")
    base_steps = 360.0 / step_angle
    rounded = round(base_steps)
    if not math.isclose(base_steps, rounded, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("360° 必须能被电机步距角整除")
    return rounded * microstep


def angle_to_steps(angle: float, step_angle: float, microstep: int) -> int:
    value = angle / 360.0 * steps_per_revolution(step_angle, microstep)
    steps = round(value)
    if angle != 0 and steps == 0:
        raise ValueError("角度太小，换算后不足一个脉冲")
    return steps


def rpm_to_frequency(rpm: float, step_angle: float, microstep: int) -> float:
    if rpm == 0:
        raise ValueError("转速不能为 0 RPM")
    return abs(rpm) * steps_per_revolution(step_angle, microstep) / 60.0


def validate_frequency(frequency: float) -> float:
    if not MIN_FREQUENCY_HZ <= frequency <= MAX_FREQUENCY_HZ:
        raise ValueError(
            f"脉冲频率必须在 {MIN_FREQUENCY_HZ:g}~"
            f"{MAX_FREQUENCY_HZ:g} Hz；当前为 {frequency:.2f} Hz。"
            "请降低转速或细分。"
        )
    return frequency


class D36A:
    def __init__(self, chip: int = 0, dry_run: bool = False) -> None:
        self.dry_run = dry_run
        self.lgpio = None
        self.handle = None

        if dry_run:
            return

        try:
            import lgpio  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "没有找到 lgpio。请运行："
                "sudo apt update && sudo apt install -y python3-lgpio"
            ) from exc

        self.lgpio = lgpio
        self.handle = lgpio.gpiochip_open(chip)

        claimed: set[int] = set()
        for pins in MOTOR_PINS.values():
            for gpio in (pins.enable, pins.direction, pins.step):
                if gpio not in claimed:
                    lgpio.gpio_claim_output(self.handle, gpio, 0)
                    claimed.add(gpio)

    def close(self) -> None:
        if self.dry_run or self.handle is None or self.lgpio is None:
            return
        for pins in MOTOR_PINS.values():
            try:
                if self.lgpio.tx_busy(
                    self.handle, pins.step, self.lgpio.TX_PWM
                ):
                    self.lgpio.gpio_free(self.handle, pins.step)
                    self.lgpio.gpio_claim_output(
                        self.handle, pins.step, 0
                    )
                else:
                    self.lgpio.gpio_write(self.handle, pins.step, 0)
                self.lgpio.gpio_write(self.handle, pins.enable, 0)
            except Exception:
                pass
        self.lgpio.gpiochip_close(self.handle)
        self.handle = None

    def __enter__(self) -> "D36A":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def move_steps(
        self,
        motor: int,
        signed_steps: int,
        frequency: float,
        hold: bool = False,
    ) -> None:
        if motor not in MOTOR_PINS:
            raise ValueError("电机通道只能是 1 或 2")
        if signed_steps == 0:
            print("步数为 0，不动作")
            return

        frequency = validate_frequency(frequency)
        pins = MOTOR_PINS[motor]
        direction_level = 1 if signed_steps > 0 else 0
        cycles = abs(signed_steps)
        duration = cycles / frequency

        print(
            f"电机{motor}: DIR={direction_level}, {cycles} 脉冲, "
            f"{frequency:.2f} Hz, 预计 {duration:.3f} 秒"
        )

        if self.dry_run:
            print("dry-run：未操作 GPIO")
            return

        assert self.lgpio is not None and self.handle is not None

        self.lgpio.gpio_write(self.handle, pins.step, 0)
        self.lgpio.gpio_write(self.handle, pins.direction, direction_level)
        time.sleep(0.00001)  # DIR setup time: > 200 ns

        self.lgpio.gpio_write(self.handle, pins.enable, 1)
        time.sleep(0.002)  # nSLEEP wake-up requirement: > 1 ms

        # 50% duty cycle leaves at least 50 us high/low at the 10 kHz limit.
        # The ATD5984 only requires at least 1 us high and 1 us low.
        self.lgpio.tx_pwm(
            self.handle,
            pins.step,
            frequency,
            50.0,
            pulse_offset=0,
            pulse_cycles=cycles,
        )
        while self.lgpio.tx_busy(self.handle, pins.step, self.lgpio.TX_PWM):
            time.sleep(0.005)

        self.lgpio.gpio_write(self.handle, pins.step, 0)
        if not hold:
            self.lgpio.gpio_write(self.handle, pins.enable, 0)

    def release(self, motor: int) -> None:
        if self.dry_run:
            return
        assert self.lgpio is not None and self.handle is not None
        pins = MOTOR_PINS[motor]
        if self.lgpio.tx_busy(self.handle, pins.step, self.lgpio.TX_PWM):
            self.lgpio.gpio_free(self.handle, pins.step)
            self.lgpio.gpio_claim_output(self.handle, pins.step, 0)
        else:
            self.lgpio.gpio_write(self.handle, pins.step, 0)
        self.lgpio.gpio_write(self.handle, pins.enable, 0)


def add_motion_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--microstep",
        type=int,
        choices=VALID_MICROSTEPS,
        default=16,
        help="D36A 拨码细分，默认 16",
    )
    parser.add_argument(
        "--step-angle",
        type=float,
        default=1.8,
        help="电机整步步距角，默认 1.8°",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="树莓派控制 D36A 双路步进电机驱动",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--chip", type=int, default=0, help="/dev/gpiochip 编号")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只计算并打印，不操作 GPIO",
    )

    subparsers = parser.add_subparsers(dest="command")

    move = subparsers.add_parser("move", help="按角度移动")
    move.add_argument("motor", type=int, choices=(1, 2), help="电机通道")
    move.add_argument("angle", type=float, help="角度；正负号控制方向")
    move.add_argument("--rpm", type=float, default=60.0, help="目标转速")
    move.add_argument(
        "--hold",
        action="store_true",
        help="动作后保持通电，按回车后释放",
    )
    add_motion_options(move)

    steps = subparsers.add_parser("steps", help="按脉冲数移动")
    steps.add_argument("motor", type=int, choices=(1, 2), help="电机通道")
    steps.add_argument("count", type=int, help="脉冲数；正负号控制方向")
    steps.add_argument("--hz", type=float, default=800.0, help="脉冲频率")
    steps.add_argument(
        "--hold",
        action="store_true",
        help="动作后保持通电，按回车后释放",
    )

    spin = subparsers.add_parser("spin", help="按给定转速运行指定秒数")
    spin.add_argument("motor", type=int, choices=(1, 2), help="电机通道")
    spin.add_argument("seconds", type=float, help="运行秒数")
    spin.add_argument(
        "--rpm",
        type=float,
        default=60.0,
        help="有符号转速；正负号控制方向",
    )
    add_motion_options(spin)

    return parser


def wait_and_release(controller: D36A, motor: int) -> None:
    try:
        input("正在保持扭矩，按 Enter 释放电机...")
    finally:
        controller.release(motor)


def run_interactive(controller: D36A) -> None:
    print("D36A 交互模式。输入 q 退出。")
    print("方向由角度正负号决定；方向相反时把角度改为负数。")

    while True:
        raw = input("\n电机通道 1/2 [q退出]: ").strip().lower()
        if raw in {"q", "quit", "exit"}:
            return
        try:
            motor = int(raw)
            if motor not in (1, 2):
                raise ValueError
            angle = float(input("角度（可为负数）: ").strip())
            rpm = float(input("转速 RPM [60]: ").strip() or "60")
            microstep = int(input("细分 1/2/4/8/16/32 [16]: ").strip() or "16")
            hold = input("动作后保持扭矩？y/N: ").strip().lower() == "y"

            signed_steps = angle_to_steps(angle, 1.8, microstep)
            frequency = rpm_to_frequency(rpm, 1.8, microstep)
            if rpm < 0:
                signed_steps = -signed_steps
            controller.move_steps(motor, signed_steps, frequency, hold)
            if hold:
                wait_and_release(controller, motor)
        except ValueError as exc:
            print(f"输入错误：{exc}")


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        with D36A(chip=args.chip, dry_run=args.dry_run) as controller:
            if args.command is None:
                run_interactive(controller)
                return 0

            if args.command == "move":
                signed_steps = angle_to_steps(
                    args.angle, args.step_angle, args.microstep
                )
                frequency = rpm_to_frequency(
                    args.rpm, args.step_angle, args.microstep
                )
                if args.rpm < 0:
                    signed_steps = -signed_steps
                controller.move_steps(
                    args.motor, signed_steps, frequency, args.hold
                )
                if args.hold and not args.dry_run:
                    wait_and_release(controller, args.motor)

            elif args.command == "steps":
                controller.move_steps(
                    args.motor, args.count, args.hz, args.hold
                )
                if args.hold and not args.dry_run:
                    wait_and_release(controller, args.motor)

            elif args.command == "spin":
                if args.seconds <= 0:
                    raise ValueError("运行秒数必须大于 0")
                frequency = rpm_to_frequency(
                    args.rpm, args.step_angle, args.microstep
                )
                signed_steps = round(frequency * args.seconds)
                if args.rpm < 0:
                    signed_steps = -signed_steps
                controller.move_steps(
                    args.motor, signed_steps, frequency, hold=False
                )

    except (RuntimeError, ValueError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\n已停止并释放电机")
        return 130

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
