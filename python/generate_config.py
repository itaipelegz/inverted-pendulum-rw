#!/usr/bin/env python3
import json
import sys
from pathlib import Path


def fmt_float(value: float) -> str:
    # Preserve typical Arduino float literal with suffix.
    text = f"{value:.6g}"
    if "." not in text and "e" not in text:
        text += ".0"
    return f"{text}f"


def write_header(data, out_path: Path) -> None:
    lines = []
    lines.append("#pragma once")
    lines.append("")
    lines.append("// Auto-generated from config/pendulum_config.json. Do not edit by hand.")
    lines.append("")

    pins = data["pins"]
    lines.append("// Pins")
    for name in [
        "RPWM_PIN",
        "LPWM_PIN",
        "R_EN_PIN",
        "L_EN_PIN",
        "AM_PIN",
        "BM_PIN",
        "AS_PIN",
        "BS_PIN",
    ]:
        lines.append(f"constexpr uint8_t {name} = {pins[name]};")
    lines.append("")

    enc = data["encoder"]
    lines.append("// Encoder")
    lines.append(f"constexpr float MOTOR_COUNTS_PER_REV_4X = {fmt_float(enc['MOTOR_COUNTS_PER_REV_4X'])};")
    lines.append(f"constexpr float MOTOR_DECODE_FACTOR = {fmt_float(enc['MOTOR_DECODE_FACTOR'])};")
    lines.append(f"constexpr bool SHAFT_USE_CHANGE = {str(enc['SHAFT_USE_CHANGE']).lower()};")
    lines.append(f"constexpr float SHAFT_PPR = {fmt_float(enc['SHAFT_PPR'])};")
    lines.append("")

    est = data["estimator"]
    lines.append("// Estimator")
    lines.append(f"constexpr unsigned long EST_PERIOD_MS = {est['EST_PERIOD_MS']}UL;")
    lines.append(f"constexpr float P_VEL_ALPHA = {fmt_float(est['P_VEL_ALPHA'])};")
    lines.append(f"constexpr float M_VEL_ALPHA = {fmt_float(est['M_VEL_ALPHA'])};")
    lines.append("")

    swing = data["swing"]
    lines.append("// Control (swing/stabilize)")
    lines.append(f"constexpr float P_VEL_DEADBAND = {fmt_float(swing['P_VEL_DEADBAND'])};")
    lines.append(f"constexpr float THETA_STAB_RAD = {fmt_float(swing['THETA_STAB_RAD'])};")
    lines.append(f"constexpr int PWM_SWING = {swing['PWM_SWING']};")
    lines.append(f"constexpr float KP = {fmt_float(swing['KP'])};")
    lines.append(f"constexpr float KD = {fmt_float(swing['KD'])};")
    lines.append(f"constexpr int PWM_MAX_STAB = {swing['PWM_MAX_STAB']};")
    lines.append("")

    motor = data["motor"]
    lines.append("// Motor")
    lines.append(f"constexpr int PWM_LIMIT = {motor['PWM_LIMIT']};")
    lines.append(f"constexpr int PWM_STEP_PER_CTRL = {motor['PWM_STEP_PER_CTRL']};")
    lines.append(f"constexpr unsigned long DIR_CHANGE_ZERO_HOLD_MS = {motor['DIR_CHANGE_ZERO_HOLD_MS']}UL;")
    lines.append("")

    ctrl = data["control"]
    lines.append("// Control loop")
    lines.append(f"constexpr unsigned long CTRL_PERIOD_MS = {ctrl['CTRL_PERIOD_MS']}UL;")
    lines.append("")

    serial = data["serial"]
    lines.append("// Serial")
    lines.append(f"constexpr unsigned long BAUDRATE = {serial['BAUDRATE']}UL;")
    lines.append("")

    safety = data["safety"]
    lines.append("// Safety")
    lines.append(
        f"constexpr float FULL_TURN_THRESHOLD_FRACTION = {fmt_float(safety['FULL_TURN_THRESHOLD_FRACTION'])};"
    )
    lines.append("")

    out_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    if len(sys.argv) != 3:
        print("Usage: generate_config.py <config.json> <output.h>")
        return 2

    config_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2])

    data = json.loads(config_path.read_text(encoding="utf-8"))
    write_header(data, out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
