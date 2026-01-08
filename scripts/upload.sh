#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

"$ROOT_DIR/scripts/build.sh"

arduino-cli upload --fqbn arduino:avr:uno -p /dev/ttyACM0 "$ROOT_DIR/src/arduino/pendulum_controller"
