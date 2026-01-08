#!/usr/bin/env bash
set -euo pipefail

arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
