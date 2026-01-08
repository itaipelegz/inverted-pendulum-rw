#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE_DIR="$ROOT_DIR/.arduino_cache"
BUILD_DIR="$ROOT_DIR/.arduino_build"

mkdir -p "$CACHE_DIR" "$BUILD_DIR"

python3 "$ROOT_DIR/python/generate_config.py" "$ROOT_DIR/config/pendulum_config.yaml" "$ROOT_DIR/include/generated_config.h"

arduino-cli compile --fqbn arduino:avr:uno \
  --build-path "$BUILD_DIR" \
  --build-cache-path "$CACHE_DIR" \
  --build-property compiler.cpp.extra_flags="-I$ROOT_DIR/include" \
  "$ROOT_DIR/src/arduino/pendulum_controller"
