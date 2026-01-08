# inverted-pendulum-rw
Reaction-wheel inverted pendulum project using Arduino Uno.

## Repo layout
- `config/` Source-of-truth configuration (JSON)
- `include/` Generated headers (`generated_config.h`)
- `python/` Configuration generator and tooling
- `scripts/` Build/upload/monitor helpers
- `src/arduino/pendulum_controller/` Arduino sketch (main firmware)
- `tests/` Experimental sketches (no unit tests yet)
- `CMakeLists.txt` Placeholder for future host tooling

## Build / upload / monitor
Requirements: `arduino-cli` with `arduino:avr:uno` installed.

- Build: `scripts/build.sh`
- Upload: `scripts/upload.sh` (uses `/dev/ttyACM0`)
- Monitor: `scripts/monitor.sh` (baudrate `115200`)

VS Code tasks are provided in `.vscode/tasks.json` and call these scripts.

## Changing configuration
1) Edit `config/pendulum_config.json`
2) Rebuild with `scripts/build.sh`

`python/generate_config.py` generates `include/generated_config.h`, and the firmware
includes those values during compilation.
