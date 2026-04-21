# Requirements

This project is a Zephyr application. It does not use a project-local Python
`requirements.txt`; Zephyr manages its Python package set from the Zephyr
workspace.

These instructions were tested on macOS with Homebrew.

## Host Tools

Install the host tools:

```sh
brew install cmake ninja dtc gperf ccache dfu-util wget python@3.12 pipx
pipx install --python /opt/homebrew/bin/python3.12 west
pipx ensurepath
```

Tested local versions:

- `west` 1.5.0
- Zephyr 4.4.99
- Zephyr SDK 1.0.1
- CMake 4.3.1
- Ninja 1.13.2
- DTC 1.7.2
- Python 3.12.13 in the Zephyr virtual environment

## Zephyr Workspace

Create a Zephyr workspace outside this application repository:

```sh
west init ~/zephyrproject
cd ~/zephyrproject
west update
```

Create a Python virtual environment for Zephyr:

```sh
/opt/homebrew/bin/python3.12 -m venv ~/zephyrproject/.venv
source ~/zephyrproject/.venv/bin/activate
python -m pip install --upgrade pip wheel west
cd ~/zephyrproject/zephyr
west zephyr-export
west packages pip --install
```

Install only the SDK toolchains needed by this project:

```sh
cd ~/zephyrproject/zephyr
source ~/zephyrproject/.venv/bin/activate
west sdk install -t arm-zephyr-eabi
west sdk install -t riscv64-zephyr-elf
```

The ARM toolchain is used by the nRF54L15 DK build. The RISC-V toolchain is
used by the ESP32-C6 build.

## Project Dependencies

Clone this repository with submodules, or initialize submodules after cloning:

```sh
git submodule update --init --recursive
```

The required MLX90614 driver is pinned as:

```text
lib/mlx90614 -> https://github.com/libdriver/mlx90614.git
```

## Hardware Wiring

The ESP32-C6 overlay uses raw GPIO numbers matching the ESP32-C6 Super Mini
board labels:

```text
Display SCK       GPIO19
Display MOSI      GPIO18
Display TFTCS     GPIO3
Display DC        GPIO20
Display RST       GPIO2
Display BL/Lite   unconnected, or 3.3 V
MLX90614 SDA      GPIO22
MLX90614 SCL      GPIO23
Heater output     GPIO14
Run/stop button   GPIO9
```

The Zephyr board target remains `xiao_esp32c6/esp32c6/hpcore` because it matches
the 4 MB ESP32-C6 module and keeps the native USB serial/JTAG monitor working.
The project overlay overrides the display, I2C, heater, and button pins with
the Super Mini GPIO numbers above.

## Build Commands

Activate the Zephyr virtual environment and run `west build` from the Zephyr
workspace so Zephyr's west extension commands are available:

```sh
source ~/zephyrproject/.venv/bin/activate
cd ~/zephyrproject/zephyr
```

Build the ESP32-C6 Super Mini display target:

```sh
west build -p always -b xiao_esp32c6/esp32c6/hpcore \
  ~/Documents/GitHub/FieldSterilization \
  -d /tmp/FieldSterilization-esp32c6-build
```

Build the nRF54L15 DK control target:

```sh
west build -p always -b nrf54l15dk/nrf54l15/cpuapp \
  ~/Documents/GitHub/FieldSterilization \
  -d /tmp/FieldSterilization-nrf54l15-build
```

Flash the last build:

```sh
west flash
```

Or flash a specific build directory:

```sh
west flash -d /tmp/FieldSterilization-esp32c6-build
```

Monitor the ESP32-C6 serial logs:

```sh
source ~/zephyrproject/.venv/bin/activate
PYTHONPATH=~/zephyrproject/zephyr/scripts/west_commands \
  python ~/zephyrproject/modules/hal/espressif/tools/idf_monitor/idf_monitor.py \
  -p /dev/cu.usbmodem2101 \
  -b 115200 \
  /tmp/FieldSterilization-esp32c6-build/zephyr/zephyr.elf \
  --eol CRLF \
  -d
```

Press `Ctrl+]` to exit the monitor.

## Application Configuration

The firmware exposes user-tunable options through `Kconfig` and `prj.conf`.
Temperatures use centi-degrees Celsius:

```text
CONFIG_FIELDSTERILIZATION_SETPOINT_CENTIC=12100
CONFIG_FIELDSTERILIZATION_MAX_SAFE_CENTIC=13500
CONFIG_FIELDSTERILIZATION_CONTROL_WINDOW_MS=1000
```

Keep `CONFIG_FIELDSTERILIZATION_CONTROL_WINDOW_MS` at `1000` or higher so the
heater output stays at or below 1 Hz.

## Notes

- `west build` may report `MIPI_DBI` as experimental for the ESP32-C6 display
  target. That comes from Zephyr's GC9X01X/MIPI-DBI display stack.
- On macOS, `west sdk install` reports that macOS host tools are not available
  from the SDK package. The Homebrew host tools above provide the pieces needed
  for these builds.
- If a future Zephyr release changes display bindings again, compare
  `boards/esp32c6.overlay` with Zephyr's built-in
  `seeed_xiao_round_display` shield overlay.
