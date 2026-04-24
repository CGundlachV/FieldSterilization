# FieldSterilization

FieldSterilization is a Zephyr RTOS firmware project for a compact temperature
control and readout device. It reads an MLX90614 infrared temperature sensor at
10 Hz, filters the object temperature for a smooth display, and uses a single
button to start or stop a guarded heater-control loop.

The display build targets an ESP32-C6 Super Mini with a 240 x 240 round
GC9A01A-compatible LCD. The screen shows a polished round instrument cluster:
the outer cyan ring is measured temperature, the slim white tick is the
setpoint, the inner amber ring is heater output duty, and the center readout
shows current temperature, setpoint, run state, and heater state on a clean
black background.

## What It Does

- Reads MLX90614 ambient and object temperature over I2C.
- Samples temperature every 100 ms for responsive logs and display motion.
- Toggles between stopped and running with one physical button.
- Runs a simple PI feedback loop against the configured temperature setpoint.
- Drives a heater-control output with software windowed PWM at 1 Hz by default.
- Forces the heater output off on sensor loss, over-temperature, missing output
  pin, or user stop.
- Retries MLX90614 initialization automatically if the sensor is not ready at
  boot.
- Keeps non-display builds useful by logging state, temperature, setpoint, duty,
  heater state, and fault state over serial.

## Control Model

The heater output is time-proportional GPIO control, not high-frequency PWM. The
default control window is 1000 ms, so output switching never exceeds 1 Hz. For a
50 percent command, the output is active for the first half of the one-second
window and inactive for the second half.

Default control settings live in `prj.conf` and are backed by `Kconfig`:

```text
CONFIG_FIELDSTERILIZATION_SETPOINT_CENTIC=12100
CONFIG_FIELDSTERILIZATION_MAX_SAFE_CENTIC=13500
CONFIG_FIELDSTERILIZATION_CONTROL_WINDOW_MS=1000
```

Temperatures are centi-degrees Celsius. The defaults are a 121.00 C setpoint and
a 135.00 C over-temperature cutoff. Tune these values for the chamber, heater,
sensor geometry, and sterilization process you are actually validating.

## Hardware Targets

### ESP32-C6 Super Mini

This is the default display target selected by `CMakeLists.txt`.

- Zephyr board target: `xiao_esp32c6/esp32c6/hpcore`
- Physical board: ESP32-C6 Super Mini
- Display: Adafruit GC9A01A 240 x 240 SPI LCD, product 6178
- Sensor: MLX90614 on I2C0
- Run/stop input: onboard BOOT-style button on GPIO9, active low
- Heater output: GPIO14, active high
- Display backlight: left unconnected by default; Adafruit pulls `BL/Lite`
  active

Configured pins:

```text
Display SCK           GPIO19
Display MOSI          GPIO18
Display TFTCS         GPIO3
Display DC            GPIO20
Display RST           GPIO2, active low
Display BL/Lite       unconnected, or 3.3 V for full brightness
MLX90614 SDA          GPIO22
MLX90614 SCL          GPIO23
Run/stop button       GPIO9
Heater output         GPIO14
```

The Zephyr target is kept on `xiao_esp32c6/esp32c6/hpcore` because it matches
the 4 MB ESP32-C6 module and gives us the working native USB serial/JTAG monitor
used on this board. The project overlay overrides the peripheral pins with raw
GPIO numbers so the wiring matches the numbers printed on the Super Mini PCB.
If you change the heater wiring, update the `heater0` alias in
`boards/esp32c6.overlay`.

The GC9A01A display is driven through Zephyr's `zephyr,mipi-dbi-spi` bus on the
ESP32-C6 `spi2` hardware peripheral, and the MLX90614 is driven by the ESP32-C6
hardware I2C controller. The firmware is not bit-banging either bus. The display
configuration also enables `CONFIG_LV_COLOR_16_SWAP` so LVGL's RGB565 pixels
arrive in the byte order expected by this panel.

### nRF54L15 DK

The nRF54L15 DK target builds the same sensor and control logic without the
round display UI.

- Board: `nrf54l15dk/nrf54l15/cpuapp`
- I2C bus: `i2c21`
- I2C SDA: P1.02
- I2C SCL: P1.03
- Run/stop input: board `SW0`
- Heater output: P1.04, active high

## Project Layout

```text
.
|-- CMakeLists.txt
|-- Kconfig
|-- README.md
|-- REQUIREMENTS.md
|-- prj.conf
|-- .gitmodules
|-- boards/
|   |-- esp32c6.conf
|   |-- esp32c6.overlay
|   |-- nrf54l15dk_nrf54l15_cpuapp.conf
|   `-- nrf54l15dk_nrf54l15_cpuapp.overlay
|-- lib/
|   `-- mlx90614/
|-- src/
|   |-- main.c
|   |-- mlx90614_read.c
|   |-- mlx90614_read.h
|   `-- mlx90614_zephyr_iface.c
`-- VERSION
```

`src/main.c` is the active firmware application.

## Setup

See `REQUIREMENTS.md` for the full environment setup. The short version is:

```sh
git submodule update --init --recursive
source ~/zephyrproject/.venv/bin/activate
cd ~/zephyrproject/zephyr
```

The MLX90614 driver must be populated under `lib/mlx90614/`. If it is missing,
CMake stops with a message telling you to initialize submodules.

## Build

Build the ESP32-C6 Super Mini display target:

```sh
west build -p always -b xiao_esp32c6/esp32c6/hpcore \
  ~/Documents/GitHub/FieldSterilization \
  -d /tmp/FieldSterilization-esp32c6-build
```

Build the nRF54L15 DK target:

```sh
west build -p always -b nrf54l15dk/nrf54l15/cpuapp \
  ~/Documents/GitHub/FieldSterilization \
  -d /tmp/FieldSterilization-nrf54l15-build
```

Flash the build you just generated:

```sh
west flash -d /tmp/FieldSterilization-esp32c6-build
```

or:

```sh
west flash -d /tmp/FieldSterilization-nrf54l15-build
```

## Operation

Open the ESP32-C6 serial monitor with:

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

The `PYTHONPATH` part is required by the Espressif monitor in this Zephyr
workspace because its output helpers import Zephyr's west build helpers.
Press `Ctrl+]` to exit.

1. Power the board with the MLX90614 connected.
2. Wait for the display or serial log to show `READY`.
3. Press the run/stop button once to start control.
4. Press the same button again to stop. Stopping always turns the heater output
   off.

The display status line shows:

- `SENSOR`: no valid MLX90614 reading yet.
- `READY`: sensor and heater output are ready; press the button to run.
- `RUN`: feedback control is active.
- `FAULT sensor`: sensor timed out while running.
- `FAULT overtemp`: filtered temperature reached the maximum safe temperature.
  The heater stays off and the fault clears automatically after the sensor
  reports a cooled temperature below the reset margin.
- `FAULT output`: heater output is missing or failed to drive.

On boards that expose a Zephyr `led0` alias, the firmware also drives the
onboard status LED:

- slow blink: ready to run
- solid on: heater output currently active
- fast blink: sensor or output fault
- double blink: over-temperature fault
- off: waiting for sensor or otherwise idle

## Troubleshooting

On boot, the ESP32-C6 build prints an I2C scan. A connected default-address
MLX90614 should appear as `0x5a`. If the scan says `no devices found`, check
that the MLX90614 is powered, grounded, and wired to GPIO22/SDA and GPIO23/SCL.
Use 4.7 kOhm pullups from SDA to 3.3 V and SCL to 3.3 V if your breakout does
not already include pullups.

## Safety Notes

This firmware provides software interlocks, but it is not a certified
sterilizer controller. Use an external thermal cutoff, fuse, power switch, and
properly rated heater driver. Validate the MLX90614 mounting, emissivity,
thermal lag, and output polarity before connecting real heating hardware.
