# FRDM-MCXA153 blinky + serial shell

Bare-metal firmware for the NXP FRDM-MCXA153 board: RGB LED control, a
serial command shell, two independent PWM outputs (P1_4 on J1-2 and P1_3), and
settings saved to flash.

Used for BAM 1 v5 regression (CY26 Q4 Goal 2): each PWM output drives a
4-20 mA module wired to a BAM pressure input, so test scripts can send a
pressure reading with `ma <mA>` (channel 1) or `ma2 <mA>` (channel 2) and
check what the BAM reports. Both channels run at the same time with their own
setpoints. Owner: Ed
(requested the project); built by Dom.

## Make commands

Run these from this directory (`BAM1_v5_Regression/MCXA153/blinky`).

| Command | What it does |
|---|---|
| `make` | Build the firmware: `build/blinky.elf`, `build/blinky.bin`, `build/blinky.map`, then print its size |
| `make flash` | Build if needed, then program the board over the on-board MCU-Link (pyocd). The chip resets and runs the new firmware |
| `make clean` | Delete the `build/` directory |
| `make clean all` | Full rebuild from scratch |
| `make && make flash` | Build, then flash (the usual edit-and-test loop) |
| `make -B` | Force every file to rebuild, even if unchanged |

`make flash` runs:

```
~/.venvs/pyocd/bin/pyocd flash -t mcxa153vlh build/blinky.elf
```

To use a different pyocd install:

```
make flash PYOCD=/path/to/pyocd
```

## Other useful pyocd commands

| Command | What it does |
|---|---|
| `~/.venvs/pyocd/bin/pyocd list` | Show connected debug probes (should list `MCU-LINK FRDM-MCXA153`) |
| `~/.venvs/pyocd/bin/pyocd reset -t mcxa153vlh` | Reset the chip without reflashing |
| `~/.venvs/pyocd/bin/pyocd erase -t mcxa153vlh --chip` | Erase the whole flash, including saved settings |
| `~/.venvs/pyocd/bin/pyocd commander -t mcxa153vlh` | Interactive debug prompt (halt, step, read/write memory) |

## Serial shell

Connect at 115200 baud, 8N1:

```
picocom -b 115200 /dev/ttyACM0      # exit: Ctrl-A then Ctrl-X
```

| Command | Effect |
|---|---|
| `help` | List commands |
| `led <color>` | Solid colour: `red green blue yellow cyan magenta white off` |
| `blink <color>` | Blink one colour |
| `cycle` | Cycle red, green, blue (default) |
| `rate <ms>` | Step period, 10 to 10000 ms (default 500) |
| `pwm` | Show PWM counts, duty and average voltage |
| `pwm <0-100>` | Set duty in %, one decimal allowed (`37.5`) |
| `pwm counts <0-1000>` | Set high time in counts |
| `ma` | Show loop current (predicted from the calibration) |
| `ma <mA>` | Set loop current, for example `ma 12.5` |
| `cal` | Show the 4-20 mA calibration and reachable range |
| `cal lo <mA>` | Record the meter reading at the present output (do this near 4 mA) |
| `cal hi <mA>` | Record the meter reading at the present output (do this near 20 mA) |
| `cal reset` | Back to 0 counts = 4 mA, 1000 counts = 20 mA |
| `pwm2`, `ma2`, `cal2` ... | Same commands for channel 2 (P1_3), e.g. `ma2 8`, `cal2 lo 3.95`. `pwm1`/`ma1`/`cal1` also work for channel 1 |
| `save` | Store current settings, including both channels' calibration, in flash (kept over power-off) |
| `defaults` | Erase saved settings, restore defaults |
| `status` | Show current state |
| `reset` | Reboot the chip |

## 4-20 mA outputs (NOYITO voltage-to-current modules)

There are two channels, each wired to its own module exactly as below:

| Channel | PWM pin | Commands |
|---|---|---|
| 1 | P1_4 (J1-2), CTIMER1 match 2 | `pwm`, `ma`, `cal` |
| 2 | P1_3, CTIMER1 match 1 | `pwm2`, `ma2`, `cal2` |

P1_3 is on the Arduino-style header (it is the LPSPI0 PCS0 pin); check the
FRDM-MCXA153 schematic or silkscreen for the exact header position. Both
channels share the 12 kHz period, but each has its own duty cycle.

Each module needs its own RC filter and its own calibration (steps below,
using `ma2`/`cal2` for channel 2). One 12-24 V supply can power both. The
module's VIN- and 0V are common, so both loops' 0V end up tied to FRDM GND:
that is fine when both BAM inputs return to the same supply negative, but
not if the BAM inputs are isolated from each other.

Per module:

The module has two terminal blocks:

- **Input block (`VIN`)**: the 0-3.3 V control voltage.
- **Output block (`AOUT+` / `0V` / `24V`)**: supply in and 4-20 mA out.
  The front silkscreen reads `AO+` for `AOUT+`.

Wiring:

```
INPUT block
  VIN+  ← J1-2 (P1_4) ──[10k]──┬── VIN+
                               ═ 2.2 µF electrolytic (− to GND)
  VIN−  ← FRDM GND (J6-8)
  third terminal: not connected

OUTPUT block
  24V   ← supply + (12-24 V)
  0V    ← supply −  and  FRDM GND (J6-8)
  AOUT+ → mA meter → 250 Ω load → 0V
```

Check the `VIN+` / `VIN−` labels on your module before wiring. Supply off:
`pwm 50` should give about 1.65 V from VIN+ to VIN−.

1. Trim the pots so the range has a little headroom: `pwm 0` should give
   slightly **below** 4 mA (about 3.8), and `pwm 100` slightly **above**
   20 mA (about 20.5). If 0 counts already gives more than 4 mA, then
   4 mA can't be reached.
2. `ma 4`, read the meter, enter it: `cal lo 3.95` (your reading).
3. `ma 20`, read the meter, enter it: `cal hi 20.12` (your reading).
4. Check with `ma 4`, `ma 12`, `ma 20`. Repeat steps 2-3 if needed.
5. `save` to keep the calibration over power-off.

Resolution is about 16 µA per count (1000 counts over 4-20 mA).

## Hardware notes

- Connect the board to the Pi with the USB-C port labelled **MCU-Link** (J15).
- RGB LED: red P3_12, green P3_13, blue P3_0 (active low).
- PWM: channel 1 on P1_4 (header J1 pin 2), channel 2 on P1_3, 12 kHz,
  1000 counts per period. Add an RC low-pass filter (for example 10 kΩ +
  1 µF) to each to get 0 to 3.3 V.
- Core clock: 48 MHz (FIRC, as left by the boot ROM).
- Saved settings live in the last 8 KB flash sector (0x1E000). `make flash`
  does not erase it. Settings saved by the older single-channel firmware are
  still read (as channel 1); the first `save` rewrites the sector in the
  two-channel format.

## One-time setup (already done on this Pi)

```
sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi binutils-arm-none-eabi
python3 -m venv ~/.venvs/pyocd
~/.venvs/pyocd/bin/pip install pyocd pyserial
~/.venvs/pyocd/bin/pyocd pack install mcxa153
```

## Files

| Path | Contents |
|---|---|
| `main.c` | LED, UART shell, PWM, commands |
| `settings.c`, `settings.h` | Save and load settings in flash via the boot ROM API |
| `Makefile` | Build and flash rules |
| `device/` | NXP MCXA153 headers, startup code, system init, linker script (from NXP DFP 26.09.00) |
| `cmsis/` | Arm CMSIS 6.1.0 core headers |
