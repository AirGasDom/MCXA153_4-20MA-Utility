# FRDM-MCXA153 blinky + serial shell

Bare-metal firmware for the NXP FRDM-MCXA153 board: RGB LED control, a
serial command shell, a PWM output on P1_4 (J1-2), and settings saved to flash.

## Make commands

Run these from this directory (`/home/dom/proj/MCXA153/blinky`).

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
| `save` | Store current settings, including calibration, in flash (kept over power-off) |
| `defaults` | Erase saved settings, restore defaults |
| `status` | Show current state |
| `reset` | Reboot the chip |

## 4-20 mA output (NOYITO voltage-to-current module)

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
- PWM: P1_4 on header J1 pin 2, 12 kHz, 1000 counts per period. Add an RC
  low-pass filter (for example 10 kΩ + 1 µF) to get 0 to 3.3 V.
- Core clock: 48 MHz (FIRC, as left by the boot ROM).
- Saved settings live in the last 8 KB flash sector (0x1E000). `make flash`
  does not erase it.

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
