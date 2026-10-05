# is31fl3736-REPL

A REPL over the is31fl3736 chip LED driver for controlling LED matrices in real-time.

Firmware: `si31fl3736-REPL/src/main.cpp` (Teensy 3.2, PlatformIO). Serial: 115200, 8N1, no flow control.
Datasheet: `si31fl3736-REPL/doc/IS31FL3736_DS.pdf`.

# Serial commands:
* help
* panel [<n 1-3>]                       -> number of panels incl. the master (default 1); panels stack
                                          in Y with the master at the bottom (8x4, 8x8 or 8x12 RGB LEDs). Resets all panels and sets
                                          SYNC (master/slaves) when n > 1
* mode pwm                              -> global PWM mode (B_EN=0)
* mode abm                              -> global Auto Breath mode (B_EN=1)
* load pwm RRGGBB RRGGBB ...            -> one hex triplet per RGB LED, LED 0 = bottom-left of the display (X first,
                                          then upward through all panels, see Panel layout);
                                          a non-zero channel also turns that dot on
* load abm mR mG mB mR mG mB ...        -> one mode per colour channel, LED 0 upward
                                          (0 = PWM, 1-3 = ABM-1..3). An ABM dot always breathes 0 -> full
                                          intensity; its PWM value does not scale the peak
* fill pwm from <led> to <led> with RRGGBB ...   -> repeat the pattern over LEDs, truncated at the end
* fill abm from <led> to <led> with m m m ...    -> repeat a per-channel mode pattern
* assign abm <n 0-3> <dot...>           -> assign by explicit dot index (panel*96 + dot)
* define abm <n 1-3> <T1> <T2> <T3> <T4> [start <1-4>] [end on|off] [loop <0-4095>]
                                        -> program ABM-n timing on all panels (raw T codes, datasheet
                                          Tables 15/16). `start` = phase the loop begins at (default T1),
                                          `end` = finish on/off (default off), `loop` = repeat count
                                          (default 0 = endless)
* gcc <0-255>                           -> Global Current Control
* reset                                 -> IC reset and re-init
* wave <RRGGBB ...> [width <leds>] [speed <leds/s>] [dir <d>] [sharp <1-8>] [fps <5-60>]
                                        -> smooth travelling wave generated on the Teensy in PWM (see "Wave modes").
                                          Colours tile over the LEDs in display order; `<d>` is right, left, up, down,
                                          bl-tr, br-tl, tl-br or tr-bl. Never an error: bad values use defaults or are
                                          clamped. `wave off`, or any command that changes the LEDs, stops it
* dump                                  -> print current shadow state

# Panel layout and assembly

This is the reference the firmware, the scripts and the final DMX display should all follow.

* **Panel:** 8 x 4 RGB LEDs on one IS31FL3736 (12 SW rows x 8 CS columns; each LED row is three SW rows,
  SW1-3 = the bottom row). Schematic: `si31fl3736-REPL/doc/DMX-LED-Panel-Schematic.pdf`.
* **Stack:** panels stack in Y into a display 8 LEDs wide and 4 LEDs tall per panel (8x8 for the 2-panel
  final display, 8x12 for the 3-panel proof of concept). **The master (panel 0, 0x50) is the BOTTOM panel;** the
  slaves sit above it (panel 1 at the bottom, nearest the webcam).
* **Numbering:** LEDs are display coordinates. LED 0 is the bottom-left of the whole display, X runs
  left to right, then rows run bottom to top through all panels, whichever chip they are on. So LED n is
  at x = n % 8, row n / 8, and `load`/`fill`/DMX all use this. (`assign abm` and `dump` use raw chip dot
  indices, panel*96 + dot.) `MASTER_AT_TOP` in `main.cpp` holds the one assumption about the stack; set it
  to `true` if the master is ever the top panel and nothing else changes.
* **Addresses:** each board's I2C address is set by its ADDR1/ADDR2 solder jumpers (JP1-JP8), not by its
  place on the ribbon: master 0x50 (ADDR1=GND, ADDR2=GND), panel 1 0x51 (ADDR1=SCL), panel 2 0x52
  (ADDR1=SDA).
* **Connection:** the 9-pin I/O connector (J1: IICRST, SDB, INT_B, SCL, SDA, SYNC, Vio, Vcc, GND) is one
  shared bus, daisy chained on a ribbon; the order along the ribbon doesn't matter electrically. The
  master drives SYNC (config SYNC=01) and the slaves take it (SYNC=10), so the ABM timers of all panels
  run together; the firmware sets this when `panel n` is issued.

# Wave modes

Three ways to get a travelling wave, all keeping the REPL available:

| Mode | Command | Colours | Look |
|------|---------|---------|------|
| ABM rainbow | `wave.py --send` | crossfades through R, G, B | steps of 3 LEDs (only 3 breath timers) |
| ABM colour wave | `wave.py --send --colors red,blue` | 7 on/off colours | each LED fades colour <-> black, still 3 phases |
| PWM wave | `wave.py --send --pwm --colors ff8000` or `wave ...` in the REPL | any RGB | smooth bump of any width |

ABM runs on the chip by itself, so a DMX controller would only set a few values. Its limits come from the
chip: three timers means three phases along the travel direction, and a dot can only fade between off and
full. A smooth `..ooO0Ooo..` bump needs many brightness levels at once, so the PWM wave is computed by the
firmware instead: `waveTick()` in `loop()` renders a frame (default 30 fps), applies a gamma table
(`GAMMA_EXP`, 2.2) and writes only the PWM rows that changed. Under DMX the controller still only needs to
supply a handful of values (colours, width, speed, direction), not 288 channels. A full redraw of 3 panels took
about 7 ms (up to ~19 ms in the worst case measured), so 30 fps has plenty of headroom; `dump` shows the
frame count and last frame time while a wave runs. Starting a wave sets PWM mode and clears all ABM
assignments; `wave off`, `reset`, `panel`, `mode`, `load`, `fill`, `assign` and `define` stop it first.

# Scripts (`scripts/`)

* `send.ps1` - sends REPL commands (stdin or `-File`) to a COM port through .NET's `SerialPort` and
  prints the replies. WSL2 can't see Windows COM ports, so scripts run from WSL call this through
  `powershell.exe`.
* `wave.py` (was `rainbow.py`) - travelling wave using Auto Breath Mode, either a rainbow or a
  single-colour wave. All three ABM slots run the same triangle wave (T1=k, T2=0, T3=k, T4=k) but start at
  different phases (T1, T4, T3 = delays 0, k, 2k). Only three timers exist, so it repeats every 3 LEDs.
  * **Rainbow (default):** each LED maps R, G, B to a rotation of the slots, so the colours crossfade
    through the primaries as the wave travels.
  * **PWM wave (`--pwm`):** the firmware animates it (see "Wave modes"); the script only sends a `wave`
    command. `--colors` takes names or any RRGGBB (no snapping), `--width` (LEDs, default 8), `--wave-speed`
    (LEDs/s, default 4, negative reverses), `--sharp 1-8` (narrower bright core), `--direction` and `--gcc`
    apply. The ABM-only `--speed/--shape/--peak/--dx/--dy` are ignored. Example:
    `python3 scripts/wave.py --send --pwm --colors red --direction up --width 10`.
  * **Colour wave (`--colors`):** each LED uses one slot for all of its lit channels, so it fades from
    black up to its colour and back to black. ABM dots always breathe to full intensity (measured: the PWM
    register does not scale the peak, and an ABM dot at PWM 0 still lights), so a channel is on or off and
    the available colours are red, green, blue, yellow, cyan, magenta and white (black = off). Give names or
    RRGGBB values; any other RRGGBB snaps to the nearest of those (each channel on if >= 0x80). Brightness
    is set with `--gcc`. The pattern is tiled over the LEDs, X first then upward (like `fill pwm`): a short
    pattern repeats and a long one is truncated. **A pattern that doesn't match the number of LEDs is not an
    error, and neither is an unrecognised colour (it shows black and prints a warning),** because a DMX
    device has nobody to report errors to.

      python3 scripts/wave.py --send          # rainbow on COM4 (omit --send to print the commands)
      python3 scripts/wave.py --send --direction down            # animate by rows, top to bottom
      python3 scripts/wave.py --send --speed 2 --direction up    # faster, bottom to top
      python3 scripts/wave.py --send --direction bl-tr           # diagonal, bottom-left to top-right
      python3 scripts/wave.py --send --colors red                # red wave fading to black
      python3 scripts/wave.py --send --colors red,blue --direction up   # alternating columns, rising
      python3 scripts/wave.py --stop --send   # reset the board

  Options: `--panels`, `--speed 0-6` (step time 0.21*2^n s), `--gcc`, `--direction right|left|up|down|bl-tr|br-tl|tl-br|tr-bl` (default `right`; `up`/`down` animate by rows,
  the corner pairs sweep diagonally, e.g. `bl-tr` = bottom-left to top-right,
  `left`/`right` by columns; `--dx/--dy` override the underlying phase steps),
  `--shape pulse|notch|saw-a|saw-b` (ABM breath profile: how the 3 slots' fade is shaped; `pulse` is the
  original rainbow timing; none of them can make the wave truly smooth),
  `--pwm --width --wave-speed --sharp` (PWM wave), `--colors NAME|RRGGBB,...` (colour wave, see above), `--peak RRGGBB` (rainbow only: per-channel
  peak, default `ff30a0`; note the measurement above, so this may have little effect), `--port`.
* `capture.py`, `montage.py` - optional webcam checking (frames and contact sheets into `captures/`,
  which is git-ignored). Run with a Windows venv with OpenCV:

      python.exe -m venv .venv-win && .venv-win\Scripts\pip install opencv-python numpy pygrabber
      .venv-win/Scripts/python.exe scripts/capture.py --index 0 --out captures/shot.jpg --exposure -8

  Camera indices shift across reboots; use `--name C920` (or `--list`) instead of `--index`
  (needs `pip install pygrabber`). Close other apps using the camera first. Very short exposures catch the multiplexing scan and show
  partial rows; lower `gcc` instead if the LEDs clip to white.
  With the diffusers on, `--exposure -7 --gain 0` shows static LEDs well (-6 washes out, -9 is black); use
  about -9 to -11 for ABM waves at `--gcc` 30-255.
