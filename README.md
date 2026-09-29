# is31fl3736-REPL

A REPL over the is31fl3736 chip LED driver for controlling LED matrices in real-time.

Firmware: `si31fl3736-REPL/src/main.cpp` (Teensy 3.2, PlatformIO). Serial: 115200, 8N1, no flow control.
Datasheet: `si31fl3736-REPL/doc/IS31FL3736_DS.pdf`.

# Serial commands:
* help
* panel [<n 1-3>]                       -> number of panels incl. the master (default 1); panels stack
                                          in Y with the master on top (8x4, 8x8 or 8x12 RGB LEDs). Resets all panels and sets
                                          SYNC (master/slaves) when n > 1
* mode pwm                              -> global PWM mode (B_EN=0)
* mode abm                              -> global Auto Breath mode (B_EN=1)
* load pwm RRGGBB RRGGBB ...            -> one hex triplet per RGB LED, LED 0 = bottom-left of the display (X first,
                                          then upward through all panels, see Panel layout);
                                          a non-zero channel also turns that dot on
* load abm mR mG mB mR mG mB ...        -> one mode per colour channel, LED 0 upward
                                          (0 = PWM, 1-3 = ABM-1..3). ABM dots breathe up to their PWM
                                          value, so `load pwm` them first
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
* dump                                  -> print current shadow state

# Panel layout and assembly

This is the reference the firmware, the scripts and the final DMX display should all follow.

* **Panel:** 8 x 4 RGB LEDs on one IS31FL3736 (12 SW rows x 8 CS columns; each LED row is three SW rows,
  SW1-3 = the bottom row). Schematic: `si31fl3736-REPL/doc/DMX-LED-Panel-Schematic.pdf`.
* **Stack:** panels stack in Y into a display 8 LEDs wide and 4 LEDs tall per panel (8x8 for the 2-panel
  final display, 8x12 for the 3-panel proof of concept). **The master (panel 0) is the TOP panel;** the
  slaves sit below it.
* **Numbering:** LEDs are display coordinates. LED 0 is the bottom-left of the whole display, X runs
  left to right, then rows run bottom to top through all panels, whichever chip they are on. So LED n is
  at x = n % 8, row n / 8, and `load`/`fill`/DMX all use this. (`assign abm` and `dump` use raw chip dot
  indices, panel*96 + dot.) `MASTER_AT_TOP` in `main.cpp` holds the one assumption about the stack; set it
  to `false` if the master is ever the bottom panel and nothing else changes.
* **Addresses:** each board's I2C address is set by its ADDR1/ADDR2 solder jumpers (JP1-JP8), not by its
  place on the ribbon: master 0x50 (ADDR1=GND, ADDR2=GND), panel 1 0x51 (ADDR1=SCL), panel 2 0x52
  (ADDR1=SDA).
* **Connection:** the 9-pin I/O connector (J1: IICRST, SDB, INT_B, SCL, SDA, SYNC, Vio, Vcc, GND) is one
  shared bus, daisy chained on a ribbon; the order along the ribbon doesn't matter electrically. The
  master drives SYNC (config SYNC=01) and the slaves take it (SYNC=10), so the ABM timers of all panels
  run together; the firmware sets this when `panel n` is issued.

# Scripts (`scripts/`)

* `send.ps1` - sends REPL commands (stdin or `-File`) to a COM port through .NET's `SerialPort` and
  prints the replies. WSL2 can't see Windows COM ports, so scripts run from WSL call this through
  `powershell.exe`.
* `rainbow.py` - moving rainbow using Auto Breath Mode. All three ABM slots run the same triangle
  wave (T1=k, T2=0, T3=k, T4=k) but start at different phases (T1, T4, T3 = delays 0, k, 2k). Each LED
  maps R, G, B to a rotation of the slots, so the rainbow travels across the panels. Only three
  timers exist, so it repeats every 3 LEDs and is limited to primary crossfades.

      python3 scripts/rainbow.py --send          # run on COM4 (omit --send to print the commands)
      python3 scripts/rainbow.py --send --direction down            # animate by rows, top to bottom
      python3 scripts/rainbow.py --send --speed 2 --direction up    # faster, bottom to top
      python3 scripts/rainbow.py --send --direction bl-tr           # diagonal, bottom-left to top-right
      python3 scripts/rainbow.py --stop --send   # reset the board

  Options: `--panels`, `--speed 0-6` (step time 0.21*2^n s), `--gcc`, `--direction right|left|up|down|bl-tr|br-tl|tl-br|tr-bl` (default `right`; `up`/`down` animate by rows,
  the corner pairs sweep diagonally, e.g. `bl-tr` = bottom-left to top-right,
  `left`/`right` by columns; `--dx/--dy` override the underlying phase steps),
  `--peak RRGGBB` (per-channel peak, default `ff30a0`: green LEDs are much brighter than red, so G is
  held back to keep yellow/orange/violet readable), `--port`.
* `capture.py`, `montage.py` - optional webcam checking (frames and contact sheets into `captures/`,
  which is git-ignored). Run with a Windows venv with OpenCV:

      python.exe -m venv .venv-win && .venv-win\Scripts\pip install opencv-python numpy pygrabber
      .venv-win/Scripts/python.exe scripts/capture.py --index 0 --out captures/shot.jpg --exposure -8

  Camera indices shift across reboots; use `--name C920` (or `--list`) instead of `--index`
  (needs `pip install pygrabber`). Close other apps using the camera first. Very short exposures catch the multiplexing scan and show
  partial rows; lower `gcc` instead if the LEDs clip to white.
