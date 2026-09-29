# is31fl3736-REPL

A REPL over the is31fl3736 chip LED driver for controlling LED matrices in real-time.

Firmware: `si31fl3736-REPL/src/main.cpp` (Teensy 3.2, PlatformIO). Serial: 115200, 8N1, no flow control.
Datasheet: `si31fl3736-REPL/doc/IS31FL3736_DS.pdf`.

# Serial commands:
* help
* panel [<n 1-3>]                       -> number of panels incl. the master (default 1); panels stack
                                          in Y (8x4, 8x8 or 8x12 RGB LEDs). Resets all panels and sets
                                          SYNC (master/slaves) when n > 1
* mode pwm                              -> global PWM mode (B_EN=0)
* mode abm                              -> global Auto Breath mode (B_EN=1)
* load pwm RRGGBB RRGGBB ...            -> one hex triplet per RGB LED, LED 0 upward (X first, then Y);
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

# Scripts (`scripts/`)

* `send.ps1` - sends REPL commands (stdin or `-File`) to a COM port through .NET's `SerialPort` and
  prints the replies. WSL2 can't see Windows COM ports, so scripts run from WSL call this through
  `powershell.exe`.
* `rainbow.py` - moving rainbow using Auto Breath Mode. All three ABM slots run the same triangle
  wave (T1=k, T2=0, T3=k, T4=k) but start at different phases (T1, T4, T3 = delays 0, k, 2k). Each LED
  maps R, G, B to a rotation of the slots, so the rainbow travels across the panels. Only three
  timers exist, so it repeats every 3 LEDs and is limited to primary crossfades.

      python3 scripts/rainbow.py --send          # run on COM4 (omit --send to print the commands)
      python3 scripts/rainbow.py --send --speed 2 --dx 0 --dy 1   # faster, travelling upward
      python3 scripts/rainbow.py --stop --send   # reset the board

  Options: `--panels`, `--speed 0-6` (step time 0.21*2^n s), `--gcc`, `--dx/--dy` (direction),
  `--peak RRGGBB` (per-channel peak, default `ff30a0`: green LEDs are much brighter than red, so G is
  held back to keep yellow/orange/violet readable), `--port`.
* `capture.py`, `montage.py` - optional webcam checking (frames and contact sheets into `captures/`,
  which is git-ignored). Run with a Windows venv with OpenCV:

      python.exe -m venv .venv-win && .venv-win\Scripts\pip install opencv-python numpy
      .venv-win/Scripts/python.exe scripts/capture.py --index 0 --out captures/shot.jpg --exposure -8

  Close other apps using the camera first. Very short exposures catch the multiplexing scan and show
  partial rows; lower `gcc` instead if the LEDs clip to white.
