#!/usr/bin/env python3
"""Moving rainbow or single-colour wave for the IS31FL3736 REPL using Auto Breath Mode.

How it works
------------
All three ABM slots get the same triangle wave (period 3k):
    T1 = k (fade in)   T2 = 0 (hold)   T3 = k (fade out)   T4 = k (off)
but begin at different points of the cycle (the 'start' option / LB bits):
    ABM-1 starts at T1 (delay 0), ABM-2 at T4 (delay k), ABM-3 at T3 (delay 2k)
so they are three copies of one wave, one third of a period apart.

Each RGB LED wires its R, G, B channels to a rotation of those slots:
    phase p:  R = slot p, G = slot p+1, B = slot p+2  (mod 3)
Any three copies of the wave a third of a period apart form a rainbow (one
primary fades in while the next fades out, the third is off). LED phase p sees
the colours of LED phase 0 delayed by p*k, so the rainbow travels toward
increasing phase. Phase = (dx*x + dy*y) mod 3, with x left->right, y bottom->top, so
--direction right/left/up/down (dx,dy = 1,0 / 2,0 / 0,1 / 0,2) picks which way it travels;
up/down animate by rows, right/left by columns. bl-tr/br-tl/tl-br/tr-bl (1,1 / 2,1 / 1,2 / 2,2)
sweep diagonally from one corner to the opposite one. --dx/--dy override it (step 2 == -1 mod 3).

The panels are treated as one tall display: the firmware numbers LEDs as display coordinates
(LED 0 = bottom-left of the whole stack, X first, then upward, whichever chip they are on), so
y = led // 8 here. This needs firmware with that mapping (see README, "Panel layout").

Only three timing slots exist, so the pattern repeats every 3 LEDs along the
travel direction.

Colour waves (--colors)
-----------------------
Each LED gets ONE slot for all of its lit channels (instead of a rotation), so it fades from black up to
its colour and back down, and the three slots make that a wave travelling in the same direction.
An ABM dot always breathes to full intensity (measured: its PWM register does not scale the peak, and
an ABM dot at PWM 0 still lights), so a channel is either on or off and only these colours exist:
red, green, blue, yellow, cyan, magenta, white (and black = off). Colours are given as names or RRGGBB;
any RRGGBB is snapped to the nearest of those (each channel on if >= 0x80), which is never an error.
Brightness is set with --gcc.
The pattern is tiled over the LEDs, X first then upward, like the firmware's 'fill pwm': a pattern
shorter than the display repeats and a longer one is truncated. An unrecognised entry is shown as black
with a warning, never a failure (a DMX device has nobody to report errors to).

Usage
-----
    wave.py                     print the REPL commands (rainbow)
    wave.py --send              send them to COM4 (through powershell.exe, WSL)
    wave.py --colors red,blue --send        colour wave: red/blue tiles fading to black
    wave.py --stop --send       reset the board / blank the LEDs

Works from WSL or from native Windows (python wave.py --send); either way it talks to the port
through powershell.exe and send.ps1. Close any serial monitor (VS Code/PlatformIO, PuTTY, Arduino, ...)
first: COM4 can only be open in one program.
"""
import argparse
import os
import subprocess
import sys

WIDTH = 8
LEDS_PER_PANEL = 32
# Table 15/16: T1/T3 code n = 0.21 * 2^n s; T2/T4 code n = 0.21 * 2^(n-1) s (0 = 0 s).
BASE_S = 0.21


# (dx, dy): phase step per column / per row. The wave travels toward increasing phase and 2 == -1 (mod 3).
DIRECTIONS = {
    "right": (1, 0), "left": (2, 0), "up": (0, 1), "down": (0, 2),
    # corner to opposite corner: BL = bottom left, TR = top right, etc.
    "bl-tr": (1, 1), "br-tl": (2, 1), "tl-br": (1, 2), "tr-bl": (2, 2),
}


NAMES = {"black": "000000", "red": "ff0000", "green": "00ff00", "blue": "0000ff",
         "yellow": "ffff00", "cyan": "00ffff", "magenta": "ff00ff", "white": "ffffff"}


def parse_colors(text):
    """Split 'red,00ff00 ...' into on/off RRGGBB tokens (each channel 00 or ff): names or hex, snapped to the
    nearest of the 8 ABM colours. Anything unrecognised becomes 000000 with a warning, never an error."""
    out = []
    for tok in text.replace(",", " ").split():
        tok = NAMES.get(tok.lower(), tok.lstrip("#"))
        if len(tok) == 6 and all(c in "0123456789abcdefABCDEF" for c in tok):
            out.append("".join("ff" if int(tok[i:i + 2], 16) >= 0x80 else "00" for i in (0, 2, 4)))
        else:
            print(f"warning: bad colour {tok!r}, using 000000", file=sys.stderr)
            out.append("000000")
    return out or ["000000"]


def build(panels, k_code, gcc, dx, dy, peak, colors=None):
    if not 0 <= k_code <= 6:
        sys.exit("--speed must be 0..6 (k = 0.21 * 2^n seconds)")
    t13 = k_code          # T1 / T3 code -> k seconds
    t4 = k_code + 1       # T4 code      -> same k seconds
    cmds = [f"panel {panels}", f"gcc {gcc}", "mode pwm"]

    # ABM breathes up to the PWM value, so PWM is the peak brightness per channel.
    n_leds = panels * LEDS_PER_PANEL
    if colors:
        # Colour wave: tile the pattern over the LEDs (repeat if short, truncate if long).
        tiled = [colors[i % len(colors)] for i in range(n_leds)]
        cmds.append("load pwm " + " ".join(tiled))
    else:
        cmds.append(f"fill pwm from 0 to {n_leds - 1} with {peak}")

    modes = []
    for led in range(n_leds):
        x = led % WIDTH
        y = led // WIDTH          # display row, 0 = bottom of the whole stack
        p = (dx * x + dy * y) % 3
        if colors:
            # One slot for each lit channel: fade colour <-> black. A channel at 00 is left on plain PWM
            # (mode 0): an ABM-assigned dot with PWM 0 does not stay dark on the chip, it lights up.
            modes += [1 + p if int(tiled[led][2 * c:2 * c + 2], 16) else 0 for c in range(3)]
        else:
            modes += [1 + (p + c) % 3 for c in range(3)]   # R, G, B
    cmds.append("load abm " + " ".join(map(str, modes)))

    cmds.append("mode abm")
    # Each define re-commits (B_EN toggle), restarting all slots from their start phase,
    # so the last one leaves all three running in step.
    cmds.append(f"define abm 1 {t13} 0 {t13} {t4} start 1")   # delay 0
    cmds.append(f"define abm 2 {t13} 0 {t13} {t4} start 4")   # delay k
    cmds.append(f"define abm 3 {t13} 0 {t13} {t4} start 3")   # delay 2k
    return cmds


def send(cmds, port):
    here = os.path.dirname(os.path.abspath(__file__))
    ps1 = os.path.join(here, "send.ps1")
    if os.name != "nt":   # WSL: powershell.exe needs a Windows path
        ps1 = subprocess.check_output(["wslpath", "-w", ps1], text=True).strip()
    p = subprocess.run(
        ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", ps1, "-Port", port],
        input="\n".join(cmds) + "\n", text=True)
    return p.returncode


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--panels", type=int, default=3, choices=(1, 2, 3))
    ap.add_argument("--speed", type=int, default=3,
                    help="k code 0-6: colour step time 0.21*2^n s (default 3 = 1.68 s, period 5.04 s)")
    ap.add_argument("--gcc", type=int, default=100)
    ap.add_argument("--direction", choices=DIRECTIONS, default="right",
                    help="which way the rainbow travels (default right); up/down move by rows, "
                         "bl-tr/br-tl/tl-br/tr-bl sweep corner to opposite corner")
    ap.add_argument("--dx", type=int, help="override phase step per column (1 = right, 2 = left, 0 = none)")
    ap.add_argument("--dy", type=int, help="override phase step per row (1 = up, 2 = down, 0 = none)")
    ap.add_argument("--peak", default="ff30a0",
                    help="RRGGBB peak brightness per channel (default ff30a0: the green LEDs are much "
                         "brighter than red, so G is held back to make yellow/orange/violet readable)")
    ap.add_argument("--colors", help="colour wave instead of a rainbow: names or RRGGBB, comma separated (red green blue yellow "
                                     "cyan magenta white; other values snap to the nearest), tiled over the LEDs "
                                     "(X first), each LED fading to black; shorter/longer than the display "
                                     "is fine (repeats/truncates)")
    ap.add_argument("--port", default="COM4", help="Windows COM port (default COM4)")
    ap.add_argument("--send", action="store_true", help="send to the board instead of printing")
    ap.add_argument("--stop", action="store_true", help="just reset the board")
    a = ap.parse_args()

    dx, dy = DIRECTIONS[a.direction]
    dx = dx if a.dx is None else a.dx
    dy = dy if a.dy is None else a.dy
    cmds = ["reset"] if a.stop else build(a.panels, a.speed, a.gcc, dx, dy, a.peak,
                                         parse_colors(a.colors) if a.colors is not None else None)
    if a.send:
        sys.exit(send(cmds, a.port))
    print("\n".join(cmds))


if __name__ == "__main__":
    main()
