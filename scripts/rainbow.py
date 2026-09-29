#!/usr/bin/env python3
"""Moving rainbow for the IS31FL3736 REPL using Auto Breath Mode.

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

Only three timing slots exist, so the pattern repeats every 3 LEDs along the
travel direction.

Usage
-----
    rainbow.py                  print the REPL commands
    rainbow.py --send           send them to COM4 (through powershell.exe, WSL)
    rainbow.py --stop --send    reset the board / blank the LEDs
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


def build(panels, k_code, gcc, dx, dy, peak):
    if not 0 <= k_code <= 6:
        sys.exit("--speed must be 0..6 (k = 0.21 * 2^n seconds)")
    t13 = k_code          # T1 / T3 code -> k seconds
    t4 = k_code + 1       # T4 code      -> same k seconds
    cmds = [f"panel {panels}", f"gcc {gcc}", "mode pwm"]

    # ABM breathes up to the PWM value, so PWM is the peak brightness per channel.
    n_leds = panels * LEDS_PER_PANEL
    cmds.append(f"fill pwm from 0 to {n_leds - 1} with {peak}")

    modes = []
    for led in range(n_leds):
        x = led % WIDTH
        y = led // WIDTH          # rows stack bottom -> top across panels
        p = (dx * x + dy * y) % 3
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
    ps1 = subprocess.check_output(["wslpath", "-w", os.path.join(here, "send.ps1")], text=True).strip()
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
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--send", action="store_true", help="send to the board instead of printing")
    ap.add_argument("--stop", action="store_true", help="just reset the board")
    a = ap.parse_args()

    dx, dy = DIRECTIONS[a.direction]
    dx = dx if a.dx is None else a.dx
    dy = dy if a.dy is None else a.dy
    cmds = ["reset"] if a.stop else build(a.panels, a.speed, a.gcc, dx, dy, a.peak)
    if a.send:
        sys.exit(send(cmds, a.port))
    print("\n".join(cmds))


if __name__ == "__main__":
    main()
