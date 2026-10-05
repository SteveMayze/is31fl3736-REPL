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
    wave.py --pwm --colors red --send       smooth PWM wave made by the firmware (any RGB colour)
    wave.py --pwm --colors rainbow --send   continuous scrolling rainbow in PWM (rainbow-bump: spectrum per bump)
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


# Breath profile of the three slots: (T1 fade in, T2 hold, T3 fade out, T4 off) in units of k = 0.21 * 2^--speed s,
# each a power of two (T2/T4 may be 0). The three slots are the same waveform a third of a period apart, so the
# period must be a multiple of 3 units and each offset must land on a segment boundary (what 'start' selects).
SHAPES = {
    "pulse": (1, 0, 1, 1),   # narrow bright pulse hopping along (the original rainbow timing)
    "notch": (1, 1, 1, 0),   # always lit except for a dark notch travelling along
    "saw-a": (1, 2, 2, 1),   # quick rise, slow fall
    "saw-b": (2, 2, 1, 1),   # slow rise, quick fall
}


def slot_timing(shape, k_code):
    """T1..T4 register codes and the 'start' value (1-4) of the 3 slots for a shape; slot p lags slot 0 by p/3 period."""
    a, b, c, d = SHAPES[shape]
    period = a + b + c + d
    bounds = [0, a, a + b, a + b + c]            # where T1..T4 begin within the cycle
    code = lambda t, n: 0 if t == 0 else k_code + t.bit_length() - 1 + n   # n=0: T1/T3 table, n=1: T2/T4 table
    codes = (code(a, 0), code(b, 1), code(c, 0), code(d, 1))
    if max(codes[0], codes[2]) > 7 or codes[1] > 8 or codes[3] > 10:
        sys.exit("--speed too high for this --shape (a T register would exceed its range)")
    lengths = (a, b, c, d)
    for base in bounds:                           # slot 0 may itself start at a later boundary
        # prefer the segment with a length: a zero-length T2/T4 would start the loop on the next segment anyway
        pick = [[i for i in range(4) if bounds[i] == (base - p * period // 3) % period and lengths[i]]
                for p in range(3)]
        if all(pick):
            return codes, [i[0] + 1 for i in pick]
    sys.exit(f"shape {shape!r} cannot offset its slots by a third of a period")


def parse_colors(text, snap=True):
    """Split 'red,00ff00 ...' into RRGGBB tokens: names or hex. With snap (ABM) each channel becomes 00 or ff, the
    nearest of the 8 ABM colours; without it (PWM wave) any colour is kept. Anything unrecognised becomes 000000
    with a warning, never an error."""
    out = []
    for tok in text.replace(",", " ").split():
        tok = NAMES.get(tok.lower(), tok.lstrip("#"))
        if len(tok) == 6 and all(c in "0123456789abcdefABCDEF" for c in tok):
            out.append("".join("ff" if int(tok[i:i + 2], 16) >= 0x80 else "00" for i in (0, 2, 4))
                       if snap else tok.lower())
        else:
            print(f"warning: bad colour {tok!r}, using 000000", file=sys.stderr)
            out.append("000000")
    return out or ["000000"]


def build(panels, k_code, gcc, dx, dy, peak, colors=None, shape="pulse"):
    if not 0 <= k_code <= 6:
        sys.exit("--speed must be 0..6 (k = 0.21 * 2^n seconds)")
    (t1, t2, t3, t4), starts = slot_timing(shape, k_code)
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
    for n, start in enumerate(starts, 1):
        cmds.append(f"define abm {n} {t1} {t2} {t3} {t4} start {start}")   # slot n lags slot 1 by (n-1)/3 period
    return cmds


def build_pwm(gcc, direction, colors, width, plateau, gap, k_code, sharp):
    """Commands for the firmware's own PWM wave (smooth, any RGB colour; colors may be ["rainbow"] for a scrolling
    rainbow or ["rainbow-bump"] for the spectrum across each bump): the Teensy animates, we only send a few parameters. --speed uses the same scale as the ABM modes: the wave advances one LED per step time
    k = 0.21 * 2^k_code seconds (0 = fastest, 6 = slowest). The firmware clamps odd values and never errors."""
    if not 0 <= k_code <= 6:
        sys.exit("--speed must be 0..6 (k = 0.21 * 2^n seconds)")
    leds_per_s = 1 / (BASE_S * 2 ** k_code)
    return [f"gcc {gcc}", "wave " + " ".join(colors or ["ffffff"]) +
            f" width {width:g} plateau {plateau:g} gap {gap:g} speed {leds_per_s:.3f} dir {direction} sharp {sharp}"]


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
    ap.add_argument("--speed", type=int,
                    help="0 (fastest) to 6 (slowest): step time k = 0.21*2^n s, for ABM and --pwm alike. ABM: one "
                         "colour step per k (default 3 = 1.68 s, period 5.04 s); --pwm: the wave moves one LED "
                         "per k (default 0 = 4.8 LEDs/s)")
    ap.add_argument("--gcc", type=int, default=100)
    ap.add_argument("--direction", choices=DIRECTIONS, default="right",
                    help="which way the rainbow travels (default right); up/down move by rows, "
                         "bl-tr/br-tl/tl-br/tr-bl sweep corner to opposite corner")
    ap.add_argument("--dx", type=int, help="override phase step per column (1 = right, 2 = left, 0 = none)")
    ap.add_argument("--dy", type=int, help="override phase step per row (1 = up, 2 = down, 0 = none)")
    ap.add_argument("--peak", default="ff30a0",
                    help="RRGGBB peak brightness per channel (default ff30a0: the green LEDs are much "
                         "brighter than red, so G is held back to make yellow/orange/violet readable)")
    ap.add_argument("--shape", choices=SHAPES, default="pulse",
                    help="breath profile of the slots (default pulse); see SHAPES in the source")
    ap.add_argument("--colors", help="colour wave instead of a rainbow: names or RRGGBB, comma separated (red green blue yellow "
                                     "cyan magenta white; other values snap to the nearest), tiled over the LEDs; "
                                     "'rainbow' = the rainbow (with --pwm: a continuous scrolling rainbow, --width LEDs "
                                     "per cycle; 'rainbow-bump' paints the spectrum across each bump instead) "
                                     "(X first), each LED fading to black; shorter/longer than the display "
                                     "is fine (repeats/truncates)")
    ap.add_argument("--pwm", action="store_true",
                    help="smooth wave generated by the firmware in PWM instead of ABM: any RGB colour, any width "
                         "(--colors, --direction, --speed, --width, --plateau, --gap, --sharp, --gcc apply; the ABM-only "
                         "options --shape/--peak/--dx/--dy are ignored)")
    ap.add_argument("--width", type=float,
                    help="--pwm: LEDs in one whole bump, rise + plateau + fall (default 8)")
    ap.add_argument("--plateau", type=float,
                    help="--pwm: LEDs held at full colour inside the bump (default 0; capped at --width)")
    ap.add_argument("--gap", type=float,
                    help="--pwm: dark LEDs between one bump and the next (default 0 = back to back)")
    ap.add_argument("--sharp", type=int, help="--pwm: 1-8, higher = narrower bright core (default 1)")
    ap.add_argument("--port", default="COM4", help="Windows COM port (default COM4)")
    ap.add_argument("--send", action="store_true", help="send to the board instead of printing")
    ap.add_argument("--stop", action="store_true", help="just reset the board")
    a = ap.parse_args()
    ignored = [f"--{n}" for n in ("width", "plateau", "gap", "sharp") if getattr(a, n) is not None]
    if ignored and not a.pwm:
        print(f"warning: {', '.join(ignored)} only apply with --pwm; ignored in the ABM modes", file=sys.stderr)
    # 'rainbow' is a reserved colour name: PWM = spectrum across each bump, ABM = the normal rainbow.
    words = [t.lower() for t in a.colors.replace(",", " ").split()] if a.colors else []
    rainbow = "rainbow-bump" if "rainbow-bump" in words else "rainbow" if "rainbow" in words else None
    if rainbow and not a.pwm:
        a.colors = None
    a.width = 8 if a.width is None else a.width
    a.plateau = 0 if a.plateau is None else a.plateau
    a.gap = 0 if a.gap is None else a.gap
    a.sharp = 1 if a.sharp is None else a.sharp

    dx, dy = DIRECTIONS[a.direction]
    dx = dx if a.dx is None else a.dx
    dy = dy if a.dy is None else a.dy
    if a.pwm and not a.stop:
        cmds = [f"panel {a.panels}"] + build_pwm(a.gcc, a.direction,
                                                 [rainbow] if rainbow else
                                                 parse_colors(a.colors, snap=False) if a.colors else [],
                                                 a.width, a.plateau, a.gap,
                                                 0 if a.speed is None else a.speed, a.sharp)
        if a.send:
            sys.exit(send(cmds, a.port))
        print("\n".join(cmds))
        return
    cmds = ["reset"] if a.stop else build(a.panels, 3 if a.speed is None else a.speed, a.gcc, dx, dy, a.peak,
                                         parse_colors(a.colors) if a.colors is not None else None, a.shape)
    if a.send:
        sys.exit(send(cmds, a.port))
    print("\n".join(cmds))


if __name__ == "__main__":
    main()
