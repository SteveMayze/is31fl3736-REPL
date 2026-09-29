"""Grab frames from a webcam (run with the Windows venv: .venv-win/Scripts/python.exe).

    capture.py --index 0 --out shot.jpg
    capture.py --index 0 --out burst.jpg --frames 6 --interval 0.5   # burst_0.jpg ...
    capture.py --index 0 --out shot.jpg --exposure -6                # manual exposure
"""
import argparse
import time

import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--out", default="shot.jpg")
    ap.add_argument("--frames", type=int, default=1)
    ap.add_argument("--interval", type=float, default=0.5)
    ap.add_argument("--exposure", type=float, help="manual exposure (DirectShow log2 s, e.g. -6); omit for auto")
    ap.add_argument("--gain", type=float, help="manual gain (0-255)")
    ap.add_argument("--wb", type=float, help="fixed white balance in kelvin (disables auto WB)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    a = ap.parse_args()

    cap = cv2.VideoCapture(a.index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise SystemExit(f"cannot open camera {a.index}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    if a.exposure is not None:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # DirectShow: manual mode
        cap.set(cv2.CAP_PROP_EXPOSURE, a.exposure)
    if a.gain is not None:
        cap.set(cv2.CAP_PROP_GAIN, a.gain)
    if a.wb is not None:
        cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        cap.set(cv2.CAP_PROP_WB_TEMPERATURE, a.wb)
    for _ in range(15):  # let auto-exposure / white balance settle
        cap.read()
    stem, dot, ext = a.out.rpartition(".")
    for i in range(a.frames):
        ok, frame = cap.read()
        if not ok:
            raise SystemExit("read failed")
        path = a.out if a.frames == 1 else f"{stem}_{i}.{ext}"
        cv2.imwrite(path, frame)
        print(path, frame.shape)
        if i + 1 < a.frames:
            time.sleep(a.interval)
    cap.release()


if __name__ == "__main__":
    main()
