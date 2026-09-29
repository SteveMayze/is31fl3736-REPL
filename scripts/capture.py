"""Grab frames from a webcam (run with the Windows venv: .venv-win/Scripts/python.exe).

    capture.py --index 0 --out shot.jpg
    capture.py --index 0 --out burst.jpg --frames 6 --interval 0.5   # burst_0.jpg ...
    capture.py --index 0 --out shot.jpg --exposure -6                # manual exposure
    capture.py --name C920 --out shot.jpg                            # pick camera by name
    capture.py --list                                                # show camera indices

Camera indices change across reboots/replugs; --name is stable. Needs pygrabber
(pip install pygrabber) to enumerate DirectShow device names.
"""
import argparse
import time

import cv2


def device_names():
    from pygrabber.dshow_graph import FilterGraph  # same order as OpenCV's CAP_DSHOW indices

    return FilterGraph().get_input_devices()


def find_index(name):
    names = device_names()
    hits = [i for i, n in enumerate(names) if name.lower() in n.lower()]
    if len(hits) != 1:
        found = ", ".join(f"{i}: {n}" for i, n in enumerate(names))
        raise SystemExit(f"--name {name!r} matched {len(hits)} cameras; available: {found}")
    return hits[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--name", help="camera name substring (e.g. C920); overrides --index")
    ap.add_argument("--list", action="store_true", help="list cameras and exit")
    ap.add_argument("--out", default="shot.jpg")
    ap.add_argument("--frames", type=int, default=1)
    ap.add_argument("--interval", type=float, default=0.5)
    ap.add_argument("--exposure", type=float, help="manual exposure (DirectShow log2 s, e.g. -6); omit for auto")
    ap.add_argument("--gain", type=float, help="manual gain (0-255)")
    ap.add_argument("--wb", type=float, help="fixed white balance in kelvin (disables auto WB)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    a = ap.parse_args()
    if a.list:
        for i, n in enumerate(device_names()):
            print(f"{i}: {n}")
        return
    if a.name:
        a.index = find_index(a.name)

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
