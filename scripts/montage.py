"""Tile a crop of several capture frames into one contact sheet (Windows venv).
    montage.py out.jpg x0 y0 x1 y1 frame1.jpg frame2.jpg ..."""
import sys

import cv2
import numpy as np

out, x0, y0, x1, y1, *files = sys.argv[1:]
x0, y0, x1, y1 = map(int, (x0, y0, x1, y1))
tiles = []
for i, f in enumerate(files):
    t = cv2.imread(f)[y0:y1, x0:x1].copy()
    cv2.putText(t, str(i), (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    tiles.append(t)
cols = 4
while len(tiles) % cols:
    tiles.append(np.zeros_like(tiles[0]))
rows = [np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)]
cv2.imwrite(out, np.vstack(rows))
