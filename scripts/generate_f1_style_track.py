#!/usr/bin/env python3
"""Build an original, simplified Yas-Marina-inspired F1TENTH circuit.

The map and CSV are generated from exactly the same closed centerline. No
external track geometry or image is copied.
"""
from pathlib import Path
import csv
import math
import json

import cv2
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "src/rmpc_f1tenth/maps/F1_STYLE"
OUT.mkdir(parents=True, exist_ok=True)

# Large separation between track sectors keeps the layout readable and avoids
# self-intersections at F1TENTH scale. First point is on the main straight.
CONTROL = np.array([
    [20, 0], [40, 0], [60, 0], [70, 4], [75, 13], [73, 22],
    [66, 27], [57, 27], [54, 31], [57, 37], [68, 39], [74, 46],
    [68, 54], [55, 57], [35, 56], [20, 57], [8, 55], [-1, 49],
    [-3, 42], [5, 37], [17, 36], [21, 30], [16, 24], [4, 23],
    [-8, 20], [-12, 13], [-10, 6], [-3, 1], [0, 0],
], dtype=float)

RESOLUTION = 0.06
HALF_WIDTH = 1.45
SPACING = 0.30
MARGIN = 8.0

# Periodic cubic interpolation parameterized by polygonal arc length.
closed = np.vstack([CONTROL, CONTROL[0]])
segments = np.linalg.norm(np.diff(closed, axis=0), axis=1)
knots = np.r_[0, np.cumsum(segments)]
period = float(knots[-1])
sx = CubicSpline(knots, closed[:, 0], bc_type="periodic")
sy = CubicSpline(knots, closed[:, 1], bc_type="periodic")
dense_u = np.linspace(0, period, 12001)
dense = np.column_stack([sx(dense_u), sy(dense_u)])
dense_s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))]
length = float(dense_s[-1])
count = round(length / SPACING)
s = np.linspace(0, length, count, endpoint=False)
u = np.interp(s, dense_s, dense_u)
xy = np.column_stack([sx(u), sy(u)])
dx, dy = sx(u, 1), sy(u, 1)
heading = np.unwrap(np.arctan2(dy, dx))
curvature = (dx * sy(u, 2) - dy * sx(u, 2)) / (dx * dx + dy * dy) ** 1.5

# Reject accidental overlaps between distant parts of the closed loop.
pairs = cKDTree(xy).query_pairs(3.5)
for a, b in pairs:
    separation = min(abs(s[a] - s[b]), length - abs(s[a] - s[b]))
    if separation > 6:
        raise RuntimeError(f"Track nearly intersects: points {a}, {b}")

origin = np.min(xy, axis=0) - MARGIN
maximum = np.max(xy, axis=0) + MARGIN
width, height = np.ceil((maximum - origin) / RESOLUTION).astype(int) + 1

def pixels(points):
    px = np.rint((points[:, 0] - origin[0]) / RESOLUTION).astype(np.int32)
    py = height - 1 - np.rint((points[:, 1] - origin[1]) / RESOLUTION).astype(np.int32)
    return np.column_stack([px, py]).reshape((-1, 1, 2))

# Black is occupied; the white ribbon is the drivable road.
image = np.zeros((height, width), dtype=np.uint8)
cv2.polylines(image, [pixels(xy)], isClosed=True, color=255,
              thickness=round(2 * HALF_WIDTH / RESOLUTION), lineType=cv2.LINE_8)
cv2.imwrite(str(OUT / "F1_STYLE_map.png"), image)
(OUT / "F1_STYLE_map.yaml").write_text(
    f"image: F1_STYLE_map.png\nresolution: {RESOLUTION}\n"
    f"origin: [{origin[0]:.6f}, {origin[1]:.6f}, 0.0]\n"
    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
)
for name in ("F1_STYLE_centerline.csv", "F1_STYLE_map_centerline.csv"):
    # The Gym bridge looks for the map YAML stem plus "_centerline.csv";
    # RMPC uses the shorter name in its own parameter file.
    with (OUT / name).open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["# x_m", "y_m", "w_tr_right_m", "w_tr_left_m"])
        for x, y in xy:
            writer.writerow([f"{x:.6f}", f"{y:.6f}", HALF_WIDTH, HALF_WIDTH])

fig, ax = plt.subplots(figsize=(9, 6), dpi=160)
ax.plot(xy[:, 0], xy[:, 1], color="#2d4059", linewidth=7, solid_capstyle="round")
ax.plot(xy[:, 0], xy[:, 1], color="#f4f4f1", linewidth=5, solid_capstyle="round")
ax.plot(xy[:, 0], xy[:, 1], color="#d9b44a", linewidth=.8, linestyle="--")
ax.plot(xy[0, 0], xy[0, 1], marker="s", markersize=8, color="#b03436")
ax.annotate("Start / main straight", tuple(xy[0]), (8, -11), arrowprops={"arrowstyle": "->"})
ax.annotate("Hairpin", (75, 13), (83, 20), arrowprops={"arrowstyle": "->"})
ax.annotate("Gentle chicane", (57, 32), (79, 32), arrowprops={"arrowstyle": "->"})
ax.set_aspect("equal")
ax.set_xlabel("Map x [m]")
ax.set_ylabel("Map y [m]")
ax.set_title("Simplified Yas-Marina-inspired circuit (original layout)")
ax.grid(alpha=.25)
fig.tight_layout()
fig.savefig(OUT / "F1_STYLE_preview.png")
plt.close(fig)

metadata = {
    "design": "Original simplified Yas-Marina-inspired circuit; not official circuit geometry",
    "length_m": length,
    "points": int(count),
    "max_abs_curvature_inv_m": float(np.max(np.abs(curvature))),
    "approx_min_radius_m": float(1 / np.max(np.abs(curvature))),
    "track_width_m": 2 * HALF_WIDTH,
    "resolution_m_per_pixel": RESOLUTION,
    "start_pose": [float(xy[0, 0]), float(xy[0, 1]), float(math.atan2(dy[0], dx[0]))],
    "map_size_px": [int(width), int(height)],
}
(OUT / "track_info.json").write_text(json.dumps(metadata, indent=2) + "\n")
print(json.dumps(metadata, indent=2))
