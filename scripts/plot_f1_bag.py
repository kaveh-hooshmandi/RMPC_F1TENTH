#!/usr/bin/env python3
"""Plot an actual F1_STYLE ROS bag against the generated centerline.

Run inside the project container; accepts the bag directory name as argv[1].
"""
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree

if len(sys.argv) != 2 or not sys.argv[1].startswith("run_") or "/" in sys.argv[1]:
    raise SystemExit("Usage: plot_f1_bag.py run_YYYYMMDD_HHMMSS")
bag_name = sys.argv[1]
bag_path = Path("/data/telemetry") / bag_name
centerline = Path("/rmpc_ws/install/rmpc_f1tenth/share/rmpc_f1tenth/maps/F1_STYLE/F1_STYLE_centerline.csv")
out = Path("/rmpc_ws/src/rmpc_f1tenth/results") / f"{bag_name}_f1_track_response.png"
if not bag_path.is_dir():
    raise SystemExit(f"Bag not found: {bag_path}")

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=""),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                        output_serialization_format="cdr"))
samples = []
while reader.has_next():
    topic, data, stamp = reader.read_next()
    if topic == "/ego_racecar/odom":
        msg = deserialize_message(data, Odometry)
        samples.append((stamp * 1e-9, msg.pose.pose.position.x,
                        msg.pose.pose.position.y))
if not samples:
    raise SystemExit("Bag contains no odometry")
samples = np.array(samples)
t = samples[:, 0] - samples[0, 0]
xy = samples[:, 1:3]
track = np.loadtxt(centerline, delimiter=",", comments="#")
cxy = track[:, :2]
idx = cKDTree(cxy).query(xy)[1]
delta = np.roll(cxy, -1, axis=0) - cxy
unit = delta / np.linalg.norm(delta, axis=1)[:, None]
normal = np.column_stack((-unit[:, 1], unit[:, 0]))
d = np.einsum("ij,ij->i", xy - cxy[idx], normal[idx])
half_width = float(track[0, 2])
left_edge = cxy + half_width * normal
right_edge = cxy - half_width * normal

fig, (ax, ay) = plt.subplots(1, 2, figsize=(11, 4.2), dpi=180,
                             gridspec_kw={"width_ratios": [1.05, 1.25]})
ax.plot(left_edge[:, 0], left_edge[:, 1], color="0.7", linewidth=1,
        label="map edges")
ax.plot(right_edge[:, 0], right_edge[:, 1], color="0.7", linewidth=1)
ax.plot(cxy[:, 0], cxy[:, 1], color="0.45", linewidth=0.8,
        linestyle="--", label="centerline")
ax.plot(xy[:, 0], xy[:, 1], color="#13679a", linewidth=1.1,
        label="recorded vehicle path")
ax.scatter([xy[0, 0]], [xy[0, 1]], s=35, color="#c13b31", marker="s",
           zorder=5, label="start")
ax.set_aspect("equal", adjustable="box")
ax.set_xlabel("Map x (m)")
ax.set_ylabel("Map y (m)")
ax.set_title("Simplified Yas Marina inspired circuit")
ax.grid(alpha=0.2)
ax.legend(fontsize=7, loc="center", bbox_to_anchor=(0.48, 0.56))

ay.plot(t, d, color="#13679a", linewidth=1.2, label="lateral offset $d(t)$")
ay.axhline(half_width, color="#c13b31", linestyle="--", linewidth=1,
           label="centerline to map edge")
ay.axhline(-half_width, color="#c13b31", linestyle="--", linewidth=1)
ay.axhline(0, color="0.6", linewidth=0.6)
ay.set_xlabel("Recorded time (s)")
ay.set_ylabel("Track relative lateral offset (m)")
ay.set_title("Recorded lateral response")
ay.set_xlim(0, float(t[-1]))
ay.set_ylim(-1.7, 1.7)
ay.grid(alpha=0.2)
ay.legend(fontsize=7, loc="upper right")
fig.tight_layout()
fig.savefig(out, bbox_inches="tight")
print(f"Saved {out}; {len(samples)} odometry samples; max |d| = {np.max(np.abs(d)):.3f} m")
