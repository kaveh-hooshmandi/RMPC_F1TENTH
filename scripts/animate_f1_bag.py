#!/usr/bin/env python3
"""Create a README GIF from recorded F1_STYLE odometry.

Run inside the project container:
python3 /project/scripts/animate_f1_bag.py run_20261112_115749
The results directory is mounted on the host.
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Polygon
import numpy as np
from PIL import Image
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="Directory name under /data/telemetry")
    args = parser.parse_args()
    if not args.bag.startswith("run_") or "/" in args.bag:
        parser.error("Supply one run_* directory name")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=f"/data/telemetry/{args.bag}", storage_id=""),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    samples = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic != "/ego_racecar/odom":
            continue
        msg = deserialize_message(data, Odometry)
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        samples.append((stamp * 1e-9, p.x, p.y, yaw, np.hypot(v.x, v.y)))
    if len(samples) < 2:
        raise SystemExit("At least two odometry samples are required")
    data = np.asarray(samples)
    data = data[np.argsort(data[:, 0], kind="stable")]
    data = data[np.r_[True, np.diff(data[:, 0]) > 0]]
    t = data[:, 0] - data[0, 0]
    if t[-1] <= 0:
        raise SystemExit("Recording must have a positive duration")
    xy = data[:, 1:3]
    yaw = np.unwrap(data[:, 3])
    track = np.loadtxt("/rmpc_ws/install/rmpc_f1tenth/share/rmpc_f1tenth/maps/F1_STYLE/F1_STYLE_centerline.csv",
                       delimiter=",", comments="#")
    cxy = track[:, :2]
    tangent = np.roll(cxy, -1, axis=0) - cxy
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    normal /= np.linalg.norm(normal, axis=1)[:, None]
    left = cxy + normal * track[:, 3, None]
    right = cxy - normal * track[:, 2, None]
    road = np.stack((left, np.roll(left, -1, axis=0),
                     np.roll(right, -1, axis=0), right), axis=1)
    bg, white, cyan, gold = "#101b2b", "#edf4ff", "#39d9ed", "#ffcb66"
    fig = plt.figure(figsize=(9.6, 6.0), dpi=100, facecolor=bg)
    ax = fig.add_axes((0.04, 0.13, 0.92, 0.70), facecolor=bg)
    ax.add_collection(PolyCollection(road, facecolors="#2a3a50", edgecolors="none"))
    for edge in (left, right):
        closed = np.vstack((edge, edge[0]))
        ax.plot(*closed.T, color="#63758e", lw=1)
    closed = np.vstack((cxy, cxy[0]))
    ax.plot(*closed.T, color="#8293aa", lw=0.8, ls=(0, (4, 6)), alpha=0.65)
    path, = ax.plot([], [], color="#218da6", lw=1.7, alpha=0.8)
    trail, = ax.plot([], [], color=cyan, lw=2.4)
    ax.scatter(*xy[0], color=gold, s=35, marker="s", zorder=5)
    # Direction marker enlarged for readability; it is not the vehicle footprint.
    car = Polygon([[0, 0], [0, 0], [0, 0]], facecolor=white, edgecolor=cyan, lw=1.3, zorder=7)
    ax.add_patch(car)
    extent = np.vstack((left, right, xy))
    ax.set_xlim(extent[:, 0].min() - 5, extent[:, 0].max() + 5)
    ax.set_ylim(extent[:, 1].min() - 5, extent[:, 1].max() + 5)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.text(0.055, 0.93, "RMPC / F1TENTH", color=white, size=22, weight="bold")
    fig.text(0.055, 0.881, "Recorded vehicle trajectory  /  F1-style circuit", color="#a7b8cf", size=11)
    hud = fig.text(0.945, 0.92, "", color=cyan, size=12, ha="right", family="monospace")
    fig.text(0.055, 0.065, "CYAN  Vehicle path     /     GOLD  Recording start", color="#a7b8cf", size=10)
    fig.text(0.945, 0.065, "8x playback  /  marker enlarged", color="#a7b8cf", size=10, ha="right")
    fig.text(0.055, 0.028, f"Source: {args.bag}   |   Original circuit inspired by Yas Marina", color="#758aa8", size=8)
    times = np.r_[np.arange(0, t[-1], 8 * 0.08), t[-1]]
    frames = []
    palette = None
    for elapsed in times:
        pos = np.array([np.interp(elapsed, t, xy[:, j]) for j in range(2)])
        angle = np.interp(elapsed, t, yaw)
        end = np.searchsorted(t, elapsed, side="right")
        start = np.searchsorted(t, max(0, elapsed - 5))
        history = np.vstack((xy[:end:8], pos))
        recent = np.vstack((xy[start:end:4], pos))
        path.set_data(*history.T)
        trail.set_data(*recent.T)
        shape = np.array([[1.5, 0], [-1, 0.8], [-0.65, 0], [-1, -0.8]])
        rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        car.set_xy(shape @ rotation.T + pos)
        speed = np.interp(elapsed, t, data[:, 4])
        hud.set_text(f"TIME {elapsed:6.1f} s\nSPEED {speed:5.2f} m/s")
        fig.canvas.draw()
        frame = Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy())
        if palette is None:
            # One shared palette avoids changing background colors between frames.
            palette = frame.quantize(colors=256)
        frames.append(frame.quantize(palette=palette, dither=Image.Dither.NONE))
    out = Path("/rmpc_ws/src/rmpc_f1tenth/results/rmpc_trajectory.gif")
    frames[0].save(out, save_all=True, append_images=frames[1:], duration=80,
                   loop=0, optimize=True, disposal=1)
    plt.close(fig)
    print(f"Saved {out}: {len(frames)} frames, {t[-1]:.2f} s recorded, {out.stat().st_size / 1e6:.2f} MB")


if __name__ == "__main__":
    main()
