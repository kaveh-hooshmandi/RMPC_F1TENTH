"""Create the controller diagram and original simplified F1-style circuit."""
from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "figures"
OUT.mkdir(exist_ok=True)

plt.rcParams.update({"font.size": 9, "font.family": "DejaVu Sans", "pdf.fonttype": 42})

fig, ax = plt.subplots(figsize=(8.4, 2.25))
ax.set_xlim(0, 12)
ax.set_ylim(0, 3)
ax.axis("off")

def box(x, y, w, h, label, face):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.06",
                                linewidth=0.8, edgecolor="#263746", facecolor=face))
    ax.text(x+w/2, y+h/2, label, ha="center", va="center", fontsize=8.2)

def arrow(a, b):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=10,
                                 linewidth=1, color="#263746"))

box(0.1, 1.1, 1.5, .8, "F1TENTH\nsimulator", "#eaf2f8")
box(2.1, 1.75, 1.65, .7, "Odometry to\nFrenet (s, d)", "#eef4e8")
box(2.1, .55, 1.65, .7, "LiDAR to local\noccupancy grid", "#eef4e8")
box(4.45, 1.75, 1.6, .7, "IMS centerline\nreference", "#eef4e8")
box(4.45, .55, 1.6, .7, "Curvature and\nobstacle speed", "#eef4e8")
box(6.8, 1.1, 2.0, .8, "GPU randomized MPC\n5,000 rollouts", "#fff1db")
box(9.6, 1.1, 2.0, .8, "Steering and\nspeed command", "#f9e9e8")
arrow((1.6,1.55),(2.08,2.08)); arrow((1.6,1.42),(2.08,.92))
arrow((3.75,2.1),(4.43,2.1)); arrow((3.75,.9),(4.43,.9))
arrow((6.05,2.1),(6.78,1.63)); arrow((6.05,.9),(6.78,1.37))
arrow((8.8,1.5),(9.58,1.5))
ax.annotate("receding-horizon feedback", xy=(.5,.3), xytext=(10.5,.3),
            arrowprops=dict(arrowstyle="-|>", color="#607485", linewidth=.8),
            ha="center", fontsize=7.5, color="#526779")
fig.tight_layout(pad=.1)
fig.savefig(OUT / "architecture.pdf", bbox_inches="tight")
plt.close(fig)

points = []
track_file = ROOT.parent / "src/rmpc_f1tenth/maps/F1_STYLE/F1_STYLE_centerline.csv"
with open(track_file, newline="") as f:
    for row in csv.reader(f):
        if row and not row[0].lstrip().startswith("#"):
            points.append((float(row[0]), float(row[1]), float(row[2]), float(row[3])))
points = np.asarray(points)
xy = points[:, :2]
direction = np.roll(xy, -1, axis=0) - xy
direction /= np.linalg.norm(direction, axis=1)[:, None]
normal = np.column_stack((-direction[:, 1], direction[:, 0]))
left = xy + points[:, 3, None] * normal
right = xy - points[:, 2, None] * normal
fig, ax = plt.subplots(figsize=(4.4, 3.25))
ax.plot(left[:, 0], left[:, 1], color="#627583", lw=.9, label="track edges")
ax.plot(right[:, 0], right[:, 1], color="#627583", lw=.9)
ax.plot(xy[:, 0], xy[:, 1], color="#1e5b7a", lw=1.2, ls="--", label="centerline")
ax.scatter(xy[0, 0], xy[0, 1], s=28, marker="s", color="#c45d37", label="start")
ax.set_aspect("equal", adjustable="datalim")
ax.set_xlabel("map x (m)")
ax.set_ylabel("map y (m)")
ax.grid(alpha=.25)
ax.legend(fontsize=7, frameon=False, loc="best")
fig.tight_layout()
fig.savefig(OUT / "f1_track.pdf", bbox_inches="tight")
plt.close(fig)
