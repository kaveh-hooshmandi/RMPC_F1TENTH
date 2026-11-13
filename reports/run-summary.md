# Local IMS simulation runs — 12 November 2026

Kaveh Hooshmandi ran the two archived simulations on an RTX 3050 using the `jamris-rmpc-f1tenth:local` image. The controller source revision is in `source_versions.txt`. Both used the IMS map and `config/controller.yaml`. Each is one asynchronous ROS run, not a repeated statistical comparison.

| Scenario | ROS bag | Detected laps | Lap time | Bag duration | Estimated minimum obstacle gap | P99 `/drive` interval |
|---|---|---:|---:|---:|---:|---:|
| Clear | `telemetry/run_20261112_113407/` | 1 | 40.628 s | 53.49 s | Not applicable | 9.51 ms |
| Obstacles | `telemetry/run_20261112_113614/` | 1 | 41.259 s | 55.08 s | 0.063 m | 10.40 ms |

The corresponding metric files are `results/run_20261112_113407_benchmark_metrics.json` and `results/run_20261112_113614_benchmark_metrics.json`. Lateral-offset, obstacle-gap, and command plots are under `results/`. The 10,000 m obstacle distance in the clear-run JSON is an evaluator placeholder, not a measured clearance. The obstacle gap is computed using a geometric model of three fixed points and circular radii; it is not a physical contact test.

The CUDA controller loaded 805 centerline points and published `/drive` commands. RViz launched. Its compatible configuration omits the local occupancy-grid visual layer due to a shader issue on this workstation; the `/local_map` topic remains active.
