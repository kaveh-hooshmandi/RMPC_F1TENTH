# Simplified Yas-Marina-inspired circuit — 12 November 2026

Kaveh Hooshmandi's new circuit is an original, simplified design, not a reproduction of official Yas Marina geometry. A script generated the map and centerline. The course length is about 322.20 m, width 2.9 m, tightest curve radius about 3.98 m, and map resolution 0.06 m/pixel. The controller uses the IMS settings with the new centerline and a 7 m/s speed cap; sampling and cost weights are unchanged.

The archived asynchronous ROS run is `telemetry/run_20261112_115749/`. Offline analysis used `./run.sh analyze run_20261112_115749 f1`:

| Measure | Recorded value |
|---|---:|
| Bag duration | 161.36 s |
| Detected laps | 2 |
| First lap | 63.696 s |
| Second lap | 59.412 s |
| Mean lap time | 61.554 s |
| P99 `/drive` interval | 6.82 ms |
| Maximum `/drive` interval | 17.93 ms |

The 10,000 m obstacle distance in the no-obstacle JSON is an evaluator placeholder. This single run does not provide a statistical comparison or physical collision validation. The timing values are message intervals, not GPU computation durations. The general analyzer's plot `results/run_20261112_115749_bag_slalom_benchmark.png` uses ±4 m guide lines; those are not the circuit boundaries, whose half-width is 1.45 m.

The paper figure `results/run_20261112_115749_f1_track_response.png` was generated with `./run.sh plot-f1 run_20261112_115749` from 40,312 odometry messages. The maximum absolute Frenet lateral offset in the bag was 0.662 m. Its gray boundary curves are geometric edges at ±1.45 m from the centerline; vehicle width is not subtracted from those limits. Source files include `src/rmpc_f1tenth/maps/F1_STYLE/track_info.json` and `results/run_20261112_115749_benchmark_metrics.json`.
