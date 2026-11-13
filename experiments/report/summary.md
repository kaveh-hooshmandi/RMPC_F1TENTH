# Controlled F1TENTH study — 12 November 2026

The fixed protocol and reproduction commands are in `../README.md`. This
summary is derived from all 100 archived per-trial JSON files and traces by
`../summarize_study.py`. The archived report files include `all_trials.csv`,
`summary.json`, figures, and the LaTeX table used in the JAMRIS paper.

| Track and condition | Pure Pursuit: completion, lap, RMS offset | RMPC 5000: completion, lap, RMS offset |
|---|---|---|
| IMS, nominal | 10/10; 49.83 ± 0.03 s; 0.042 ± 0.001 m | 10/10; 53.51 ± 2.09 s; 0.184 ± 0.024 m |
| New circuit, nominal | 10/10; 58.92 ± 0.05 s; 0.056 ± 0.001 m | 10/10; 61.16 ± 2.01 s; 0.269 ± 0.018 m |
| New circuit, low-grip sector | 10/10; 59.19 s; 0.145 m | 10/10; 61.75 s; 0.334 m |

The RMPC 1000/2500/5000 mean RMS offsets were 0.218/0.214/0.184 m on IMS
and 0.312/0.278/0.269 m on the new circuit. Across the same settings, mean
per-trial median synchronized solve-call durations were 0.881/0.894/0.919 ms
on IMS and 0.886/0.900/0.919 ms on the new circuit. The maximum individual
solve call among all 100 trials was 5.72 ms. All nominal and low-grip trials
completed a lap without a Gym collision or center crossing beyond nominal
map half-width.

The paired-seed difference (RMPC 5000 minus Pure Pursuit) in RMS lateral
offset was 0.142 m on IMS (95% paired bootstrap interval 0.128–0.156 m)
and 0.213 m on the new circuit (0.201–0.223 m). Lap differences were
3.68 s (2.42–4.86 s) and 2.23 s (1.11–3.49 s). The intervals use 10,000
resamples of ten independent seed pairs. They describe the chosen initial
perturbation, speed and LiDAR-noise protocol; they do not measure performance
on unseen tracks, larger disturbances or different tuning.

The 20 low-grip runs reduce simulator friction from 1.0489 to 0.35 over
15–35% of new-circuit progress while controller parameters stay fixed.
Compared with nominal, mean RMS offset rose by 0.089 m for Pure Pursuit and
0.065 m for RMPC. These are descriptive differences under one synthetic
friction schedule. Compute timing excludes ROS transport and simulator work;
the 20 ms simulation period does not establish a deployed deadline guarantee.
