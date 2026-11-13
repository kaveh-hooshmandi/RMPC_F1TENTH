# Controlled comparison and sensitivity study

This folder contains a synchronous experiment runner around the actual C++/CUDA
controller and the installed F1TENTH Gym. It adds Pure Pursuit, direct solve-call
timing, repeated seeded trials and an unannounced low-grip sector.

The controller source is extended locally. The original ROS executable remains
installed in the image; the experiments run the separately rebuilt executable
at `/rmpc_ws/build/rmpc_f1tenth/rmpc_node`. No dependencies are installed by
`benchmark-build`. After recreating the container, rebuild this executable.

## Reproduce

Stop any live simulator, controller, recorder and RViz before benchmarking to
avoid competing CPU/GPU work. From `RMPC_F1TENTH`:

```bash
./run.sh down
./run.sh up
./run.sh benchmark-build
./run.sh benchmark nominal my_nominal_repeat
./run.sh benchmark slippery my_slippery_repeat
```

Use new output names for a new study. Completed trials can be resumed only if
the manifest still matches the script and binary. A partially written trial
requires a new output directory or explicit recovery; it is never silently
dropped. The paper dataset is `study_v1/` (80 trials) and `slippery_v1/` (20).
The fixed report script audits these two datasets and regenerates the figures:

```bash
./run.sh report
```

This report step uses NumPy/Matplotlib in the project container. It reads
all scheduled results and fails if a result is missing or trace checks fail.
`report/all_trials.csv`, `report/summary.json`, PDF/PNG figures, and a LaTeX table
are generated from the archived traces. The manuscript is in `../paper/` within this repository.

## Protocol fixed before main trials

- Nominal: two maps (IMS and the new circuit), Pure Pursuit plus RMPC with 1000,
  2500 or 5000 samples, ten independent trials per condition: 80 runs total.
- Stress: new circuit, Pure Pursuit and RMPC 5000, ten trials each. Simulator
  friction is 0.35 between 15% and 35% of centerline progress, and 1.0489 outside.
  The controller's assumed friction stays at 0.523. This is a synthetic abrupt
  coefficient change, without a wet-road sensor model or adaptive controller.
- Seeds 81000--81009 pair initial conditions and LiDAR noise between methods.
  Controller RNG uses the same trial seed. The entire controller process and
  simulator are restarted each trial; the ten runs are independent restarts,
  not ten consecutive laps. Run order is shuffled with seed 20261112.
- Start at centerline point 0 from rest. Add normal lateral error with SD 0.05 m
  clipped at +/-0.12 m and yaw error with SD 0.01 rad clipped at +/-0.03 rad.
  LiDAR: 819 beams, 270 degrees, 0.01 m noise SD, 0.275 m forward sensor offset.
  Ground-truth odometry is provided; localisation noise is not tested here.
- Apply one command every 20 ms of simulation time; integrate with RK4 at
  10 ms. Each command is held for two physics steps, checking collision after
  both. The simulator waits for computation. Wall-clock speed has no influence
  on the simulated trajectory, and this does not test ROS scheduling deadlines.
- Both methods share the original curvature/forward-distance speed policy,
  speed cap 7 m/s, steering limits and map/perception pipeline. RMPC retains
  its original 100-stage horizon, 20 ms prediction step, seven basis coefficients
  and weights. Pure Pursuit targets a point 1.5 m ahead in centerline arc length,
  interpolates that segment, and uses `atan2(2*L*y_goal, x_goal^2+y_goal^2)`.
  It enforces the same 0.4189 rad angle and 3.2 rad/s command slew limits.
- Nominal simulator vehicle: mass 3.74 kg, friction 1.0489, body width 0.31 m,
  length 0.58 m. Controller prediction retains mass 3.47 kg, friction 0.523 and
  width 0.2032 m. These inherited model mismatches are disclosed in the paper.
- One complete unwrapped centerline length is one trial; collision or 180 s
  simulation time terminates it early. Progress and signed offset use nearest
  segment projection, including the closing segment. Finish time is quantized
  to the 20 ms control step. Collision comes from Gym's collision model, not
  from the geometric obstacle-gap placeholder used in the old evaluator.
- A monotonic CPU timer encloses the synchronized RMPC `solve()` call: sampling,
  rollouts, transfers and host selection are included. It is not GPU-kernel-only
  time. A second timer encloses perception plus the controller callback and
  command publication, excluding stdin/stdout, simulation and visualization.
  PP decision timing covers its geometric steering calculation; the full
  pipeline timer is also retained. First-second timing samples are excluded
  from steady summaries; all raw timings and all-state tracking errors remain.

## Validation and interpretation

Pilot seed 70001 was excluded from the study. Both methods completed both maps
without tuning after those pilots. Repeating the new-circuit RMPC pilot gave
identical recorded states and commands; timings are expected to vary.
The report verifies schedule completeness, monotonic time, finite states,
limits, completion distance, collision flags, and RMS consistency.

`trace.npz` contains column names and all sampled states, commands, progress,
friction, collision flags and timings. Each trial has a saved controller YAML,
log and result JSON. Manifests include binary, script, map/centerline hashes
and simulator vehicle parameters. Raw synchronous traces are NPZ files; these
controlled experiments do not create ROS bags. Earlier asynchronous ROS runs
and their bags remain separate and should not be pooled with these results.

Statistics use trial means and sample SD, not individual control steps as
independent observations. Paired bootstrap intervals resample the ten seed
pairs (10000 resamples). They characterize this narrow perturbation protocol,
not all tracks or operating conditions. A good Pure Pursuit result is retained
even if it outperforms RMPC; no algorithmic superiority or new control method
is assumed in advance.
