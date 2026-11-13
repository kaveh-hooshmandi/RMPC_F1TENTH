#!/usr/bin/env python3
"""Seeded, synchronous 50 Hz comparison using the actual CUDA controller.

No ROS scheduling or GUI is in the feedback loop. Each trial starts a new
controller process and simulator; raw states, commands and timings are saved.
"""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
import os
from pathlib import Path
import select
import subprocess
import time

import numpy as np
import yaml
from scipy.spatial import cKDTree
from f1tenth_gym.envs import F110Env
from f1tenth_gym.envs.env_config import (EnvConfig, ControlConfig, SimulationConfig,
    ObservationConfig, ResetConfig, LoopCounterMode)
from f1tenth_gym.envs.dynamic_models import DynamicModel, get_f1tenth_vehicle_parameters
from f1tenth_gym.envs.integrators import IntegratorType
from f1tenth_gym.envs.lidar import LiDARConfig
from f1tenth_gym.envs.observation import ObservationType
from f1tenth_gym.envs.reset import ResetStrategy
from f1tenth_gym_ros.gym_bridge import _load_track_from_yaml

MAP_ROOT = Path('/rmpc_ws/install/rmpc_f1tenth/share/rmpc_f1tenth/maps')
EXE = '/rmpc_ws/build/rmpc_f1tenth/rmpc_node'
DT = 0.02
BEAMS = 819
ANGLE_MIN = -3*np.pi/4
ANGLE_INC = (3*np.pi/2)/(BEAMS-1)

class Progress:
    def __init__(self, csv):
        a = np.loadtxt(csv, delimiter=',', comments='#')
        self.xy, self.widths = a[:, :2], a[:, 2:]
        self.dv = np.roll(self.xy, -1, axis=0)-self.xy
        self.ds = np.linalg.norm(self.dv, axis=1)
        self.s = np.r_[0, np.cumsum(self.ds[:-1])]
        self.length = float(sum(self.ds))
        self.tree = cKDTree(self.xy)
    def project(self, p):
        i = int(self.tree.query(p)[1])
        # Project to the nearer of the two adjacent segments, including seam.
        choices = []
        for j in ((i-1) % len(self.xy), i):
            u = float(np.clip(np.dot(p-self.xy[j], self.dv[j])/self.ds[j]**2, 0, 1))
            residual = p-self.xy[j]-u*self.dv[j]
            tangent = self.dv[j]/self.ds[j]
            d = float(np.dot(residual, [-tangent[1], tangent[0]]))
            choices.append((np.linalg.norm(residual), (self.s[j]+u*self.ds[j]) % self.length, d, j))
        _, s, d, j = min(choices)
        return float(s), d, j

def response(proc, timeout=30):
    if not select.select([proc.stdout], [], [], timeout)[0]:
        raise TimeoutError('Controller response timed out')
    line = proc.stdout.readline()
    if not line:
        raise RuntimeError(f'Controller exited: {proc.poll()}')
    return line.strip()

def trial(root, track_name, method, samples, seed, timeout, slippery=False):
    tag = f'{track_name}_{method}_{samples}_s{seed}'
    folder = root/tag
    folder.mkdir()
    map_name = 'IMS' if track_name == 'ims' else 'F1_STYLE'
    base = MAP_ROOT/map_name
    progress = Progress(base/f'{map_name}_centerline.csv')
    rng = np.random.default_rng(seed)
    lateral0 = float(np.clip(rng.normal(0, .05), -.12, .12))
    yaw0 = float(np.clip(rng.normal(0, .01), -.03, .03))
    tangent = progress.dv[0]/progress.ds[0]
    start_xy = progress.xy[0] + lateral0*np.array([-tangent[1], tangent[0]])
    pose = np.array([[*start_xy, np.arctan2(tangent[1], tangent[0])+yaw0]])
    p = yaml.safe_load(Path('/project/config/controller.yaml').read_text())
    p['rmpc_node']['ros__parameters'].update({
        'waypoints_file': f'maps/{map_name}/{map_name}_centerline.csv',
        'controller_kind': method, 'benchmark_stdio': True,
        'visualization_enabled': False, 'pp_lookahead': 1.5})
    p['rmpc_node']['ros__parameters']['solver']['samples'] = samples
    p['rmpc_node']['ros__parameters']['solver']['seed'] = seed
    p['rmpc_node']['ros__parameters']['speed_limits']['v_max'] = 7.0
    config = folder/'controller.yaml'
    config.write_text(yaml.safe_dump(p))
    track, _ = _load_track_from_yaml(base/f'{map_name}_map.yaml', 1.)
    lidar = LiDARConfig(enabled=True, num_beams=BEAMS, range_min=.05, range_max=25.,
        angle_min=ANGLE_MIN, angle_max=3*np.pi/4, noise_std=.01,
        base_link_to_lidar_tf=(.275, 0., 0.))
    vehicle = get_f1tenth_vehicle_parameters()
    active_mu = vehicle.mu
    cfg = EnvConfig(map_name=track, params=vehicle,
        seed=seed, num_agents=1, control_config=ControlConfig(),
        simulation_config=SimulationConfig(timestep=.01, integrator_timestep=.01,
            integrator=IntegratorType.RK4, dynamics_model=DynamicModel.ST,
            loop_counter=LoopCounterMode.TOGGLE, compute_frenet_frame=False, max_laps=None),
        observation_config=ObservationConfig(type=ObservationType.DIRECT),
        reset_config=ResetConfig(strategy=ResetStrategy.MAP_RANDOM_STATIC),
        lidar_config=lidar, render_enabled=False)
    sim = F110Env(config=cfg, render_mode=None)
    sim.reset(seed=seed, options={'poses': pose})
    stderr = (folder/'controller.log').open('w')
    proc = subprocess.Popen([EXE, '--ros-args', '--params-file', str(config),
        '--log-level', 'warn'], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=stderr, text=True, bufsize=1,
        env={**os.environ, 'RCUTILS_LOGGING_USE_STDOUT': '0', 'ROS_DOMAIN_ID': '68'})
    rows = []
    outcome = 'timeout'
    wall0 = time.monotonic()
    try:
        while response(proc) != 'READY':
            pass
        prev_s, _, _ = progress.project(start_xy)
        distance = 0.
        for step in range(int(timeout/DT)):
            state = sim.sim.state
            x, y, yaw = map(float, state.poses[0])
            # An unannounced low-grip sector: controller friction stays 0.523.
            s_now, _, _ = progress.project(np.array([x, y]))
            desired_mu = .35 if slippery and .15 <= s_now/progress.length < .35 else vehicle.mu
            if desired_mu != active_mu:
                sim.update_params(replace(vehicle, mu=desired_mu))
                active_mu = desired_mu
            speed = float(state.standard_state[0, 3])
            beta = float(state.standard_state[0, 6])
            vx, vy = speed*np.cos(beta), speed*np.sin(beta)
            wz = float(state.standard_state[0, 5])
            scan = np.nan_to_num(state.scans[0], nan=25., posinf=25., neginf=.05)
            values = [x, y, yaw, vx, vy, wz, BEAMS, ANGLE_MIN, ANGLE_INC, .05, 25., *scan]
            proc.stdin.write(' '.join(format(float(v), '.9g') for v in values)+'\n')
            proc.stdin.flush()
            steer, command_speed, decision_ms, pipeline_ms = map(float, response(proc).split())
            if not np.all(np.isfinite([steer, command_speed, decision_ms, pipeline_ms])):
                raise ValueError('Nonfinite controller output')
            collided = False
            for _ in range(2):
                sim.step(np.array([[steer, command_speed]], dtype=float))
                collided |= bool(sim.sim.state.collisions[0])
                if collided:
                    break
            t = float(sim.sim.state.sim_time)
            px, py = map(float, sim.sim.state.poses[0, :2])
            s, d, j = progress.project(np.array([px, py]))
            ds = (s-prev_s+progress.length/2) % progress.length-progress.length/2
            distance += ds
            prev_s = s
            width = float(progress.widths[j, 1 if d >= 0 else 0])
            rows.append([t, px, py, d, distance, speed, steer, command_speed,
                         decision_ms, pipeline_ms, int(collided), width, active_mu])
            if collided:
                outcome = 'collision'
                break
            if distance >= progress.length:
                outcome = 'completed'
                break
        arr = np.asarray(rows)
        # First second excluded from steady-state latency summaries only.
        timing = arr[arr[:, 0] >= 1.0]
        if not len(timing):
            timing = arr
        result = dict(track=track_name, method=method, samples=samples, seed=seed,
            scenario='slippery_sector' if slippery else 'nominal',
            outcome=outcome, duration_s=float(arr[-1, 0]),
            lap_s=float(arr[-1, 0]) if outcome == 'completed' else None,
            progress_m=float(arr[-1, 4]), track_length_m=progress.length,
            rms_d_m=float(np.sqrt(np.mean(arr[:, 3]**2))), max_abs_d_m=float(max(abs(arr[:, 3]))),
            center_outside_fraction=float(np.mean(abs(arr[:, 3]) > arr[:, 11])),
            decision_median_ms=float(np.median(timing[:, 8])),
            decision_p99_ms=float(np.percentile(timing[:, 8], 99)),
            pipeline_p99_ms=float(np.percentile(timing[:, 9], 99)),
            decision_over_20ms_fraction=float(np.mean(timing[:, 8] > 20)),
            decision_max_all_ms=float(max(arr[:, 8])),
            initial_lateral_m=lateral0, initial_yaw_error_rad=yaw0,
            control_dt_s=DT, physics_dt_s=.01, v_max_mps=7., lidar_noise_std_m=.01,
            steps=len(arr), wall_s=time.monotonic()-wall0,
            map_sha256=hashlib.sha256((base/f'{map_name}_map.png').read_bytes()).hexdigest(),
            centerline_sha256=hashlib.sha256((base/f'{map_name}_centerline.csv').read_bytes()).hexdigest())
        np.savez_compressed(folder/'trace.npz', data=arr,
            columns=np.array(['t','x','y','d','progress','speed','steer','command_speed',
                              'decision_ms','pipeline_ms','collision','half_width','mu']))
        (folder/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        return result
    finally:
        proc.stdin.close()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        stderr.close()
        sim.close()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--seeds', type=int, default=10)
    parser.add_argument('--seed-base', type=int, default=81000)
    parser.add_argument('--pilot', action='store_true')
    parser.add_argument('--slippery', action='store_true')
    parser.add_argument('--timeout', type=float, default=180.)
    args = parser.parse_args()
    root = Path(args.output)
    root.mkdir(parents=True, exist_ok=True)
    conditions = [('pp', 5000), ('rmpc', 1000), ('rmpc', 2500), ('rmpc', 5000)]
    if args.pilot or args.slippery:
        conditions = [('pp', 5000), ('rmpc', 5000)]
    tracks = ('f1',) if args.slippery else ('ims', 'f1')
    schedule = [(track, method, samples, args.seed_base+i) for i in range(args.seeds)
                for track in tracks for method, samples in conditions]
    np.random.default_rng(20261112).shuffle(schedule)
    manifest = dict(protocol='synchronous_fixed_50hz_v1', schedule=schedule,
        slippery=args.slippery,
        simulator_vehicle={k: v for k, v in asdict(get_f1tenth_vehicle_parameters()).items()
                           if isinstance(v, (float, int)) and np.isfinite(v)},
        controller_binary_sha256=hashlib.sha256(Path(EXE).read_bytes()).hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        note='Fresh process and simulator per trial; paired seeds; first lap from rest; no tuning on study seeds.')
    manifest_path = root/'manifest.json'
    if manifest_path.exists() and json.loads(manifest_path.read_text()) != json.loads(json.dumps(manifest)):
        raise RuntimeError('Manifest changed: choose a fresh output directory')
    manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
    for n, (track, method, samples, seed) in enumerate(schedule, 1):
        result_path = root/f'{track}_{method}_{samples}_s{seed}'/'result.json'
        if result_path.exists():
            print(f'SKIP {result_path}', flush=True)
            continue
        result = trial(root, track, method, samples, seed, args.timeout, args.slippery)
        print(json.dumps({'trial': n, 'of': len(schedule), **result}), flush=True)

if __name__ == '__main__':
    main()
