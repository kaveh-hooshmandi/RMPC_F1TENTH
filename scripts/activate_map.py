"""Activate the Nav2 map server after the simulator launch has created it."""
import subprocess
import time


def ros(*args):
    return subprocess.run(['ros2', 'lifecycle', *args], capture_output=True,
                          text=True, timeout=8)


for attempt in range(12):
    try:
        state = ros('get', '/map_server')
        if state.returncode == 0:
            if state.stdout.startswith('unconfigured'):
                result = ros('set', '/map_server', 'configure')
                if result.returncode != 0:
                    raise RuntimeError(result.stderr or result.stdout)
                state = ros('get', '/map_server')
            if state.stdout.startswith('inactive'):
                result = ros('set', '/map_server', 'activate')
                if result.returncode != 0:
                    raise RuntimeError(result.stderr or result.stdout)
            print('Map server active.', flush=True)
            break
    except (subprocess.TimeoutExpired, RuntimeError) as exc:
        print(f'Map activation attempt {attempt + 1}: {exc}', flush=True)
    time.sleep(1)
else:
    raise SystemExit('Map server could not be activated; the simulator may have exited.')
