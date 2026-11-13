"""Launch the simulator, RMPC controller, map activation, and optional RViz."""
from pathlib import Path
from datetime import datetime, timezone

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CONFIG = Path('/project/config')
TRACKS = {
    'f1': ('sim_f1_style.yaml', 'f1_style_controller.yaml'),
    'clear': ('sim.yaml', 'controller.yaml'),
    'obstacles': ('sim_obstacles.yaml', 'controller.yaml'),
}


def setup(context):
    track = LaunchConfiguration('track').perform(context)
    if track not in TRACKS:
        raise ValueError(f'track must be one of {", ".join(TRACKS)}')
    sim_yaml, controller_yaml = TRACKS[track]
    sim_launch = Path(get_package_share_directory('f1tenth_gym_ros')) / 'launch/gym_bridge_launch.py'
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim_launch)),
            launch_arguments={
                'config': str(CONFIG / sim_yaml),
                'open_foxglove': 'false',
            }.items(),
        ),
        Node(
            package='rmpc_f1tenth', executable='rmpc_node',
            parameters=[str(CONFIG / controller_yaml)], output='screen',
        ),
        # The existing image's lifecycle manager lacks a runtime library.
        # Activate the map server through its standard ROS lifecycle service.
        TimerAction(period=4.0, actions=[ExecuteProcess(
            cmd=['python3', '/project/scripts/activate_map.py'], output='screen',
        )]),
    ]
    if LaunchConfiguration('rviz').perform(context).lower() in ('true', '1', 'yes'):
        actions.append(Node(package='rviz2', executable='rviz2',
                            arguments=['-d', str(CONFIG / 'rviz-compatible.rviz')],
                            output='screen'))
    if LaunchConfiguration('record').perform(context).lower() in ('true', '1', 'yes'):
        bag = 'run_' + datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%SZ')
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', f'/data/telemetry/{bag}',
                 '/ego_racecar/odom', '/drive'], output='screen',
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('track', default_value='f1',
                              description='f1, clear, or obstacles'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz in the same terminal'),
        DeclareLaunchArgument('record', default_value='false',
                              description='Record odometry and drive in the same terminal'),
        OpaqueFunction(function=setup),
    ])
