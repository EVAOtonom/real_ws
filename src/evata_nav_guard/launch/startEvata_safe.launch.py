import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.dirname(os.path.realpath(__file__))

    def gnome_tab(title, command, restart_delay=2.0):
        # --wait + exec are important: when the ROS command dies, gnome-terminal
        # returns too; ExecuteProcess can then respawn it automatically.
        return ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--wait',
                '--tab',
                '-t', title,
                '--',
                'bash', '-c',
                f'exec {command}',
            ],
            output='screen',
            respawn=True,
            respawn_delay=restart_delay,
        )

    aks = gnome_tab('AKS', 'ros2 run reel_evata Aks')
    microstrain = gnome_tab(
        'MICROSTRAIN IMU',
        'ros2 launch microstrain_inertial_driver microstrain_launch.py')
    rslidar = gnome_tab('RSLIDAR', 'ros2 launch rslidar_sdk start.py')
    zed = gnome_tab(
        'ZED2i',
        'ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i')
    lio = gnome_tab('LIO_SAM', 'ros2 launch lio_sam run.launch.py')
    lidar_localization = gnome_tab(
        'lidar localization',
        'ros2 launch lidar_localization_ros2 lidar_localization.launch.py')

    supervisor = Node(
        package='evata_nav_guard',
        executable='evata_safety_supervisor.py',
        name='evata_safety_supervisor',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[{
            'sensor_timeout_sec': 1.0,
            'startup_grace_sec': 15.0,
            'recovery_stable_sec': 2.0,
            # These are substring matches, case-insensitive.
            # If one of your real node names differs, edit ONLY this list.
            'required_node_fragments': [
                'aks',
                'microstrain',
                'rslidar',
                'zed',
                'lio',
                'localization',
                'controller_server',
                'planner_server',
                'bt_navigator',
            ],
        }]
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'navReel_safe.launch.py'))
    )

    return LaunchDescription([
        # Safety supervisor starts first and defaults to STOP.
        supervisor,

        TimerAction(period=0.0, actions=[aks]),
        TimerAction(period=0.2, actions=[microstrain]),
        TimerAction(period=0.4, actions=[rslidar]),
        TimerAction(period=0.6, actions=[zed]),
        TimerAction(period=0.8, actions=[lio]),
        TimerAction(period=1.0, actions=[lidar_localization]),

        # Nav2 may start while sensors are initializing; the guard keeps the BT
        # paused until all health checks are stable.
        TimerAction(period=2.0, actions=[nav]),
    ])
