import copy
import os
import tempfile
import xml.etree.ElementTree as ET

import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def _find_bt_node(root):
    main_id = root.attrib.get('main_tree_to_execute')
    behavior_trees = list(root.findall('.//BehaviorTree'))
    if not behavior_trees:
        raise RuntimeError('No <BehaviorTree> found in Nav2 default BT XML')
    if main_id:
        for bt in behavior_trees:
            if bt.attrib.get('ID') == main_id:
                return bt
    return behavior_trees[0]


def _make_guarded_bt(source_xml, output_xml):
    tree = ET.parse(source_xml)
    root = tree.getroot()
    bt = _find_bt_node(root)
    children = list(bt)
    if not children:
        raise RuntimeError(f'Nav2 BT has no root child: {source_xml}')

    for child in children:
        bt.remove(child)

    guard = ET.Element('EvataSafetyPause', {'health_topic': '/evata/system_healthy'})
    if len(children) == 1:
        guard.append(children[0])
    else:
        sequence = ET.Element('Sequence', {'name': 'EvataGuardedRootSequence'})
        for child in children:
            sequence.append(child)
        guard.append(sequence)
    bt.append(guard)

    try:
        ET.indent(tree, space='  ')
    except AttributeError:
        pass
    tree.write(output_xml, encoding='utf-8', xml_declaration=True)


def _default_nav2_plugins():
    try:
        default_params = os.path.join(
            get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')
        with open(default_params, 'r') as f:
            data = yaml.safe_load(f) or {}
        return list(
            data.get('bt_navigator', {})
                .get('ros__parameters', {})
                .get('plugin_lib_names', [])
        )
    except Exception:
        return []


def _make_safe_params(original_params):
    with open(original_params, 'r') as f:
        params = yaml.safe_load(f) or {}

    bt_params = params.setdefault('bt_navigator', {}).setdefault('ros__parameters', {})

    plugins = list(bt_params.get('plugin_lib_names', []) or [])
    if not plugins:
        plugins = _default_nav2_plugins()
    if 'evata_safety_pause_bt_node' not in plugins:
        plugins.append('evata_safety_pause_bt_node')
    bt_params['plugin_lib_names'] = plugins

    bt_share = get_package_share_directory('nav2_bt_navigator')
    bt_dir = os.path.join(bt_share, 'behavior_trees')

    nav_to_pose_src = os.path.join(bt_dir, 'navigate_to_pose_w_replanning_and_recovery.xml')
    nav_to_pose_out = os.path.join(tempfile.gettempdir(), 'evata_nav_to_pose_guarded.xml')
    _make_guarded_bt(nav_to_pose_src, nav_to_pose_out)
    bt_params['default_nav_to_pose_bt_xml'] = nav_to_pose_out

    nav_through_src = os.path.join(bt_dir, 'navigate_through_poses_w_replanning_and_recovery.xml')
    if os.path.exists(nav_through_src):
        nav_through_out = os.path.join(tempfile.gettempdir(), 'evata_nav_through_poses_guarded.xml')
        _make_guarded_bt(nav_through_src, nav_through_out)
        bt_params['default_nav_through_poses_bt_xml'] = nav_through_out

    output = os.path.join(tempfile.gettempdir(), 'evata_nav2_safe_params.yaml')
    with open(output, 'w') as f:
        yaml.safe_dump(params, f, sort_keys=False)
    return output


def generate_launch_description():
    dir_path = os.path.dirname(os.path.realpath(__file__))
    src_dir = dir_path.split('/install')[0]

    sdf_path = os.path.join(src_dir, 'src', 'reel_nav', 'final_deneme.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_config_dir = os.path.join(
        src_dir, 'src', 'reel_nav', 'config', 'nav2_evata_view.rviz')

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    ekf_params_file = os.path.join(
        src_dir, 'src', 'reel_nav', 'config', 'ekf.yaml')

    map_default = os.path.join(src_dir, 'src', 'reel_nav', 'map', 'harita.yaml')
    original_params = os.path.join(src_dir, 'src', 'reel_nav', 'config', 'test.yaml')
    safe_params = _make_safe_params(original_params)

    map_dir = LaunchConfiguration('map', default=map_default)

    doc = xacro.parse(open(sdf_path))
    xacro.process_doc(doc)

    nav2_guarded = GroupAction([
        # Critical: Nav2 no longer publishes directly to the robot's /cmd_vel.
        # The supervisor is the ONLY publisher that should reach /cmd_vel.
        SetRemap(src='cmd_vel', dst='cmd_vel_nav'),
        SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_file_dir, 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': safe_params,
                'autostart': 'true',
            }.items()
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=map_default,
            description='Full path to map file to load'),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': doc.toxml(),
            }]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': doc.toxml(),
            }]
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_nav',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.10,
                'min_height': -0.05,
                'max_height': 1.80,
                'angle_min': -3.141592653589793,
                'angle_max': 3.141592653589793,
                'angle_increment': 0.008726646259972,
                'scan_time': 0.105,
                'range_min': 0.50,
                'range_max': 12.0,
                'use_inf': True,
            }],
            remappings=[
                ('cloud_in', '/rslidar_points'),
                ('scan', '/scan_nav'),
            ]
        ),

        nav2_guarded,

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odom')],
            arguments=['--ros-args', '--log-level', 'info']
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odometry/global')]
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/gnss_1/llh_position'),
                ('odometry/filtered', '/odometry/global'),
                ('odometry/gps', '/odometry/gps'),
                ('gps/filtered', '/gps/filtered'),
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),

        Node(
            package='reel_evata',
            executable='OdometerListener',
            name='encoder_odom_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
