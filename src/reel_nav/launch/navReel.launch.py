import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # ============================================================
    # CPU YERLESIMI - Ryzen 9 9955HX / 32 logical CPU hedefli
    # ============================================================
    # CPU 0-1  : OS / DDS / genel sistem
    # CPU 2-3  : Nav2 controller_server + local costmap
    # CPU 4-5  : Nav2 planner_server + global costmap
    # CPU 6-7  : pointcloud_to_laserscan
    # CPU 8    : velocity_smoother
    # CPU 9    : local EKF
    # CPU 10   : global EKF + navsat_transform
    # CPU 11   : AMCL + map_server
    # CPU 12   : BT navigator + behavior server
    # CPU 13   : robot/joint state + encoder odometry
    # CPU 14-15: RViz
    # CPU 16-31: SMT kardeşleri ve diğer perception süreçleri için serbest
    #
    # Nav2 Humble bringup varsayılanı use_composition=True olduğundan tüm
    # Nav2 düğümleri tek component_container_isolated içinde toplanabiliyor.
    # Aşağıda composition kapatılıyor ve Nav2 süreçleri ayrı PID'lere ayrılıyor.

    # ============================================================
    # WORKSPACE / PACKAGE PATH TESPITI
    # ============================================================
    # Launch dosyasi iki farkli yerden calisabilir:
    #   1) /home/.../real_ws/src/reel_nav/launch/navReel.launch.py
    #   2) /home/.../real_ws/install/reel_nav/share/reel_nav/launch/navReel.launch.py
    #
    # Eski `dir_path.split('/install')[0]` yaklasimi 1. durumda
    # /launch/src/reel_nav/... gibi hatali, tekrarlanmis yol uretiyordu.
    dir_path = os.path.dirname(os.path.realpath(__file__))
    normalized_dir = dir_path.replace('\\\\', '/')

    if '/src/reel_nav/' in normalized_dir:
        workspace_root = normalized_dir.split('/src/reel_nav/')[0]
    elif '/install/reel_nav/' in normalized_dir:
        workspace_root = normalized_dir.split('/install/reel_nav/')[0]
    else:
        # Son guvenli fallback: reel_nav'in install share yolundan workspace'i bul.
        reel_nav_share = get_package_share_directory('reel_nav').replace('\\\\', '/')
        if '/install/reel_nav/' in reel_nav_share:
            workspace_root = reel_nav_share.split('/install/reel_nav/')[0]
        else:
            raise RuntimeError(
                f"reel_nav workspace root bulunamadi. launch_dir={dir_path}, "
                f"share_dir={reel_nav_share}"
            )

    package_src_dir = os.path.join(workspace_root, 'src', 'reel_nav')

    sdf_path = os.path.join(
        package_src_dir,
        'final_deneme.sdf'
    )

    # Erken ve acik hata ver: daha sonra xacro tarafinda anlamsiz hata gorulmesin.
    if not os.path.isfile(sdf_path):
        raise FileNotFoundError(
            f"final_deneme.sdf bulunamadi: {sdf_path}"
        )

    use_sim_time = LaunchConfiguration(
        'use_sim_time',
        default='false'
    )

    rviz_config_dir = os.path.join(
        package_src_dir,
        'config',
        'nav2_evata_view.rviz'
    )

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    ekf_params_file = os.path.join(
        package_src_dir,
        'config',
        'ekf.yaml'
    )

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            package_src_dir,
            'map',
            'harita.yaml'
        )
    )

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            package_src_dir,
            'config',
            'test.yaml'
        )
    )

    # Xacro dosyasını oku ve işle
    doc = xacro.parse(open(sdf_path))
    xacro.process_doc(doc)


    return LaunchDescription([

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                package_src_dir,
                'map',
                'harita.yaml'
            ),
            description='Full path to map file to load'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                package_src_dir,
                'config',
                'test.yaml'
            ),
            description='Full path to param file to load'
        ),



        # ============================================================
        # SABİT DÖNÜŞLER
        # ============================================================

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_map_to_odom',
        #     output='log',
        #     arguments=[
        #         '0', '0', '0',
        #         '0', '0', '0',
        #         'map', 'odom'
        #     ]
        # ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_odom_to_base',
        #     output='log',
        #     arguments=[
        #         '0', '0', '0',
        #         '0', '0', '0',
        #         'odom', 'base_footprint'
        #     ]
        # ),

        # ============================================================
        # JOINT STATE PUBLISHER
        # ============================================================

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            prefix=['taskset -c 13'],
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': doc.toxml()
                }
            ]
        ),

        # ============================================================
        # ROBOT STATE PUBLISHER
        # ============================================================

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            prefix=['taskset -c 13'],
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': doc.toxml()
                }
            ]
        ),

        # ============================================================
        # POINTCLOUD TO LASERSCAN
        # ============================================================

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_nav',
            output='screen',
            prefix=['taskset -c 6-7'],
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'target_frame': 'base_footprint',
                    'transform_tolerance': 0.20,

                    'min_height': -0.05,
                    'max_height': 1.00,

                    # Sadece ön 270 derece
                    # Arka sağ 45° + arka sol 45° görülmeyecek
                    'angle_min': -2.356194490192345,   # -135°
                    'angle_max':  2.356194490192345,   # +135°

                    'angle_increment': 0.008726646259972,  # 0.5°
                    'scan_time': 0.105,

                    'range_min': 0.50,
                    'range_max': 12.0,

                    'use_inf': True
                }
            ],
            remappings=[
                ('cloud_in', '/rslidar_points'),
                ('scan', '/scan_nav')
            ]
        ),

        # ============================================================
        # NAV2 BRINGUP
        # ============================================================

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    nav2_launch_file_dir,
                    'bringup_launch.py'
                )
            ),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'autostart': 'true',
                # Humble bringup varsayilani True. Ayrı process = scheduler daha iyi dağıtır.
                'use_composition': 'False',
                'use_respawn': 'False',
                'log_level': 'warn'
            }.items()
        ),

        # ============================================================
        # NAV2 PROCESS CPU AFFINITY
        # ============================================================
        # use_composition=False ile Nav2 süreçleri ayrı PID olarak başlar.
        # 4 saniye sonra her kritik süreci kendi CPU grubuna sabitliyoruz.
        # taskset başarısız olursa launch devam eder; sadece uyarı basar.
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash', '-lc',
                        r'''
set +e
pin_proc() {
    pattern="$1"
    cpus="$2"
    pid="$(pgrep -f "^.*/lib/${pattern}( |$)" | head -n1)"
    if [ -n "$pid" ]; then
        taskset -pc "$cpus" "$pid" >/dev/null 2>&1 && \
            echo "[CPU-AFFINITY] ${pattern} -> CPU ${cpus} (PID ${pid})" || \
            echo "[CPU-AFFINITY][WARN] ${pattern} pinlenemedi"
    else
        echo "[CPU-AFFINITY][WARN] ${pattern} PID bulunamadi"
    fi
}

pin_proc 'nav2_controller/controller_server'       '2-3'
pin_proc 'nav2_planner/planner_server'             '4-5'
pin_proc 'nav2_velocity_smoother/velocity_smoother' '8'
pin_proc 'nav2_amcl/amcl'                          '11'
pin_proc 'nav2_map_server/map_server'              '11'
pin_proc 'nav2_bt_navigator/bt_navigator'          '12'
pin_proc 'nav2_behaviors/behavior_server'          '12'
pin_proc 'nav2_smoother/smoother_server'           '12'
pin_proc 'nav2_waypoint_follower/waypoint_follower' '12'
'''
                    ],
                    output='screen'
                )
            ]
        ),

        # ============================================================
        # LOCAL EKF
        # ============================================================

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            prefix=['taskset -c 9'],
            parameters=[
                ekf_params_file,
                {
                    'use_sim_time': use_sim_time
                }
            ],
            remappings=[
                ('odometry/filtered', '/odom')
            ],
            arguments=[
                '--ros-args',
                '--log-level',
                'info'
            ]
        ),

        # ============================================================
        # GLOBAL EKF
        # ============================================================

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            prefix=['taskset -c 10'],
            parameters=[
                ekf_params_file,
                {
                    'use_sim_time': use_sim_time
                }
            ],
            remappings=[
                ('odometry/filtered', '/odometry/global')
            ]
        ),

        # ============================================================
        # NAVSAT TRANSFORM
        # ============================================================

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            prefix=['taskset -c 10'],
            parameters=[
                ekf_params_file,
                {
                    'use_sim_time': use_sim_time
                }
            ],
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/gnss_1/llh_position'),
                ('odometry/filtered', '/odometry/global'),
                ('odometry/gps', '/odometry/gps'),
                ('gps/filtered', '/gps/filtered')
            ],
            arguments=[
                '--ros-args',
                '--log-level',
                'info'
            ]
        ),

        # ============================================================
        # ENCODER ODOMETRİ
        # ============================================================

        Node(
            package='reel_evata',
            executable='OdometerListener',
            name='encoder_odom_publisher',
            output='screen',
            prefix=['taskset -c 13'],
            parameters=[
                {
                    'use_sim_time': use_sim_time
                }
            ]
        ),

        # ============================================================
        # ZED NODE
        # ============================================================

        # Node(
        #     package='zed_wrapper',
        #     executable='zed_node',
        #     name='zed_node',
        #     output='screen',
        #     parameters=[
        #         {
        #             'use_sim_time': use_sim_time
        #         }
        #     ]
        # ),

        # ============================================================
        # RVIZ
        # ============================================================

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            prefix=['taskset -c 14-15'],
            arguments=[
                '-d',
                rviz_config_dir
            ],
            parameters=[
                {
                    'use_sim_time': use_sim_time
                }
            ],
            output='screen'
        ),

    ])
