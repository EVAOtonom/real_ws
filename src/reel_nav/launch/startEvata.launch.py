from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():

    # GNOME Terminal'de yeni sekme açma fonksiyonu
    def gnome_tab(title, command):
        return ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--tab',
                '-t',
                title,
                '--',
                'bash',
                '-c',
                f'{command}; exec bash'
            ],
            output='screen'
        )

    # 1. AKS
    aks = gnome_tab(
        'AKS',
        'ros2 run reel_evata Aks'
    )

    # 2. MICROSTRAIN IMU
    microstrain = gnome_tab(
        'MICROSTRAIN IMU',
        'ros2 launch microstrain_inertial_driver microstrain_launch.py'
    )

    # 3. RSLIDAR
    rslidar = gnome_tab(
        'RSLIDAR',
        'ros2 launch rslidar_sdk start.py'
    )

    # 4. ZED2i Wrapper
    zed = gnome_tab(
        'ZED2i',
        'ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i camera_name:=zed2i'
    )
   
    # 6. lio_sam 
    lio = gnome_tab(
        'LIO_SAM',
        'ros2 launch lio_sam run.launch.py '
    )    
    
    # 7. lidar localization
    lidar_localization = gnome_tab(
        'lidar localization',
        'ros2 launch lidar_localization_ros2 lidar_localization.launch.py '
    )       
    
    # 8. nav2 
    nav = gnome_tab(
        'NAV2',
        'ros2 launch reel_nav navReel.launch.py  '
    )      

    return LaunchDescription([
        TimerAction(
            period=0.0,
            actions=[aks]
        ),

        TimerAction(
            period=0.1,
            actions=[microstrain]
        ),

        TimerAction(
            period=0.2,
            actions=[rslidar]
        ),

        TimerAction(
            period=0.3,
            actions=[zed]
        ),
        
        TimerAction(
            period=0.4,
            actions=[lio]
        ),        

        TimerAction(
            period=0.5,
            actions=[lidar_localization]
        ),

        TimerAction(
            period=0.6,
            actions=[nav]
        ),                
    ])
