from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():

    # ============================================================
    # CPU DAĞILIMI
    # ============================================================
    #
    # Mevcut navReel.launch.py:
    #
    #   CPU 0-1   -> OS / DDS / genel sistem
    #   CPU 2-15  -> Nav2 + EKF + pointcloud_to_laserscan + RViz
    #
    # Bu start launch'ta geri kalan CPU'ları diğer ağır sistemlere
    # ayırıyoruz.
    #
    #   CPU 16-21 -> LIO-SAM
    #   CPU 22-25 -> LiDAR Localization
    #   CPU 26-29 -> ZED2i
    #   CPU 30    -> AKS + MicroStrain
    #   CPU 31    -> RSLIDAR
    #
    # LIO-SAM config:
    #   numberOfCores: 6
    #
    # olduğundan ona tam 6 logical CPU bırakıyoruz.
    #
    # lidar_localization kendi numberOfCores parametresini kullanmaya
    # devam eder; taskset sadece hangi CPU havuzunda çalışabileceğini
    # sınırlar.
    #
    # NAV2 burada taskset ALMIYOR.
    # navReel.launch.py kendi iç CPU dağılımını yapıyor.
    # ============================================================

    CPU_LIO_SAM = "16-21"
    CPU_LIDAR_LOCALIZATION = "22-25"
    CPU_ZED = "26-29"
    CPU_AKS_IMU = "30"
    CPU_RSLIDAR = "31"


    # ============================================================
    # GNOME TERMINAL TAB
    # ============================================================

    def gnome_tab(title, command, cpus=None):

        if cpus is not None:
            command = f'taskset -c {cpus} {command}'

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


    # ============================================================
    # 1. AKS
    # ============================================================

    aks = gnome_tab(
        'AKS',
        'ros2 run reel_evata Aks',
        CPU_AKS_IMU
    )


    # ============================================================
    # 2. MICROSTRAIN IMU
    # ============================================================

    microstrain = gnome_tab(
        'MICROSTRAIN IMU',
        'ros2 launch microstrain_inertial_driver microstrain_launch.py',
        CPU_AKS_IMU
    )


    # ============================================================
    # 3. RSLIDAR
    # ============================================================

    rslidar = gnome_tab(
        'RSLIDAR',
        'ros2 launch rslidar_sdk start.py',
        CPU_RSLIDAR
    )


    # ============================================================
    # 4. ZED2i
    # ============================================================

    zed = gnome_tab(
        'ZED2i',
        'ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i',
        CPU_ZED
    )


    # ============================================================
    # 5. LIO-SAM
    # ============================================================

    lio = gnome_tab(
        'LIO_SAM',

        # Eğer localization-only launch dosyasını kullanıyorsan:
        'ros2 launch lio_sam run.launch.py',

        # Eski run.launch.py kullanacaksan üstteki satırı:
        # 'ros2 launch lio_sam run.launch.py'
        #
        # olarak değiştir.

        CPU_LIO_SAM
    )


    # ============================================================
    # 6. LIDAR LOCALIZATION
    # ============================================================

    lidar_localization = gnome_tab(
        'LIDAR LOCALIZATION',
        'ros2 launch lidar_localization_ros2 lidar_localization.launch.py',
        CPU_LIDAR_LOCALIZATION
    )


    # ============================================================
    # 7. NAV2
    # ============================================================
    #
    # BURADA taskset YOK.
    #
    # navReel.launch.py zaten kendi içinde:
    #
    # controller_server
    # planner_server
    # velocity_smoother
    # EKF
    # pointcloud_to_laserscan
    # RViz
    #
    # gibi node'ları CPU'lara dağıtıyor.
    # ============================================================

    nav = gnome_tab(
        'NAV2',
        'ros2 launch reel_nav navReel.launch.py'
    )


    # ============================================================
    # BAŞLATMA SIRASI
    # ============================================================
    #
    # Eski:
    #
    # 0.0
    # 0.1
    # 0.2
    # 0.3
    # ...
    #
    # şeklinde neredeyse hepsini aynı anda başlatıyordu.
    #
    # ZED + LIO-SAM + Nav2 aynı anda initialize olduğunda
    # anlık CPU / RAM / DDS yükü çok artabilir.
    #
    # Bu nedenle sensör -> odometri -> localization -> Nav2
    # sırasıyla başlatıyoruz.
    # ============================================================

    return LaunchDescription([

        # --------------------------------------------------------
        # AKS
        # --------------------------------------------------------

        TimerAction(
            period=0.0,
            actions=[aks]
        ),


        # --------------------------------------------------------
        # MICROSTRAIN
        # --------------------------------------------------------

        TimerAction(
            period=1.0,
            actions=[microstrain]
        ),


        # --------------------------------------------------------
        # RSLIDAR
        # --------------------------------------------------------

        TimerAction(
            period=2.0,
            actions=[rslidar]
        ),


        # --------------------------------------------------------
        # ZED
        # --------------------------------------------------------

        TimerAction(
            period=4.0,
            actions=[zed]
        ),


        # --------------------------------------------------------
        # LIO-SAM
        #
        # LiDAR + IMU'nun önce ayağa kalkmasını bekliyoruz.
        # --------------------------------------------------------

        TimerAction(
            period=7.0,
            actions=[lio]
        ),


        # --------------------------------------------------------
        # LIDAR LOCALIZATION
        #
        # LIO-SAM odometrisinin oluşmasına zaman veriyoruz.
        # --------------------------------------------------------

        TimerAction(
            period=10.0,
            actions=[lidar_localization]
        ),


        # --------------------------------------------------------
        # NAV2
        #
        # Sensor + odometry + localization zinciri hazırlandıktan
        # sonra navigation başlıyor.
        # --------------------------------------------------------

        TimerAction(
            period=13.0,
            actions=[nav]
        ),
    ])
