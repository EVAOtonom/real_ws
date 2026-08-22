from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():

    # ============================================================
    # CPU STRATEJISI
    # ============================================================
    #
    # navReel.launch.py zaten kendi icinde:
    #
    #   CPU 2-15 -> Nav2 / EKF / pointcloud_to_laserscan / RViz
    #
    # kullaniyor.
    #
    # Bu nedenle agir perception / localization proseslerini:
    #
    #   fiziksel core 8-15 + SMT kardesleri
    #   CPU 8,24,9,25,...,15,31
    #
    # GENIS ortak havuzuna birakiyoruz.
    #
    # Neden tek tek bolmuyoruz?
    #
    #   RSLIDAR'i tek CPU'ya kilitlemek packet decode / cloud publish
    #   darboğazi olusturabilir.
    #
    #   LIO-SAM her an tum CPU'larini kullanmaz. O bosken ZED veya
    #   lidar_localization ayni havuzdaki bos CPU'lari kullanabilir.
    #
    #   Linux scheduler 16 logical CPU icinde dinamik dagitim yapar.
    #
    # Nav2'nin kritik CPU'larina perception prosesleri giremez.
    #
    # Fiziksel core 0 (CPU 0,16) OS / DDS icin bos.
    # Fiziksel core 1 (CPU 1,17) AKS + MicroStrain icin.
    # ============================================================

    # Fiziksel CORE 8-15 ve SMT kardesleri:
    # core8  -> CPU 8,24
    # core9  -> CPU 9,25
    # ...
    # core15 -> CPU 15,31
    CPU_HEAVY_POOL = "8,24,9,25,10,26,11,27,12,28,13,29,14,30,15,31"

    # Fiziksel CORE 1 yalnız AKS + MicroStrain icin.
    CPU_CONTROL_IO = "1,17"


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
    # AKS
    # ============================================================
    #
    # AKS'i yapay olarak tek CPU'ya sikistirmiyoruz.
    # Kontrol / seri haberlesme callback'leri Linux scheduler tarafindan
    # uygun CPU'da calisabilsin.
    # ============================================================

    aks = gnome_tab(
        'AKS',
        'ros2 run reel_evata Aks',
        CPU_CONTROL_IO
    )


    # ============================================================
    # MICROSTRAIN IMU
    # ============================================================

    microstrain = gnome_tab(
        'MICROSTRAIN IMU',
        'ros2 launch microstrain_inertial_driver microstrain_launch.py',
        CPU_CONTROL_IO
    )


    # ============================================================
    # RSLIDAR
    # ============================================================
    #
    # ONEMLI:
    # Eskiden tek logical CPU verilmesi driver'i bogabiliyordu.
    # Artik 16 logical CPU'luk havuzda serbest.
    # ============================================================

    rslidar = gnome_tab(
        'RSLIDAR',
        'ros2 launch rslidar_sdk start.py',
        CPU_HEAVY_POOL
    )


    # ============================================================
    # ZED2i
    # ============================================================

    zed = gnome_tab(
        'ZED2i',
        'ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i',
        CPU_HEAVY_POOL
    )


    # ============================================================
    # LIO-SAM
    # ============================================================

    lio = gnome_tab(
        'LIO_SAM',
        'ros2 launch lio_sam run.launch.py '
        'params_file:=/home/otonom/real_ws/src/LIO-SAM/config/params.yaml',
        CPU_HEAVY_POOL
    )


    # ============================================================
    # LIDAR LOCALIZATION
    # ============================================================

    lidar_localization = gnome_tab(
        'LIDAR LOCALIZATION',
        'ros2 launch lidar_localization_ros2 lidar_localization.launch.py',
        CPU_HEAVY_POOL
    )


    # ============================================================
    # NAV2
    # ============================================================
    #
    # Burada ekstra taskset YOK.
    # navReel.launch.py kendi process-level affinity ayarlarini kullaniyor.
    # ============================================================

    nav = gnome_tab(
        'NAV2',
        'ros2 launch reel_nav navReel.launch.py'
    )


    # ============================================================
    # BASLATMA SIRASI
    # ============================================================
    #
    # Aynı anda initialization patlamasi yaratmamak icin sensörlerden
    # navigasyona dogru kademeli baslatiliyor.
    # ============================================================

    return LaunchDescription([

        TimerAction(
            period=0.0,
            actions=[aks]
        ),

        TimerAction(
            period=1.0,
            actions=[microstrain]
        ),

        TimerAction(
            period=2.0,
            actions=[rslidar]
        ),

        TimerAction(
            period=4.0,
            actions=[zed]
        ),

        # IMU + LiDAR stabil veri üretmeye baslasin.
        TimerAction(
            period=7.0,
            actions=[lio]
        ),

        # LIO odometri / correction zincirine zaman ver.
        TimerAction(
            period=10.0,
            actions=[lidar_localization]
        ),

        # Localization sistemi oturduktan sonra Nav2.
        TimerAction(
            period=13.0,
            actions=[nav]
        ),
    ])
