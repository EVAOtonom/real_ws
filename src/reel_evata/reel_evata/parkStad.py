#!/usr/bin/env python3

import copy
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import ComputePathToPose, SmoothPath, FollowPath


# ============================================================
# KULLANICI AYARLARI
# ============================================================

# Kullanılacak park yerini 1 ile 7 arasında seç.
PARK_YERI = 6

# /initialpose verileri map frame'inde kaydedildi.
HEDEF_FRAME = 'map'

# Kod yalnızca sistemin konum üretmeye başladığını kontrol etmek
# için /odom topic'ini dinler. Planner başlangıç pozunu TF'den alır.
ODOM_TOPIC = '/odom'

# Her park yeri kendi /initialpose quaternion yönelimini kullanır.

# Park noktasından önce tamamen düz ilerleme mesafesi.
# Planner ilk segmenti mevcut konumdan bu düzleşme pozuna üretir.
DUZ_GIRIS_MESAFESI = 1.5


# ============================================================
# NAV2 PLUGIN İSİMLERİ
# ============================================================

PLANNER_ID = 'GridBased'
SMOOTHER_ID = 'simple_smoother'
CONTROLLER_ID = 'FollowPath'
GOAL_CHECKER_ID = 'general_goal_checker'
PROGRESS_CHECKER_ID = ''

SMOOTHING_MAX_SECONDS = 5
CHECK_FOR_COLLISIONS = True
FALLBACK_TO_RAW_APPROACH = True
SERVER_KONTROL_SURESI = 0.5


# ============================================================
# PARK NOKTALARI
# ============================================================

PARK_NOKTALARI = {
    1: {
        'x': -1.5524296760559082,
        'y': -4.330573558807373,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.23641480662278433,
        'ow': 0.9716522213269064,
    },

    2: {
        'x': -2.7012860774993896,
        'y': -6.6166605949401855,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.2353716069633958,
        'ow': 0.9719054514897366,
    },

    3: {
        'x': -3.586357831954956,
        'y': -9.08237075805664,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.23468104604017762,
        'ow': 0.9720724286952531,
    },

    4: {
        'x': -4.865077018737793,
        'y': -11.354307174682617,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.22640529027876505,
        'ow': 0.9740331845136428,
    },

    5: {
        'x': -6.256798267364502,
        'y': -13.632193565368652,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.20336931553117824,
        'ow': 0.9791020996302582,
    },

    6: {
        'x': -7.167178630828857,
        'y': -15.849114418029785,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.2245825854104875,
        'ow': 0.9744550591640135,
    },

    7: {
        'x': -8.716404914855957,
        'y': -18.396080017089844,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.24538884871729577,
        'ow': 0.9694247329861149,
    },
}

class Nav2TwoStageParking(Node):
    """
    İki Nav2 planner segmenti üretir:

      1) Mevcut araç pozu -> düzleşme pozu
      2) Düzleşme pozu -> final park pozu

    İlk segmentte Nav2, costmap ve Ackermann kinematiğine göre
    uygun yaklaşma yolunu kendisi hesaplar.

    İkinci segment hedef yönelimiyle aynı doğrultuda ve aynı
    yaw üzerinde olduğundan final park girişi düz kalır.
    """

    def __init__(self):
        super().__init__('nav2_two_stage_parking')

        self.planner_client = ActionClient(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
        )

        self.smoother_client = ActionClient(
            self,
            SmoothPath,
            'smooth_path',
        )

        self.controller_client = ActionClient(
            self,
            FollowPath,
            'follow_path',
        )

        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.raw_path_pub = self.create_publisher(
            Path,
            '/parking/full_raw_path',
            path_qos,
        )

        self.final_path_pub = self.create_publisher(
            Path,
            '/parking/full_follow_path',
            path_qos,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self.odom_callback,
            20,
        )

        if PARK_YERI not in PARK_NOKTALARI:
            raise ValueError(
                'PARK_YERI 1 ile 7 arasında olmalıdır.'
            )

        if DUZ_GIRIS_MESAFESI <= 0.5:
            raise ValueError(
                'DUZ_GIRIS_MESAFESI 0.5 metreden büyük olmalıdır.'
            )

        self.current_pose = None
        self.align_pose = None
        self.final_pose = None

        self.approach_path = None
        self.straight_path = None
        self.raw_full_path = None

        self.started = False
        self.feedback_counter = 0

        self.timer = self.create_timer(
            SERVER_KONTROL_SURESI,
            self.wait_for_ready,
        )

        self.get_logger().info(
            '\nİki aşamalı Nav2 park sistemi başlatıldı.\n'
            f'Park yeri: {PARK_YERI}\n'
            f'Hedef frame: {HEDEF_FRAME}\n'
            f'Düz giriş mesafesi: '
            f'{DUZ_GIRIS_MESAFESI:.1f} m\n'
            'Final yönelim: seçilen park yerinin '
            'quaternion değeri'
        )

    # ========================================================
    # POZ YARDIMCILARI
    # ========================================================

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        """
        Quaternion değerini yaw açısına dönüştürür.

        Sonuç radyan cinsindendir.
        """
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(
            siny_cosp,
            cosy_cosp,
        )

    @staticmethod
    def yaw_to_quaternion(yaw):
        """
        Yaw açısını quaternion değerine dönüştürür.
        """
        return (
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        )

    def create_pose(self, x, y, z, yaw):
        """
        Verilen konum ve yaw değerlerinden PoseStamped oluşturur.
        """
        pose = PoseStamped()

        pose.header.frame_id = HEDEF_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)

        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def odom_callback(self, msg):
        """
        Aracın güncel odometri pozunu kaydeder.

        İlk planner çağrısında başlangıç pozu TF üzerinden alınır.
        Bu veri sistemin odometri üretmeye başladığını kontrol etmek
        amacıyla tutulur.
        """
        self.current_pose = {
            'x': float(msg.pose.pose.position.x),
            'y': float(msg.pose.pose.position.y),
            'z': float(msg.pose.pose.position.z),
            'yaw': self.quaternion_to_yaw(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ),
            'frame_id': msg.header.frame_id,
        }

    def create_targets(self):
        """
        Seçilen park noktası için düzleşme ve final hedefini üretir.
        """
        goal = PARK_NOKTALARI[PARK_YERI]

        goal_x = float(goal['x'])
        goal_y = float(goal['y'])
        goal_z = float(goal['z'])

        # Her park yerinin kendi quaternion değerini yaw'a dönüştür.
        final_yaw = self.quaternion_to_yaw(
            float(goal['ox']),
            float(goal['oy']),
            float(goal['oz']),
            float(goal['ow']),
        )

        final_yaw_deg = math.degrees(final_yaw)

        # Park yönündeki birim doğrultu vektörü.
        heading_x = math.cos(final_yaw)
        heading_y = math.sin(final_yaw)

        # Final park pozunun gerisinde, aynı doğrultuda bulunan
        # düzleşme pozunu hesapla.
        align_x = (
            goal_x
            - DUZ_GIRIS_MESAFESI * heading_x
        )

        align_y = (
            goal_y
            - DUZ_GIRIS_MESAFESI * heading_y
        )

        self.align_pose = self.create_pose(
            align_x,
            align_y,
            goal_z,
            final_yaw,
        )

        self.final_pose = self.create_pose(
            goal_x,
            goal_y,
            goal_z,
            final_yaw,
        )

        self.get_logger().info(
            '\nNav2 hedefleri:\n'
            f'1. Düzleşme pozu: '
            f'X={align_x:.3f}, '
            f'Y={align_y:.3f}, '
            f'Yaw={final_yaw_deg:.1f}°\n'
            f'2. Final park pozu: '
            f'X={goal_x:.3f}, '
            f'Y={goal_y:.3f}, '
            f'Yaw={final_yaw_deg:.1f}°'
        )

    # ========================================================
    # SERVER HAZIRLIĞI
    # ========================================================

    def wait_for_ready(self):
        """
        Nav2 action serverlarını ve odometri verisini bekler.
        """
        if self.started:
            return

        if self.current_pose is None:
            self.get_logger().info(
                f'{ODOM_TOPIC} verisi bekleniyor...'
            )
            return

        planner_ready = self.planner_client.wait_for_server(
            timeout_sec=0.05,
        )

        smoother_ready = self.smoother_client.wait_for_server(
            timeout_sec=0.05,
        )

        controller_ready = self.controller_client.wait_for_server(
            timeout_sec=0.05,
        )

        missing = []

        if not planner_ready:
            missing.append(
                'compute_path_to_pose'
            )

        if not smoother_ready:
            missing.append(
                'smooth_path'
            )

        if not controller_ready:
            missing.append(
                'follow_path'
            )

        if missing:
            self.get_logger().info(
                'Nav2 action server bekleniyor: '
                + ', '.join(missing)
            )
            return

        self.create_targets()

        self.started = True
        self.timer.cancel()

        self.request_approach_path()

    # ========================================================
    # SEGMENT 1: MEVCUT POZ -> DÜZLEŞME POZU
    # ========================================================

    def request_approach_path(self):
        """
        Aracın mevcut pozundan düzleşme pozuna yol ister.
        """
        goal = ComputePathToPose.Goal()

        goal.goal = self.align_pose
        goal.planner_id = PLANNER_ID

        # Nav2 mevcut başlangıç pozunu TF'den kendi alır.
        goal.use_start = False

        self.get_logger().info(
            'Planner segment 1 hesaplıyor: '
            'mevcut araç pozu -> düzleşme pozu'
        )

        future = self.planner_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.approach_goal_response_callback
        )

    def approach_goal_response_callback(self, future):
        """
        Birinci planner hedefinin kabul edilme sonucunu işler.
        """
        try:
            goal_handle = future.result()

        except Exception as error:
            self.get_logger().error(
                'Yaklaşma planner hedefi gönderilemedi: '
                f'{error}'
            )
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                'Yaklaşma planner hedefi reddedildi.'
            )
            return

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.approach_result_callback
        )

    def approach_result_callback(self, future):
        """
        Birinci planner segmentinin sonucunu işler.
        """
        try:
            wrapped_result = future.result()

        except Exception as error:
            self.get_logger().error(
                'Yaklaşma planner sonucu alınamadı: '
                f'{error}'
            )
            return

        if (
            wrapped_result.status
            != GoalStatus.STATUS_SUCCEEDED
        ):
            self.log_planner_failure(
                wrapped_result,
                'Yaklaşma segmenti',
            )
            return

        path = wrapped_result.result.path

        if path is None or len(path.poses) < 2:
            self.get_logger().error(
                'Yaklaşma planner geçersiz yol döndürdü.'
            )
            return

        self.approach_path = path

        self.get_logger().info(
            'Yaklaşma segmenti oluşturuldu. '
            f'Poz sayısı: {len(path.poses)}'
        )

        self.request_straight_path()

    # ========================================================
    # SEGMENT 2: DÜZLEŞME POZU -> FİNAL PARK POZU
    # ========================================================

    def request_straight_path(self):
        """
        Düzleşme pozundan final park pozuna yol ister.
        """
        goal = ComputePathToPose.Goal()

        goal.start = copy.deepcopy(
            self.align_pose
        )

        goal.goal = copy.deepcopy(
            self.final_pose
        )

        goal.planner_id = PLANNER_ID

        # İkinci segmentin başlangıcı açıkça düzleşme pozudur.
        goal.use_start = True

        self.get_logger().info(
            'Planner segment 2 hesaplıyor: '
            'düzleşme pozu -> final park pozu'
        )

        future = self.planner_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.straight_goal_response_callback
        )

    def straight_goal_response_callback(self, future):
        """
        İkinci planner hedefinin kabul edilme sonucunu işler.
        """
        try:
            goal_handle = future.result()

        except Exception as error:
            self.get_logger().error(
                'Düz segment planner hedefi gönderilemedi: '
                f'{error}'
            )
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                'Düz segment planner hedefi reddedildi.'
            )
            return

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.straight_result_callback
        )

    def straight_result_callback(self, future):
        """
        İkinci planner segmentinin sonucunu işler.
        """
        try:
            wrapped_result = future.result()

        except Exception as error:
            self.get_logger().error(
                'Düz segment planner sonucu alınamadı: '
                f'{error}'
            )
            return

        if (
            wrapped_result.status
            != GoalStatus.STATUS_SUCCEEDED
        ):
            self.log_planner_failure(
                wrapped_result,
                'Düz final segmenti',
            )
            return

        path = wrapped_result.result.path

        if path is None or len(path.poses) < 2:
            self.get_logger().error(
                'Düz final planner geçersiz yol döndürdü.'
            )
            return

        self.straight_path = path

        self.get_logger().info(
            'Düz final segmenti oluşturuldu. '
            f'Poz sayısı: {len(path.poses)}'
        )

        self.raw_full_path = self.combine_paths(
            self.approach_path,
            self.straight_path,
        )

        self.raw_path_pub.publish(
            self.raw_full_path
        )

        self.get_logger().info(
            '\nTam ham Nav2 yolu RViz için yayınlandı:\n'
            'Topic: /parking/full_raw_path\n'
            f'Frame: {self.raw_full_path.header.frame_id}\n'
            f'Poz sayısı: '
            f'{len(self.raw_full_path.poses)}'
        )

        self.request_smoothing()

    # ========================================================
    # YAKLAŞMA SEGMENTİNİ YUMUŞAT
    # ========================================================

    def request_smoothing(self):
        """
        Yalnızca yaklaşma segmentini yumuşatır.

        Final düz segment değiştirilmez.
        """
        if self.approach_path is None:
            self.get_logger().error(
                'Yumuşatılacak yaklaşma yolu yok.'
            )
            return

        goal = SmoothPath.Goal()

        goal.path = copy.deepcopy(
            self.approach_path
        )

        goal.smoother_id = SMOOTHER_ID

        goal.max_smoothing_duration = Duration(
            sec=int(SMOOTHING_MAX_SECONDS),
            nanosec=0,
        )

        goal.check_for_collisions = (
            CHECK_FOR_COLLISIONS
        )

        self.get_logger().info(
            'Yalnızca yaklaşma segmenti yumuşatılıyor. '
            'Final düz segment korunacak.'
        )

        future = self.smoother_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.smoother_goal_response_callback
        )

    def smoother_goal_response_callback(self, future):
        """
        Smoother hedefinin kabul edilme sonucunu işler.
        """
        try:
            goal_handle = future.result()

        except Exception as error:
            self.get_logger().error(
                'Smoother hedefi gönderilemedi: '
                f'{error}'
            )

            self.handle_smoothing_failure()
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                'Smoother yaklaşma yolunu reddetti.'
            )

            self.handle_smoothing_failure()
            return

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.smoother_result_callback
        )

    def smoother_result_callback(self, future):
        """
        Smoother sonucunu işler.
        """
        try:
            wrapped_result = future.result()

        except Exception as error:
            self.get_logger().error(
                'Smoother sonucu alınamadı: '
                f'{error}'
            )

            self.handle_smoothing_failure()
            return

        if (
            wrapped_result.status
            != GoalStatus.STATUS_SUCCEEDED
        ):
            self.get_logger().error(
                'Smoother başarısız. '
                f'Durum: {wrapped_result.status}'
            )

            self.handle_smoothing_failure()
            return

        smoothed_approach = (
            wrapped_result.result.path
        )

        if (
            smoothed_approach is None
            or len(smoothed_approach.poses) < 2
        ):
            self.get_logger().error(
                'Smoother geçersiz yaklaşma yolu döndürdü.'
            )

            self.handle_smoothing_failure()
            return

        # Birleşim noktasını planner'ın özgün düzleşme
        # pozuna sabitle.
        smoothed_approach.poses[-1] = copy.deepcopy(
            self.straight_path.poses[0]
        )

        final_path = self.combine_paths(
            smoothed_approach,
            self.straight_path,
        )

        self.publish_and_follow(
            final_path
        )

    def handle_smoothing_failure(self):
        """
        Smoother başarısız olduğunda ham yaklaşma yoluna geçer.
        """
        if FALLBACK_TO_RAW_APPROACH:
            self.get_logger().warn(
                'Yumuşatma başarısız. '
                'Ham iki segmentli Nav2 yolu kullanılacak.'
            )

            self.publish_and_follow(
                self.raw_full_path
            )

        else:
            self.get_logger().error(
                'Yumuşatma başarısız olduğu için araç '
                'hareket ettirilmeyecek.'
            )

    # ========================================================
    # PATH BİRLEŞTİRME
    # ========================================================

    def combine_paths(self, first_path, second_path):
        """
        Yaklaşma ve düz final segmentlerini tek Path yapar.
        """
        combined = Path()

        combined.header = copy.deepcopy(
            first_path.header
        )

        combined.header.stamp = (
            self.get_clock().now().to_msg()
        )

        combined.poses = copy.deepcopy(
            first_path.poses
        )

        # Düzleşme pozu iki path'te de bulunduğu için
        # ikinci path'in ilk pozunu tekrar ekleme.
        for pose in second_path.poses[1:]:
            combined.poses.append(
                copy.deepcopy(pose)
            )

        for pose in combined.poses:
            pose.header.frame_id = (
                combined.header.frame_id
            )

            pose.header.stamp = (
                combined.header.stamp
            )

        return combined

    # ========================================================
    # FOLLOW PATH
    # ========================================================

    def publish_and_follow(self, path):
        """
        Oluşturulan yolu yayınlar ve FollowPath action'a gönderir.
        """
        if path is None or len(path.poses) < 2:
            self.get_logger().error(
                'Controller için geçerli yol yok.'
            )
            return

        path.header.stamp = (
            self.get_clock().now().to_msg()
        )

        for pose in path.poses:
            pose.header.frame_id = (
                path.header.frame_id
            )

            pose.header.stamp = (
                path.header.stamp
            )

        self.final_path_pub.publish(
            path
        )

        self.get_logger().info(
            '\nController yolu RViz için yayınlandı:\n'
            'Topic: /parking/full_follow_path\n'
            f'Frame: {path.header.frame_id}\n'
            f'Poz sayısı: {len(path.poses)}'
        )

        goal = FollowPath.Goal()

        goal.path = path
        goal.controller_id = CONTROLLER_ID
        goal.goal_checker_id = GOAL_CHECKER_ID

        if hasattr(
            goal,
            'progress_checker_id',
        ):
            goal.progress_checker_id = (
                PROGRESS_CHECKER_ID
            )

        future = self.controller_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )

        future.add_done_callback(
            self.controller_goal_response_callback
        )

    def controller_goal_response_callback(self, future):
        """
        FollowPath hedefinin kabul edilme sonucunu işler.
        """
        try:
            goal_handle = future.result()

        except Exception as error:
            self.get_logger().error(
                'Controller hedefi gönderilemedi: '
                f'{error}'
            )
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                'Controller park yolunu reddetti.'
            )
            return

        self.get_logger().info(
            f'{PARK_YERI}. park yolu controller '
            'tarafından kabul edildi.'
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.controller_result_callback
        )

    def feedback_callback(self, feedback_msg):
        """
        FollowPath geri bildirimlerini belirli aralıklarla yazdırır.
        """
        self.feedback_counter += 1

        if self.feedback_counter % 10 != 0:
            return

        feedback = feedback_msg.feedback

        distance = getattr(
            feedback,
            'distance_to_goal',
            None,
        )

        speed = getattr(
            feedback,
            'speed',
            None,
        )

        texts = []

        if distance is not None:
            texts.append(
                f'Kalan mesafe: {distance:.2f} m'
            )

        if speed is not None:
            texts.append(
                f'Hız: {speed:.2f} m/s'
            )

        if texts:
            self.get_logger().info(
                ' | '.join(texts)
            )

    def controller_result_callback(self, future):
        """
        FollowPath sonucunu işler.
        """
        try:
            wrapped_result = future.result()

        except Exception as error:
            self.get_logger().error(
                'Controller sonucu alınamadı: '
                f'{error}'
            )
            return

        status = wrapped_result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Araç {PARK_YERI}. park yerine ulaştı.'
            )

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(
                'Park işlemi iptal edildi.'
            )

        elif status == GoalStatus.STATUS_ABORTED:
            result = wrapped_result.result

            error_code = getattr(
                result,
                'error_code',
                'bilinmiyor',
            )

            self.get_logger().error(
                'Controller park yolunu takip edemedi. '
                f'Hata kodu: {error_code}'
            )

        else:
            self.get_logger().warn(
                'Beklenmeyen controller durum kodu: '
                f'{status}'
            )

    # ========================================================
    # HATA RAPORU
    # ========================================================

    def log_planner_failure(
        self,
        wrapped_result,
        segment_name,
    ):
        """
        Planner hatalarını ayrıntılı olarak loglar.
        """
        result = wrapped_result.result

        error_code = getattr(
            result,
            'error_code',
            'Humble sürümünde mevcut değil',
        )

        error_msg = getattr(
            result,
            'error_msg',
            '',
        )

        self.get_logger().error(
            f'{segment_name} oluşturulamadı. '
            f'Action durumu={wrapped_result.status}, '
            f'hata kodu={error_code}, '
            f'hata={error_msg}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = Nav2TwoStageParking()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            'Nav2 park sistemi kapatılıyor.'
        )

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
