#!/usr/bin/env python3

import copy
import json
import math
import time

import rclpy
import tf2_ros

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import ComputePathThroughPoses, FollowPath, NavigateToPose
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


# ============================================================
# GENEL AYARLAR
# ============================================================

MAP_FRAME = 'map'
BASE_FRAME = 'base_footprint'
ODOM_TOPIC = '/odom'
DETECTED_SIGNS_TOPIC = '/detected_signs'
DETECTED_PARKING_DETAIL_TOPIC = '/detected_parking_sign'


# ============================================================
# PARK ALANI TETİK POLİGONU - YENİ 4 NOKTA
# ============================================================

PARK_TETIK_BOLGESI = [
    (-18.085346221923828,   2.321505546569824),
    (-32.98799133300781,   -9.707924842834473),
    (-24.771465301513672, -21.28096580505371),
    (-13.22526741027832,  -10.970020294189453),
]


# ============================================================
# PARK ALANINA GİRİNCE NORMAL FollowPath HIZINI DÜŞÜR
# ============================================================
# test(2).yaml içindeki normal FollowPath desired_linear_vel = 1.0.
# Trigger alanına girince yalnızca staging/arama waypointleri için 0.35'e çekilir.
# Cüllop park zaten ParkFollowPath kullanıyor ve YAML'da 0.35 m/s.

NORMAL_FOLLOWPATH_DESIRED_LINEAR_VEL = 1.0
PARK_AREA_FOLLOWPATH_DESIRED_LINEAR_VEL = 0.35


# ============================================================
# 1. PARK LEVHASI ARAMA WAYPOINTİ
# ============================================================

STAGING_X = -16.769689559936523
STAGING_Y = -16.44771957397461
STAGING_YAW_DEG = -46.507636965324


# ============================================================
# 2. PARK LEVHASI ARAMA WAYPOINTİ
# ============================================================

SECOND_SEARCH_X = -12.92992877960205
SECOND_SEARCH_Y = -18.595855712890625
SECOND_SEARCH_Z = 0.0
SECOND_SEARCH_OZ = -0.07136896896585412
SECOND_SEARCH_OW = 0.9974499838431754
SECOND_SEARCH_YAW_RAD = -0.142859389838442


# ============================================================
# DURUŞ / TARAMA
# ============================================================

STOP_LINEAR_THRESHOLD = 0.05
STOP_ANGULAR_THRESHOLD = 0.05
STOP_HOLD_SECONDS = 1.0
CANCEL_WAIT_SECONDS = 0.50
PARK_SIGN_SCAN_SECONDS = 10.0
PARK_REQUIRED_DETECTIONS = 1


# ============================================================
# SABİT FALLBACK PARK
# ============================================================
# İki 10 saniyelik aramada da PARK görülmezse RANDOM YOK.
# Buradaki park numarasına gidilir. İstediğin parkı sadece bu satırdan değiştir.

FIXED_FALLBACK_PARK_INDEX = 1


# Dış levha tespit node'undan gelebilecek park-yasak sınıfları.
NO_PARKING_CLASS_ALIASES = {
    'park yasak',
    'park yasagi',
    'park etmek yasak',
    'no parking',
    'noparking',
}

# ============================================================
# CÜLLOP PARK GEOMETRİSİ
# ============================================================

# Final park noktasından önce aracın tamamen park yönüne dönmüş
# olarak ilerleyeceği düz giriş mesafesi.
DUZ_GIRIS_MESAFESI = 1.5

# Park 1-6 için ortak viraj hazırlık geometrisi.
VIRAJ_HAZIRLIK_MESAFESI = 5.0
VIRAJ_SAG_OFSET_MESAFESI = 3.0
VIRAJ_HAZIRLIK_YAW_OFFSET_DEG = 50.0

UC_ASAMALI_PARK_YERLERI = {1, 2, 3, 4, 5, 6}
IKI_ASAMALI_PARK_YERLERI = {7, 8, 9}


# ============================================================
# NAV2 PARK PLANNER / CONTROLLER
# ============================================================

PLANNER_ID = 'ParkingGrid'
CONTROLLER_ID = 'ParkFollowPath'
GOAL_CHECKER_ID = 'general_goal_checker'
PROGRESS_CHECKER_ID = ''

NORMAL_XY_GOAL_TOLERANCE = 1.0
NORMAL_YAW_GOAL_TOLERANCE = 0.7

PARK_XY_GOAL_TOLERANCE = 0.20
PARK_YAW_GOAL_TOLERANCE = 0.10


# ============================================================
# 9 PARK NOKTASI - GÜNCEL /initialpose VERİLERİ
# ============================================================

PARK_NOKTALARI = {
    1: {
        'x': 8.250296601444525,
        'y': -12.817874193377493,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.3951797498614198,
        'ow': 0.9186038130224943,
    },

    2: {
        'x': 6.45089466290189,
        'y': -14.697705345067515,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.403016542761911,
        'ow': 0.9151926935133589,
    },

    3: {
        'x': 4.4505624291607635,
        'y': -16.36755313048985,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.40193285638014364,
        'ow': 0.915669142737757,
    },

    4: {
        'x': 2.5272573901948214,
        'y': -18.141262344159436,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.3921338931106277,
        'ow': 0.9199081529554474,
    },

    5: {
        'x': 0.8757253042189346,
        'y': -19.88546176393868,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.40774293304523923,
        'ow': 0.9130967640681165,
    },

    6: {
        'x': -1.1223330574834662,
        'y': -21.72483809701007,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.3925955420529421,
        'ow': 0.9197112266141784,
    },

    7: {
        'x': -3.0231264310682135,
        'y': -23.474967796717102,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.3925955420529421,
        'ow': 0.9197112266141784,
    },

    8: {
        'x': -4.9255425048955175,
        'y': -25.023106196647312,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.39253672967745207,
        'ow': 0.9197363295282681,
    },

    9: {
        'x': -6.94339338075001,
        'y': -26.825295069938328,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.39253672967745207,
        'ow': 0.9197363295282681,
    },
}


class ExternalSignParkingManager(Node):
    STATE_NORMAL = 'NORMAL'
    STATE_CANCELING = 'CANCELING'
    STATE_STAGING_NAV = 'STAGING_NAV'
    STATE_SECOND_SEARCH_NAV = 'SECOND_SEARCH_NAV'
    STATE_WAIT_STOP = 'WAIT_STOP'
    STATE_WAIT_PARK_SIGN = 'WAIT_PARK_SIGN'
    STATE_WAIT_PARK_SERVERS = 'WAIT_PARK_SERVERS'
    STATE_PARK_TOLERANCE = 'PARK_TOLERANCE'
    STATE_PARK_PLANNING = 'PARK_PLANNING'
    STATE_PARK_FOLLOWING = 'PARK_FOLLOWING'
    STATE_DONE = 'DONE'
    STATE_ERROR = 'ERROR'

    def __init__(self):
        super().__init__('external_sign_parking_manager')

        # TF: araç konumu map -> base_footprint üzerinden alınır.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ----------------------------------------------------
        # SUBSCRIBERS
        # ----------------------------------------------------
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, 20)

        # Ana levha tespit node'unun mevcut genel çıktısı.
        self.create_subscription(
            String,
            DETECTED_SIGNS_TOPIC,
            self.detected_signs_callback,
            20,
        )

        # Ana levha tespit node'unun PARK/PARK-YASAK için map koordinatlı çıktısı.
        self.create_subscription(
            String,
            DETECTED_PARKING_DETAIL_TOPIC,
            self.detected_parking_sign_callback,
            20,
        )

        # ----------------------------------------------------
        # PARK DEBUG PUBLISHERS
        # ----------------------------------------------------
        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.goal_poses_pub = self.create_publisher(PoseArray, '/parking/goal_poses', path_qos)
        self.planner_path_pub = self.create_publisher(Path, '/parking/planner_path', path_qos)
        self.follow_path_pub = self.create_publisher(Path, '/parking/follow_path', path_qos)

        # ----------------------------------------------------
        # NAV2 ACTION CLIENTS
        # ----------------------------------------------------
        self.navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.planner_client = ActionClient(self, ComputePathThroughPoses, 'compute_path_through_poses')
        self.controller_client = ActionClient(self, FollowPath, 'follow_path')

        # Aktif normal navigasyonu iptal etmek için.
        self.cancel_navigate_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.cancel_navigate_through_client = self.create_client(CancelGoal, '/navigate_through_poses/_action/cancel_goal')
        self.cancel_follow_client = self.create_client(CancelGoal, '/follow_path/_action/cancel_goal')

        # Goal tolerance + trigger alanı hız değişikliği aynı controller_server servisinden.
        self.set_params_client = self.create_client(SetParameters, '/controller_server/set_parameters')

        # ----------------------------------------------------
        # STATE / LOCALIZATION
        # ----------------------------------------------------
        self.state = self.STATE_NORMAL
        self.zone_triggered = False
        self.current_linear_speed = 999.0
        self.current_angular_speed = 999.0
        self.odom_received = False
        self.stopped_since = None
        self.last_robot_map_pose = None
        self.first_position_logged = False

        # ----------------------------------------------------
        # STAGING / SEARCH NAV
        # ----------------------------------------------------
        self.cancel_started_at = None
        self.staging_goal_sent = False
        self.staging_goal_handle = None
        self.second_search_goal_sent = False
        self.second_search_goal_handle = None

        # ----------------------------------------------------
        # EXTERNAL PARK SIGN SEARCH
        # ----------------------------------------------------
        self.park_detection_count = 0
        self.park_detection_points = []
        self.park_search_started_at = None
        self.park_search_attempt = 0
        self.park_sign_search_permanently_disabled = False
        self.no_parking_slots = set()
        self.no_parking_detection_points = []
        self.selected_park_index = None
        self.selection_reason = None
        self.last_generic_park_log = 0.0

        # ----------------------------------------------------
        # TRIGGER AREA SPEED
        # ----------------------------------------------------
        self.park_area_speed_request_pending = False
        self.park_area_speed_active = False
        self.restore_speed_request_pending = False
        self.restore_speed_future = None

        # ----------------------------------------------------
        # CÜLLOP PARK RUNTIME
        # ----------------------------------------------------
        self.turn_pose = None
        self.align_pose = None
        self.final_pose = None
        self.goal_poses = []
        self.goal_labels = []
        self.planned_path = None
        self.feedback_counter = 0
        self.parking_started = False
        self.parking_finished = False

        self.tolerance_request_pending = False
        self.park_tolerance_active = False
        self.restore_request_pending = False
        self.restore_future = None

        self.control_timer = self.create_timer(0.10, self.control_loop)


        self.get_logger().info(
            '\n========== EXTERNAL LEVHA + CÜLLOP PARK ==========\n'
            f'Tetik poligonu: {PARK_TETIK_BOLGESI}\n'
            f'Trigger alanı FollowPath hızı: {PARK_AREA_FOLLOWPATH_DESIRED_LINEAR_VEL:.2f} m/s\n'
            f'1. arama waypoint: X={STAGING_X:.3f}, Y={STAGING_Y:.3f}, '
            f'Yaw={STAGING_YAW_DEG:.2f}°\n'
            f'2. arama waypoint: X={SECOND_SEARCH_X:.3f}, Y={SECOND_SEARCH_Y:.3f}, '
            f'Yaw={math.degrees(SECOND_SEARCH_YAW_RAD):.2f}°\n'
            f'Her arama: {PARK_SIGN_SCAN_SECONDS:.0f} sn\n'
            f'Sabit fallback park: #{FIXED_FALLBACK_PARK_INDEX}\n'
            f'Levha topicleri: {DETECTED_SIGNS_TOPIC} + {DETECTED_PARKING_DETAIL_TOPIC}\n'
            f'Park planner/controller: {PLANNER_ID} / {CONTROLLER_ID}'
        )

    # ========================================================
    # GEOMETRİ
    # ========================================================

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def yaw_to_quaternion(yaw):
        return (
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        )

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def create_pose(self, x, y, z, yaw):
        pose = PoseStamped()
        pose.header.frame_id = MAP_FRAME
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

    @staticmethod
    def point_on_segment(px, py, x1, y1, x2, y2, eps=1e-6):
        cross = (px - x1) * (y2 - y1) - (py - y1) * (x2 - x1)
        if abs(cross) > eps:
            return False
        dot = (px - x1) * (px - x2) + (py - y1) * (py - y2)
        return dot <= eps

    @classmethod
    def point_in_polygon(cls, x, y, polygon):
        for i in range(len(polygon)):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % len(polygon)]
            if cls.point_on_segment(x, y, x1, y1, x2, y2):
                return True

        inside = False
        j = len(polygon) - 1

        for i in range(len(polygon)):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            intersects = (
                (yi > y) != (yj > y)
                and x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi
            )
            if intersects:
                inside = not inside
            j = i

        return inside

    def get_robot_pose_map_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                BASE_FRAME,
                rclpy.time.Time(),
            )
            q = transform.transform.rotation
            return (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                self.quaternion_to_yaw(q.x, q.y, q.z, q.w),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None

    def get_robot_pose_map(self):
        # Araç konumu yalnızca TF ağacındaki
        # map -> base_footprint dönüşümünden alınır.
        return self.get_robot_pose_map_tf()

    # ========================================================
    # TF KONUM / ODOM
    # ========================================================

    def odom_callback(self, msg):
        self.odom_received = True
        self.current_linear_speed = float(msg.twist.twist.linear.x)
        self.current_angular_speed = float(msg.twist.twist.angular.z)

    # ========================================================
    # ANA STATE MACHINE
    # ========================================================

    def control_loop(self):
        robot_pose = self.get_robot_pose_map()
        if robot_pose is not None:
            self.last_robot_map_pose = robot_pose

            if not self.first_position_logged:
                self.first_position_logged = True
                x, y, yaw = robot_pose
                inside = self.point_in_polygon(x, y, PARK_TETIK_BOLGESI)
                self.get_logger().warn(
                    '\n[KONUM KONTROL - 1 KEZ]\n'
                    f'TF map -> base_footprint: X={x:.3f}, Y={y:.3f}, '
                    f'Yaw={math.degrees(yaw):.1f}°\n'
                    f'Park alanında mı: {"EVET" if inside else "HAYIR"}'
                )

        # 1) NORMAL -> trigger alanı
        if self.state == self.STATE_NORMAL:
            if self.zone_triggered or robot_pose is None:
                return

            x, y, _ = robot_pose
            if self.point_in_polygon(x, y, PARK_TETIK_BOLGESI):
                self.zone_triggered = True
                self.state = self.STATE_CANCELING
                self.cancel_started_at = time.monotonic()

                self.get_logger().warn(
                    '\n[PARK ALANI TETİKLENDİ]\n'
                    f'Araç park alanına girdi: X={x:.3f}, Y={y:.3f}\n'
                    f'FollowPath hızı {PARK_AREA_FOLLOWPATH_DESIRED_LINEAR_VEL:.2f} m/s yapılacak.\n'
                    'Mevcut normal Nav2 waypoint/route iptal ediliyor.'
                )

                self.request_park_area_speed()
                self.cancel_active_navigation()
            return

        # 2) CANCEL -> hız gerçekten düştükten sonra 1. waypoint
        if self.state == self.STATE_CANCELING:
            if self.cancel_started_at is None:
                self.cancel_started_at = time.monotonic()

            if not self.park_area_speed_active:
                self.request_park_area_speed()
                return

            if (
                not self.staging_goal_sent
                and time.monotonic() - self.cancel_started_at >= CANCEL_WAIT_SECONDS
            ):
                self.send_staging_goal()
            return

        # 3) Search waypoint tamamlandı -> araç tam dursun
        if self.state == self.STATE_WAIT_STOP:
            linear_ok = abs(self.current_linear_speed) <= STOP_LINEAR_THRESHOLD
            angular_ok = abs(self.current_angular_speed) <= STOP_ANGULAR_THRESHOLD

            if linear_ok and angular_ok:
                now = time.monotonic()
                if self.stopped_since is None:
                    self.stopped_since = now

                if now - self.stopped_since >= STOP_HOLD_SECONDS:
                    self.start_park_sign_scan()
            else:
                self.stopped_since = None
            return

        # 4) Dış levha node'undan PARK bekle
        if self.state == self.STATE_WAIT_PARK_SIGN:
            if self.park_search_started_at is None:
                self.park_search_started_at = time.monotonic()

            elapsed = time.monotonic() - self.park_search_started_at
            if elapsed >= PARK_SIGN_SCAN_SECONDS:
                self.finish_park_sign_scan_and_select()
            return

        # 5) Cüllop park serverları
        if self.state == self.STATE_WAIT_PARK_SERVERS:
            self.try_start_selected_parking()
            return

    # ========================================================
    # AKTİF NORMAL NAVİGASYONU İPTAL ET
    # ========================================================

    def cancel_active_navigation(self):
        clients = (
            (self.cancel_navigate_client, 'NavigateToPose'),
            (self.cancel_navigate_through_client, 'NavigateThroughPoses'),
            (self.cancel_follow_client, 'FollowPath'),
        )

        for client, name in clients:
            if not client.wait_for_service(timeout_sec=0.05):
                self.get_logger().info(f'[CANCEL] {name} cancel servisi hazır değil; atlandı.')
                continue

            future = client.call_async(CancelGoal.Request())
            future.add_done_callback(
                lambda f, n=name: self.cancel_service_callback(f, n)
            )

    def cancel_service_callback(self, future, action_name):
        try:
            response = future.result()
            self.get_logger().info(
                f'[CANCEL] {action_name}: '
                f'{len(response.goals_canceling)} goal iptal sürecinde.'
            )
        except Exception as error:
            self.get_logger().warn(
                f'[CANCEL] {action_name} cevabı alınamadı: {error}'
            )

    # ========================================================
    # STAGING WAYPOINT
    # ========================================================

    def send_staging_goal(self):
        if self.staging_goal_sent:
            return

        if not self.navigate_client.wait_for_server(timeout_sec=0.05):
            # Server henüz hazır değilse state CANCELING'de kal ve tekrar dene.
            return

        self.staging_goal_sent = True

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose(
            STAGING_X,
            STAGING_Y,
            0.0,
            math.radians(STAGING_YAW_DEG),
        )

        self.state = self.STATE_STAGING_NAV

        self.get_logger().warn(
            '\n[STAGING GOAL]\n'
            'Eski waypoint iptal edildi. 1. park levhası arama waypointi gönderiliyor.\n'
            f'X={STAGING_X:.3f}, Y={STAGING_Y:.3f}, '
            f'Yaw={STAGING_YAW_DEG:.2f}°'
        )

        future = self.navigate_client.send_goal_async(goal_msg)
        future.add_done_callback(self.staging_goal_response_callback)

    def staging_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.finish_with_error(f'[STAGING] Goal gönderilemedi: {error}')
            return

        if not goal_handle.accepted:
            self.finish_with_error('[STAGING] NavigateToPose goal reddedildi.')
            return

        self.staging_goal_handle = goal_handle
        self.get_logger().info('[STAGING] Goal kabul edildi.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.staging_result_callback)

    def staging_result_callback(self, future):
        try:
            wrapped_result = future.result()
        except Exception as error:
            self.finish_with_error(f'[STAGING] Sonuç alınamadı: {error}')
            return

        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.stopped_since = None
            self.park_search_attempt = 1
            self.state = self.STATE_WAIT_STOP
            self.get_logger().warn(
                '[STAGING] Waypoint tamamlandı. '
                '1. levha taramasından önce araç tam duruşu doğrulanıyor.'
            )
            return

        self.finish_with_error(
            f'[STAGING] Başarısız. action_status={wrapped_result.status}'
        )

    # ========================================================
    # İKİNCİ PARK LEVHASI TARAMA NOKTASINA GİT
    # ========================================================

    def send_second_search_goal(self):
        if self.second_search_goal_sent:
            return

        if self.park_sign_search_permanently_disabled:
            return

        if not self.navigate_client.wait_for_server(timeout_sec=0.05):
            self.finish_with_error(
                '[2. TARAMA] navigate_to_pose action server hazır değil.'
            )
            return

        self.second_search_goal_sent = True
        self.park_search_started_at = None
        self.reset_park_confirmation()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose(
            SECOND_SEARCH_X,
            SECOND_SEARCH_Y,
            SECOND_SEARCH_Z,
            SECOND_SEARCH_YAW_RAD,
        )

        self.state = self.STATE_SECOND_SEARCH_NAV

        self.get_logger().warn(
            '\n[1. TARAMADA PARK LEVHASI BULUNAMADI]\n'
            'İkinci ve SON tarama noktasına gidiliyor.\n'
            f'X={SECOND_SEARCH_X:.3f}, Y={SECOND_SEARCH_Y:.3f}, '
            f'Yaw={math.degrees(SECOND_SEARCH_YAW_RAD):.2f}°\n'
            'Bu geçiş şimdilik normal NavigateToPose path ile yapılır; '
            'özel yay geometrisi uygulanmaz.'
        )

        future = self.navigate_client.send_goal_async(goal_msg)
        future.add_done_callback(self.second_search_goal_response_callback)

    def second_search_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.finish_with_error(
                f'[2. TARAMA] Goal gönderilemedi: {error}'
            )
            return

        if not goal_handle.accepted:
            self.finish_with_error(
                '[2. TARAMA] NavigateToPose goal reddedildi.'
            )
            return

        self.second_search_goal_handle = goal_handle
        self.get_logger().info('[2. TARAMA] Goal kabul edildi.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.second_search_result_callback)

    def second_search_result_callback(self, future):
        try:
            wrapped_result = future.result()
        except Exception as error:
            self.finish_with_error(
                f'[2. TARAMA] Sonuç alınamadı: {error}'
            )
            return

        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.stopped_since = None
            self.park_search_attempt = 2
            self.state = self.STATE_WAIT_STOP

            self.get_logger().warn(
                '[2. TARAMA] Noktaya ulaşıldı. '
                'İkinci ve SON levha taramasından önce tam duruş doğrulanıyor.'
            )
            return

        self.finish_with_error(
            '[2. TARAMA] NavigateToPose başarısız. '
            f'action_status={wrapped_result.status}'
        )

    # ========================================================
    # DIŞ PARK LEVHASI TOPIC TARAMASI - SADECE 2 KEZ
    # ========================================================

    def start_park_sign_scan(self):
        if self.state == self.STATE_WAIT_PARK_SIGN:
            return

        if self.park_sign_search_permanently_disabled:
            return

        if self.park_search_attempt not in (1, 2):
            self.finish_with_error(
                f'Geçersiz park levhası arama numarası: '
                f'{self.park_search_attempt}'
            )
            return

        self.reset_park_confirmation()
        self.park_search_started_at = time.monotonic()
        self.state = self.STATE_WAIT_PARK_SIGN

        if self.park_search_attempt == 1:
            location_text = 'STAGING / İLK TARAMA'
        else:
            location_text = 'İKİNCİ NOKTA / SON TARAMA'

        self.get_logger().warn(
            '\n[DIŞ PARK LEVHASI TOPIC TARAMASI BAŞLADI]\n'
            f'Arama: {self.park_search_attempt}/2 - {location_text}\n'
            f'Süre: {PARK_SIGN_SCAN_SECONDS:.0f} sn\n'
            'Geçerli PARK tabelası görülür görülmez timeout beklenmeden '
            'park seçilecek. PARK YASAK görülürse ilgili slot elenecek.'
        )

    def finish_park_sign_scan_and_select(self):
        if self.state != self.STATE_WAIT_PARK_SIGN:
            return
        if self.park_sign_search_permanently_disabled:
            return

        elapsed = 0.0
        if self.park_search_started_at is not None:
            elapsed = time.monotonic() - self.park_search_started_at

        # Dış levha detector'ünden map koordinatlı PARK geldiyse en yakın uygun slot.
        if self.park_detection_count >= PARK_REQUIRED_DETECTIONS and self.park_detection_points:
            avg_x = sum(p[0] for p in self.park_detection_points) / len(self.park_detection_points)
            avg_y = sum(p[1] for p in self.park_detection_points) / len(self.park_detection_points)

            selected_index, selected_distance = self.find_nearest_park(
                avg_x,
                avg_y,
                excluded=self.no_parking_slots,
            )

            if selected_index is not None:
                self.selected_park_index = selected_index
                self.selection_reason = f'EXTERNAL_PARK_SIGN_SEARCH_{self.park_search_attempt}'
                self.park_sign_search_permanently_disabled = True
                self.park_search_started_at = None

                self.get_logger().warn(
                    '\n[DIŞ LEVHA NODE -> PARK BULUNDU]\n'
                    f'Arama: {self.park_search_attempt}/2\n'
                    f'Tespit süresi: {elapsed:.1f} sn\n'
                    f'Levha ortalama map: X={avg_x:.3f}, Y={avg_y:.3f}\n'
                    f'PARK YASAK slotlar: {sorted(self.no_parking_slots)}\n'
                    f'Seçilen en yakın park: #{selected_index}\n'
                    f'Levha -> park mesafesi: {selected_distance:.2f} m'
                )
                self.state = self.STATE_WAIT_PARK_SERVERS
                return

        # 1. 10 saniye bitti -> 2. waypoint.
        if self.park_search_attempt == 1:
            self.park_search_started_at = None
            self.reset_park_confirmation()
            self.send_second_search_goal()
            return

        # 2. 10 saniye de bitti -> RANDOM YOK, sabit park.
        if self.park_search_attempt == 2:
            self.park_sign_search_permanently_disabled = True
            self.park_search_started_at = None

            if FIXED_FALLBACK_PARK_INDEX not in PARK_NOKTALARI:
                self.finish_with_error(
                    f'FIXED_FALLBACK_PARK_INDEX geçersiz: #{FIXED_FALLBACK_PARK_INDEX}'
                )
                return

            if FIXED_FALLBACK_PARK_INDEX in self.no_parking_slots:
                self.finish_with_error(
                    f'Sabit fallback park #{FIXED_FALLBACK_PARK_INDEX}, PARK YASAK olarak işaretlendi.'
                )
                return

            self.selected_park_index = FIXED_FALLBACK_PARK_INDEX
            self.selection_reason = 'FIXED_FALLBACK_AFTER_TWO_SCANS'

            self.get_logger().warn(
                '\n[2. TARAMADA DA PARK YOK - SABİT FALLBACK]\n'
                f'Tarama süresi: {elapsed:.1f} sn\n'
                'Random seçim KALDIRILDI.\n'
                f'Gidilecek sabit park: #{FIXED_FALLBACK_PARK_INDEX}'
            )
            self.state = self.STATE_WAIT_PARK_SERVERS
            return

        self.finish_with_error(
            f'Beklenmeyen park levhası arama durumu: {self.park_search_attempt}'
        )

    # ========================================================
    # DIŞ LEVHA TESPİT NODE'U
    # ========================================================

    @staticmethod
    def normalize_sign_class_name(class_name):
        name = str(class_name).strip().lower()
        translations = str.maketrans({
            'ı': 'i', 'ş': 's', 'ğ': 'g', 'ü': 'u', 'ö': 'o', 'ç': 'c',
        })
        name = name.translate(translations)
        name = name.replace('_', ' ').replace('-', ' ')
        return ' '.join(name.split())

    def is_no_parking_class(self, class_name):
        name = self.normalize_sign_class_name(class_name)
        if name in NO_PARKING_CLASS_ALIASES:
            return True
        return ('park' in name and 'yasak' in name) or ('no parking' in name)

    def reset_park_confirmation(self):
        self.park_detection_count = 0
        self.park_detection_points = []

    def detected_signs_callback(self, msg):
        # Bu topic mevcut levha kodunun genel çıktısıdır: {"park": mesafe, ...}
        # Park seçimi için map koordinatı gereken asıl veri /detected_parking_sign'dan gelir.
        if self.state != self.STATE_WAIT_PARK_SIGN:
            return
        if self.park_sign_search_permanently_disabled:
            return

        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(data, dict):
            return

        for class_name in data.keys():
            normalized = self.normalize_sign_class_name(class_name)
            if 'park' in normalized and not self.is_no_parking_class(class_name):
                now = time.monotonic()
                if now - self.last_generic_park_log >= 1.0:
                    self.last_generic_park_log = now
                    self.get_logger().info(
                        '[DIŞ LEVHA TOPIC] PARK görüldü; map koordinatlı detay bekleniyor.'
                    )
                break

    def detected_parking_sign_callback(self, msg):
        if self.state != self.STATE_WAIT_PARK_SIGN:
            return
        if self.park_sign_search_permanently_disabled:
            return

        try:
            data = json.loads(msg.data)
        except Exception as error:
            self.get_logger().warn(f'[PARK DETAIL JSON] Parse hatası: {error}')
            return

        if not isinstance(data, dict):
            return

        class_name = data.get('class_name', data.get('class', ''))
        normalized = self.normalize_sign_class_name(class_name)

        try:
            map_x = float(data['map_x'])
            map_y = float(data['map_y'])
        except (KeyError, TypeError, ValueError):
            return

        distance = data.get('distance', None)
        confidence = data.get('confidence', None)

        # PARK YASAK detayları da dış detector'dan alınır.
        if self.is_no_parking_class(class_name):
            forbidden_index, slot_distance = self.find_nearest_park(map_x, map_y)
            if forbidden_index is None:
                return
            self.no_parking_detection_points.append((map_x, map_y, forbidden_index))
            if forbidden_index not in self.no_parking_slots:
                self.no_parking_slots.add(forbidden_index)
                self.get_logger().warn(
                    '\n[DIŞ LEVHA -> PARK YASAK]\n'
                    f'Levha map=({map_x:.2f}, {map_y:.2f})\n'
                    f'Elenecek park: #{forbidden_index}\n'
                    f'Levha -> slot: {slot_distance:.2f} m'
                )
            return

        if 'park' not in normalized:
            return

        self.park_detection_count += 1
        self.park_detection_points.append((map_x, map_y))

        extra = []
        if confidence is not None:
            extra.append(f'conf={confidence}')
        if distance is not None:
            extra.append(f'dist={distance}m')

        self.get_logger().warn(
            f'[DIŞ PARK TESPİTİ] #{self.park_detection_count} | '
            f'map=({map_x:.2f}, {map_y:.2f})'
            + ((' | ' + ' | '.join(extra)) if extra else '')
        )

        if self.park_detection_count >= PARK_REQUIRED_DETECTIONS:
            self.finish_park_sign_scan_and_select()

    def find_nearest_park(self, sign_x, sign_y, excluded=None):
        excluded = set() if excluded is None else set(excluded)
        best_index = None
        best_distance = float('inf')

        for park_index, park in PARK_NOKTALARI.items():
            if park_index in excluded:
                continue
            distance = math.hypot(float(park['x']) - sign_x, float(park['y']) - sign_y)
            if distance < best_distance:
                best_distance = distance
                best_index = park_index

        return best_index, best_distance

    # ========================================================
    # TRIGGER ALANI HIZ YÖNETİMİ
    # ========================================================

    def make_followpath_speed_request(self, desired_speed):
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                'FollowPath.desired_linear_vel',
                Parameter.Type.DOUBLE,
                float(desired_speed),
            ).to_parameter_msg(),
        ]
        return request

    def request_park_area_speed(self):
        if self.park_area_speed_active or self.park_area_speed_request_pending:
            return
        if not self.set_params_client.service_is_ready():
            return

        self.park_area_speed_request_pending = True
        request = self.make_followpath_speed_request(PARK_AREA_FOLLOWPATH_DESIRED_LINEAR_VEL)
        future = self.set_params_client.call_async(request)
        future.add_done_callback(self.park_area_speed_result_callback)

    def park_area_speed_result_callback(self, future):
        self.park_area_speed_request_pending = False
        try:
            response = future.result()
        except Exception as error:
            self.finish_with_error(f'Park alanı hız parametresi gönderilemedi: {error}')
            return

        successful, reason = self.parameter_response_successful(response, expected_count=1)
        if not successful:
            self.finish_with_error(f'Park alanı hızı ayarlanamadı: {reason}')
            return

        self.park_area_speed_active = True
        self.get_logger().warn(
            f'[PARK ALANI HIZI AKTİF] FollowPath.desired_linear_vel = '
            f'{PARK_AREA_FOLLOWPATH_DESIRED_LINEAR_VEL:.2f} m/s'
        )

    def restore_normal_followpath_speed(self):
        if not self.park_area_speed_active:
            return self.restore_speed_future
        if self.restore_speed_request_pending:
            return self.restore_speed_future
        if not self.set_params_client.service_is_ready():
            return None

        self.restore_speed_request_pending = True
        request = self.make_followpath_speed_request(NORMAL_FOLLOWPATH_DESIRED_LINEAR_VEL)
        self.restore_speed_future = self.set_params_client.call_async(request)
        self.restore_speed_future.add_done_callback(self.restore_normal_speed_result_callback)
        return self.restore_speed_future

    def restore_normal_speed_result_callback(self, future):
        self.restore_speed_request_pending = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'Normal FollowPath hızı geri yüklenemedi: {error}')
            return

        successful, reason = self.parameter_response_successful(response, expected_count=1)
        if not successful:
            self.get_logger().error(f'Normal FollowPath hızı geri yüklenemedi: {reason}')
            return

        self.park_area_speed_active = False
        self.get_logger().info(
            f'Normal FollowPath hızı geri yüklendi: {NORMAL_FOLLOWPATH_DESIRED_LINEAR_VEL:.2f} m/s'
        )

    # CÜLLOP PARK: SERVER HAZIRLIK + TOLERANCE
    # ========================================================

    def try_start_selected_parking(self):
        if self.parking_started or self.parking_finished:
            return

        if self.selected_park_index not in PARK_NOKTALARI:
            self.finish_with_error(
                f'Geçersiz park index: {self.selected_park_index}'
            )
            return

        if not self.odom_received:
            return

        planner_ready = self.planner_client.wait_for_server(timeout_sec=0.05)
        controller_ready = self.controller_client.wait_for_server(timeout_sec=0.05)
        param_ready = self.set_params_client.service_is_ready()

        missing = []
        if not planner_ready:
            missing.append('compute_path_through_poses')
        if not controller_ready:
            missing.append('follow_path')
        if not param_ready:
            missing.append('/controller_server/set_parameters')

        if missing:
            return

        self.parking_started = True
        self.state = self.STATE_PARK_TOLERANCE

        self.get_logger().warn(
            '\n[CÜLLOP PARK BAŞLATILIYOR]\n'
            f'Seçilen park: #{self.selected_park_index}\n'
            f'Seçim nedeni: {self.selection_reason}\n'
            f'Planner: {PLANNER_ID}\n'
            f'Controller: {CONTROLLER_ID}'
        )

        self.request_park_goal_tolerance()

    def make_goal_tolerance_request(self, xy_tolerance, yaw_tolerance):
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                'general_goal_checker.xy_goal_tolerance',
                Parameter.Type.DOUBLE,
                float(xy_tolerance),
            ).to_parameter_msg(),
            Parameter(
                'general_goal_checker.yaw_goal_tolerance',
                Parameter.Type.DOUBLE,
                float(yaw_tolerance),
            ).to_parameter_msg(),
        ]
        return request

    @staticmethod
    def parameter_response_successful(response, expected_count=None):
        if response is None:
            return False, 'SetParameters cevabı boş.'

        if expected_count is not None and len(response.results) != expected_count:
            return (
                False,
                f'{expected_count} parametre sonucu bekleniyordu, {len(response.results)} geldi.',
            )

        for result in response.results:
            if not result.successful:
                return False, result.reason

        return True, ''

    def request_park_goal_tolerance(self):
        if self.tolerance_request_pending:
            return

        self.tolerance_request_pending = True
        request = self.make_goal_tolerance_request(
            PARK_XY_GOAL_TOLERANCE,
            PARK_YAW_GOAL_TOLERANCE,
        )

        self.get_logger().info(
            'Park goal tolerance ayarlanıyor: '
            f'XY={PARK_XY_GOAL_TOLERANCE:.2f} m, '
            f'Yaw={PARK_YAW_GOAL_TOLERANCE:.3f} rad'
        )

        future = self.set_params_client.call_async(request)
        future.add_done_callback(self.park_goal_tolerance_result_callback)

    def park_goal_tolerance_result_callback(self, future):
        self.tolerance_request_pending = False

        try:
            response = future.result()
        except Exception as error:
            self.finish_with_error(
                f'Park goal tolerance servisi hata verdi: {error}'
            )
            return

        successful, reason = self.parameter_response_successful(response, expected_count=2)
        if not successful:
            self.finish_with_error(
                f'Park goal tolerance ayarlanamadı: {reason}'
            )
            return

        self.park_tolerance_active = True
        self.get_logger().info(
            'PARK goal tolerance AKTİF: '
            f'XY={PARK_XY_GOAL_TOLERANCE:.2f} m | '
            f'Yaw={PARK_YAW_GOAL_TOLERANCE:.3f} rad'
        )

        self.create_parking_targets(self.selected_park_index)
        self.state = self.STATE_PARK_PLANNING
        self.request_path_through_poses()

    # ========================================================
    # CÜLLOP PARK HEDEFLERİNİ OLUŞTUR
    # ========================================================

    def create_parking_targets(self, park_index):
        goal = PARK_NOKTALARI[park_index]

        goal_x = float(goal['x'])
        goal_y = float(goal['y'])
        goal_z = float(goal['z'])

        final_yaw = self.quaternion_to_yaw(
            float(goal['ox']),
            float(goal['oy']),
            float(goal['oz']),
            float(goal['ow']),
        )

        heading_x = math.cos(final_yaw)
        heading_y = math.sin(final_yaw)

        # Tüm parklarda düzleşme pozu.
        align_x = goal_x - DUZ_GIRIS_MESAFESI * heading_x
        align_y = goal_y - DUZ_GIRIS_MESAFESI * heading_y

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

        self.turn_pose = None
        self.goal_poses = []
        self.goal_labels = []

        # PARK 1-6: viraj hazırlık + düzleşme + final.
        if park_index in UC_ASAMALI_PARK_YERLERI:
            right_x = math.sin(final_yaw)
            right_y = -math.cos(final_yaw)

            turn_x = (
                align_x
                - VIRAJ_HAZIRLIK_MESAFESI * heading_x
                + VIRAJ_SAG_OFSET_MESAFESI * right_x
            )
            turn_y = (
                align_y
                - VIRAJ_HAZIRLIK_MESAFESI * heading_y
                + VIRAJ_SAG_OFSET_MESAFESI * right_y
            )
            turn_yaw = self.normalize_angle(
                final_yaw + math.radians(VIRAJ_HAZIRLIK_YAW_OFFSET_DEG)
            )

            self.turn_pose = self.create_pose(
                turn_x,
                turn_y,
                goal_z,
                turn_yaw,
            )

            self.goal_poses = [
                copy.deepcopy(self.turn_pose),
                copy.deepcopy(self.align_pose),
                copy.deepcopy(self.final_pose),
            ]
            self.goal_labels = [
                'VİRAJ BAŞLANGIÇ POSE',
                'DÜZLEŞME POSE',
                'FINAL PARK POSE',
            ]

        # PARK 7-8: düzleşme + final.
        elif park_index in IKI_ASAMALI_PARK_YERLERI:
            self.goal_poses = [
                copy.deepcopy(self.align_pose),
                copy.deepcopy(self.final_pose),
            ]
            self.goal_labels = [
                'DÜZLEŞME POSE',
                'FINAL PARK POSE',
            ]
        else:
            self.finish_with_error(f'Park modu tanımsız: #{park_index}')
            return

        self.get_logger().info(
            f'\nPark #{park_index} için {len(self.goal_poses)} yönlü pose oluşturuldu:'
        )

        for index, (label, pose) in enumerate(
            zip(self.goal_labels, self.goal_poses),
            start=1,
        ):
            q = pose.pose.orientation
            yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
            self.get_logger().info(
                f'{index}. {label} | '
                f'X={pose.pose.position.x:.3f} | '
                f'Y={pose.pose.position.y:.3f} | '
                f'Yaw={math.degrees(yaw):.2f}°'
            )

        self.publish_goal_poses()

    def publish_goal_poses(self):
        msg = PoseArray()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = [copy.deepcopy(pose.pose) for pose in self.goal_poses]
        self.goal_poses_pub.publish(msg)

    # ========================================================
    # COMPUTE PATH THROUGH POSES
    # ========================================================

    def request_path_through_poses(self):
        if len(self.goal_poses) < 2:
            self.finish_with_error('Planner için yeterli goal pose yok.')
            return

        now = self.get_clock().now().to_msg()
        for pose in self.goal_poses:
            pose.header.frame_id = MAP_FRAME
            pose.header.stamp = now

        goal = ComputePathThroughPoses.Goal()
        goal.goals = [copy.deepcopy(pose) for pose in self.goal_poses]
        goal.planner_id = PLANNER_ID

        # Başlangıç pozunu Nav2 TF'den alır.
        goal.use_start = False

        route_text = '\n'.join(
            f'{index} -> {label}'
            for index, label in enumerate(self.goal_labels, start=1)
        )

        self.get_logger().info(
            '\nComputePathThroughPoses gönderiliyor.\n'
            f'Planner: {PLANNER_ID}\n'
            f'Park: #{self.selected_park_index}\n'
            f'Pose sayısı: {len(self.goal_poses)}\n'
            f'{route_text}'
        )

        future = self.planner_client.send_goal_async(goal)
        future.add_done_callback(self.planner_goal_response_callback)

    def planner_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.finish_with_error(f'Planner goal gönderilemedi: {error}')
            return

        if not goal_handle.accepted:
            self.finish_with_error(
                'ComputePathThroughPoses planner goal reddedildi.'
            )
            return

        self.get_logger().info(
            'ComputePathThroughPoses planner goal kabul edildi.'
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.planner_result_callback)

    def planner_result_callback(self, future):
        try:
            wrapped_result = future.result()
        except Exception as error:
            self.finish_with_error(f'Planner sonucu alınamadı: {error}')
            return

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            result = wrapped_result.result
            error_code = getattr(
                result,
                'error_code',
                'Humble sürümünde mevcut değil',
            )
            error_msg = getattr(result, 'error_msg', '')
            self.finish_with_error(
                'ComputePathThroughPoses başarısız. '
                f'Action status={wrapped_result.status}, '
                f'error_code={error_code}, error={error_msg}'
            )
            return

        path = wrapped_result.result.path
        if path is None or len(path.poses) < 2:
            self.finish_with_error('Planner geçerli path oluşturamadı.')
            return

        self.planned_path = path
        self.planner_path_pub.publish(path)

        self.get_logger().info(
            '\nPlanner path oluşturuldu.\n'
            f'Goal pose sayısı: {len(self.goal_poses)}\n'
            f'Path pose sayısı: {len(path.poses)}'
        )

        self.send_path_to_controller(path)

    # ========================================================
    # FOLLOW PATH
    # ========================================================

    def send_path_to_controller(self, path):
        if path is None or len(path.poses) < 2:
            self.finish_with_error('Controller için geçerli path yok.')
            return

        path.header.stamp = self.get_clock().now().to_msg()
        if not path.header.frame_id:
            path.header.frame_id = MAP_FRAME

        for pose in path.poses:
            pose.header.frame_id = path.header.frame_id
            pose.header.stamp = path.header.stamp

        self.follow_path_pub.publish(path)

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = CONTROLLER_ID
        goal.goal_checker_id = GOAL_CHECKER_ID

        if hasattr(goal, 'progress_checker_id'):
            goal.progress_checker_id = PROGRESS_CHECKER_ID

        self.state = self.STATE_PARK_FOLLOWING

        self.get_logger().info(
            '\nPath controller\'a gönderiliyor.\n'
            f'Controller: {CONTROLLER_ID}\n'
            f'Goal checker: {GOAL_CHECKER_ID}'
        )

        future = self.controller_client.send_goal_async(
            goal,
            feedback_callback=self.controller_feedback_callback,
        )
        future.add_done_callback(self.controller_goal_response_callback)

    def controller_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.finish_with_error(f'Controller goal gönderilemedi: {error}')
            return

        if not goal_handle.accepted:
            self.finish_with_error('FollowPath controller goal reddedildi.')
            return

        self.get_logger().info(
            f'Park #{self.selected_park_index} yolu '
            f'{CONTROLLER_ID} tarafından kabul edildi.'
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.controller_result_callback)

    def controller_feedback_callback(self, feedback_msg):
        self.feedback_counter += 1
        if self.feedback_counter % 10 != 0:
            return

        feedback = feedback_msg.feedback
        distance = getattr(feedback, 'distance_to_goal', None)
        speed = getattr(feedback, 'speed', None)

        texts = []
        if distance is not None:
            texts.append(f'Kalan mesafe: {distance:.2f} m')
        if speed is not None:
            texts.append(f'Hız: {speed:.2f} m/s')
        if texts:
            self.get_logger().info(' | '.join(texts))

    def controller_result_callback(self, future):
        try:
            wrapped_result = future.result()
        except Exception as error:
            self.finish_with_error(f'Controller sonucu alınamadı: {error}')
            return

        status = wrapped_result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.parking_finished = True
            self.state = self.STATE_DONE
            self.get_logger().warn(
                '\n===========================================\n'
                f'PARK #{self.selected_park_index} BAŞARIYLA TAMAMLANDI\n'
                f'Seçim nedeni: {self.selection_reason}\n'
                f'Pose sayısı: {len(self.goal_poses)}\n'
                f'Planner/Controller: {PLANNER_ID} / {CONTROLLER_ID}\n'
                '==========================================='
            )
            self.restore_normal_goal_tolerance()
            self.restore_normal_followpath_speed()
            return

        if status == GoalStatus.STATUS_CANCELED:
            self.parking_finished = True
            self.state = self.STATE_ERROR
            self.get_logger().warn('Park FollowPath iptal edildi.')
            self.restore_normal_goal_tolerance()
            self.restore_normal_followpath_speed()
            return

        result = wrapped_result.result
        error_code = getattr(result, 'error_code', 'bilinmiyor')
        error_msg = getattr(result, 'error_msg', '')

        self.parking_finished = True
        self.state = self.STATE_ERROR
        self.get_logger().error(
            'Controller path takip başarısız. '
            f'status={status}, error_code={error_code}, error={error_msg}'
        )
        self.restore_normal_goal_tolerance()
        self.restore_normal_followpath_speed()

    # ========================================================
    # NORMAL GOAL TOLERANCE GERİ YÜKLE
    # ========================================================

    def restore_normal_goal_tolerance(self):
        if not self.park_tolerance_active:
            return self.restore_future

        if self.restore_request_pending:
            return self.restore_future

        if not self.set_params_client.service_is_ready():
            self.get_logger().warn(
                'controller_server set_parameters servisi hazır değil; '
                'normal tolerance geri yüklenemedi.'
            )
            return None

        self.restore_request_pending = True

        request = self.make_goal_tolerance_request(
            NORMAL_XY_GOAL_TOLERANCE,
            NORMAL_YAW_GOAL_TOLERANCE,
        )

        self.restore_future = self.set_params_client.call_async(request)
        self.restore_future.add_done_callback(
            self.restore_goal_tolerance_result_callback
        )
        return self.restore_future

    def restore_goal_tolerance_result_callback(self, future):
        self.restore_request_pending = False

        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f'Normal goal tolerance geri yüklenirken hata: {error}'
            )
            return

        successful, reason = self.parameter_response_successful(response, expected_count=2)
        if not successful:
            self.get_logger().error(
                f'Normal goal tolerance geri yüklenemedi: {reason}'
            )
            return

        self.park_tolerance_active = False
        self.get_logger().info(
            'NORMAL goal tolerance geri yüklendi: '
            f'XY={NORMAL_XY_GOAL_TOLERANCE:.2f} m | '
            f'Yaw={NORMAL_YAW_GOAL_TOLERANCE:.3f} rad'
        )

    # ========================================================
    # HATA
    # ========================================================

    def finish_with_error(self, message):
        self.get_logger().error(message)
        self.parking_finished = True
        self.park_sign_search_permanently_disabled = True
        self.state = self.STATE_ERROR
        self.restore_normal_goal_tolerance()
        self.restore_normal_followpath_speed()


def main(args=None):
    rclpy.init(args=args)
    node = ExternalSignParkingManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('External levha + cüllop park node kapatılıyor.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
