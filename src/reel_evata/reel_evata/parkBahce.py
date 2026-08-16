#!/usr/bin/env python3

import copy
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import ComputePathToPose, FollowPath
from rcl_interfaces.srv import SetParameters


# ============================================================
# KULLANICI AYARLARI
# ============================================================

# 1 - 7 arasında kullanılacak park yeri
PARK_YERI = 5

HEDEF_FRAME = 'map'
ODOM_TOPIC = '/odom'

# Final park noktasından önce oluşturulan düzleşme pozu
DUZ_GIRIS_MESAFESI = 3.0


# ============================================================
# PARK İÇİN NAV2 PROFİLLERİ
# ============================================================

# YAML:
#
# ParkingGrid:
#   motion_model_for_search: "REEDS_SHEPP"
#
# ParkFollowPath:
#   allow_reversing: true
#
PLANNER_ID = 'ParkingGrid'
CONTROLLER_ID = 'ParkFollowPath'

# Artık tek goal checker kullanıyoruz.
GOAL_CHECKER_ID = 'general_goal_checker'

PROGRESS_CHECKER_ID = ''

SERVER_KONTROL_SURESI = 0.5


# ============================================================
# GOAL TOLERANCE
# ============================================================

# NORMAL NAVİGASYON
NORMAL_XY_GOAL_TOLERANCE = 1.0
NORMAL_YAW_GOAL_TOLERANCE = 0.7


# PARK
#
# 15 cm konum toleransı
# 0.05 rad ≈ 2.86 derece yaw toleransı
#
PARK_XY_GOAL_TOLERANCE = 0.20
PARK_YAW_GOAL_TOLERANCE = 0.1


# ============================================================
# PARK NOKTALARI
# ============================================================

PARK_NOKTALARI = {

    1: {
        'x': -0.13177430629730225,
        'y': -26.073577880859375,
        'z': 0.0,

        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.7410746945859217,
        'ow': 0.6714225919973076,
    },

    2: {
        'x': 2.2548341751098633,
        'y': -26.556270599365234,
        'z': 0.0,

        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.7401743133961566,
        'ow': 0.6724150398292175,
    },

    3: {
        'x': 4.65485143661499,
        'y': -26.811031341552734,
        'z': 0.0,

        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.7449845281495763,
        'ow': 0.6670817437299219,
    },

    4: {
        'x': 7.242579460144043,
        'y': -27.481422424316406,
        'z': 0.0,

        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.7565551210266418,
        'ow': 0.6539299265581622,
    },

    5: {
        'x': 9.790084838867188,
        'y': -27.535062789916992,
        'z': 0.0,

        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.7445136677023241,
        'ow': 0.6676072188079105,
    },

    6: {
        'x': 12.310771942138672,
        'y': -27.870264053344727,
        'z': 0.0,

        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.7535441785355289,
        'ow': 0.6573972702979645,
    },

    7: {
        'x': 14.764419555664062,
        'y': -28.20547103881836,
        'z': 0.0,

        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.7529680222193897,
        'ow': 0.6580571080955061,
    },
}


# ============================================================
# PARK NODE
# ============================================================

class Nav2TwoStageParking(Node):

    def __init__(self):

        super().__init__(
            'nav2_two_stage_parking'
        )


        # ====================================================
        # PLANNER
        # ====================================================

        self.planner_client = ActionClient(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
        )


        # ====================================================
        # CONTROLLER
        # ====================================================

        self.controller_client = ActionClient(
            self,
            FollowPath,
            'follow_path',
        )


        # ====================================================
        # CONTROLLER SERVER PARAMETER CLIENT
        #
        # Park başında:
        #
        # XY  = 0.15
        # Yaw = 0.05
        #
        # Park sonunda:
        #
        # XY  = 1.0
        # Yaw = 0.7
        #
        # ====================================================

        self.set_params_client = self.create_client(
            SetParameters,
            '/controller_server/set_parameters',
        )


        # ====================================================
        # RVIZ PATH PUBLISHERS
        # ====================================================

        path_qos = QoSProfile(
            depth=1
        )

        path_qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
        )


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


        # ====================================================
        # ODOM
        # ====================================================

        self.odom_sub = self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self.odom_callback,
            20,
        )


        # ====================================================
        # AYAR KONTROLLERİ
        # ====================================================

        if PARK_YERI not in PARK_NOKTALARI:

            raise ValueError(
                'PARK_YERI 1 ile 7 arasında olmalıdır.'
            )


        if DUZ_GIRIS_MESAFESI <= 0.5:

            raise ValueError(
                'DUZ_GIRIS_MESAFESI '
                '0.5 metreden büyük olmalıdır.'
            )


        # ====================================================
        # RUNTIME
        # ====================================================

        self.current_pose = None

        self.align_pose = None

        self.final_pose = None


        self.approach_path = None

        self.straight_path = None

        self.raw_full_path = None


        self.started = False

        self.finished = False

        self.feedback_counter = 0


        # Goal tolerance değişimi devam ediyor mu?
        self.tolerance_request_pending = False


        # Park toleransı şu anda aktif mi?
        self.park_tolerance_active = False


        # Normal toleransa dönüş isteği gönderildi mi?
        self.restore_request_pending = False

        self.restore_future = None


        # ====================================================
        # SERVER KONTROL TIMER
        # ====================================================

        self.timer = self.create_timer(
            SERVER_KONTROL_SURESI,
            self.wait_for_ready,
        )


        self.get_logger().info(

            '\n'
            'İki aşamalı Nav2 park sistemi başlatıldı.\n'

            f'Park yeri: {PARK_YERI}\n'

            f'Hedef frame: {HEDEF_FRAME}\n'

            f'Düz giriş mesafesi: '
            f'{DUZ_GIRIS_MESAFESI:.1f} m\n'

            f'Planner: {PLANNER_ID}\n'

            f'Controller: {CONTROLLER_ID}\n'

            f'Goal checker: {GOAL_CHECKER_ID}\n'

            f'Park XY toleransı: '
            f'{PARK_XY_GOAL_TOLERANCE:.2f} m\n'

            f'Park yaw toleransı: '
            f'{PARK_YAW_GOAL_TOLERANCE:.3f} rad '
            f'('
            f'{math.degrees(PARK_YAW_GOAL_TOLERANCE):.1f}°'
            f')\n'

            'Harici SimpleSmoother: KAPALI'
        )


    # ========================================================
    # QUATERNION -> YAW
    # ========================================================

    @staticmethod
    def quaternion_to_yaw(
        x,
        y,
        z,
        w,
    ):

        siny_cosp = (
            2.0
            * (
                w * z
                + x * y
            )
        )


        cosy_cosp = (
            1.0
            - 2.0
            * (
                y * y
                + z * z
            )
        )


        return math.atan2(
            siny_cosp,
            cosy_cosp,
        )


    # ========================================================
    # YAW -> QUATERNION
    # ========================================================

    @staticmethod
    def yaw_to_quaternion(
        yaw
    ):

        return (
            0.0,
            0.0,

            math.sin(
                yaw / 2.0
            ),

            math.cos(
                yaw / 2.0
            ),
        )


    # ========================================================
    # POSE OLUŞTUR
    # ========================================================

    def create_pose(
        self,
        x,
        y,
        z,
        yaw,
    ):

        pose = PoseStamped()


        pose.header.frame_id = (
            HEDEF_FRAME
        )


        pose.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )


        pose.pose.position.x = (
            float(x)
        )

        pose.pose.position.y = (
            float(y)
        )

        pose.pose.position.z = (
            float(z)
        )


        qx, qy, qz, qw = (
            self.yaw_to_quaternion(
                yaw
            )
        )


        pose.pose.orientation.x = qx

        pose.pose.orientation.y = qy

        pose.pose.orientation.z = qz

        pose.pose.orientation.w = qw


        return pose


    # ========================================================
    # ODOM
    # ========================================================

    def odom_callback(
        self,
        msg,
    ):

        self.current_pose = {

            'x':
                float(
                    msg.pose.pose.position.x
                ),

            'y':
                float(
                    msg.pose.pose.position.y
                ),

            'z':
                float(
                    msg.pose.pose.position.z
                ),

            'yaw':
                self.quaternion_to_yaw(

                    msg.pose.pose.orientation.x,

                    msg.pose.pose.orientation.y,

                    msg.pose.pose.orientation.z,

                    msg.pose.pose.orientation.w,
                ),

            'frame_id':
                msg.header.frame_id,
        }


    # ========================================================
    # GOAL TOLERANCE REQUEST OLUŞTUR
    # ========================================================

    def make_goal_tolerance_request(
        self,
        xy_tolerance,
        yaw_tolerance,
    ):

        request = (
            SetParameters.Request()
        )


        request.parameters = [

            Parameter(
                'general_goal_checker.xy_goal_tolerance',
                Parameter.Type.DOUBLE,
                float(
                    xy_tolerance
                ),
            ).to_parameter_msg(),


            Parameter(
                'general_goal_checker.yaw_goal_tolerance',
                Parameter.Type.DOUBLE,
                float(
                    yaw_tolerance
                ),
            ).to_parameter_msg(),
        ]


        return request


    # ========================================================
    # PARAMETRE CEVABINI KONTROL ET
    # ========================================================

    @staticmethod
    def parameter_response_successful(
        response
    ):

        if response is None:

            return (
                False,
                'SetParameters cevabı boş.'
            )


        if len(
            response.results
        ) != 2:

            return (
                False,
                'Beklenen 2 parametre sonucu yerine '
                f'{len(response.results)} sonuç geldi.'
            )


        for result in response.results:

            if not result.successful:

                return (
                    False,
                    result.reason,
                )


        return (
            True,
            ''
        )


    # ========================================================
    # PARK TOLERANSINI AKTİF ET
    # ========================================================

    def request_park_goal_tolerance(
        self
    ):

        if self.tolerance_request_pending:

            return


        self.tolerance_request_pending = (
            True
        )


        request = (
            self.make_goal_tolerance_request(

                PARK_XY_GOAL_TOLERANCE,

                PARK_YAW_GOAL_TOLERANCE,
            )
        )


        self.get_logger().info(

            'Park goal tolerance ayarlanıyor: '

            f'XY='
            f'{PARK_XY_GOAL_TOLERANCE:.2f} m, '

            f'Yaw='
            f'{PARK_YAW_GOAL_TOLERANCE:.3f} rad '

            f'('
            f'{math.degrees(PARK_YAW_GOAL_TOLERANCE):.1f}°'
            f')'
        )


        future = (
            self.set_params_client
            .call_async(
                request
            )
        )


        future.add_done_callback(
            self.park_goal_tolerance_result_callback
        )


    # ========================================================
    # PARK TOLERANSI CEVABI
    # ========================================================

    def park_goal_tolerance_result_callback(
        self,
        future,
    ):

        self.tolerance_request_pending = (
            False
        )


        try:

            response = (
                future.result()
            )


        except Exception as error:

            self.get_logger().error(

                'Park goal tolerance servisi '
                'hata verdi: '

                f'{error}'
            )

            return


        successful, reason = (
            self.parameter_response_successful(
                response
            )
        )


        if not successful:

            self.get_logger().error(

                'Park goal tolerance '
                'ayarlanamadı: '

                f'{reason}'
            )

            return


        # ====================================================
        # PARK TOLERANSI BAŞARIYLA AKTİF
        # ====================================================

        self.park_tolerance_active = (
            True
        )


        self.get_logger().info(

            'PARK goal tolerance AKTİF: '

            f'XY='
            f'{PARK_XY_GOAL_TOLERANCE:.2f} m | '

            f'Yaw='
            f'{PARK_YAW_GOAL_TOLERANCE:.3f} rad '

            f'('
            f'{math.degrees(PARK_YAW_GOAL_TOLERANCE):.1f}°'
            f')'
        )


        # ====================================================
        # TOLERANS AYARLANDIKTAN SONRA PARK BAŞLA
        # ====================================================

        self.create_targets()


        self.started = True


        if self.timer is not None:

            self.timer.cancel()


        self.request_approach_path()


    # ========================================================
    # NORMAL TOLERANSI GERİ YÜKLE
    # ========================================================

    def restore_normal_goal_tolerance(
        self
    ):

        # Zaten normal ayardaysa bir şey yapma.
        if not self.park_tolerance_active:

            return self.restore_future


        # Zaten restore isteği devam ediyorsa
        # tekrar istek gönderme.
        if self.restore_request_pending:

            return self.restore_future


        if not (
            self.set_params_client
            .service_is_ready()
        ):

            self.get_logger().warn(

                'controller_server '
                'set_parameters servisi hazır değil. '
                'Normal goal tolerance '
                'henüz geri yüklenemedi.'
            )

            return None


        self.restore_request_pending = (
            True
        )


        request = (
            self.make_goal_tolerance_request(

                NORMAL_XY_GOAL_TOLERANCE,

                NORMAL_YAW_GOAL_TOLERANCE,
            )
        )


        self.restore_future = (
            self.set_params_client
            .call_async(
                request
            )
        )


        self.restore_future.add_done_callback(
            self.restore_goal_tolerance_result_callback
        )


        return self.restore_future


    # ========================================================
    # NORMAL TOLERANS RESTORE CEVABI
    # ========================================================

    def restore_goal_tolerance_result_callback(
        self,
        future,
    ):

        self.restore_request_pending = (
            False
        )


        try:

            response = (
                future.result()
            )


        except Exception as error:

            self.get_logger().error(

                'Normal goal tolerance '
                'geri yüklenirken hata: '

                f'{error}'
            )

            return


        successful, reason = (
            self.parameter_response_successful(
                response
            )
        )


        if not successful:

            self.get_logger().error(

                'Normal goal tolerance '
                'geri yüklenemedi: '

                f'{reason}'
            )

            return


        self.park_tolerance_active = (
            False
        )


        self.get_logger().info(

            'NORMAL goal tolerance '
            'geri yüklendi: '

            f'XY='
            f'{NORMAL_XY_GOAL_TOLERANCE:.2f} m | '

            f'Yaw='
            f'{NORMAL_YAW_GOAL_TOLERANCE:.3f} rad '

            f'('
            f'{math.degrees(NORMAL_YAW_GOAL_TOLERANCE):.1f}°'
            f')'
        )


    # ========================================================
    # PARK HEDEFLERİNİ OLUŞTUR
    # ========================================================

    def create_targets(
        self
    ):

        goal = (
            PARK_NOKTALARI[
                PARK_YERI
            ]
        )


        goal_x = float(
            goal['x']
        )

        goal_y = float(
            goal['y']
        )

        goal_z = float(
            goal['z']
        )


        # ====================================================
        # PARK YAW
        # ====================================================

        final_yaw = (
            self.quaternion_to_yaw(

                float(
                    goal['ox']
                ),

                float(
                    goal['oy']
                ),

                float(
                    goal['oz']
                ),

                float(
                    goal['ow']
                ),
            )
        )


        final_yaw_deg = (
            math.degrees(
                final_yaw
            )
        )


        # ====================================================
        # PARK YÖN VEKTÖRÜ
        # ====================================================

        heading_x = (
            math.cos(
                final_yaw
            )
        )


        heading_y = (
            math.sin(
                final_yaw
            )
        )


        # ====================================================
        # DÜZLEŞME POZU
        # ====================================================

        align_x = (

            goal_x

            - DUZ_GIRIS_MESAFESI
            * heading_x
        )


        align_y = (

            goal_y

            - DUZ_GIRIS_MESAFESI
            * heading_y
        )


        self.align_pose = (
            self.create_pose(

                align_x,

                align_y,

                goal_z,

                final_yaw,
            )
        )


        self.final_pose = (
            self.create_pose(

                goal_x,

                goal_y,

                goal_z,

                final_yaw,
            )
        )


        self.get_logger().info(

            '\n'
            'Nav2 park hedefleri:\n'

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
    # NAV2 SERVER HAZIRLIĞI
    # ========================================================

    def wait_for_ready(
        self
    ):

        if self.started:

            return


        if self.finished:

            return


        if self.tolerance_request_pending:

            return


        # ====================================================
        # ODOM
        # ====================================================

        if self.current_pose is None:

            self.get_logger().info(

                f'{ODOM_TOPIC} '
                'verisi bekleniyor...'
            )

            return


        # ====================================================
        # PLANNER
        # ====================================================

        planner_ready = (
            self.planner_client
            .wait_for_server(
                timeout_sec=0.05,
            )
        )


        # ====================================================
        # CONTROLLER
        # ====================================================

        controller_ready = (
            self.controller_client
            .wait_for_server(
                timeout_sec=0.05,
            )
        )


        # ====================================================
        # PARAMETER SERVICE
        # ====================================================

        param_service_ready = (
            self.set_params_client
            .service_is_ready()
        )


        missing = []


        if not planner_ready:

            missing.append(
                'compute_path_to_pose'
            )


        if not controller_ready:

            missing.append(
                'follow_path'
            )


        if not param_service_ready:

            missing.append(
                '/controller_server/set_parameters'
            )


        if missing:

            self.get_logger().info(

                'Beklenen servis/action: '

                + ', '.join(
                    missing
                )
            )

            return


        # ====================================================
        # ÖNEMLİ
        #
        # Önce park goal tolerance daraltılır.
        #
        # Başarılı callback geldikten sonra
        # planner başlatılır.
        # ====================================================

        self.request_park_goal_tolerance()


    # ========================================================
    # SEGMENT 1
    #
    # MEVCUT ARAÇ
    # ->
    # DÜZLEŞME POZU
    # ========================================================

    def request_approach_path(
        self
    ):

        goal = (
            ComputePathToPose.Goal()
        )


        goal.goal = copy.deepcopy(
            self.align_pose
        )


        goal.planner_id = (
            PLANNER_ID
        )


        # Başlangıcı Nav2 TF'den alsın.
        goal.use_start = False


        self.get_logger().info(

            'Planner segment 1 hesaplıyor: '

            'mevcut araç pozu '
            '-> düzleşme pozu '

            f'[{PLANNER_ID}]'
        )


        future = (
            self.planner_client
            .send_goal_async(
                goal
            )
        )


        future.add_done_callback(
            self.approach_goal_response_callback
        )


    # ========================================================
    # SEGMENT 1 GOAL RESPONSE
    # ========================================================

    def approach_goal_response_callback(
        self,
        future,
    ):

        try:

            goal_handle = (
                future.result()
            )


        except Exception as error:

            self.finish_with_error(

                'Yaklaşma planner hedefi '
                'gönderilemedi: '

                f'{error}'
            )

            return


        if not goal_handle.accepted:

            self.finish_with_error(

                'Yaklaşma planner '
                'hedefi reddedildi.'
            )

            return


        result_future = (
            goal_handle
            .get_result_async()
        )


        result_future.add_done_callback(
            self.approach_result_callback
        )


    # ========================================================
    # SEGMENT 1 RESULT
    # ========================================================

    def approach_result_callback(
        self,
        future,
    ):

        try:

            wrapped_result = (
                future.result()
            )


        except Exception as error:

            self.finish_with_error(

                'Yaklaşma planner '
                'sonucu alınamadı: '

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


            self.finish_with_error(

                'Park yaklaşma planner '
                'hatası nedeniyle durdu.',

                log_message=False,
            )

            return


        path = (
            wrapped_result
            .result
            .path
        )


        if (
            path is None
            or len(
                path.poses
            ) < 2
        ):

            self.finish_with_error(

                'Yaklaşma planner '
                'geçersiz yol döndürdü.'
            )

            return


        self.approach_path = (
            path
        )


        self.get_logger().info(

            'Yaklaşma segmenti '
            'oluşturuldu. '

            f'Poz sayısı: '
            f'{len(path.poses)}'
        )


        self.request_straight_path()


    # ========================================================
    # SEGMENT 2
    #
    # DÜZLEŞME
    # ->
    # FINAL PARK
    # ========================================================

    def request_straight_path(
        self
    ):

        goal = (
            ComputePathToPose.Goal()
        )


        goal.start = copy.deepcopy(
            self.align_pose
        )


        goal.goal = copy.deepcopy(
            self.final_pose
        )


        goal.planner_id = (
            PLANNER_ID
        )


        # İkinci segment başlangıcını
        # açıkça align_pose yap.
        goal.use_start = True


        self.get_logger().info(

            'Planner segment 2 hesaplıyor: '

            'düzleşme pozu '
            '-> final park pozu '

            f'[{PLANNER_ID}]'
        )


        future = (
            self.planner_client
            .send_goal_async(
                goal
            )
        )


        future.add_done_callback(
            self.straight_goal_response_callback
        )


    # ========================================================
    # SEGMENT 2 GOAL RESPONSE
    # ========================================================

    def straight_goal_response_callback(
        self,
        future,
    ):

        try:

            goal_handle = (
                future.result()
            )


        except Exception as error:

            self.finish_with_error(

                'Düz segment planner '
                'hedefi gönderilemedi: '

                f'{error}'
            )

            return


        if not goal_handle.accepted:

            self.finish_with_error(

                'Düz segment planner '
                'hedefi reddedildi.'
            )

            return


        result_future = (
            goal_handle
            .get_result_async()
        )


        result_future.add_done_callback(
            self.straight_result_callback
        )


    # ========================================================
    # SEGMENT 2 RESULT
    # ========================================================

    def straight_result_callback(
        self,
        future,
    ):

        try:

            wrapped_result = (
                future.result()
            )


        except Exception as error:

            self.finish_with_error(

                'Düz segment planner '
                'sonucu alınamadı: '

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


            self.finish_with_error(

                'Park final planner '
                'hatası nedeniyle durdu.',

                log_message=False,
            )

            return


        path = (
            wrapped_result
            .result
            .path
        )


        if (
            path is None
            or len(
                path.poses
            ) < 2
        ):

            self.finish_with_error(

                'Düz final planner '
                'geçersiz yol döndürdü.'
            )

            return


        self.straight_path = (
            path
        )


        self.get_logger().info(

            'Düz final segmenti '
            'oluşturuldu. '

            f'Poz sayısı: '
            f'{len(path.poses)}'
        )


        # ====================================================
        # PATH BİRLEŞTİR
        # ====================================================

        self.raw_full_path = (
            self.combine_paths(

                self.approach_path,

                self.straight_path,
            )
        )


        # ====================================================
        # RVIZ
        # ====================================================

        self.raw_path_pub.publish(
            self.raw_full_path
        )


        self.get_logger().info(

            '\n'
            'Tam ham park yolu oluşturuldu.\n'

            'Topic: '
            '/parking/full_raw_path\n'

            f'Frame: '
            f'{self.raw_full_path.header.frame_id}\n'

            f'Poz sayısı: '
            f'{len(self.raw_full_path.poses)}\n'

            'SmoothPath uygulanmayacak.'
        )


        # ====================================================
        # REEDS_SHEPP CUSP'LARI KORU
        #
        # HARİCİ SMOOTHER YOK
        # ====================================================

        self.publish_and_follow(
            self.raw_full_path
        )


    # ========================================================
    # PATH BİRLEŞTİR
    # ========================================================

    def combine_paths(
        self,
        first_path,
        second_path,
    ):

        combined = Path()


        combined.header = (
            copy.deepcopy(
                first_path.header
            )
        )


        combined.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )


        combined.poses = (
            copy.deepcopy(
                first_path.poses
            )
        )


        # align_pose iki path'te de var.
        # İkinci path'in ilk pozunu tekrar ekleme.
        for pose in (
            second_path.poses[1:]
        ):

            combined.poses.append(

                copy.deepcopy(
                    pose
                )
            )


        for pose in (
            combined.poses
        ):

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

    def publish_and_follow(
        self,
        path,
    ):

        if (
            path is None
            or len(
                path.poses
            ) < 2
        ):

            self.finish_with_error(

                'Controller için '
                'geçerli yol yok.'
            )

            return


        # ====================================================
        # HEADER
        # ====================================================

        path.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )


        for pose in (
            path.poses
        ):

            pose.header.frame_id = (
                path.header.frame_id
            )


            pose.header.stamp = (
                path.header.stamp
            )


        # ====================================================
        # RVIZ
        # ====================================================

        self.final_path_pub.publish(
            path
        )


        self.get_logger().info(

            '\n'
            'Controller yolu '
            'RViz için yayınlandı:\n'

            'Topic: '
            '/parking/full_follow_path\n'

            f'Frame: '
            f'{path.header.frame_id}\n'

            f'Poz sayısı: '
            f'{len(path.poses)}\n'

            f'Controller: '
            f'{CONTROLLER_ID}\n'

            f'Goal checker: '
            f'{GOAL_CHECKER_ID}'
        )


        # ====================================================
        # FOLLOW PATH
        # ====================================================

        goal = (
            FollowPath.Goal()
        )


        goal.path = (
            path
        )


        # Park için özel controller
        goal.controller_id = (
            CONTROLLER_ID
        )


        # ====================================================
        # TEK GOAL CHECKER
        #
        # Ama parametreleri park başında:
        #
        # XY  = 0.15
        # Yaw = 0.05
        #
        # olarak değiştirildi.
        # ====================================================

        goal.goal_checker_id = (
            GOAL_CHECKER_ID
        )


        if hasattr(
            goal,
            'progress_checker_id',
        ):

            goal.progress_checker_id = (
                PROGRESS_CHECKER_ID
            )


        future = (
            self.controller_client
            .send_goal_async(

                goal,

                feedback_callback=
                    self.feedback_callback,
            )
        )


        future.add_done_callback(
            self.controller_goal_response_callback
        )


    # ========================================================
    # CONTROLLER GOAL RESPONSE
    # ========================================================

    def controller_goal_response_callback(
        self,
        future,
    ):

        try:

            goal_handle = (
                future.result()
            )


        except Exception as error:

            self.finish_with_error(

                'Controller hedefi '
                'gönderilemedi: '

                f'{error}'
            )

            return


        if not goal_handle.accepted:

            self.finish_with_error(

                'Controller '
                'park yolunu reddetti.'
            )

            return


        self.get_logger().info(

            f'{PARK_YERI}. park yolu '

            f'{CONTROLLER_ID} '

            'tarafından kabul edildi.'
        )


        result_future = (
            goal_handle
            .get_result_async()
        )


        result_future.add_done_callback(
            self.controller_result_callback
        )


    # ========================================================
    # CONTROLLER FEEDBACK
    # ========================================================

    def feedback_callback(
        self,
        feedback_msg,
    ):

        self.feedback_counter += 1


        if (
            self.feedback_counter
            % 10
            != 0
        ):

            return


        feedback = (
            feedback_msg.feedback
        )


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

                f'Kalan mesafe: '
                f'{distance:.2f} m'
            )


        if speed is not None:

            texts.append(

                f'Hız: '
                f'{speed:.2f} m/s'
            )


        if texts:

            self.get_logger().info(

                ' | '.join(
                    texts
                )
            )


    # ========================================================
    # CONTROLLER RESULT
    # ========================================================

    def controller_result_callback(
        self,
        future,
    ):

        try:

            wrapped_result = (
                future.result()
            )


        except Exception as error:

            self.finish_with_error(

                'Controller sonucu '
                'alınamadı: '

                f'{error}'
            )

            return


        status = (
            wrapped_result.status
        )


        # ====================================================
        # BAŞARILI PARK
        # ====================================================

        if (
            status
            == GoalStatus.STATUS_SUCCEEDED
        ):

            self.finished = True


            self.get_logger().info(

                f'Araç '
                f'{PARK_YERI}. '

                'park yerine ulaştı.'
            )


            # Park bitti.
            # Normal navigation toleransını geri getir.
            self.restore_normal_goal_tolerance()

            return


        # ====================================================
        # CANCEL
        # ====================================================

        if (
            status
            == GoalStatus.STATUS_CANCELED
        ):

            self.finished = True


            self.get_logger().warn(

                'Park işlemi '
                'iptal edildi.'
            )


            self.restore_normal_goal_tolerance()

            return


        # ====================================================
        # ABORT
        # ====================================================

        if (
            status
            == GoalStatus.STATUS_ABORTED
        ):

            result = (
                wrapped_result.result
            )


            error_code = getattr(

                result,

                'error_code',

                'bilinmiyor',
            )


            error_msg = getattr(

                result,

                'error_msg',

                '',
            )


            self.finished = True


            self.get_logger().error(

                'Controller park yolunu '
                'takip edemedi. '

                f'Hata kodu: '
                f'{error_code} '

                f'Hata: '
                f'{error_msg}'
            )


            self.restore_normal_goal_tolerance()

            return


        # ====================================================
        # BEKLENMEYEN DURUM
        # ====================================================

        self.finished = True


        self.get_logger().warn(

            'Beklenmeyen controller '
            'durum kodu: '

            f'{status}'
        )


        self.restore_normal_goal_tolerance()


    # ========================================================
    # ORTAK HATA / BİTİŞ
    # ========================================================

    def finish_with_error(
        self,
        message,
        log_message=True,
    ):

        if self.finished:

            return


        self.finished = True


        if log_message:

            self.get_logger().error(
                message
            )


        # Park toleransı aktifse
        # hata durumunda normale dön.
        self.restore_normal_goal_tolerance()


    # ========================================================
    # PLANNER HATA RAPORU
    # ========================================================

    def log_planner_failure(
        self,
        wrapped_result,
        segment_name,
    ):

        result = (
            wrapped_result.result
        )


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

            f'{segment_name} '

            'oluşturulamadı. '

            f'Action durumu='
            f'{wrapped_result.status}, '

            f'hata kodu='
            f'{error_code}, '

            f'hata='
            f'{error_msg}'
        )


# ============================================================
# MAIN
# ============================================================

def main(
    args=None
):

    rclpy.init(
        args=args
    )


    node = (
        Nav2TwoStageParking()
    )


    try:

        rclpy.spin(
            node
        )


    except KeyboardInterrupt:

        node.get_logger().info(

            'Nav2 park sistemi '
            'kapatılıyor.'
        )


    finally:

        # ====================================================
        # ÖNEMLİ:
        #
        # Kullanıcı Ctrl+C ile parkı yarıda kapatırsa
        # park toleransı controller_server üzerinde
        # kalmasın.
        #
        # Normal:
        #
        # XY  = 1.0
        # Yaw = 0.7
        #
        # geri yüklenmeye çalışılır.
        # ====================================================

        if (
            rclpy.ok()
            and node.park_tolerance_active
        ):

            if not (
                node.set_params_client
                .service_is_ready()
            ):

                node.set_params_client.wait_for_service(
                    timeout_sec=1.0
                )


            future = (
                node.restore_normal_goal_tolerance()
            )


            if future is not None:

                try:

                    rclpy.spin_until_future_complete(

                        node,

                        future,

                        timeout_sec=2.0,
                    )


                except Exception as error:

                    node.get_logger().error(

                        'Kapanışta normal '
                        'goal tolerance '

                        'geri yüklenemedi: '

                        f'{error}'
                    )


        node.destroy_node()


        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()