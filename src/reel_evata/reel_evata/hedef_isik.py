#!/usr/bin/env python3

import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Twist

from tf2_ros import Buffer, TransformListener, TransformException


class SequentialNav2(Node):

    def __init__(self):
        super().__init__('sequential_nav2_goals')

        # NAV2 ACTION CLIENT
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # ROTA DOSYASI
        self.route_file = os.path.expanduser(
            "~/real_ws/src/reel_evata/reel_evata/rota.json"
        )

        self.goals = self._load_route()


        # TF dönüşümü

        self.map_frame = 'map'
        self.robot_frame = 'base_footprint'

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # Son başarılı map koordinatı
        self.current_map_x = None
        self.current_map_y = None

        # TF hata logunu sürekli basmamak için
        self.last_tf_warning_time = 0.0

        # KIRMIZI IŞIK SİSTEMİ
        self.motion_enabled = True
        self.current_goal_handle = None
        self._red_light_timeout_timer = None
        self._cancel_future = None

        #PATH MESAFE KONTROLU
        self.previous_path_distance = None
        self.latest_path_distance = None

        self.route_extension_threshold = 60.0

        self.skip_due_to_route_extension = False
        self.route_extension_cancel_requested = False

        # TRAFİK IŞIĞI BÖLGELERİ
        self.traffic_light_zones = [

            {
                'x_min': -86.3,
                'x_max': -71.1,
                'y_min': -22.3,
                'y_max': -7.99
            },

            {
                'x_min': -27.6,
                'x_max': -16.7,
                'y_min': -4.29,
                'y_max': 7.73
            },

            {
                'x_min': -73.2,
                'x_max': -62.9,
                'y_min': -16.4,
                'y_max': -5.62
            },

        ]

        # PUBLISHERS

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.motor_power_pub = self.create_publisher(
            Int8,
            '/stm/motor_power',
            10
        )

        # SUBSCRIBERS

        self.create_subscription(
            String,
            '/detected_signs',
            self._sign_callback,
            10
        )

        self.get_logger().info(
            'Araç konumu TF üzerinden alınacak: '
            f'{self.map_frame} -> {self.robot_frame}'
        )

    # ROTA YÜKLE

    def _load_route(self):
        """
        rota.json -> navigasyon.rota'dan
        (x, y, oz, ow, wait) listesi yükle.
        """

        try:
            with open(
                self.route_file,
                'r',
                encoding='utf-8'
            ) as f:
                data = json.load(f)

        except Exception as e:
            self.get_logger().error(
                f'Rota dosyası okunamadı '
                f'({self.route_file}): {e}'
            )
            return []

        try:
            route_list = data['navigasyon']['rota']

        except (KeyError, TypeError) as e:
            self.get_logger().error(
                f'rota.json içinde navigasyon.rota '
                f'bulunamadı: {e}'
            )
            return []

        goals = []

        for item in route_list:

            try:
                x = float(item['x'])
                y = float(item['y'])

                qz = float(item['oz'])
                qw = float(item['ow'])

                wait = float(
                    item.get('wait', 0.0)
                )

            except (
                KeyError,
                TypeError,
                ValueError
            ) as e:

                point_id = (
                    item.get('id', '?')
                    if isinstance(item, dict)
                    else '?'
                )

                self.get_logger().error(
                    f'rota.json içindeki '
                    f'{point_id} nolu nokta '
                    f'geçersiz: {e}'
                )

                continue

            goals.append(
                (
                    x,
                    y,
                    qz,
                    qw,
                    wait
                )
            )

        self.get_logger().info(
            f'{len(goals)} Hedef yüklendi.'
        )

        return goals

    # =============================================================
    # MAP -> BASE_FOOTPRINT KONUMU
    # =============================================================

    def _get_robot_map_position(self):
        """
        Aracın MAP koordinat sistemindeki konumunu TF'den alır.

        Kullanılan TF:

            map -> base_footprint

        Return:
            (x, y)

        TF alınamazsa:
            None
        """

        try:

            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            self.current_map_x = x
            self.current_map_y = y

            return x, y

        except TransformException as e:

            # Her callback'te warning yağdırmasın.
            now = time.monotonic()

            if now - self.last_tf_warning_time > 2.0:

                self.get_logger().warn(
                    f'TF alınamadı: '
                    f'{self.map_frame} -> '
                    f'{self.robot_frame} | {e}'
                )

                self.last_tf_warning_time = now

            return None

    # =============================================================
    # SIGN CALLBACK
    # =============================================================

    def _sign_callback(self, msg):

        try:

            data = json.loads(msg.data)

            if 'kirmizi' in data:

                self._handle_red_light(
                    data['kirmizi']
                )

            else:

                self._handle_green_light()

        except Exception as e:

            self.get_logger().error(
                f'Sign JSON parse hatası: {e}'
            )

    # =============================================================
    # TRAFİK IŞIĞI ZONE KONTROLÜ
    # =============================================================

    def _in_traffic_light_zone(self):
        """
        Aracın konumu artık ODOM'dan değil,
        MAP -> BASE_FOOTPRINT TF'sinden alınır.
        """

        position = self._get_robot_map_position()

        if position is None:

            self.get_logger().warn(
                'Araç MAP konumu alınamadığı için '
                'traffic-light zone kontrolü yapılamadı.'
            )

            return False

        x, y = position

        # Debug istersen açabilirsin:
        #
        # self.get_logger().info(
        #     f'MAP araç konumu: X={x:.2f}, Y={y:.2f}'
        # )

        for i, zone in enumerate(
            self.traffic_light_zones
        ):

            inside = (
                zone['x_min'] <= x <= zone['x_max']
                and
                zone['y_min'] <= y <= zone['y_max']
            )

            if inside:

                self.get_logger().info(
                    f'Araç trafik ışığı zone '
                    f'{i + 1} içinde. '
                    f'MAP: X={x:.2f}, Y={y:.2f}'
                )

                return True

        return False

    # =============================================================
    # MOTOR DURDUR
    # =============================================================

    def _stop_motors(self):

        motor_msg = Int8()
        motor_msg.data = 0

        self.motor_power_pub.publish(
            motor_msg
        )

        stop_msg = Twist()

        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0

        self.cmd_vel_pub.publish(
            stop_msg
        )

        self.get_logger().info(
            'Motor gücü kapatıldı '
            '(motor_power=0, cmd_vel=0).'
        )

    # =============================================================
    # KIRMIZI IŞIK TIMEOUT
    # =============================================================

    def _red_light_timeout(self):

        if self._red_light_timeout_timer is not None:

            self._red_light_timeout_timer.cancel()

            self._red_light_timeout_timer = None

        self.get_logger().warn(
            '20 saniye geçti, ışık görülemedi - '
            'OTOMATIK HAREKETE GEÇİLİYOR'
        )

        self._handle_green_light()

    # =============================================================
    # KIRMIZI IŞIK
    # =============================================================

    def _handle_red_light(
        self,
        distance
    ):

        # Önce aracın MAP koordinatına bak.
        if not self._in_traffic_light_zone():

            self.get_logger().info(
                'Kırmızı ışık görüldü fakat '
                'araç MAP üzerinde trigger zone '
                'dışında. Görmezden geliniyor.'
            )

            return

        if self.motion_enabled:

            self.get_logger().info(
                'KIRMIZI IŞIK '
                '(MAP zone içi) - DURULUYOR'
            )

            self.motion_enabled = False

            self._stop_motors()

            # Aktif Nav2 goal varsa iptal et.
            if self.current_goal_handle is not None:

                self._cancel_future = (
                    self.current_goal_handle
                    .cancel_goal_async()
                )

        # Her yeni kırmızı algılandığında
        # timeout sıfırlansın.
        if self._red_light_timeout_timer is not None:

            self._red_light_timeout_timer.cancel()

        self._red_light_timeout_timer = (
            self.create_timer(
                20.0,
                self._red_light_timeout
            )
        )

        self.get_logger().info(
            'Kırmızı ışık görüldü - '
            '20 saniye timeout başlatıldı '
            '(timer reset)'
        )

    # =============================================================
    # YEŞİL IŞIK
    # =============================================================

    def _handle_green_light(self):
        """
        Yeşil ışık kontrolü
        """

        if self._red_light_timeout_timer is not None:

            self._red_light_timeout_timer.cancel()

            self._red_light_timeout_timer = None

        if not self.motion_enabled:

            self.get_logger().info(
                'Kırmızı ışık yok - DEVAM EDİLİYOR'
            )

            self.motion_enabled = True

    # =============================================================
    # HAREKET İZNİ BEKLE
    # =============================================================

    def _wait_until_motion_enabled(self):
        """
        motion_enabled tekrar True olana kadar
        node'u canlı tut.

        Yeşil ışık veya 20sn timeout sonrası çıkar.
        """

        while not self.motion_enabled:

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

    # =============================================================
    # NAV2 GOAL OLUŞTUR
    # =============================================================

    def create_goal(
        self,
        x,
        y,
        qz,
        qw
    ):
        """
        NavigateToPose goal mesajı oluştur.
        """

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'

        goal_msg.pose.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        # =========================================================
        # POSITION
        # =========================================================

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # =========================================================
        # ORIENTATION
        # =========================================================

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0

        goal_msg.pose.pose.orientation.z = float(qz)
        goal_msg.pose.pose.orientation.w = float(qw)

        return goal_msg

    # =============================================================
    # NAV2 FEEDBACK
    # =============================================================

    def feedback_callback(
        self,
        feedback_msg
    ):

        feedback = feedback_msg.feedback

        try:

            distance = float(
                feedback.distance_remaining
            )

            if distance <= 0.0:

                self.get_logger().info(
                    '[ROTA KONTROL] '
                    'Mesafe 0.0 geldi, gerçek path mesafesi '
                    'bekleniyor...',
                    throttle_duration_sec=2.0
                )

                return

            # =================================================
            # İLK GERÇEK MESAFE
            # =================================================

            if self.previous_path_distance is None:

                self.previous_path_distance = distance
                self.latest_path_distance = distance

                self.get_logger().info(
                    f'[ROTA KONTROL] '
                    f'İlk gerçek kalan mesafe: '
                    f'{distance:.2f} m'
                )

                return

            # =================================================
            # ÖNCEKİ FEEDBACK -> YENİ FEEDBACK
            #
            # Örnek:
            #
            # Önceki: 45 m
            # Yeni:    56 m
            #
            # Uzama: +11 m
            # =================================================

            route_extension = (
                distance
                -
                self.previous_path_distance
            )

            self.get_logger().info(
                f'[ROTA KONTROL] '
                f'Önceki={self.previous_path_distance:.2f} m | '
                f'Anlık={distance:.2f} m | '
                f'Değişim={route_extension:+.2f} m',
                throttle_duration_sec=10.0
            )

            # =================================================
            # KALAN PATH BİR ANDA 10 METRE VEYA DAHA FAZLA
            # UZADIYSA
            # =================================================

            if (
                route_extension
                >= self.route_extension_threshold
                and
                not self.route_extension_cancel_requested
            ):

                self.get_logger().warn(
                    f'[ROTA UZADI] '
                    f'{self.previous_path_distance:.2f} m -> '
                    f'{distance:.2f} m | '
                    f'Uzama: +{route_extension:.2f} m'
                )

                self.get_logger().warn(
                    'Aktif hedef ve sonraki hedef '
                    'ATLANACAK.'
                )

                self.skip_due_to_route_extension = True
                self.route_extension_cancel_requested = True

                if self.current_goal_handle is not None:

                    self._cancel_future = (
                        self.current_goal_handle
                        .cancel_goal_async()
                    )

            # Bir sonraki feedback için güncelle
            self.previous_path_distance = distance
            self.latest_path_distance = distance

        except Exception as e:

            self.get_logger().error(
                f'Feedback işleme hatası: {e}'
            )

    # =============================================================
    # DURAK BEKLEME
    # =============================================================

    def _wait_seconds(
        self,
        wait_sec
    ):
        """
        Durakta bekleme.

        rota.json'dan gelen süre kadar.
        """

        if (
            wait_sec is None
            or
            wait_sec <= 0.0
        ):

            return

        self.get_logger().info(
            f'Bu durakta '
            f'{wait_sec:.1f} sn bekleniyor...'
        )

        end_time = (
            time.monotonic()
            +
            wait_sec
        )

        while time.monotonic() < end_time:

            # Callback'ler çalışmaya devam etsin.
            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

        self.get_logger().info(
            'Bekleme tamamlandı, '
            'sıradaki durağa geçiliyor.'
        )

    # =============================================================
    # TÜM HEDEFLERİ GEZ
    # =============================================================

    def navigate_all(self):

        if not self.goals:

            self.get_logger().error(
                'rota.json boş veya '
                'yüklenemedi, hedef yok.'
            )

            return False

        # =========================================================
        # NAV2 SERVER BEKLE
        # =========================================================

        self.get_logger().info(
            'NavigateToPose action server bekleniyor...'
        )

        self.nav_client.wait_for_server()

        self.get_logger().info(
            'NavigateToPose action server hazır.'
        )

        # =========================================================
        # TF'NİN GELMESİNİ KONTROL ET
        # =========================================================

        self.get_logger().info(
            f'{self.map_frame} -> '
            f'{self.robot_frame} TF bekleniyor...'
        )

        tf_wait_start = time.monotonic()

        while rclpy.ok():

            position = (
                self._get_robot_map_position()
            )

            if position is not None:

                x_map, y_map = position

                self.get_logger().info(
                    'MAP konumu hazır: '
                    f'X={x_map:.3f}, '
                    f'Y={y_map:.3f}'
                )

                break

            # TF callback'lerini işle
            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            # 10 saniyede bir bilgi ver.
            if (
                time.monotonic()
                -
                tf_wait_start
                >
                10.0
            ):

                self.get_logger().warn(
                    f'Hala '
                    f'{self.map_frame} -> '
                    f'{self.robot_frame} '
                    f'TF bekleniyor...'
                )

                tf_wait_start = (
                    time.monotonic()
                )

        # =========================================================
        # HEDEFLERİ SIRAYLA GÖNDER
        # =========================================================

        index = 0

        while index < len(self.goals):

            (
                x,
                y,
                qz,
                qw,
                wait
            ) = self.goals[index]

            self.get_logger().info(
                '=========================================='
            )

            self.get_logger().info(
                f'HEDEF '
                f'{index + 1}/'
                f'{len(self.goals)}'
            )

            self.get_logger().info(
                f'X={x:.3f}  Y={y:.3f}'
            )

            # -----------------------------------------------------
            # İstersen her hedef öncesinde aracın
            # güncel MAP koordinatını da gör.
            # -----------------------------------------------------

            robot_position = (
                self._get_robot_map_position()
            )

            if robot_position is not None:

                robot_x, robot_y = (
                    robot_position
                )

                self.get_logger().info(
                    'Araç MAP konumu: '
                    f'X={robot_x:.3f} '
                    f'Y={robot_y:.3f}'
                )

            # -----------------------------------------------------
            # Kırmızı ışıktaysak yeni goal gönderme.
            # -----------------------------------------------------

            self._wait_until_motion_enabled()

            # =====================================================
            # YENİ HEDEF İÇİN ROTA UZAMA KONTROLÜNÜ SIFIRLA
            # =====================================================

            self.previous_path_distance = None
            self.latest_path_distance = None

            self.skip_due_to_route_extension = False
            self.route_extension_cancel_requested = False
            
            # -----------------------------------------------------
            # Goal oluştur
            # -----------------------------------------------------

            goal_msg = self.create_goal(
                x,
                y,
                qz,
                qw
            )

            # -----------------------------------------------------
            # Goal gönder
            # -----------------------------------------------------

            send_goal_future = (
                self.nav_client
                .send_goal_async(
                    goal_msg,
                    feedback_callback=(
                        self.feedback_callback
                    )
                )
            )

            rclpy.spin_until_future_complete(
                self,
                send_goal_future
            )

            goal_handle = (
                send_goal_future.result()
            )

            if goal_handle is None:

                self.get_logger().error(
                    f'Hedef {index + 1}: '
                    f'goal handle alınamadı.'
                )

                return False

            if not goal_handle.accepted:

                self.get_logger().error(
                    f'Hedef {index + 1} '
                    f'Nav2 tarafından REDDEDİLDİ.'
                )

                return False

            self.current_goal_handle = (
                goal_handle
            )

            # -----------------------------------------------------
            # Sonuç bekle
            # -----------------------------------------------------

            result_future = (
                goal_handle
                .get_result_async()
            )

            rclpy.spin_until_future_complete(
                self,
                result_future
            )

            result_response = (
                result_future.result()
            )

            self.current_goal_handle = None

            if result_response is None:

                self.get_logger().error(
                    f'Hedef {index + 1}: '
                    f'sonuç alınamadı.'
                )

                return False

            status = (
                result_response.status
            )

            # =====================================================
            # BAŞARILI
            # =====================================================

            if (
                status
                ==
                GoalStatus.STATUS_SUCCEEDED
            ):

                self.get_logger().info(
                    f'HEDEF '
                    f'{index + 1} '
                    f'BAŞARILI'
                )

                self._wait_seconds(
                    wait
                )

                index += 1

                continue

            # =====================================================
            # CANCEL
            # =====================================================

            elif (
                status
                ==
                GoalStatus.STATUS_CANCELED
            ):

                # =====================================================
                # ROTA UZAMASI NEDENİYLE İPTAL
                # =====================================================

                if self.skip_due_to_route_extension:

                    self.get_logger().warn(
                        '================================================'
                    )

                    self.get_logger().warn(
                        f'ROTA UZAMASI NEDENİYLE '
                        f'HEDEF {index + 1} ATLANDI.'
                    )

                    old_index = index

                    # -------------------------------------------------
                    # Aktif hedef + bir sonraki hedefi atla
                    # -------------------------------------------------

                    index += 2

                    if index < len(self.goals):

                        self.get_logger().warn(
                            f'HEDEF {old_index + 1} ve '
                            f'HEDEF {old_index + 2} atlandı.'
                        )

                        self.get_logger().warn(
                            f'Yeni hedef: '
                            f'{index + 1}/{len(self.goals)}'
                        )

                    else:

                        self.get_logger().warn(
                            'Aktif hedef ve sonraki hedef '
                            'atlandıktan sonra rota tamamlandı.'
                        )

                    self.get_logger().warn(
                        '================================================'
                    )

                    continue


                # =====================================================
                # KIRMIZI IŞIK NEDENİYLE İPTAL
                # =====================================================

                if not self.motion_enabled:

                    self.get_logger().warn(
                        f'HEDEF '
                        f'{index + 1} '
                        f'kırmızı ışık nedeniyle '
                        f'iptal edildi, '
                        f'yeşil ışık bekleniyor...'
                    )

                    self._wait_until_motion_enabled()

                    self.get_logger().info(
                        'Yeşil ışık/timeout — '
                        'aynı hedef tekrar gönderiliyor.'
                    )

                    continue


                # =====================================================
                # DİĞER CANCEL
                # =====================================================

                self.get_logger().error(
                    f'HEDEF '
                    f'{index + 1} '
                    f'İPTAL EDİLDİ.'
                )

                return False

        # =========================================================
        # TÜM HEDEFLER TAMAMLANDI
        # =========================================================


        self.get_logger().info(
            'TÜM NAV2 HEDEFLERİ '
            'BAŞARIYLA TAMAMLANDI'
        )

        return True


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = SequentialNav2()

    try:

        node.navigate_all()

    except KeyboardInterrupt:

        node.get_logger().warn(
            'Program kullanıcı tarafından '
            'durduruldu.'
        )

    except Exception as e:

        node.get_logger().error(
            f'Beklenmeyen hata: {e}'
        )

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':
    main()