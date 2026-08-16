#!/usr/bin/env python3

import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class SequentialNav2(Node):

    def __init__(self):
        super().__init__('sequential_nav2_goals')

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # =========================================================
        # ROTA DOSYASI
        # ros2_ws/src/reel_evata/reel_evata/rota.json
        # Format:
        # {"navigasyon": {"rota": [{"id":.., "x":.., "y":.., "oz":.., "ow":.., "wait": 0}, ...]}}
        # =========================================================
        self.route_file = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'rota.json'
        )

        self.goals = self._load_route()

    def _load_route(self):
        """rota.json -> navigasyon.rota'dan (x, y, oz, ow, wait) listesi yükle."""

        try:
            with open(self.route_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(
                f'Rota dosyası okunamadı ({self.route_file}): {e}'
            )
            return []

        try:
            route_list = data['navigasyon']['rota']
        except (KeyError, TypeError) as e:
            self.get_logger().error(
                f'rota.json içinde navigasyon.rota bulunamadı: {e}'
            )
            return []

        goals = []

        for item in route_list:
            try:
                x = float(item['x'])
                y = float(item['y'])
                qz = float(item['oz'])
                qw = float(item['ow'])
                wait = float(item.get('wait', 0.0))
            except (KeyError, TypeError, ValueError) as e:
                point_id = item.get('id', '?') if isinstance(item, dict) else '?'
                self.get_logger().error(
                    f'rota.json içindeki {point_id} nolu nokta geçersiz: {e}'
                )
                continue

            goals.append((x, y, qz, qw, wait))

        self.get_logger().info(
            f'rota.json içinden {len(goals)} hedef yüklendi.'
        )

        return goals

    def create_goal(self, x, y, qz, qw):
        """NavigateToPose goal mesajı oluştur."""

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # Orientation quaternion
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = float(qz)
        goal_msg.pose.pose.orientation.w = float(qw)

        return goal_msg

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        try:
            distance = feedback.distance_remaining

            self.get_logger().info(
                f'Kalan mesafe: {distance:.2f} m',
                throttle_duration_sec=1.0
            )
        except Exception:
            pass

    def _wait_seconds(self, wait_sec):
        """Durakta bekleme (rota.json'dan gelen süre kadar, senkron)."""

        if wait_sec is None or wait_sec <= 0.0:
            return

        self.get_logger().info(
            f'⏳ Bu durakta {wait_sec:.1f} sn bekleniyor...'
        )

        end_time = time.monotonic() + wait_sec

        while time.monotonic() < end_time:
            # Node'u canlı tut, callback'leri işle (0.1 sn adımlarla)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(
            'Bekleme tamamlandı, sıradaki durağa geçiliyor.'
        )

    def navigate_all(self):

        if not self.goals:
            self.get_logger().error(
                'rota.json boş veya yüklenemedi, hedef yok.'
            )
            return False

        self.get_logger().info(
            'NavigateToPose action server bekleniyor...'
        )

        self.nav_client.wait_for_server()

        self.get_logger().info(
            f'Nav2 hazır. Toplam {len(self.goals)} hedef var.'
        )

        # =========================================================
        # HEDEFLERİ SIRAYLA GÖNDER
        # =========================================================

        for index, goal_data in enumerate(self.goals):

            x, y, qz, qw, wait = goal_data

            self.get_logger().info('')
            self.get_logger().info(
                '=========================================='
            )
            self.get_logger().info(
                f'HEDEF {index + 1}/{len(self.goals)}'
            )
            self.get_logger().info(
                f'X={x:.3f}  Y={y:.3f}'
            )
            self.get_logger().info(
                f'Quaternion Z={qz:.6f} W={qw:.6f}'
            )
            self.get_logger().info(
                '=========================================='
            )

            goal_msg = self.create_goal(
                x,
                y,
                qz,
                qw
            )

            # Goal gönder
            send_goal_future = self.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )

            rclpy.spin_until_future_complete(
                self,
                send_goal_future
            )

            goal_handle = send_goal_future.result()

            # -----------------------------------------------------
            # NAV2 GOAL'I REDDETTİ
            # -----------------------------------------------------
            if goal_handle is None:
                self.get_logger().error(
                    f'Hedef {index + 1}: goal handle alınamadı.'
                )
                return False

            if not goal_handle.accepted:
                self.get_logger().error(
                    f'Hedef {index + 1} Nav2 tarafından REDDEDİLDİ.'
                )
                return False

            self.get_logger().info(
                f'Hedef {index + 1} kabul edildi.'
            )

            # -----------------------------------------------------
            # HEDEFİN SONUCUNU BEKLE
            # -----------------------------------------------------
            result_future = goal_handle.get_result_async()

            rclpy.spin_until_future_complete(
                self,
                result_future
            )

            result_response = result_future.result()

            if result_response is None:
                self.get_logger().error(
                    f'Hedef {index + 1}: sonuç alınamadı.'
                )
                return False

            status = result_response.status

            # -----------------------------------------------------
            # SUCCESS
            # -----------------------------------------------------
            if status == GoalStatus.STATUS_SUCCEEDED:

                self.get_logger().info(
                    f'✅ HEDEF {index + 1} BAŞARILI'
                )

                # Durakta bekleme (rota.json'dan gelen süre > 0 ise)
                self._wait_seconds(wait)

                # For döngüsü otomatik olarak
                # bir sonraki hedefe geçecek.

            # -----------------------------------------------------
            # CANCEL
            # -----------------------------------------------------
            elif status == GoalStatus.STATUS_CANCELED:

                self.get_logger().error(
                    f'❌ HEDEF {index + 1} İPTAL EDİLDİ.'
                )

                return False

            # -----------------------------------------------------
            # ABORT
            # -----------------------------------------------------
            elif status == GoalStatus.STATUS_ABORTED:

                self.get_logger().error(
                    f'❌ HEDEF {index + 1} ABORTED.'
                )

                return False

            else:

                self.get_logger().error(
                    f'❌ HEDEF {index + 1} başarısız. '
                    f'Status={status}'
                )

                return False

        # =========================================================
        # TÜM HEDEFLER TAMAMLANDI
        # =========================================================

        self.get_logger().info('')
        self.get_logger().info(
            '=========================================='
        )
        self.get_logger().info(
            '🏁 TÜM NAV2 HEDEFLERİ BAŞARIYLA TAMAMLANDI'
        )
        self.get_logger().info(
            '=========================================='
        )

        return True


def main(args=None):

    rclpy.init(args=args)

    node = SequentialNav2()

    try:
        node.navigate_all()

    except KeyboardInterrupt:
        node.get_logger().warn(
            'Program kullanıcı tarafından durduruldu.'
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