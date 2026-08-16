#!/usr/bin/env python3

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
        # DOSYADAKI POZLAR
        # Format:
        # (x, y, orientation_z, orientation_w)
        # =========================================================
        self.goals = [

            # 1
            (
                -84.20746612548828,
                -6.594117164611816,
                -0.23425603688405539,
                0.9721749375412719
            ),

            # 2
            (
                -72.11151885986328,
                -4.927000999450684,
                0.5174728381804424,
                0.8556996329001653
            ),

            # 3
            (
                -51.11619567871094,
                5.383429527282715,
                -0.18979385222140596,
                0.9818239626628387
            ),

            # 4
            (
                -60.57512283325195,
                -12.046930313110352,
                -0.9857168765468909,
                0.1684109239053137
            ),

            # 5
            (
                -64.89634704589844,
                -19.882232666015625,
                -0.5465650118756582,
                0.8374166751345244
            ),

            # 6
            (
                -56.6627311706543,
                -24.29261016845703,
                0.16755688331452337,
                0.9858624096971763
            ),

            # 7
            (
                -48.55339050292969,
                -5.776865005493164,
                0.5343865668940246,
                0.8452402008442441
            ),

            # 8
            (
                -32.31331253051758,
                15.132672309875488,
                -0.24656515563213827,
                0.969126216768538
            ),

            # 9
            (
                -28.58000946044922,
                -4.491061210632324,
                -0.8368783759706584,
                0.547388878068155
            ),

            # 10
            (
                -28.344215393066406,
                -14.212835311889648,
                -0.3156101639248107,
                0.9488889420935172
            ),
        ]

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

    def navigate_all(self):

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

            x, y, qz, qw = goal_data

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
