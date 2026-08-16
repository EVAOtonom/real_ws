import rclpy
import os
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int8
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Twist

import math
import json


class RealGPSNavigator(Node):
    def __init__(self):
        super().__init__('real_gps_navigator')

        # Timers
        self._wait_timer = None
        self._retry_timer = None
        self._red_light_timeout_timer = None

        # Waypoint & rejoin sistemi
        self.saved_goal = None
        self.rejoin_waypoints = []
        self.rejoin_active = False
        self.saved_main_goal = None
        self.wait_after_cancel_sec = 0.0

        # Doğrudan verilen duraklar (x, y, ox, oy, oz, ow, wait)
        # 'wait': her durakta kaç saniye bekleneceği (saniye cinsinden)
        self.waypoints = [
        
            {
                'x': -56.026,
                'y': 30.474,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.7077857265109235,
                'ow': 0.7064271833298579,
                'wait': 0.0
            },
            {
                 'x': -74.270,
                 'y': -2.310,
                 'ox': 0.0,
                 'oy': 0.0,
                 'oz': -0.707774,
                 'ow': 0.706439,
                 'wait': 0.0     
            },
            #1.DURAK     
            {
                'x': -89.078,
                'y': 3.936,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.707774,
                'ow': 0.706439,
                'wait': 15.0     
             },          
            {
                'x': -82.545,
                'y': 7.751,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.707774,
                'ow': 0.706439,
                'wait': 0.0     
             },
            #2.DURAK    
            {
                'x': -83.046,
                'y': -28.150,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.707774,
                'ow': 0.706439,
                'wait': 15.0     
             },    
            {
                'x': -66.673,
                'y': -33.831,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.707774,
                'ow': 0.706439,
                'wait': 0.0     
             },
            #3.DURAK    
            {
                'x': -48.243,
                'y': -5.388,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.707774,
                'ow': 0.706439,
                'wait': 15.0     
             },    
            {
                'x': -18.470,
                'y': -19.540,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.707774,
                'ow': 0.706439,
                'wait': 20.0     
             },                                                                             
        ]

        # Trafik ışığı alanları
        self.traffic_light_zones = [
            {'x_min': 3.7, 'x_max': 10.3, 'y_min': 41.9, 'y_max': 48.5}
            # 2. ışık 
            # 3. ışık
        ]

        # Load rejoin points from file
        self.load_rejoin_points()

        self.current_index = 0
        self.current_pose = None
        self.current_yaw = None

        # State
        self.paused = False
        self.motion_enabled = True
        self.goal_handle = None

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.motor_power_pub = self.create_publisher(Int8, '/stm/motor_power', 10)

        # Action client
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._client.wait_for_server()

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, 'nav_cmd', self.control_callback, 10)
        self.create_subscription(String, '/detected_signs', self._sign_callback, 10)

        # Rejoin check timer
        self.create_timer(1.0, self.check_rejoin_waypoint)

        self.get_logger().info("RealGPS Navigator başlatıldı.")
        self.send_next_goal()

    def load_rejoin_points(self):
        """Rejoin waypoint'lerini txt dosyasından yükle"""
        filename = os.path.expanduser("~/real_ws/src/reel_evata/reel_evata/rejoin_waypoints.txt")

        if not os.path.exists(filename):
            self.get_logger().warn(f"Rejoin waypoints dosyası bulunamadı: {filename}")
            return

        try:
            with open(filename, "r") as f:
                for line in f:
                    line = line.strip()
                    if line == "" or line.startswith("#"):
                        continue

                    values = line.split()
                    if len(values) >= 2:
                        self.rejoin_waypoints.append({
                            "x": float(values[0]),
                            "y": float(values[1])
                        })

            self.get_logger().info(
                f"{len(self.rejoin_waypoints)} rejoin waypoint yüklendi."
            )
        except Exception as e:
            self.get_logger().error(f"Rejoin waypoint yükleme hatası: {e}")

    def odom_callback(self, msg):
        """Odometry callback"""
        self.current_pose = msg.pose.pose

        q = self.current_pose.orientation
        (_, _, self.current_yaw) = euler_from_quaternion([
            q.x, q.y, q.z, q.w
        ])

    def check_rejoin_waypoint(self):
        """Rejoin waypoint kontrol et"""
        if self.current_pose is None:
            return

        if self.rejoin_active:
            return

        best_wp = None
        best_distance = 999

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        for wp in self.rejoin_waypoints:
            dx = wp["x"] - robot_x
            dy = wp["y"] - robot_y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance > 10.0:
                continue

            bearing = math.atan2(dy, dx)
            angle = math.degrees(bearing - self.current_yaw)

            while angle > 180:
                angle -= 360
            while angle < -180:
                angle += 360

            self.get_logger().debug(
                f"Robot=({robot_x:.2f},{robot_y:.2f}) "
                f"WP=({wp['x']:.2f},{wp['y']:.2f}) "
                f"dist={distance:.2f} angle={angle:.1f}"
            )

            if angle > -10 or angle < -80:
                continue

            if distance < best_distance:
                best_distance = distance
                best_wp = wp

        if best_wp is None:
            return

        self.get_logger().warn(
            f"REJOIN BULUNDU: ({best_wp['x']:.2f}, {best_wp['y']:.2f})"
        )
        self.start_rejoin(best_wp)

    def start_rejoin(self, wp):
        """Rejoin başlat"""
        self.get_logger().info("Rejoin waypoint bulundu.")

        self.rejoin_active = True
        self.saved_main_goal = {
            "target": self.waypoints[self.current_index],
            "index": self.current_index
        }

        self.rejoin_waypoints.remove(wp)

        # Yönü hesapla
        if self.current_pose is not None:
            dx = wp["x"] - self.current_pose.position.x
            dy = wp["y"] - self.current_pose.position.y
            yaw = math.atan2(dy, dx)
        else:
            yaw = self.current_yaw if hasattr(self, 'current_yaw') else 0.0

        q = quaternion_from_euler(0.0, 0.0, yaw)

        self.rejoin_goal = {
            "x": wp["x"],
            "y": wp["y"],
            "ox": q[0],
            "oy": q[1],
            "oz": q[2],
            "ow": q[3]
        }

        if self.goal_handle is not None:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._rejoin_cancel_done)
        else:
            self.send_goal(self.rejoin_goal, is_rejoin=True)

    def _rejoin_cancel_done(self, future):
        self.get_logger().info("Ana hedef iptal edildi. Rejoin hedefi gönderiliyor.")
        self.goal_handle = None

    def _start_wait_then_proceed(self):
        """Durakta bekleme başlat"""
        if self._wait_timer is not None:
            self._wait_timer.cancel()
            self._wait_timer = None

        self._wait_timer = self.create_timer(
            self.wait_after_cancel_sec, self._on_wait_timer_done
        )

    def _on_wait_timer_done(self):
        """Bekleme bitti, sonraki waypoint'e geç"""
        if self._wait_timer is not None:
            self._wait_timer.cancel()
            self._wait_timer = None
        self.get_logger().info("Bekleme tamamlandı, sıradaki durağa geçiliyor.")
        self._proceed_to_next()

    def _proceed_to_next(self):
        """Sonraki waypoint'e geç"""
        self.current_index += 1
        self.saved_goal = None
        if self.current_index < len(self.waypoints):
            self.send_next_goal()
        else:
            self.get_logger().info("Tüm duraklara ulaşıldı.")

    def send_next_goal(self):
        """Sonraki hedefi gönder"""
        if self.paused:
            self.get_logger().info('Navigasyon duraklatıldı.')
            return

        if not self.motion_enabled:
            self.get_logger().info("Hareket devre dışı, hedef gönderilmeyecek.")
            return

        if self.current_index >= len(self.waypoints):
            return

        wp = self.waypoints[self.current_index]
        self.get_logger().info(
            f'Hedef {self.current_index + 1}/{len(self.waypoints)}: '
            f"XY ({wp['x']:.2f}, {wp['y']:.2f})"
        )
        self.send_goal(wp)

    def send_goal(self, wp, is_rejoin=False):
        """Hedef gönder"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wp['x']
        goal_msg.pose.pose.position.y = wp['y']
        goal_msg.pose.pose.position.z = wp.get('z', 0.0)
        goal_msg.pose.pose.orientation.x = wp.get('ox', 0.0)
        goal_msg.pose.pose.orientation.y = wp.get('oy', 0.0)
        goal_msg.pose.pose.orientation.z = wp.get('oz', 0.0)
        goal_msg.pose.pose.orientation.w = wp.get('ow', 1.0)

        self.get_logger().info(f"Hedef gönderiliyor: XY ({wp['x']:.2f}, {wp['y']:.2f})")
        send_goal_future = self._client.send_goal_async(goal_msg)
        if is_rejoin:
            send_goal_future.add_done_callback(self.rejoin_goal_response_callback)
        else:
            send_goal_future.add_done_callback(self.goal_response_callback)

    def rejoin_goal_response_callback(self, future):
        """Rejoin goal response"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Rejoin reddedildi.")
            return

        self.get_logger().info("REJOIN KABUL EDİLDİ")
        self.goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.rejoin_result_callback)

    def rejoin_result_callback(self, future):
        """Rejoin result callback"""
        result = future.result()

        self.get_logger().warn(f"REJOIN RESULT status={result.status}")

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Rejoin tamamlandı.")
            self.rejoin_active = False

            self.current_index = self.saved_main_goal["index"]
            self.send_goal(self.saved_main_goal["target"])
            self.saved_main_goal = None
        else:
            self.get_logger().warn(f"Rejoin başarısız: {result.status}")
            self.rejoin_active = False
            self.saved_main_goal = None

    def control_callback(self, msg):
        """nav_cmd kontrolü (red/green)"""
        if msg.data == 'red':
            self.motion_enabled = False
            self.get_logger().info("Hareket durduruldu (nav_cmd).")

            if self.current_index < len(self.waypoints):
                self.saved_goal = {
                    'target': self.waypoints[self.current_index],
                    'index': self.current_index
                }

            self._stop_motors()

            if self.goal_handle is not None:
                self.get_logger().info("Aktif hedef iptal ediliyor...")
                cancel_future = self.goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._cancel_done_callback)

            if self._wait_timer is not None:
                self._wait_timer.cancel()
                self._wait_timer = None

        elif msg.data == 'green':
            self.motion_enabled = True
            self.get_logger().info("Hareket yeniden başlatılıyor (nav_cmd).")
            if self.saved_goal:
                wp = self.saved_goal['target']
                self.current_index = self.saved_goal['index']
                self.send_goal(wp)

    def _sign_callback(self, msg):
        """Trafik ışığı algılama callback"""
        try:
            data = json.loads(msg.data)
            if 'kirmizi' in data:
                self._handle_red_light(data['kirmizi'])
            else:
                self._handle_green_light()
        except Exception as e:
            self.get_logger().error(f"Sign JSON parse hatası: {e}")

    def _in_traffic_light_zone(self):
        """Traffic light zone içinde mi kontrol et"""
        if self.current_pose is None:
            return False
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        for zone in self.traffic_light_zones:
            if zone['x_min'] <= x <= zone['x_max'] and \
               zone['y_min'] <= y <= zone['y_max']:
                return True
        return False

    def _stop_motors(self):
        """Motor'u durdur (motor_power ve cmd_vel)"""
        # Motor power = 0
        motor_msg = Int8()
        motor_msg.data = 0
        self.motor_power_pub.publish(motor_msg)

        # cmd_vel = 0
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)

        self.get_logger().info("Motor gücü kapatıldı (motor_power=0, cmd_vel=0).")

    def _red_light_timeout(self):
        """20 saniyede yeşil görülmezse otomatik hareket et"""
        if self._red_light_timeout_timer is not None:
            self._red_light_timeout_timer.cancel()
            self._red_light_timeout_timer = None

        self.get_logger().warn(
            "20 saniye geçti, ışık görülemedi - OTOMATIK HAREKETE GEÇİLİYOR"
        )
        self._handle_green_light()

    def _handle_red_light(self, distance):
        """Kırmızı ışık kontrolü"""
        if self.rejoin_active:
            self.get_logger().info("Rejoin aktif, kırmızı ışık görmezden geliniyor.")
            return

        if not self._in_traffic_light_zone():
            self.get_logger().info(
                "Kırmızı ışık ama trigger zone dışındayız, görmezden geliniyor."
            )
            return

        # Zone içindeyiz — hedefi kaydet
        if self.current_index < len(self.waypoints) and not self.saved_goal:
            self.saved_goal = {
                'target': self.waypoints[self.current_index],
                'index': self.current_index
            }

        # Bu kısım sadece ilk kez durduğu zaman çalışsın (motor'u durdur)
        if self.motion_enabled:
            self.get_logger().info(f"KIRMIZI IŞIK (zone içi) - DURULUYOR")
            self.motion_enabled = False

            self._stop_motors()

            if self.goal_handle is not None:
                cancel_future = self.goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._cancel_done_callback)

            if self._wait_timer is not None:
                self._wait_timer.cancel()
                self._wait_timer = None

        # 20 saniye timeout timer'ını HER SEFERINDE yeniden başlat
        # (kırmızı görmeye devam etse bile timer reset olur)
        if self._red_light_timeout_timer is not None:
            self._red_light_timeout_timer.cancel()

        self._red_light_timeout_timer = self.create_timer(
            20.0, self._red_light_timeout
        )
        self.get_logger().info("Kırmızı ışık görüldü - 20 saniye timeout başlatıldı (timer reset)")

    def _handle_green_light(self):
        """Yeşil ışık kontrolü"""
        # Timer'ı cancel et
        if self._red_light_timeout_timer is not None:
            self._red_light_timeout_timer.cancel()
            self._red_light_timeout_timer = None

        if not self.motion_enabled:
            self.get_logger().info("Kırmızı ışık yok - DEVAM EDİLİYOR")
            self.motion_enabled = True

            if self.saved_goal:
                self.current_index = self.saved_goal['index']
                self.send_goal(self.saved_goal['target'])

    def _cancel_done_callback(self, future):
        """Hedef iptal callback"""
        self.get_logger().info("Hedef iptal isteği işlendi.")
        self.goal_handle = None

    def goal_response_callback(self, future):
        """Hedef yanıt callback"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Hedef reddedildi! 1 saniye sonra tekrar denenecek.')
            self._retry_timer = self.create_timer(1.0, self._retry_current_goal)
            return

        self.get_logger().info('Hedef kabul edildi. Bekleniyor...')
        self.goal_handle = goal_handle

        if not self.saved_goal and self.current_index < len(self.waypoints):
            self.saved_goal = {
                'target': self.waypoints[self.current_index],
                'index': self.current_index
            }

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def _retry_current_goal(self):
        """Hedef tekrar dene"""
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None
        self.send_next_goal()

    def result_callback(self, future):
        """Hedef sonuç callback"""
        result = future.result()

        self.get_logger().warn(
            f"NORMAL RESULT CALLBACK status={result.status} rejoin={self.rejoin_active}"
        )

        if result.status == GoalStatus.STATUS_CANCELED and self.rejoin_active:
            self.get_logger().info("Eski hedef halt oldu. Rejoin hedefi gönderiliyor.")
            self.send_goal(self.rejoin_goal, is_rejoin=True)
            return

        if result.status == GoalStatus.STATUS_CANCELED and not self.rejoin_active:
            self.get_logger().info("Hedef iptal edildi (trafik ışığı), bekleniyor.")
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            wait_time = self.waypoints[self.current_index].get("wait", 0.0)
            self.saved_goal = None
            self.goal_handle = None

            if wait_time > 0:
                self.wait_after_cancel_sec = wait_time
                self.get_logger().info(f"Hedefe ulaşıldı. {wait_time:.0f} saniye bekleniyor...")
                self._start_wait_then_proceed()
            else:
                self.get_logger().info("Hedefe ulaşıldı. Sonraki hedefe geçiliyor.")
                self._proceed_to_next()
        else:
            self.get_logger().warn(f"Hedef beklenmeyen durumla tamamlandı. Status: {result.status}")


def main(args=None):
    rclpy.init(args=args)
    node = RealGPSNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
