#!/usr/bin/env python3
import csv
import json
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Int8, Int16, Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def color_rgba(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def speed_color(speed, vmax):
    """0 m/s = red, vmax/2 = yellow, vmax = green."""
    ratio = clamp(abs(speed) / max(vmax, 1e-6), 0.0, 1.0)
    if ratio < 0.5:
        return color_rgba(1.0, 2.0 * ratio, 0.0)
    return color_rgba(2.0 * (1.0 - ratio), 1.0, 0.0)


class EMAFilter:
    def __init__(self, alpha):
        self.alpha = float(alpha)
        self.value = None

    def update(self, x):
        x = float(x)
        if self.value is None:
            self.value = x
        else:
            self.value = self.alpha * x + (1.0 - self.alpha) * self.value
        return self.value


class SpeedTraceLogger(Node):
    def __init__(self):
        super().__init__('speed_trace_logger')

        # Frames / topics
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odometer_topic', '/stm/read_odometer')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('motor_power_topic', '/stm/motor_power')
        self.declare_parameter('steering_topic', '/stm/steering_angle')
        self.declare_parameter('brake_topic', '/stm/brake')
        self.declare_parameter('obstacle_topic', '/obstacle_detected')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('marker_topic', '/speed_trace/markers')

        # Sampling / display
        self.declare_parameter('sample_period', 0.10)        # CSV: 10 Hz
        self.declare_parameter('marker_publish_period', 0.50)
        self.declare_parameter('label_distance', 0.50)
        self.declare_parameter('label_speed_delta', 0.15)
        self.declare_parameter('point_size', 0.12)
        self.declare_parameter('text_size', 0.28)
        self.declare_parameter('speed_color_max', 0.80)
        self.declare_parameter('ema_alpha', 0.30)

        # Event thresholds
        self.declare_parameter('curve_angular_threshold', 0.10)
        self.declare_parameter('stall_target_speed_threshold', 0.45)
        self.declare_parameter('stall_actual_speed_threshold', 0.15)
        self.declare_parameter('stall_motor_threshold', 35)
        self.declare_parameter('accel_spike_threshold', 0.80)
        self.declare_parameter('power_spike_threshold', 4)
        self.declare_parameter('event_cooldown', 1.0)

        # File naming
        self.declare_parameter('test_name', 'test')
        self.declare_parameter('csv_directory', '~/speed_trace/logs')

        gp = lambda n: self.get_parameter(n).value
        self.target_frame = str(gp('target_frame'))
        self.base_frame = str(gp('base_frame'))
        self.odometer_topic = str(gp('odometer_topic'))
        self.cmd_vel_topic = str(gp('cmd_vel_topic'))
        self.motor_power_topic = str(gp('motor_power_topic'))
        self.steering_topic = str(gp('steering_topic'))
        self.brake_topic = str(gp('brake_topic'))
        self.obstacle_topic = str(gp('obstacle_topic'))
        self.path_topic = str(gp('path_topic'))
        self.marker_topic = str(gp('marker_topic'))

        self.sample_period = float(gp('sample_period'))
        self.marker_publish_period = float(gp('marker_publish_period'))
        self.label_distance = float(gp('label_distance'))
        self.label_speed_delta = float(gp('label_speed_delta'))
        self.point_size = float(gp('point_size'))
        self.text_size = float(gp('text_size'))
        self.speed_color_max = float(gp('speed_color_max'))

        self.curve_angular_threshold = float(gp('curve_angular_threshold'))
        self.stall_target_speed_threshold = float(gp('stall_target_speed_threshold'))
        self.stall_actual_speed_threshold = float(gp('stall_actual_speed_threshold'))
        self.stall_motor_threshold = int(gp('stall_motor_threshold'))
        self.accel_spike_threshold = float(gp('accel_spike_threshold'))
        self.power_spike_threshold = int(gp('power_spike_threshold'))
        self.event_cooldown = float(gp('event_cooldown'))

        self.test_name = ''.join(
            c if c.isalnum() or c in ('-', '_') else '_' for c in str(gp('test_name'))
        )
        csv_dir = os.path.expanduser(str(gp('csv_directory')))
        os.makedirs(csv_dir, exist_ok=True)

        # TF
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.vel_filter = EMAFilter(float(gp('ema_alpha')))
        self.last_odom = None
        self.last_odom_time = None
        self.current_velocity = None

        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.motor_power = 0
        self.steering_angle = 0
        self.brake = False
        self.obstacle = False

        self.path_points = []
        self.path_frame_ok = False
        self.path_frame_warning_printed = False

        self.last_sample_speed = None
        self.last_sample_time = None
        self.last_sample_motor = 0
        self.last_label_xy = None
        self.last_label_speed = None
        self.last_event_time = {}

        # RViz history
        self.speed_points = []
        self.speed_colors = []
        self.text_markers = []
        self.event_markers = []
        self.next_text_id = 1
        self.next_event_id = 1
        self.last_marker_publish_monotonic = 0.0

        # CSV + metadata
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(csv_dir, f'{self.test_name}_{stamp}.csv')
        self.meta_path = os.path.join(csv_dir, f'{self.test_name}_{stamp}_meta.json')

        self.csv_file = open(self.csv_path, 'w', newline='', buffering=1)
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'elapsed_s',
            'map_x_m', 'map_y_m', 'yaw_deg',
            'actual_speed_m_s', 'target_speed_m_s', 'speed_error_m_s',
            'accel_m_s2',
            'target_angular_z_rad_s', 'curvature_1_m',
            'motor_power', 'steering_angle', 'brake', 'obstacle',
            'path_error_m', 'events'
        ])

        metadata = {
            'created_at': datetime.now().isoformat(timespec='seconds'),
            'test_name': self.test_name,
            'target_frame': self.target_frame,
            'base_frame': self.base_frame,
            'topics': {
                'odometer': self.odometer_topic,
                'cmd_vel': self.cmd_vel_topic,
                'motor_power': self.motor_power_topic,
                'steering': self.steering_topic,
                'brake': self.brake_topic,
                'obstacle': self.obstacle_topic,
                'path': self.path_topic,
                'markers': self.marker_topic,
            },
            'thresholds': {
                'curve_angular_threshold': self.curve_angular_threshold,
                'stall_target_speed_threshold': self.stall_target_speed_threshold,
                'stall_actual_speed_threshold': self.stall_actual_speed_threshold,
                'stall_motor_threshold': self.stall_motor_threshold,
                'accel_spike_threshold': self.accel_spike_threshold,
                'power_spike_threshold': self.power_spike_threshold,
            }
        }
        with open(self.meta_path, 'w') as f:
            json.dump(metadata, f, indent=2)

        # Subscriptions
        self.create_subscription(Float32, self.odometer_topic, self.odom_callback, 20)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 20)
        self.create_subscription(Int8, self.motor_power_topic, self.motor_callback, 20)
        self.create_subscription(Int16, self.steering_topic, self.steering_callback, 20)
        self.create_subscription(Bool, self.brake_topic, self.brake_callback, 20)
        self.create_subscription(Int8, self.obstacle_topic, self.obstacle_callback, 20)
        self.create_subscription(Path, self.path_topic, self.path_callback, 10)

        # Transient-local helps RViz joining later receive the latest full history message.
        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, marker_qos)

        self.start_monotonic = time.monotonic()
        self.timer = self.create_timer(self.sample_period, self.timer_callback)

        self.get_logger().info(f'CSV: {self.csv_path}')
        self.get_logger().info(f'Metadata: {self.meta_path}')
        self.get_logger().info(f'RViz MarkerArray: {self.marker_topic}')
        self.get_logger().info(f'TF: {self.target_frame} <- {self.base_frame}')
        self.get_logger().info('Ctrl+C ile bitir; CSV her satırda flush edilir.')

    # -------------------- callbacks --------------------

    def odom_callback(self, msg: Float32):
        now = time.monotonic()
        odom_cm = float(msg.data)

        if self.last_odom is None:
            self.last_odom = odom_cm
            self.last_odom_time = now
            return

        dt = now - self.last_odom_time
        if dt <= 1e-6:
            return

        velocity_mps = (odom_cm - self.last_odom) / 100.0 / dt
        self.current_velocity = self.vel_filter.update(velocity_mps)
        self.last_odom = odom_cm
        self.last_odom_time = now

    def cmd_vel_callback(self, msg: Twist):
        self.target_linear_x = float(msg.linear.x)
        self.target_angular_z = float(msg.angular.z)

    def motor_callback(self, msg: Int8):
        self.motor_power = int(msg.data)

    def steering_callback(self, msg: Int16):
        self.steering_angle = int(msg.data)

    def brake_callback(self, msg: Bool):
        self.brake = bool(msg.data)

    def obstacle_callback(self, msg: Int8):
        self.obstacle = (int(msg.data) == 1)

    def path_callback(self, msg: Path):
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            self.path_frame_ok = False
            self.path_points = []
            if not self.path_frame_warning_printed:
                self.get_logger().warning(
                    f'Path frame "{msg.header.frame_id}", target_frame "{self.target_frame}". '
                    'Path error hesaplanmayacak. path_topic veya frame ayarını kontrol et.'
                )
                self.path_frame_warning_printed = True
            return

        # Subsample to avoid expensive nearest-point search on very dense paths.
        poses = msg.poses
        step = max(1, len(poses) // 1000)
        self.path_points = [
            (float(p.pose.position.x), float(p.pose.position.y))
            for p in poses[::step]
        ]
        self.path_frame_ok = bool(self.path_points)

    # -------------------- helpers --------------------

    @staticmethod
    def quaternion_to_yaw(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def path_error(self, x, y):
        if not self.path_frame_ok or not self.path_points:
            return math.nan
        return min(math.hypot(x - px, y - py) for px, py in self.path_points)

    def detect_events(self, speed, accel):
        events = []

        if abs(self.target_angular_z) >= self.curve_angular_threshold:
            events.append('CURVE')

        if self.obstacle:
            events.append('OBSTACLE')

        if self.brake:
            events.append('BRAKE')

        # "RPP wants to move, motor is high, but vehicle is nearly stopped."
        if (
            abs(self.target_linear_x) >= self.stall_target_speed_threshold
            and abs(speed) <= self.stall_actual_speed_threshold
            and self.motor_power >= self.stall_motor_threshold
            and not self.brake
            and not self.obstacle
        ):
            events.append('STALL')

        if abs(accel) >= self.accel_spike_threshold:
            events.append('ACCEL_SPIKE')

        if abs(self.motor_power - self.last_sample_motor) >= self.power_spike_threshold:
            events.append('POWER_SPIKE')

        return events

    def should_add_label(self, x, y, speed):
        if self.last_label_xy is None:
            return True
        moved = math.hypot(x - self.last_label_xy[0], y - self.last_label_xy[1])
        speed_delta = abs(speed - self.last_label_speed)
        return moved >= self.label_distance or speed_delta >= self.label_speed_delta

    def add_speed_label(self, x, y, speed, speed_error):
        m = Marker()
        m.header.frame_id = self.target_frame
        m.ns = 'speed_text'
        m.id = self.next_text_id
        self.next_text_id += 1
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.28
        m.pose.orientation.w = 1.0
        m.scale.z = self.text_size
        m.color = color_rgba(1.0, 1.0, 1.0)
        m.text = (
            f'G:{speed:.2f} H:{self.target_linear_x:.2f}\n'
            f'Δ:{speed_error:+.2f} M:{self.motor_power} D:{self.steering_angle}'
        )
        # lifetime=0 => permanent while RViz/node history exists
        m.lifetime.sec = 0
        m.lifetime.nanosec = 0
        self.text_markers.append(m)
        self.last_label_xy = (x, y)
        self.last_label_speed = speed

    def add_event_marker(self, event, x, y, speed):
        now = time.monotonic()
        if now - self.last_event_time.get(event, -1e9) < self.event_cooldown:
            return
        self.last_event_time[event] = now

        if event == 'STALL':
            c = color_rgba(1.0, 0.0, 0.0)
        elif event in ('BRAKE', 'OBSTACLE'):
            c = color_rgba(1.0, 0.3, 0.0)
        elif event in ('ACCEL_SPIKE', 'POWER_SPIKE'):
            c = color_rgba(1.0, 0.0, 1.0)
        else:
            c = color_rgba(0.2, 0.7, 1.0)

        sphere = Marker()
        sphere.header.frame_id = self.target_frame
        sphere.ns = 'events'
        sphere.id = self.next_event_id
        self.next_event_id += 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = x
        sphere.pose.position.y = y
        sphere.pose.position.z = 0.12
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.35
        sphere.scale.y = 0.35
        sphere.scale.z = 0.20
        sphere.color = c
        sphere.lifetime.sec = 0
        self.event_markers.append(sphere)

        text = Marker()
        text.header.frame_id = self.target_frame
        text.ns = 'events'
        text.id = self.next_event_id
        self.next_event_id += 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = 0.55
        text.pose.orientation.w = 1.0
        text.scale.z = 0.32
        text.color = c
        text.text = f'{event}\nG:{speed:.2f} H:{self.target_linear_x:.2f} M:{self.motor_power}'
        text.lifetime.sec = 0
        self.event_markers.append(text)

    def publish_markers(self):
        now_msg = self.get_clock().now().to_msg()
        arr = MarkerArray()

        points = Marker()
        points.header.frame_id = self.target_frame
        points.header.stamp = now_msg
        points.ns = 'speed_points'
        points.id = 0
        points.type = Marker.POINTS
        points.action = Marker.ADD
        points.pose.orientation.w = 1.0
        points.scale.x = self.point_size
        points.scale.y = self.point_size
        points.points = self.speed_points
        points.colors = self.speed_colors
        points.lifetime.sec = 0
        arr.markers.append(points)

        for marker in self.text_markers:
            marker.header.stamp = now_msg
            arr.markers.append(marker)

        for marker in self.event_markers:
            marker.header.stamp = now_msg
            arr.markers.append(marker)

        self.marker_pub.publish(arr)

    # -------------------- main sampling --------------------

    def timer_callback(self):
        if self.current_velocity is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, self.base_frame, Time()
            )
        except TransformException as exc:
            self.get_logger().warning(f'TF alınamadı: {exc}', throttle_duration_sec=2.0)
            return

        x = float(tf.transform.translation.x)
        y = float(tf.transform.translation.y)
        yaw_deg = math.degrees(self.quaternion_to_yaw(tf.transform.rotation))
        speed = float(self.current_velocity)

        now_mono = time.monotonic()
        if self.last_sample_speed is None or self.last_sample_time is None:
            accel = 0.0
        else:
            dt = max(1e-6, now_mono - self.last_sample_time)
            accel = (speed - self.last_sample_speed) / dt

        speed_error = self.target_linear_x - speed
        if abs(self.target_linear_x) > 0.05:
            curvature = self.target_angular_z / self.target_linear_x
        else:
            curvature = math.nan

        p_error = self.path_error(x, y)
        events = self.detect_events(speed, accel)

        # CSV
        self.csv_writer.writerow([
            datetime.now().isoformat(timespec='milliseconds'),
            f'{now_mono - self.start_monotonic:.3f}',
            f'{x:.6f}', f'{y:.6f}', f'{yaw_deg:.3f}',
            f'{speed:.4f}', f'{self.target_linear_x:.4f}', f'{speed_error:.4f}',
            f'{accel:.4f}',
            f'{self.target_angular_z:.4f}',
            '' if math.isnan(curvature) else f'{curvature:.5f}',
            self.motor_power, self.steering_angle, int(self.brake), int(self.obstacle),
            '' if math.isnan(p_error) else f'{p_error:.4f}',
            '|'.join(events) if events else 'NORMAL',
        ])
        self.csv_file.flush()

        # RViz points
        p = Point(x=x, y=y, z=0.05)
        self.speed_points.append(p)
        self.speed_colors.append(speed_color(speed, self.speed_color_max))

        if self.should_add_label(x, y, speed):
            self.add_speed_label(x, y, speed, speed_error)

        # CURVE is too frequent to create an event marker every second; keep it in CSV only.
        for event in events:
            if event != 'CURVE':
                self.add_event_marker(event, x, y, speed)

        if now_mono - self.last_marker_publish_monotonic >= self.marker_publish_period:
            self.publish_markers()
            self.last_marker_publish_monotonic = now_mono

        self.last_sample_speed = speed
        self.last_sample_time = now_mono
        self.last_sample_motor = self.motor_power

    def destroy_node(self):
        try:
            self.publish_markers()
        except Exception:
            pass
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
            self.get_logger().info(f'CSV kaydedildi: {self.csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeedTraceLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
