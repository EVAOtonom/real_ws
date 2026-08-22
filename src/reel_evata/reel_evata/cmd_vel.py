#!/usr/bin/env python3.9
"""
cmd_vel_subscriber
-------------------
nav2'nin /cmd_vel (Twist) mesajlarini dinler, STM'e uc komut yayinlar:
  /stm/motor_power     (Int8)
  /stm/steering_angle  (Int16)
  /stm/brake           (Bool)
  /stm/reverse_command (Bool)

Bu versiyon Ackermann kinematiğine uygun olarak güncellenmiştir.

NOT:
/detected_signs levha/kirmizi isik sistemi tamamen kaldirilmistir.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Int16, Bool, Float32
from time import monotonic
import os
import csv
from datetime import datetime
import math


class PID:
    """Basit PID kontrolcu; anti-windup (integral clamp) ve cikis limiti ile."""
    def __init__(self, kp, ki, kd, out_min, out_max, integral_limit=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral_limit = integral_limit if integral_limit is not None else out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0:
            return self.out_min

        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        base_power = self.out_min if error > 0 else 0.0
        output = base_power + (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        return max(self.out_min, min(self.out_max, output))


class EMAFilter:
    """Ustel hareketli ortalama - odometri hiz tahminini yumusatmak icin."""
    def __init__(self, alpha):
        self.alpha = alpha
        self.value = None

    def update(self, new_value):
        self.value = new_value if self.value is None else self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value


class RateLimiter:
    """Bir degerin birim zamanda (birim/saniye) ne kadar degisebilecegini sinirlar."""
    def __init__(self, max_rate_up, max_rate_down=None):
        self.max_rate_up = max_rate_up
        self.max_rate_down = max_rate_down if max_rate_down is not None else max_rate_up
        self.value = 0.0

    def update(self, target, dt):
        if dt <= 0:
            return self.value

        delta = target - self.value
        delta = max(-self.max_rate_down * dt, min(self.max_rate_up * dt, delta))
        self.value += delta
        return self.value

    def reset(self, value=0.0):
        self.value = value


class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        # ==================== KINEMATIK PARAMETRELER ====================
        self.declare_parameter('wheelbase', 1.55)
        self.declare_parameter('min_turning_radius', 2.7)

        # ==================== HIZ PARAMETRELERI ====================
        self.declare_parameter('max_motor_power', 36)
        self.declare_parameter('min_motor_power', 28)
        self.declare_parameter('max_velocity', 0.8)
        self.declare_parameter('vel_kp', 2.0)
        self.declare_parameter('vel_ki', 1.5)
        self.declare_parameter('vel_kd', 0.8)
        self.declare_parameter('max_accel', 0.20)
        self.declare_parameter('max_decel', 0.35)

        # ==================== GERI VITES HIZ PARAMETRELERI ====================
        self.declare_parameter('max_reverse_velocity', 0.2)
        self.declare_parameter('reverse_min_motor_power', 32)
        self.declare_parameter('reverse_max_motor_power', 39)
        self.declare_parameter('reverse_vel_kp', 2.0)
        self.declare_parameter('reverse_vel_ki', 0.5)
        self.declare_parameter('reverse_vel_kd', 0.5)
        self.declare_parameter('reverse_max_accel', 0.35)
        self.declare_parameter('reverse_max_decel', 0.8)
        self.declare_parameter('reverse_overspeed_brake_margin', 0.50)
        self.declare_parameter('reverse_stall_boost_rate', 1.0)
        self.declare_parameter('reverse_absolute_max_motor_power', 39)

        # Yon degistirirken arac hareketliyse once durdur.
        self.declare_parameter('gear_shift_velocity_threshold', 0.05)

        # ==================== FREN ====================
        self.declare_parameter('overspeed_brake_margin', 0.5)
        self.declare_parameter('stall_velocity_threshold', 0.08)

        # ==================== DIREKSIYON ====================
        self.declare_parameter('steer_max_left', 160)
        self.declare_parameter('steer_max_right', -160)
        self.declare_parameter('angular_z_max', 0.35)
        self.declare_parameter('angular_z_min', -0.35)
        self.declare_parameter('angular_gain', 1.0)
        self.declare_parameter('angular_deadband', 0.03)
        self.declare_parameter('steering_max_rate_dps', 28.0)

        # ==================== DONUSTE GUC ARTISI ====================
        self.declare_parameter('curvature_power_boost_enable', False)
        self.declare_parameter('curvature_free_zone', 0.25)
        self.declare_parameter('turn_power_boost_max', 0.5)

        # ==================== KALKIS / STALL ====================
        self.declare_parameter('stall_boost_rate', 0.7)
        self.declare_parameter('absolute_max_motor_power', 38)

        # ==================== WATCHDOG ====================
        self.declare_parameter('cmd_vel_timeout', 0.30)
        self.declare_parameter('odom_timeout', 1.00)
        self.declare_parameter('status_log_hz', 2.0)
        self.declare_parameter('csv_log_hz', 20.0)
        self.declare_parameter('csv_flush_hz', 1.0)

        self._load_params()
        self.add_on_set_parameters_callback(self._on_param_update)

        # ==================== PUBLISHERS ====================
        self.steering_angle_pub = self.create_publisher(Int16, '/stm/steering_angle', 1)
        self.motor_power_pub = self.create_publisher(Int8, '/stm/motor_power', 1)
        self.brake_pub = self.create_publisher(Bool, '/stm/brake', 1)
        self.reverse_pub = self.create_publisher(Bool, '/stm/reverse_command', 1)

        # ==================== SUBSCRIBERS ====================
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)
        self.create_subscription(Float32, '/stm/read_odometer', self.odom_callback, 1)
        self.create_subscription(Int8, '/obstacle_detected', self.obstacle_callback, 1)

        # ==================== DURUM DEGISKENLERI ====================
        self.current_velocity = 0.0
        self.target_velocity = 0.0
        self.target_steering_deg = 0.0
        self.last_steering_deg = 0
        self.last_motor_power = 0
        self.last_brake = False
        self.angular_z_filtered = 0.0

        self.last_odom = None
        self.last_odom_time = None
        self.last_odom_rx_time = None
        self.last_cmd_vel_time = None
        self.last_safety_reason = None
        self.obstacle_detected = False

        self.stall_timer = 0.0
        self.reverse_stall_timer = 0.0
        self.last_pid_time = None

        # Geri vites
        self.reverse_active = False
        self.desired_reverse = False

        self.vel_filter = EMAFilter(0.3)
        self.velocity_ramp = RateLimiter(self.max_accel, self.max_decel)
        self.reverse_velocity_ramp = RateLimiter(self.reverse_max_accel, self.reverse_max_decel)
        self.steering_ramp = RateLimiter(self.steering_max_rate_dps)

        self.velocity_pid = PID(
            kp=self.vel_kp, ki=self.vel_ki, kd=self.vel_kd,
            out_min=float(self.min_motor_power), out_max=float(self.max_motor_power),
            integral_limit=(self.max_motor_power - self.min_motor_power) * 3.0
        )

        self.reverse_velocity_pid = PID(
            kp=self.reverse_vel_kp, ki=self.reverse_vel_ki, kd=self.reverse_vel_kd,
            out_min=float(self.reverse_min_motor_power), out_max=float(self.reverse_max_motor_power),
            integral_limit=(self.reverse_max_motor_power - self.reverse_min_motor_power) * 3.0
        )

        # Baslangicta ileri vites
        self.reverse_pub.publish(Bool(data=False))

        # ==================== CSV LOGGING ====================
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(os.getcwd(), f"cmd_vel_log_{timestamp_str}.csv")
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "Timestamp", "HedefLinX_m_s", "AnlikHiz_m_s", "MotorGucu", "TurnBoost",
            "StallBoost", "HedefAngZ_rad_s", "HedefAci_deg", "YayinAci_deg", "Fren", "GeriVites"
        ])

        self.last_csv_log_time = 0.0
        self.last_csv_flush_time = monotonic()
        self.last_status_log_time = 0.0

        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info(f'CmdVel Node baslatildi. Veriler kaydediliyor: {self.csv_filename}')

    def _load_params(self):
        gp = lambda name: self.get_parameter(name).value

        self.wheelbase = gp('wheelbase')
        self.min_turning_radius = gp('min_turning_radius')

        self.max_motor_power = gp('max_motor_power')
        self.min_motor_power = gp('min_motor_power')
        self.max_velocity = gp('max_velocity')
        self.vel_kp = gp('vel_kp')
        self.vel_ki = gp('vel_ki')
        self.vel_kd = gp('vel_kd')
        self.max_accel = gp('max_accel')
        self.max_decel = gp('max_decel')

        # ==================== GERI VITES ====================
        self.max_reverse_velocity = gp('max_reverse_velocity')
        self.reverse_min_motor_power = gp('reverse_min_motor_power')
        self.reverse_max_motor_power = gp('reverse_max_motor_power')
        self.reverse_vel_kp = gp('reverse_vel_kp')
        self.reverse_vel_ki = gp('reverse_vel_ki')
        self.reverse_vel_kd = gp('reverse_vel_kd')
        self.reverse_max_accel = gp('reverse_max_accel')
        self.reverse_max_decel = gp('reverse_max_decel')
        self.reverse_overspeed_brake_margin = gp('reverse_overspeed_brake_margin')
        self.reverse_stall_boost_rate = gp('reverse_stall_boost_rate')
        self.reverse_absolute_max_motor_power = gp('reverse_absolute_max_motor_power')
        self.gear_shift_velocity_threshold = gp('gear_shift_velocity_threshold')

        # ==================== FREN ====================
        self.overspeed_brake_margin = gp('overspeed_brake_margin')
        self.stall_velocity_threshold = gp('stall_velocity_threshold')

        # ==================== DIREKSIYON ====================
        self.STEER_MAX_LEFT = gp('steer_max_left')
        self.STEER_MAX_RIGHT = gp('steer_max_right')
        self.max_steer_mag = max(abs(self.STEER_MAX_LEFT), abs(self.STEER_MAX_RIGHT))
        self.angular_z_max = gp('angular_z_max')
        self.angular_z_min = gp('angular_z_min')
        self.angular_z_mag = max(abs(self.angular_z_max), abs(self.angular_z_min))
        self.angular_gain = gp('angular_gain')
        self.angular_deadband = gp('angular_deadband')
        self.steering_max_rate_dps = gp('steering_max_rate_dps')

        # ==================== DONUS GUCU ====================
        self.curvature_power_boost_enable = gp('curvature_power_boost_enable')
        self.curvature_free_zone = gp('curvature_free_zone')
        self.turn_power_boost_max = gp('turn_power_boost_max')

        # ==================== STALL ====================
        self.stall_boost_rate = gp('stall_boost_rate')
        self.absolute_max_motor_power = gp('absolute_max_motor_power')

        # ==================== WATCHDOG ====================
        self.cmd_vel_timeout = gp('cmd_vel_timeout')
        self.odom_timeout = gp('odom_timeout')
        self.status_log_hz = gp('status_log_hz')
        self.csv_log_hz = gp('csv_log_hz')
        self.csv_flush_hz = gp('csv_flush_hz')

        # ==================== STEERING RATIO ====================
        self.max_physical_steer_deg = math.degrees(math.atan(self.wheelbase / self.min_turning_radius))
        self.steering_ratio = self.max_steer_mag / self.max_physical_steer_deg

        self.get_logger().info(f'[KINEMATIK] Wheelbase: {self.wheelbase}m, R_min: {self.min_turning_radius}m')
        self.get_logger().info(f'[KINEMATIK] Max Fiziksel Aci: {self.max_physical_steer_deg:.1f}°, Steering Ratio: {self.steering_ratio:.2f}')

    def _on_param_update(self, params):
        from rcl_interfaces.msg import SetParametersResult

        for p in params:
            if hasattr(self, p.name):
                setattr(self, p.name, p.value)
                self.get_logger().info(f'[PARAM] {p.name} = {p.value}')

            if p.name in ('angular_z_max', 'angular_z_min'):
                self.angular_z_mag = max(abs(self.angular_z_max), abs(self.angular_z_min))

            if p.name == 'steering_max_rate_dps':
                self.steering_ramp.max_rate_up = p.value
                self.steering_ramp.max_rate_down = p.value

        return SetParametersResult(successful=True)

    def obstacle_callback(self, msg: Int8):
        was_detected = self.obstacle_detected
        self.obstacle_detected = (msg.data == 1)

        if self.obstacle_detected and not was_detected:
            self.get_logger().warn('[ENGEL] Engel algilandi! Arac durdurulacak.')
            self.velocity_pid.reset()
            self.reverse_velocity_pid.reset()

    def odom_callback(self, msg: Float32):
        now = monotonic()
        self.last_odom_rx_time = now

        if self.last_odom is None:
            self.last_odom = msg.data
            self.last_odom_time = now
            return

        dt = now - self.last_odom_time
        if dt <= 0:
            return

        velocity_mps = ((msg.data - self.last_odom) / 100.0) / dt
        self.current_velocity = self.vel_filter.update(velocity_mps)

        self.last_odom = msg.data
        self.last_odom_time = now

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_vel_time = monotonic()

        if self.obstacle_detected:
            self.target_velocity = 0.0
            return

        if abs(msg.linear.x) <= 1e-3:
            self.target_velocity = 0.0
            self.target_steering_deg = 0.0
            self.angular_z_filtered = 0.0
            return

        if msg.linear.x > 0.0:
            self.desired_reverse = False
            self.target_velocity = min(msg.linear.x, self.max_velocity)
        else:
            self.desired_reverse = False
            self.target_velocity = max(msg.linear.x, -self.max_reverse_velocity)

        raw_angular = msg.angular.z
        if abs(raw_angular) < self.angular_deadband:
            raw_angular = 0.0

        raw_angular = max(self.angular_z_min, min(self.angular_z_max, raw_angular * self.angular_gain))
        self.angular_z_filtered = raw_angular

        # ==================== ACKERMANN ====================
        if abs(self.target_velocity) > 0.01:
            physical_steer_rad = math.atan((raw_angular * self.wheelbase) / self.target_velocity)
            physical_steer_deg = math.degrees(physical_steer_rad)
        else:
            physical_steer_deg = 0.0

        target_steer = -physical_steer_deg * self.steering_ratio
        self.target_steering_deg = max(self.STEER_MAX_RIGHT, min(self.STEER_MAX_LEFT, target_steer))

    def timer_callback(self):
        now = monotonic()
        dt = 0.02 if self.last_pid_time is None else max(1e-3, now - self.last_pid_time)
        self.last_pid_time = now

        # ==================== VITES GECISI ====================
        if self.desired_reverse != self.reverse_active:
            if abs(self.current_velocity) > self.gear_shift_velocity_threshold:
                self.last_motor_power = 0
                self.last_brake = True

                self.motor_power_pub.publish(Int8(data=0))
                self.brake_pub.publish(Bool(data=True))
                self.steering_angle_pub.publish(Int16(data=self.last_steering_deg))
                self.reverse_pub.publish(Bool(data=self.reverse_active))

                self.velocity_pid.reset()
                self.reverse_velocity_pid.reset()
                self.velocity_ramp.reset(0.0)
                self.reverse_velocity_ramp.reset(0.0)
                self.stall_timer = 0.0
                self.reverse_stall_timer = 0.0
                return

            self.reverse_active = self.desired_reverse
            self.reverse_pub.publish(Bool(data=self.reverse_active))

            self.velocity_pid.reset()
            self.reverse_velocity_pid.reset()
            self.velocity_ramp.reset(0.0)
            self.reverse_velocity_ramp.reset(0.0)
            self.stall_timer = 0.0
            self.reverse_stall_timer = 0.0

            self.get_logger().info('[VITES] GERI' if self.reverse_active else '[VITES] ILERI')
        else:
            self.reverse_pub.publish(Bool(data=self.reverse_active))

        # ==================== DIREKSIYON ====================
        smoothed_steer = self.steering_ramp.update(self.target_steering_deg, dt)
        self.last_steering_deg = int(round(smoothed_steer))
        curvature_fraction = abs(self.last_steering_deg) / self.max_steer_mag

        if self.curvature_power_boost_enable and curvature_fraction > self.curvature_free_zone:
            curvature_boost_fraction = (curvature_fraction - self.curvature_free_zone) / (1.0 - self.curvature_free_zone)
        else:
            curvature_boost_fraction = 0.0

        turn_boost = 0.0
        stall_boost = 0.0
        is_stop_command = (abs(self.target_velocity) <= 1e-3)

        if is_stop_command:
            self.last_motor_power = 0
            self.last_brake = (abs(self.current_velocity) > self.stall_velocity_threshold)
            self.velocity_pid.reset()
            self.reverse_velocity_pid.reset()
            self.stall_timer = 0.0
            self.reverse_stall_timer = 0.0

        # ==================== GERI VITES ====================
        elif self.target_velocity < 0.0:
            ramped_reverse_speed = self.reverse_velocity_ramp.update(abs(self.target_velocity), dt)
            current_reverse_speed = abs(self.current_velocity)

            if (current_reverse_speed - ramped_reverse_speed) >= self.reverse_overspeed_brake_margin:
                self.last_motor_power = 0
                self.last_brake = True
                self.reverse_velocity_pid.reset()
                self.reverse_stall_timer = 0.0
            else:
                reverse_error = ramped_reverse_speed - current_reverse_speed
                base_power = self.reverse_velocity_pid.compute(reverse_error, dt)
                turn_boost = curvature_boost_fraction * self.turn_power_boost_max

                if current_reverse_speed < self.stall_velocity_threshold:
                    self.reverse_stall_timer += dt
                    stall_boost = self.reverse_stall_timer * self.reverse_stall_boost_rate
                else:
                    self.reverse_stall_timer = 0.0

                motor_power = min(self.reverse_absolute_max_motor_power, base_power + turn_boost + stall_boost)
                self.last_motor_power = int(round(motor_power))
                self.last_brake = False

            self.velocity_pid.reset()
            self.stall_timer = 0.0

        # ==================== ILERI VITES ====================
        else:
            ramped_target = self.velocity_ramp.update(self.target_velocity, dt)

            if (self.current_velocity - ramped_target) >= self.overspeed_brake_margin:
                self.last_motor_power = 0
                self.last_brake = True
                self.velocity_pid.reset()
                self.stall_timer = 0.0
            else:
                error = ramped_target - self.current_velocity
                base_power = self.velocity_pid.compute(error, dt)
                turn_boost = curvature_boost_fraction * self.turn_power_boost_max

                if self.current_velocity < self.stall_velocity_threshold:
                    self.stall_timer += dt
                    stall_boost = self.stall_timer * self.stall_boost_rate
                else:
                    self.stall_timer = 0.0

                motor_power = min(self.absolute_max_motor_power, base_power + turn_boost + stall_boost)
                self.last_motor_power = int(round(motor_power))
                self.last_brake = False

            self.reverse_velocity_pid.reset()
            self.reverse_stall_timer = 0.0

        # ==================== STM YAYIN ====================
        self.motor_power_pub.publish(Int8(data=self.last_motor_power))
        self.brake_pub.publish(Bool(data=self.last_brake))
        self.steering_angle_pub.publish(Int16(data=self.last_steering_deg))

        # ==================== CSV ====================
        fren_durumu = "EVET" if self.last_brake else "HAYIR"
        geri_vites_durumu = "EVET" if self.reverse_active else "HAYIR"

        if self.csv_log_hz > 0.0 and (now - self.last_csv_log_time) >= (1.0 / self.csv_log_hz):
            self.last_csv_log_time = now
            current_time_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        if self.csv_flush_hz > 0.0 and (now - self.last_csv_flush_time) >= (1.0 / self.csv_flush_hz):
            self.csv_file.flush()
            self.last_csv_flush_time = now

        # ==================== TERMINAL ====================
        if self.status_log_hz > 0.0 and (now - self.last_status_log_time) >= (1.0 / self.status_log_hz):
            self.last_status_log_time = now
            vites_str = "GERI" if self.reverse_active else "ILERI"
            self.get_logger().info(
                f'HedefLinX: {self.target_velocity:5.2f} | AnlikHiz: {self.current_velocity:5.2f} | '
                f'Vites: {vites_str:5s} | Motor: {self.last_motor_power:3d} (turn={turn_boost:.1f}, '
                f'stall={stall_boost:.1f}) | HedefAngZ: {self.angular_z_filtered:+.3f} | '
                f'HedefAci: {self.target_steering_deg:+6.1f} | YayinAci: {self.last_steering_deg:4d} | '
                f'Fren: {fren_durumu}'
            )

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
            self.get_logger().info('CSV dosyasi guvenle kapatildi.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()