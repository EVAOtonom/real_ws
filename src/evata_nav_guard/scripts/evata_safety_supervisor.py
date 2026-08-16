#!/usr/bin/env python3

import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
)

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool


class EvataSafetySupervisor(Node):
    """
    Fail-safe supervisor.

    - /imu/data and /rslidar_points must remain fresh.
    - Critical ROS node name fragments must be present.
    - Nav2 publishes to /cmd_vel_nav; only this node publishes to /cmd_vel.
    - On any fault, /cmd_vel is forced to zero and /evata/system_healthy=false.
    - Recovery requires all checks to stay healthy continuously for a stabilization period.
    """

    def __init__(self):
        super().__init__('evata_safety_supervisor')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('lidar_topic', '/rslidar_points')
        self.declare_parameter('nav_cmd_topic', '/cmd_vel_nav')
        self.declare_parameter('robot_cmd_topic', '/cmd_vel')
        self.declare_parameter('health_topic', '/evata/system_healthy')
        self.declare_parameter('sensor_timeout_sec', 1.0)
        self.declare_parameter('startup_grace_sec', 15.0)
        self.declare_parameter('recovery_stable_sec', 2.0)
        self.declare_parameter(
            'required_node_fragments',
            [
                'aks',
                'microstrain',
                'rslidar',
                'zed',
                'lio',
                'localization',
                'controller_server',
                'planner_server',
                'bt_navigator',
            ]
        )

        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.lidar_topic = str(self.get_parameter('lidar_topic').value)
        self.nav_cmd_topic = str(self.get_parameter('nav_cmd_topic').value)
        self.robot_cmd_topic = str(self.get_parameter('robot_cmd_topic').value)
        self.health_topic = str(self.get_parameter('health_topic').value)
        self.sensor_timeout = float(self.get_parameter('sensor_timeout_sec').value)
        self.startup_grace = float(self.get_parameter('startup_grace_sec').value)
        self.recovery_stable = float(self.get_parameter('recovery_stable_sec').value)
        self.required_node_fragments: List[str] = [
            str(x).lower() for x in self.get_parameter('required_node_fragments').value
        ]

        now = time.monotonic()
        self.start_time = now
        self.last_imu = 0.0
        self.last_lidar = 0.0
        self.healthy_since = None
        self.system_healthy = False
        self.last_nav_cmd = Twist()

        health_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.health_pub = self.create_publisher(Bool, self.health_topic, health_qos)
        self.cmd_pub = self.create_publisher(Twist, self.robot_cmd_topic, 10)

        self.create_subscription(
            Imu, self.imu_topic, self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, self.lidar_topic, self._lidar_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, self.nav_cmd_topic, self._cmd_cb, 10)

        self.timer = self.create_timer(0.05, self._tick)  # 20 Hz safety output
        self.log_timer = self.create_timer(1.0, self._status_log)

        self._publish_health(False)
        self._publish_stop()
        self.get_logger().info('EVATA safety supervisor started in FAIL-SAFE state')

    def _imu_cb(self, _msg):
        self.last_imu = time.monotonic()

    def _lidar_cb(self, _msg):
        self.last_lidar = time.monotonic()

    def _cmd_cb(self, msg: Twist):
        self.last_nav_cmd = msg

    def _node_fragment_health(self):
        names = []
        for name, namespace in self.get_node_names_and_namespaces():
            full = f'{namespace}/{name}'.replace('//', '/').lower()
            names.append(full)

        missing = [
            fragment for fragment in self.required_node_fragments
            if not any(fragment in node_name for node_name in names)
        ]
        return len(missing) == 0, missing

    def _raw_health(self):
        now = time.monotonic()

        # During startup we are intentionally stopped but do not treat missing
        # initial samples as a permanent fault.
        imu_ok = self.last_imu > 0.0 and (now - self.last_imu) <= self.sensor_timeout
        lidar_ok = self.last_lidar > 0.0 and (now - self.last_lidar) <= self.sensor_timeout
        nodes_ok, missing_nodes = self._node_fragment_health()

        return imu_ok and lidar_ok and nodes_ok, imu_ok, lidar_ok, missing_nodes

    def _tick(self):
        now = time.monotonic()
        raw_ok, _imu_ok, _lidar_ok, _missing = self._raw_health()

        if now - self.start_time < self.startup_grace:
            raw_ok = False

        if not raw_ok:
            self.healthy_since = None
            if self.system_healthy:
                self.system_healthy = False
                self._publish_health(False)
                self.get_logger().error('SYSTEM FAULT -> NAV2 PAUSE + HARD STOP')
            self._publish_stop()
            return

        if self.healthy_since is None:
            self.healthy_since = now

        stable = (now - self.healthy_since) >= self.recovery_stable
        if not stable:
            self._publish_stop()
            return

        if not self.system_healthy:
            self.system_healthy = True
            self._publish_health(True)
            self.get_logger().info('SYSTEM STABLE -> NAV2 RESUME + CMD_VEL ENABLED')

        self.cmd_pub.publish(self.last_nav_cmd)

    def _publish_health(self, value: bool):
        msg = Bool()
        msg.data = value
        self.health_pub.publish(msg)

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _status_log(self):
        raw_ok, imu_ok, lidar_ok, missing = self._raw_health()
        self.get_logger().info(
            f'health={self.system_healthy} raw={raw_ok} '
            f'imu={imu_ok} lidar={lidar_ok} missing_nodes={missing}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = EvataSafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best effort stop on clean shutdown.
        for _ in range(3):
            node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
