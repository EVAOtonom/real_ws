#!/usr/bin/env python3

import math
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformBroadcaster,
    TransformException
)


def quaternion_multiply(q1, q2):
    """
    q = q1 * q2

    Quaternion format:
    [x, y, z, w]
    """

    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

    return [x, y, z, w]


def quaternion_inverse(q):
    """
    Unit quaternion inverse.
    """

    x, y, z, w = q

    norm_sq = x*x + y*y + z*z + w*w

    if norm_sq < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]

    return [
        -x / norm_sq,
        -y / norm_sq,
        -z / norm_sq,
         w / norm_sq
    ]


def rotate_vector(q, v):
    """
    Quaternion ile 3B vektör döndür.
    """

    q_vec = [v[0], v[1], v[2], 0.0]
    q_inv = quaternion_inverse(q)

    result = quaternion_multiply(
        quaternion_multiply(q, q_vec),
        q_inv
    )

    return [result[0], result[1], result[2]]


def normalize_quaternion(q):
    norm = math.sqrt(
        q[0] * q[0] +
        q[1] * q[1] +
        q[2] * q[2] +
        q[3] * q[3]
    )

    if norm < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]

    return [
        q[0] / norm,
        q[1] / norm,
        q[2] / norm,
        q[3] / norm
    ]


def yaw_from_quaternion(q):
    x, y, z, w = q

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(siny_cosp, cosy_cosp)


class InitialPoseTFReset(Node):

    def __init__(self):
        super().__init__('initialpose_tf_reset')

        # ==========================================================
        # PARAMETRELER
        # ==========================================================

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('publish_rate', 50.0)

        self.map_frame = (
            self.get_parameter('map_frame')
            .get_parameter_value()
            .string_value
        )

        self.odom_frame = (
            self.get_parameter('odom_frame')
            .get_parameter_value()
            .string_value
        )

        self.base_frame = (
            self.get_parameter('base_frame')
            .get_parameter_value()
            .string_value
        )

        self.initialpose_topic = (
            self.get_parameter('initialpose_topic')
            .get_parameter_value()
            .string_value
        )

        publish_rate = (
            self.get_parameter('publish_rate')
            .get_parameter_value()
            .double_value
        )

        # ==========================================================
        # TF
        # ==========================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.tf_broadcaster = TransformBroadcaster(
            self
        )

        # Hesapladığımız map -> odom
        self.map_to_odom_translation = [
            0.0,
            0.0,
            0.0
        ]

        self.map_to_odom_rotation = [
            0.0,
            0.0,
            0.0,
            1.0
        ]

        self.transform_initialized = False

        # ==========================================================
        # INITIALPOSE SUBSCRIBER
        # ==========================================================

        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            self.initialpose_callback,
            10
        )

        # ==========================================================
        # TF YAYIN TIMER
        # ==========================================================

        period = 1.0 / max(publish_rate, 1.0)

        self.timer = self.create_timer(
            period,
            self.publish_map_to_odom
        )

        self.get_logger().info(
            'InitialPose TF Reset node baslatildi.'
        )

        self.get_logger().info(
            f'InitialPose topic : {self.initialpose_topic}'
        )

        self.get_logger().info(
            f'TF zinciri        : '
            f'{self.map_frame} -> '
            f'{self.odom_frame} -> '
            f'{self.base_frame}'
        )

    # ==============================================================
    # INITIALPOSE
    # ==============================================================

    def initialpose_callback(self, msg):

        self.get_logger().info(
            '/initialpose alindi.'
        )

        # ----------------------------------------------------------
        # İstenen MAP -> BASE_FOOTPRINT
        # ----------------------------------------------------------

        desired_translation = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]

        desired_rotation = normalize_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

        # ----------------------------------------------------------
        # Mevcut ODOM -> BASE_FOOTPRINT
        #
        # lookup_transform(target, source)
        #
        # target = odom
        # source = base_footprint
        #
        # sonuç:
        #
        # T_odom_base
        # ----------------------------------------------------------

        try:

            tf_odom_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time()
            )

        except TransformException as ex:

            self.get_logger().error(
                f'{self.odom_frame} -> '
                f'{self.base_frame} TF bulunamadi: {ex}'
            )

            return

        odom_base_translation = [
            tf_odom_base.transform.translation.x,
            tf_odom_base.transform.translation.y,
            tf_odom_base.transform.translation.z
        ]

        odom_base_rotation = normalize_quaternion([
            tf_odom_base.transform.rotation.x,
            tf_odom_base.transform.rotation.y,
            tf_odom_base.transform.rotation.z,
            tf_odom_base.transform.rotation.w
        ])

        # ==========================================================
        # T_map_odom =
        #
        # T_map_base_desired *
        # inverse(T_odom_base)
        # ==========================================================

        # ----------------------------------------------------------
        # Rotasyon
        #
        # R_map_odom =
        # R_map_base * inverse(R_odom_base)
        # ----------------------------------------------------------

        odom_base_rotation_inv = quaternion_inverse(
            odom_base_rotation
        )

        map_odom_rotation = quaternion_multiply(
            desired_rotation,
            odom_base_rotation_inv
        )

        map_odom_rotation = normalize_quaternion(
            map_odom_rotation
        )

        # ----------------------------------------------------------
        # Translation
        #
        # p_map_base =
        # p_map_odom +
        # R_map_odom * p_odom_base
        #
        # dolayısıyla:
        #
        # p_map_odom =
        # p_map_base -
        # R_map_odom * p_odom_base
        # ----------------------------------------------------------

        rotated_odom_base = rotate_vector(
            map_odom_rotation,
            odom_base_translation
        )

        map_odom_translation = [
            desired_translation[0] - rotated_odom_base[0],
            desired_translation[1] - rotated_odom_base[1],
            desired_translation[2] - rotated_odom_base[2]
        ]

        # ----------------------------------------------------------
        # Kaydet
        # ----------------------------------------------------------

        self.map_to_odom_translation = map_odom_translation
        self.map_to_odom_rotation = map_odom_rotation

        self.transform_initialized = True

        desired_yaw = math.degrees(
            yaw_from_quaternion(desired_rotation)
        )

        odom_yaw = math.degrees(
            yaw_from_quaternion(odom_base_rotation)
        )

        map_odom_yaw = math.degrees(
            yaw_from_quaternion(map_odom_rotation)
        )

        self.get_logger().info(
            '\n'
            '=============================================\n'
            '       INITIAL POSE UYGULANDI\n'
            '=============================================\n'
            f'Istenen map -> base_footprint:\n'
            f'  x   = {desired_translation[0]:.3f}\n'
            f'  y   = {desired_translation[1]:.3f}\n'
            f'  yaw = {desired_yaw:.2f} derece\n'
            '\n'
            f'Mevcut odom -> base_footprint:\n'
            f'  x   = {odom_base_translation[0]:.3f}\n'
            f'  y   = {odom_base_translation[1]:.3f}\n'
            f'  yaw = {odom_yaw:.2f} derece\n'
            '\n'
            f'Hesaplanan map -> odom:\n'
            f'  x   = {map_odom_translation[0]:.3f}\n'
            f'  y   = {map_odom_translation[1]:.3f}\n'
            f'  yaw = {map_odom_yaw:.2f} derece\n'
            '============================================='
        )

        # Beklemeden ilk TF'yi gönder
        self.publish_map_to_odom()

    # ==============================================================
    # MAP -> ODOM YAYINLA
    # ==============================================================

    def publish_map_to_odom(self):

        if not self.transform_initialized:
            return

        tf_msg = TransformStamped()

        tf_msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.odom_frame

        tf_msg.transform.translation.x = (
            self.map_to_odom_translation[0]
        )

        tf_msg.transform.translation.y = (
            self.map_to_odom_translation[1]
        )

        tf_msg.transform.translation.z = (
            self.map_to_odom_translation[2]
        )

        tf_msg.transform.rotation.x = (
            self.map_to_odom_rotation[0]
        )

        tf_msg.transform.rotation.y = (
            self.map_to_odom_rotation[1]
        )

        tf_msg.transform.rotation.z = (
            self.map_to_odom_rotation[2]
        )

        tf_msg.transform.rotation.w = (
            self.map_to_odom_rotation[3]
        )

        self.tf_broadcaster.sendTransform(
            tf_msg
        )


def main(args=None):

    rclpy.init(args=args)

    node = InitialPoseTFReset()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
