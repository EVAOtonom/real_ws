#!/usr/bin/env python3

import os
from collections import deque
from typing import Optional, Tuple

import numpy as np
import rclpy

from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from tf2_ros import (
    Buffer,
    TransformException,
    TransformListener,
)


class LaneMapper(Node):
    """
    /lane_pointcloud üzerindeki şerit noktalarını /odom pozuna göre
    odom frame'inde biriktirir ve /lane_map olarak yayınlar.

    Girişler:
        /lane_pointcloud : sensor_msgs/PointCloud2
        /odom            : nav_msgs/Odometry

    Çıkış:
        /lane_map        : sensor_msgs/PointCloud2
    """

    POINT_FIELD_DTYPES = {
        PointField.INT8: np.dtype("i1"),
        PointField.UINT8: np.dtype("u1"),
        PointField.INT16: np.dtype("i2"),
        PointField.UINT16: np.dtype("u2"),
        PointField.INT32: np.dtype("i4"),
        PointField.UINT32: np.dtype("u4"),
        PointField.FLOAT32: np.dtype("f4"),
        PointField.FLOAT64: np.dtype("f8"),
    }

    def __init__(self):
        super().__init__("lane_mapper")

        # Topic parametreleri
        self.declare_parameter(
            "lane_topic",
            "/lane_pointcloud"
        )

        self.declare_parameter(
            "odom_topic",
            "/odom"
        )

        self.declare_parameter(
            "output_topic",
            "/lane_map"
        )

        # Odom mesajında child_frame_id boşsa kullanılacak frame.
        self.declare_parameter(
            "base_frame",
            "base_link"
        )

        # Aynı hücreye düşen şerit noktaları birleştirilir.
        self.declare_parameter(
            "voxel_size",
            0.10
        )

        # Lane mesajı ile odom mesajı arasındaki maksimum zaman farkı.
        self.declare_parameter(
            "max_odom_age",
            0.25
        )

        # Tutulacak odom mesajı sayısı.
        self.declare_parameter(
            "odom_buffer_size",
            300
        )

        # Haritanın yayın frekansı.
        self.declare_parameter(
            "publish_rate",
            2.0
        )

        # Haritadaki maksimum voxel sayısı.
        self.declare_parameter(
            "max_voxels",
            500000
        )

        # Şerit noktalarının sensöre göre minimum ve maksimum mesafesi.
        self.declare_parameter(
            "min_range",
            0.20
        )

        self.declare_parameter(
            "max_range",
            40.0
        )

        # Odom frame'indeki Z filtresi.
        self.declare_parameter(
            "min_map_z",
            -3.0
        )

        self.declare_parameter(
            "max_map_z",
            3.0
        )

        # TF bulunamazsa lane bulutunu odom child frame'inde varsay.
        # Normalde false kalmalı.
        self.declare_parameter(
            "assume_lane_in_base_frame",
            False
        )

        # Program kapanırken CSV kaydetmek için.
        # Boş bırakılırsa kayıt yapılmaz.
        self.declare_parameter(
            "save_csv_path",
            ""
        )

        self.lane_topic = self.get_parameter(
            "lane_topic"
        ).value

        self.odom_topic = self.get_parameter(
            "odom_topic"
        ).value

        self.output_topic = self.get_parameter(
            "output_topic"
        ).value

        self.default_base_frame = self.get_parameter(
            "base_frame"
        ).value

        self.voxel_size = float(
            self.get_parameter("voxel_size").value
        )

        self.max_odom_age = float(
            self.get_parameter("max_odom_age").value
        )

        odom_buffer_size = int(
            self.get_parameter("odom_buffer_size").value
        )

        self.publish_rate = float(
            self.get_parameter("publish_rate").value
        )

        self.max_voxels = int(
            self.get_parameter("max_voxels").value
        )

        self.min_range = float(
            self.get_parameter("min_range").value
        )

        self.max_range = float(
            self.get_parameter("max_range").value
        )

        self.min_map_z = float(
            self.get_parameter("min_map_z").value
        )

        self.max_map_z = float(
            self.get_parameter("max_map_z").value
        )

        self.assume_lane_in_base_frame = bool(
            self.get_parameter(
                "assume_lane_in_base_frame"
            ).value
        )

        self.save_csv_path = self.get_parameter(
            "save_csv_path"
        ).value

        if self.voxel_size <= 0.0:
            raise ValueError(
                "voxel_size sıfırdan büyük olmalıdır."
            )

        if self.publish_rate <= 0.0:
            raise ValueError(
                "publish_rate sıfırdan büyük olmalıdır."
            )

        # Odom geçmişi
        self.odom_buffer = deque(
            maxlen=odom_buffer_size
        )

        # Voxel:
        #
        # key = (voxel_x, voxel_y, voxel_z)
        # value = [sum_x, sum_y, sum_z, point_count]
        #
        # Aynı bölge tekrar görüldüğünde ortalama alınır.
        self.voxel_map = {}

        self.current_map_frame = ""
        self.last_lane_stamp = None

        self.received_lane_clouds = 0
        self.accepted_lane_clouds = 0
        self.rejected_lane_clouds = 0
        self.total_received_lane_points = 0
        self.total_accepted_lane_points = 0

        # TF yalnızca kamera/sensör -> odom child frame dönüşümü için.
        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.lane_subscription = self.create_subscription(
            PointCloud2,
            self.lane_topic,
            self.lane_callback,
            qos_profile_sensor_data
        )

        self.map_publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos_profile_sensor_data
        )

        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_map
        )

        # Mor renk
        self.purple_rgb = self.pack_rgb_float(
            red=180,
            green=0,
            blue=255
        )

        self.get_logger().info(
            "Lane Mapper başlatıldı.\n"
            f"Lane topic  : {self.lane_topic}\n"
            f"Odom topic  : {self.odom_topic}\n"
            f"Harita topic: {self.output_topic}\n"
            f"Voxel boyutu: {self.voxel_size:.3f} m\n"
            f"Odom toleransı: {self.max_odom_age:.3f} s"
        )

    @staticmethod
    def stamp_to_seconds(stamp) -> float:
        return (
            float(stamp.sec) +
            float(stamp.nanosec) * 1e-9
        )

    @staticmethod
    def pack_rgb_float(
        red: int,
        green: int,
        blue: int
    ) -> np.float32:
        rgb_uint32 = np.uint32(
            (int(red) << 16) |
            (int(green) << 8) |
            int(blue)
        )

        return np.asarray(
            [rgb_uint32],
            dtype=np.uint32
        ).view(np.float32)[0]

    @staticmethod
    def quaternion_to_rotation_matrix(
        x: float,
        y: float,
        z: float,
        w: float
    ) -> np.ndarray:
        norm = np.sqrt(
            x * x +
            y * y +
            z * z +
            w * w
        )

        if norm < 1e-12:
            return np.eye(
                3,
                dtype=np.float32
            )

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        return np.array(
            [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w)
                ],
                [
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w)
                ],
                [
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y)
                ]
            ],
            dtype=np.float32
        )

    def get_field(
        self,
        cloud_msg: PointCloud2,
        field_name: str
    ) -> PointField:
        for field in cloud_msg.fields:
            if field.name == field_name:
                return field

        available_fields = [
            field.name
            for field in cloud_msg.fields
        ]

        raise ValueError(
            f"'{field_name}' alanı bulunamadı. "
            f"Mevcut alanlar: {available_fields}"
        )

    def get_numpy_datatype(
        self,
        field: PointField,
        is_bigendian: bool
    ) -> np.dtype:
        if field.datatype not in self.POINT_FIELD_DTYPES:
            raise ValueError(
                f"Desteklenmeyen PointField datatype: "
                f"{field.datatype}, alan: {field.name}"
            )

        datatype = self.POINT_FIELD_DTYPES[
            field.datatype
        ]

        if datatype.itemsize > 1:
            byte_order = ">" if is_bigendian else "<"

            datatype = datatype.newbyteorder(
                byte_order
            )

        return datatype

    def pointcloud_to_xyz(
        self,
        cloud_msg: PointCloud2
    ) -> np.ndarray:
        """
        PointCloud2 içinden yalnızca x, y, z alanlarını doğrudan
        byte buffer üzerinden okur.

        Böylece rgb, ring, timestamp gibi farklı tipteki alanlardan
        etkilenmez.
        """

        if cloud_msg.width == 0 or cloud_msg.height == 0:
            return np.empty(
                (0, 3),
                dtype=np.float32
            )

        x_field = self.get_field(
            cloud_msg,
            "x"
        )

        y_field = self.get_field(
            cloud_msg,
            "y"
        )

        z_field = self.get_field(
            cloud_msg,
            "z"
        )

        structured_dtype = np.dtype(
            {
                "names": [
                    "x",
                    "y",
                    "z"
                ],
                "formats": [
                    self.get_numpy_datatype(
                        x_field,
                        cloud_msg.is_bigendian
                    ),
                    self.get_numpy_datatype(
                        y_field,
                        cloud_msg.is_bigendian
                    ),
                    self.get_numpy_datatype(
                        z_field,
                        cloud_msg.is_bigendian
                    )
                ],
                "offsets": [
                    x_field.offset,
                    y_field.offset,
                    z_field.offset
                ],
                "itemsize": cloud_msg.point_step
            }
        )

        point_array = np.ndarray(
            shape=(
                cloud_msg.height,
                cloud_msg.width
            ),
            dtype=structured_dtype,
            buffer=memoryview(cloud_msg.data),
            strides=(
                cloud_msg.row_step,
                cloud_msg.point_step
            )
        )

        x = np.asarray(
            point_array["x"],
            dtype=np.float32
        ).reshape(-1)

        y = np.asarray(
            point_array["y"],
            dtype=np.float32
        ).reshape(-1)

        z = np.asarray(
            point_array["z"],
            dtype=np.float32
        ).reshape(-1)

        xyz = np.column_stack(
            (x, y, z)
        ).astype(
            np.float32,
            copy=False
        )

        valid_mask = np.isfinite(
            xyz
        ).all(axis=1)

        return xyz[valid_mask]

    def odom_callback(
        self,
        odom_msg: Odometry
    ):
        if not odom_msg.header.frame_id:
            self.get_logger().warning(
                "/odom mesajının header.frame_id alanı boş.",
                throttle_duration_sec=2.0
            )
            return

        self.odom_buffer.append(
            odom_msg
        )

    def find_closest_odom(
        self,
        lane_stamp
    ) -> Tuple[Optional[Odometry], float]:
        if not self.odom_buffer:
            return None, float("inf")

        lane_time = self.stamp_to_seconds(
            lane_stamp
        )

        closest_odom = min(
            self.odom_buffer,
            key=lambda odom: abs(
                self.stamp_to_seconds(
                    odom.header.stamp
                ) - lane_time
            )
        )

        time_difference = abs(
            self.stamp_to_seconds(
                closest_odom.header.stamp
            ) - lane_time
        )

        return closest_odom, time_difference

    def transform_sensor_to_body(
        self,
        points: np.ndarray,
        sensor_frame: str,
        body_frame: str,
        stamp
    ) -> np.ndarray:
        if points.shape[0] == 0:
            return points

        if sensor_frame == body_frame:
            return points

        try:
            transform = self.tf_buffer.lookup_transform(
                body_frame,
                sensor_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=0.10)
            )

        except TransformException as exact_error:
            try:
                # Statik TF zaman uyuşmazlığı yaşarsa en güncel TF.
                transform = self.tf_buffer.lookup_transform(
                    body_frame,
                    sensor_frame,
                    Time(),
                    timeout=Duration(seconds=0.10)
                )

            except TransformException as latest_error:
                if self.assume_lane_in_base_frame:
                    self.get_logger().warning(
                        f"TF bulunamadı: "
                        f"{sensor_frame} -> {body_frame}. "
                        "Noktalar base frame'de varsayılıyor.",
                        throttle_duration_sec=2.0
                    )

                    return points

                raise RuntimeError(
                    f"TF bulunamadı: "
                    f"{sensor_frame} -> {body_frame}. "
                    f"İlk hata: {exact_error}. "
                    f"Son hata: {latest_error}"
                ) from latest_error

        translation = transform.transform.translation
        quaternion = transform.transform.rotation

        rotation_matrix = (
            self.quaternion_to_rotation_matrix(
                quaternion.x,
                quaternion.y,
                quaternion.z,
                quaternion.w
            )
        )

        translation_vector = np.array(
            [
                translation.x,
                translation.y,
                translation.z
            ],
            dtype=np.float32
        )

        return (
            points @ rotation_matrix.T
            + translation_vector
        ).astype(
            np.float32,
            copy=False
        )

    def transform_body_to_odom(
        self,
        points: np.ndarray,
        odom_msg: Odometry
    ) -> np.ndarray:
        """
        nav_msgs/Odometry içindeki poz:

            header.frame_id <- child_frame_id

        dönüşümünü ifade eder.
        """

        pose = odom_msg.pose.pose

        rotation_matrix = (
            self.quaternion_to_rotation_matrix(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            )
        )

        translation_vector = np.array(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z
            ],
            dtype=np.float32
        )

        return (
            points @ rotation_matrix.T
            + translation_vector
        ).astype(
            np.float32,
            copy=False
        )

    def filter_lane_points(
        self,
        sensor_points: np.ndarray
    ) -> np.ndarray:
        if sensor_points.shape[0] == 0:
            return sensor_points

        ranges = np.linalg.norm(
            sensor_points,
            axis=1
        )

        valid_mask = (
            (ranges >= self.min_range) &
            (ranges <= self.max_range)
        )

        return sensor_points[valid_mask]

    def update_voxel_map(
        self,
        points: np.ndarray
    ):
        if points.shape[0] == 0:
            return

        # Odom frame'indeki yükseklik filtresi
        height_mask = (
            (points[:, 2] >= self.min_map_z) &
            (points[:, 2] <= self.max_map_z)
        )

        points = points[height_mask]

        if points.shape[0] == 0:
            return

        voxel_indices = np.floor(
            points / self.voxel_size
        ).astype(np.int64)

        unique_voxels, inverse_indices, counts = np.unique(
            voxel_indices,
            axis=0,
            return_inverse=True,
            return_counts=True
        )

        voxel_sums = np.zeros(
            (unique_voxels.shape[0], 3),
            dtype=np.float64
        )

        np.add.at(
            voxel_sums,
            inverse_indices,
            points
        )

        for voxel_index, point_sum, point_count in zip(
            unique_voxels,
            voxel_sums,
            counts
        ):
            voxel_key = (
                int(voxel_index[0]),
                int(voxel_index[1]),
                int(voxel_index[2])
            )

            existing_voxel = self.voxel_map.get(
                voxel_key
            )

            if existing_voxel is not None:
                existing_voxel[0] += point_sum[0]
                existing_voxel[1] += point_sum[1]
                existing_voxel[2] += point_sum[2]
                existing_voxel[3] += int(point_count)

            elif len(self.voxel_map) < self.max_voxels:
                self.voxel_map[voxel_key] = [
                    float(point_sum[0]),
                    float(point_sum[1]),
                    float(point_sum[2]),
                    int(point_count)
                ]

    def lane_callback(
        self,
        lane_msg: PointCloud2
    ):
        self.received_lane_clouds += 1

        try:
            if not lane_msg.header.frame_id:
                raise ValueError(
                    "/lane_pointcloud frame_id alanı boş."
                )

            closest_odom, odom_age = (
                self.find_closest_odom(
                    lane_msg.header.stamp
                )
            )

            if closest_odom is None:
                self.rejected_lane_clouds += 1

                self.get_logger().warning(
                    "Henüz /odom mesajı alınmadı.",
                    throttle_duration_sec=2.0
                )
                return

            if odom_age > self.max_odom_age:
                self.rejected_lane_clouds += 1

                self.get_logger().warning(
                    f"Lane ve odom zamanları uyuşmuyor. "
                    f"Fark: {odom_age:.3f} s, "
                    f"izin verilen: {self.max_odom_age:.3f} s",
                    throttle_duration_sec=1.0
                )
                return

            new_map_frame = (
                closest_odom.header.frame_id
            )

            if (
                self.current_map_frame and
                new_map_frame != self.current_map_frame
            ):
                self.get_logger().warning(
                    f"Odom frame değişti: "
                    f"{self.current_map_frame} -> {new_map_frame}. "
                    "Eski lane haritası temizleniyor."
                )

                self.voxel_map.clear()

            self.current_map_frame = new_map_frame

            body_frame = (
                closest_odom.child_frame_id
                if closest_odom.child_frame_id
                else self.default_base_frame
            )

            sensor_points = self.pointcloud_to_xyz(
                lane_msg
            )

            self.total_received_lane_points += (
                sensor_points.shape[0]
            )

            sensor_points = self.filter_lane_points(
                sensor_points
            )

            # Kamera/sensör frame -> aracın body frame'i
            body_points = self.transform_sensor_to_body(
                points=sensor_points,
                sensor_frame=lane_msg.header.frame_id,
                body_frame=body_frame,
                stamp=lane_msg.header.stamp
            )

            # Aracın body frame'i -> odom frame
            odom_points = self.transform_body_to_odom(
                points=body_points,
                odom_msg=closest_odom
            )

            self.update_voxel_map(
                odom_points
            )

            self.last_lane_stamp = (
                lane_msg.header.stamp
            )

            self.accepted_lane_clouds += 1
            self.total_accepted_lane_points += (
                odom_points.shape[0]
            )

            if self.accepted_lane_clouds % 20 == 0:
                self.get_logger().info(
                    f"Lane map güncellendi | "
                    f"Bulut noktası: {odom_points.shape[0]} | "
                    f"Voxel sayısı: {len(self.voxel_map)} | "
                    f"Odom farkı: {odom_age:.3f} s | "
                    f"Frame: {self.current_map_frame}"
                )

        except Exception as error:
            self.rejected_lane_clouds += 1

            self.get_logger().warning(
                f"Lane bulutu haritaya eklenemedi: {error}",
                throttle_duration_sec=1.0
            )

    def get_map_points(self) -> np.ndarray:
        if not self.voxel_map:
            return np.empty(
                (0, 3),
                dtype=np.float32
            )

        map_points = np.empty(
            (len(self.voxel_map), 3),
            dtype=np.float32
        )

        for index, voxel_data in enumerate(
            self.voxel_map.values()
        ):
            point_count = max(
                int(voxel_data[3]),
                1
            )

            map_points[index, 0] = (
                voxel_data[0] / point_count
            )

            map_points[index, 1] = (
                voxel_data[1] / point_count
            )

            map_points[index, 2] = (
                voxel_data[2] / point_count
            )

        return map_points

    def create_lane_map_message(
        self,
        map_points: np.ndarray
    ) -> PointCloud2:
        output_dtype = np.dtype(
            [
                ("x", "<f4"),
                ("y", "<f4"),
                ("z", "<f4"),
                ("rgb", "<f4")
            ]
        )

        output_array = np.empty(
            map_points.shape[0],
            dtype=output_dtype
        )

        output_array["x"] = map_points[:, 0]
        output_array["y"] = map_points[:, 1]
        output_array["z"] = map_points[:, 2]
        output_array["rgb"] = self.purple_rgb

        output_msg = PointCloud2()

        output_msg.header = Header()
        output_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        output_msg.header.frame_id = (
            self.current_map_frame
        )

        output_msg.height = 1
        output_msg.width = map_points.shape[0]

        output_msg.fields = [
            PointField(
                name="x",
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name="y",
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name="z",
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name="rgb",
                offset=12,
                datatype=PointField.FLOAT32,
                count=1
            )
        ]

        output_msg.is_bigendian = False
        output_msg.point_step = output_dtype.itemsize

        output_msg.row_step = (
            output_msg.point_step *
            output_msg.width
        )

        output_msg.is_dense = True
        output_msg.data = output_array.tobytes()

        return output_msg

    def publish_map(self):
        if not self.current_map_frame:
            return

        if not self.voxel_map:
            return

        map_points = self.get_map_points()

        if map_points.shape[0] == 0:
            return

        output_msg = self.create_lane_map_message(
            map_points
        )

        self.map_publisher.publish(
            output_msg
        )

    def save_map_to_csv(self):
        if not self.save_csv_path:
            return

        map_points = self.get_map_points()

        if map_points.shape[0] == 0:
            self.get_logger().warning(
                "Lane map boş olduğu için CSV kaydedilmedi."
            )
            return

        output_path = os.path.expanduser(
            self.save_csv_path
        )

        output_directory = os.path.dirname(
            output_path
        )

        if output_directory:
            os.makedirs(
                output_directory,
                exist_ok=True
            )

        np.savetxt(
            output_path,
            map_points,
            delimiter=",",
            header="x,y,z",
            comments="",
            fmt="%.6f"
        )

        self.get_logger().info(
            f"Lane map CSV olarak kaydedildi: "
            f"{output_path} | "
            f"Nokta sayısı: {map_points.shape[0]}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = LaneMapper()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            "Lane Mapper kapatılıyor."
        )

    finally:
        node.save_map_to_csv()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()