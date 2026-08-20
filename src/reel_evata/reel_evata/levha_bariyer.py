#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header
from visualization_msgs.msg import Marker, MarkerArray

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from nav2_msgs.srv import ClearEntireCostmap


# ==============================================================
# DUVAR NOKTALARINI OLUŞTUR
# ==============================================================

def generate_wall_points(
    cx: float,
    cy: float,
    yaw: float,
    length: float,
    width: float,
    z_min: float = 0.0,
    z_max: float = 2.0,
    xy_res: float = 0.7,
    z_res: float = 0.8,
) -> list:

    points = []

    x_local = -length / 2.0

    while x_local <= length / 2.0:

        y_local = -width / 2.0

        while y_local <= width / 2.0:

            wx = (
                cx
                + x_local * math.cos(yaw)
                - y_local * math.sin(yaw)
            )

            wy = (
                cy
                + x_local * math.sin(yaw)
                + y_local * math.cos(yaw)
            )

            z = z_min

            while z <= z_max:

                points.append([
                    wx,
                    wy,
                    z
                ])

                z += z_res

            y_local += xy_res

        x_local += xy_res

    return points


class SignDynamicObstacle(Node):

    def __init__(self):

        super().__init__("levha_dinamik_engel")

        # ======================================================
        # AYARLAR
        # ======================================================

        self.sign_trigger_distance = 10.0

        self.wall_width = 0.10

        self.wall_z_min = 0.0
        self.wall_z_max = 2.0

        self.wall_edge_margin = 0.5

        # Bariyer 20 saniye aktif
        self.wall_duration = 20.0

        self.walls = {}

        self.next_marker_id = 0

        # JSON
        self.barriers = self.load_barriers()

        # TF
        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # SUBSCRIBER

        self.sign_sub = self.create_subscription(
            String,
            "/detected_signs",
            self.sign_callback,
            10
        )

        # MARKER

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/sign_wall_markers",
            10
        )

        # LOCAL COSTMAP

        self.obstacle_pub = self.create_publisher(
            PointCloud2,
            "/sign_obstacles",
            10
        )

        # GLOBAL COSTMAP

        self.obstacle_global_pub = self.create_publisher(
            PointCloud2,
            "/sign_obstacles_global",
            10
        )

        # ======================================================
        # COSTMAP CLEAR
        # ======================================================

        self.clear_local_cli = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap"
        )

        self.clear_global_cli = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap"
        )


        # Hem yayın yap
        # Hem süresi dolan bariyerleri sil
        # ======================================================

        self.publish_timer = self.create_timer(
            0.2,
            self.update_walls
        )

        self.get_logger().info(
            "=============================================="
        )

        self.get_logger().info(
            "Levha Dinamik Engel Node Baslatildi"
        )

    # JSON
    # ==========================================================

    def load_barriers(self):

        try:

            path = (
                "/home/otonom/real_ws/src/reel_evata/"
                "reel_evata/barriers.json"
            )

            with open(path, "r") as f:
                data = json.load(f)

            self.get_logger().info(
                f"{len(data['junctions'])} kavsak yuklendi"
            )

            return data["junctions"]

        except Exception as e:

            self.get_logger().error(
                f"Barrier dosyasi okunamadi: {e}"
            )

            return []

    # LEVHA CALLBACK
    # ==========================================================

    def sign_callback(self, msg: String):

        try:

            data = json.loads(msg.data)

        except Exception as e:

            self.get_logger().warn(
                f"JSON parse hatasi: {e}"
            )

            return

        # KAVŞAK
        # ======================================================

        junction = self.find_current_junction()

        if junction is None:
            return

        # ROBOT YAW
        # ======================================================

        robot_yaw_deg = self.get_robot_map_yaw_deg()

        if robot_yaw_deg is None:
            return

        # ANGLE SET
        # ======================================================

        angle_set = self.find_angle_set(
            junction,
            robot_yaw_deg
        )

        if angle_set is None:

            self.get_logger().warn(
                f"[NO ANGLE SET] "
                f"Junction={junction['id']} "
                f"yaw={robot_yaw_deg:.2f}"
            )

            return

        self.get_logger().info(
            f"[ANGLE SET] "
            f"{angle_set['name']} secildi "
            f"yaw={robot_yaw_deg:.2f}"
        )

        active_barriers = angle_set["barriers"]

        # LEVHA -> BARIYER
        # ======================================================

        sign_map = {

            "sol": [
                "front",
                "right"
            ],

            "sag": [
                "front",
                "left"
            ],

            "ileriden_sola": [
                "front",
                "right"
            ],

            "ileriden_saga": [
                "front",
                "left"
            ],

            "soladonulmez": [
                "left"
            ],

            "sagadonulmez": [
                "right"
            ],

            "girisiyok": {
                "front"
            },
        }

        # ======================================================
        # LEVHALAR
        # ======================================================

        for sign_key, barrier_names in sign_map.items():

            if sign_key not in data:
                continue

            try:

                distance = float(
                    data[sign_key]
                )

            except Exception:

                continue

            if distance > self.sign_trigger_distance:
                continue

            self.get_logger().warn(
                f"[SIGN DETECTED] "
                f"{sign_key} = {distance:.2f}m"
            )

            # ==================================================
            # BARIYERLER
            # ==================================================

            for barrier_name in barrier_names:

                wall_key = (
                    f"{junction['id']}_"
                    f"{angle_set['name']}_"
                    f"{barrier_name}"
                )

                # ------------------------------------------------
                # ZATEN AKTİF
                # ------------------------------------------------

                if wall_key in self.walls:

                    self.get_logger().info(
                        f"[ALREADY ACTIVE] "
                        f"{wall_key}"
                    )

                    continue

                # ------------------------------------------------
                # BARIYER BUL
                # ------------------------------------------------

                barrier = active_barriers.get(
                    barrier_name
                )

                if barrier is None:

                    self.get_logger().warn(
                        f"[BARRIER NOT FOUND] "
                        f"Set={angle_set['name']} "
                        f"Barrier={barrier_name}"
                    )

                    continue

                # ------------------------------------------------
                # OLUŞTUR
                # ------------------------------------------------

                self.get_logger().warn(
                    f"[BARRIER SELECT] "
                    f"Junction={junction['id']} | "
                    f"Set={angle_set['name']} | "
                    f"Yaw={robot_yaw_deg:.2f} | "
                    f"Levha={sign_key} | "
                    f"Bariyer={barrier_name}"
                )

                self.create_wall(
                    wall_key,
                    barrier
                )

    # ==========================================================
    # ROBOT MAP YAW
    # ==========================================================

    def get_robot_map_yaw_deg(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                rclpy.time.Time()
            )

        except Exception as e:

            self.get_logger().warn(
                f"Robot map yaw TF hatasi: {e}"
            )

            return None

        q = transform.transform.rotation

        _, _, yaw_rad = euler_from_quaternion([
            q.x,
            q.y,
            q.z,
            q.w
        ])

        yaw_deg = math.degrees(
            yaw_rad
        )

        if yaw_deg < 0:
            yaw_deg += 360.0

        return yaw_deg

    # ==========================================================
    # CURRENT JUNCTION
    # ==========================================================

    def find_current_junction(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                rclpy.time.Time()
            )

        except Exception as e:

            self.get_logger().warn(
                f"TF lookup hatasi: {e}"
            )

            return None

        robot_x = (
            transform.transform.translation.x
        )

        robot_y = (
            transform.transform.translation.y
        )

        for junction in self.barriers:

            area = junction["trigger_area"]

            x_min = area.get(
                "x_min",
                area.get("xmin")
            )

            x_max = area.get(
                "x_max",
                area.get("xmax")
            )

            y_min = area.get(
                "y_min",
                area.get("ymin")
            )

            y_max = area.get(
                "y_max",
                area.get("ymax")
            )

            if (
                x_min <= robot_x <= x_max
                and
                y_min <= robot_y <= y_max
            ):

                self.get_logger().info(
                    f"[JUNCTION] "
                    f"{junction['id']} "
                    f"robot=("
                    f"{robot_x:.2f},"
                    f"{robot_y:.2f})"
                )

                return junction

        return None

    # ==========================================================
    # ANGLE SET
    # ==========================================================

    def find_angle_set(
        self,
        junction,
        robot_yaw_deg
    ):

        angle_sets = junction.get(
            "angle_sets",
            []
        )

        for angle_set in angle_sets:

            min_yaw = float(
                angle_set["min_yaw"]
            )

            max_yaw = float(
                angle_set["max_yaw"]
            )

            # Normal aralık
            if min_yaw <= max_yaw:

                if (
                    min_yaw
                    <= robot_yaw_deg
                    <= max_yaw
                ):

                    return angle_set

            # 360 üzerinden geçen aralık
            else:

                if (
                    robot_yaw_deg >= min_yaw
                    or
                    robot_yaw_deg <= max_yaw
                ):

                    return angle_set

        return None

    # ==========================================================
    # DUVAR OLUŞTUR
    #
    # ÇOK ÖNEMLİ:
    #
    # Burada map -> odom YOK.
    #
    # JSON koordinatı doğrudan MAP.
    # ==============================================================

    def create_wall(
        self,
        wall_key,
        barrier
    ):

        x1 = float(
            barrier["x1"]
        )

        y1 = float(
            barrier["y1"]
        )

        x2 = float(
            barrier["x2"]
        )

        y2 = float(
            barrier["y2"]
        )

        # ======================================================
        # MERKEZ
        # ======================================================

        cx = (
            x1 + x2
        ) / 2.0

        cy = (
            y1 + y2
        ) / 2.0

        # ======================================================
        # YÖN
        # ======================================================

        dx = x2 - x1
        dy = y2 - y1

        length = math.hypot(
            dx,
            dy
        )

        length += (
            2.0 *
            self.wall_edge_margin
        )

        yaw = math.atan2(
            dy,
            dx
        )

        # ======================================================
        # MARKER ID
        # ======================================================

        marker_id = self.next_marker_id

        self.next_marker_id += 1

        # ======================================================
        # SÜRE
        #
        # ROS clock üzerinden tutuluyor.
        # ======================================================

        now_sec = (
            self.get_clock()
            .now()
            .nanoseconds
            / 1e9
        )

        expire_time = (
            now_sec
            + self.wall_duration
        )

        # ======================================================
        # DUVAR
        # ======================================================

        self.walls[wall_key] = {

            "cx": cx,
            "cy": cy,

            "yaw": yaw,

            "length": length,

            "marker_id": marker_id,

            "expire_time": expire_time,

            "map_x1": x1,
            "map_y1": y1,

            "map_x2": x2,
            "map_y2": y2,
        }

        self.get_logger().warn(
            f"=============================================="
        )

        self.get_logger().warn(
            f"[WALL CREATED]"
        )

        self.get_logger().warn(
            f"KEY      = {wall_key}"
        )

        self.get_logger().warn(
            f"MAP      = ({cx:.2f}, {cy:.2f})"
        )

        self.get_logger().warn(
            f"YAW      = {math.degrees(yaw):.2f} deg"
        )

        self.get_logger().warn(
            f"LENGTH   = {length:.2f} m"
        )

        self.get_logger().warn(
            f"EXPIRES  = {self.wall_duration:.1f} sec"
        )

        self.get_logger().warn(
            f"=============================================="
        )

        # ======================================================
        # BARIYERİ HEMEN YAYINLA
        #
        # 0.2 saniyelik timerı beklemiyoruz.
        # ======================================================

        self.publish_all_walls()

    # ==========================================================
    # AKTİF BARIYERLERİ GÜNCELLE
    #
    # 1) Süresi dolanları sil
    # 2) Kalanların tamamını yayınla
    #
    # Böylece costmap sürekli güncel kalır.
    # ==========================================================

    def update_walls(self):

        now_sec = (
            self.get_clock()
            .now()
            .nanoseconds
            / 1e9
        )

        expired_keys = []

        for wall_key, wall in list(
            self.walls.items()
        ):

            if now_sec >= wall["expire_time"]:

                expired_keys.append(
                    wall_key
                )

        # ======================================================
        # SÜRESİ DOLANLARI SİL
        # ======================================================

        for wall_key in expired_keys:

            self.remove_wall(
                wall_key
            )

        # ======================================================
        # GÜNCEL BARIYERLERİ YAYINLA
        # ======================================================

        self.publish_all_walls()

    # ==========================================================
    # TÜM AKTİF DUVARLARI YAYINLA
    #
    # BURASI KRİTİK.
    #
    # walls boş olsa bile BOŞ CLOUD yayınlanıyor.
    # ==============================================================

    def publish_all_walls(self):

        marker_array = MarkerArray()

        all_points = []

        # ======================================================
        # AKTİF BARIYERLER
        # ======================================================

        for wall_key, wall in self.walls.items():

            marker_array.markers.append(
                self._build_marker(
                    wall,
                    wall_key
                )
            )

            pts = generate_wall_points(
                wall["cx"],
                wall["cy"],
                wall["yaw"],
                wall["length"],
                self.wall_width,
                self.wall_z_min,
                self.wall_z_max,
            )

            all_points.extend(
                pts
            )

        # ======================================================
        # MARKER YAYINLA
        # ======================================================

        if marker_array.markers:

            self.marker_pub.publish(
                marker_array
            )

        # ======================================================
        # POINT CLOUD
        #
        # AKTİF BARIYER YOKSA BİLE BOŞ CLOUD OLUŞTUR.
        #
        # Eski kodun önemli avantajlarından biri buydu.
        # ==============================================================

        header = Header()

        header.frame_id = "map"

        header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        cloud = point_cloud2.create_cloud_xyz32(
            header,
            all_points
        )

        # ======================================================
        # LOCAL
        # ======================================================

        self.obstacle_pub.publish(
            cloud
        )

        # ======================================================
        # GLOBAL
        # ======================================================

        self.obstacle_global_pub.publish(
            cloud
        )

    # ==========================================================
    # MARKER
    # ==========================================================

    def _build_marker(
        self,
        wall,
        label
    ):

        marker = Marker()

        marker.header.frame_id = "map"

        marker.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        marker.ns = "sign_obstacles"

        marker.id = wall["marker_id"]

        marker.type = Marker.CUBE

        marker.action = Marker.ADD

        # ======================================================
        # POZİSYON
        # ======================================================

        marker.pose.position.x = wall["cx"]
        marker.pose.position.y = wall["cy"]

        marker.pose.position.z = (
            self.wall_z_max
            + self.wall_z_min
        ) / 2.0

        # ======================================================
        # YAW
        # ======================================================

        q = quaternion_from_euler(
            0.0,
            0.0,
            wall["yaw"]
        )

        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        # ======================================================
        # BOYUT
        # ======================================================

        marker.scale.x = wall["length"]

        marker.scale.y = self.wall_width

        marker.scale.z = (
            self.wall_z_max
            - self.wall_z_min
        )

        # ======================================================
        # GÖRÜNÜM
        # ======================================================

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.85

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        return marker

    # ==========================================================
    # TEK BARIYERİ SİL
    # ==========================================================

    def remove_wall(
        self,
        wall_key
    ):

        if wall_key not in self.walls:
            return

        wall = self.walls.pop(
            wall_key
        )

        # ======================================================
        # MARKER DELETE
        # ======================================================

        marker = Marker()

        marker.header.frame_id = "map"

        marker.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        marker.ns = "sign_obstacles"

        marker.id = wall["marker_id"]

        marker.action = Marker.DELETE

        marker_array = MarkerArray()

        marker_array.markers.append(
            marker
        )

        self.marker_pub.publish(
            marker_array
        )

        # ======================================================
        # ÖNCE GÜNCEL POINT CLOUD'U YAYINLA
        #
        # Böylece silinen bariyer cloud'dan da çıkar.
        # ==============================================================

        self.publish_all_walls()

        # ======================================================
        # COSTMAP TEMİZLE
        # ======================================================

        self._call_clear_costmaps()

        self.get_logger().warn(
            f"=============================================="
        )

        self.get_logger().warn(
            f"[WALL TIMEOUT]"
        )

        self.get_logger().warn(
            f"{wall_key} -> "
            f"{self.wall_duration:.1f} saniye doldu."
        )

        self.get_logger().warn(
            f"[WALL REMOVED]"
        )

        self.get_logger().warn(
            f"Kalan aktif bariyer = "
            f"{len(self.walls)}"
        )

        self.get_logger().warn(
            f"=============================================="
        )

    # ==========================================================
    # COSTMAP CLEAR
    # ==========================================================

    def _call_clear_costmaps(self):

        req = ClearEntireCostmap.Request()

        # ======================================================
        # LOCAL
        # ======================================================

        if self.clear_local_cli.service_is_ready():

            self.clear_local_cli.call_async(
                ClearEntireCostmap.Request()
            )

            self.get_logger().info(
                "[COSTMAP] Local costmap clear cagrildi."
            )

        else:

            self.get_logger().warn(
                "[COSTMAP] Local clear servisi hazir degil."
            )

        # ======================================================
        # GLOBAL
        # ======================================================

        if self.clear_global_cli.service_is_ready():

            self.clear_global_cli.call_async(
                ClearEntireCostmap.Request()
            )

            self.get_logger().info(
                "[COSTMAP] Global costmap clear cagrildi."
            )

        else:

            self.get_logger().warn(
                "[COSTMAP] Global clear servisi hazir degil."
            )

    # ==========================================================
    # DESTROY
    # ==========================================================

    def destroy_node(self):

        self.walls.clear()

        super().destroy_node()


# ==============================================================
# MAIN
# ==============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = SignDynamicObstacle()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()