#!/usr/bin/env python3.10

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import logging
from sensor_msgs_py import point_cloud2
from ament_index_python.packages import get_package_share_directory
import math
import json
import time
import tf2_ros

logging.getLogger('ultralytics').setLevel(logging.ERROR)

class SignDetectorWithNavigation(Node):
    def __init__(self):
        super().__init__('sign_detector_navigation_node')

        dir_path = os.path.dirname(os.path.realpath(__file__))
        src_dir = dir_path.split('/install')[0]  # install kısmını çıkar
        model_path = os.path.join(src_dir, 'src', 'reel_evata', 'reel_evata', 'utils', 'bestEVA.pt')

        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.fx = 277.0
        self.latest_pointcloud = None
        self.last_detection_time = time.time()
        self.detection_interval = 0.15

        # Levha takibi: her levha class_name'e göre gerçek bir OpenCV tracker ile izlenir.
        # Böylece levha bir kez YOLO ile bulunduktan sonra her karede baştan aranmaz;
        # YOLO sadece belirli aralıklarla takibi doğrulamak/düzeltmek için çalışır.
        # self.trackers[class_name] = {'tracker', 'bbox' (x1,y1,x2,y2), 'confidence', 'missed_detections'}
        self.trackers = {}
        self.max_missed_detections = 5  # bu kadar YOLO turunda tespit edilmezse takip bırakılır
        
        # Map and navigation related
        self.parking_locations = {}  # Store detected parking sign locations
        self.map_data = None
        self.map_origin = None
        self.map_resolution = 0.05
        
        # TF2 for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.color_image_callback, 10)
        self.create_subscription(CameraInfo, "/zed/zed_node/rgb/camera_info", self.camera_info_callback, 10)
        self.create_subscription(PointCloud2, "/zed/zed_node/point_cloud/cloud_registered", self.point_cloud_callback, 10)

        
        # Publishers
        self.sign_publisher = self.create_publisher(String, "/detected_signs", 10)
        # PARK/PARK-YASAK için map koordinatlı detay; park manager bunu kullanır.
        self.parking_detail_publisher = self.create_publisher(String, "/detected_parking_sign", 10)
        
        self.get_logger().info("Sign Detector publish-only initialized (/detected_signs + /detected_parking_sign)")

    def point_cloud_callback(self, msg):
        self.latest_pointcloud = msg
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, msg.point_step)
            # x,y,z alanlarının byte offset'lerini fields'ten al (genelde 0,4,8 ama garanti için kontrol edelim)
            xyz = arr[:, :, 0:12].view(np.float32).reshape(msg.height, msg.width, 3)
            self.pointcloud_xyz = xyz
        except Exception as e:
            self.get_logger().error(f"PointCloud numpy conversion error: {e}")
            self.pointcloud_xyz = None
    
    def camera_info_callback(self, msg):
        self.fx = msg.k[0]

    def get_point_from_pointcloud(self, center_x, center_y):
        """O(1) direkt numpy indeksleme - tracker/uvs bug'ından etkilenmez."""
        if getattr(self, 'pointcloud_xyz', None) is None:
            return None, None, None
        height, width, _ = self.pointcloud_xyz.shape
        cx = min(max(center_x, 0), width - 1)
        cy = min(max(center_y, 0), height - 1)
        x, y, z = self.pointcloud_xyz[cy, cx]
        if np.isnan(x) or np.isnan(y) or np.isnan(z) or np.isinf(z):
            return None, None, None
        return float(x), float(y), float(z)

    def _create_cv_tracker(self):
        """OpenCV sürümüne göre uyumlu bir tracker oluşturur (KCF: hız/doğruluk dengesi iyi)."""
        try:
            return cv2.TrackerKCF_create()
        except AttributeError:
            return cv2.legacy.TrackerKCF_create()

    def _update_trackers(self, frame):
        """Her karede çağrılır: mevcut takipçileri günceller, YOLO çalışmasa bile levhaların
        konumunu takip eder. Tracker konumu tamamen kaybederse ilgili levha bırakılır."""
        for class_name in list(self.trackers.keys()):
            info = self.trackers[class_name]
            success, box = info['tracker'].update(frame)
            if success:
                x, y, w, h = box
                info['bbox'] = (int(x), int(y), int(x + w), int(y + h))
            else:
                self.get_logger().info(f"'{class_name}' takibi kayboldu, bırakılıyor")
                del self.trackers[class_name]

    def _sync_trackers_with_detections(self, resized_image, scale_x, scale_y):
        """YOLO ile periyodik tespit yapar; yeni levhalar için tracker başlatır,
        mevcut takipçileri tespitle düzeltir (drift önlenir), uzun süre tespit
        edilmeyen levhaları bırakır."""
        results = self.model(resized_image, verbose=False)

        detected_signs = {}
        for r in results:
            for box in r.boxes:
                if box.conf > 0.6:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    x1 = int(x1 * scale_x)
                    y1 = int(y1 * scale_y)
                    x2 = int(x2 * scale_x)
                    y2 = int(y2 * scale_y)
                    class_name = self.model.names[int(box.cls)]
                    confidence = float(box.conf)
                    detected_signs[class_name] = (x1, y1, x2, y2, confidence)

        for class_name, (x1, y1, x2, y2, confidence) in detected_signs.items():
            if class_name in self.trackers:
                # Zaten takip ediliyor: tracker'ı tespitle yeniden hizala (drift düzeltme)
                info = self.trackers[class_name]
                info['tracker'] = self._create_cv_tracker()
                info['tracker'].init(self.annotated_image, (x1, y1, x2 - x1, y2 - y1))
                info['bbox'] = (x1, y1, x2, y2)
                info['confidence'] = confidence
                info['missed_detections'] = 0
            else:
                # Yeni levha: yeni tracker başlat
                tracker = self._create_cv_tracker()
                tracker.init(self.annotated_image, (x1, y1, x2 - x1, y2 - y1))
                self.trackers[class_name] = {
                    'tracker': tracker,
                    'bbox': (x1, y1, x2, y2),
                    'confidence': confidence,
                    'missed_detections': 0
                }
                self.get_logger().info(f"Yeni levha takibe alındı: {class_name}")

        # Bu turda YOLO ile tekrar tespit edilmeyen takipçiler için sayaç arttır
        for class_name in list(self.trackers.keys()):
            if class_name not in detected_signs:
                self.trackers[class_name]['missed_detections'] += 1
                if self.trackers[class_name]['missed_detections'] > self.max_missed_detections:
                    self.get_logger().info(f"'{class_name}' uzun süredir doğrulanamadı, takip bırakılıyor")
                    del self.trackers[class_name]

    def transform_to_map_frame(self, x, y, z, source_frame="camera_link"):
        """Transform coordinates from camera frame to map frame"""
        try:
            # Create a pose in the camera frame
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = source_frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = float(x)
            pose_stamped.pose.position.y = float(y)
            pose_stamped.pose.position.z = float(z)
            pose_stamped.pose.orientation.w = 1.0

            # Transform to map frame
            transform = self.tf_buffer.lookup_transform('map', source_frame, rclpy.time.Time())
            transformed_pose = do_transform_pose(pose_stamped.pose, transform)
            
            return transformed_pose.position.x, transformed_pose.position.y, transformed_pose.position.z
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return None, None, None

    @staticmethod
    def normalize_sign_class_name(class_name):
        name = str(class_name).strip().lower()
        translations = str.maketrans({
            'ı': 'i', 'ş': 's', 'ğ': 'g', 'ü': 'u', 'ö': 'o', 'ç': 'c',
        })
        name = name.translate(translations)
        name = name.replace('_', ' ').replace('-', ' ')
        return ' '.join(name.split())

    def is_no_parking_class(self, class_name):
        name = self.normalize_sign_class_name(class_name)
        return (
            name in {'park yasak', 'park yasagi', 'park etmek yasak', 'no parking', 'noparking'}
            or ('park' in name and 'yasak' in name)
            or ('no parking' in name)
        )

    def publish_parking_detail(self, class_name, distance, confidence, camera_x, camera_y, camera_z):
        if camera_x is None:
            return

        source_frame = 'camera_link'
        if self.latest_pointcloud is not None and self.latest_pointcloud.header.frame_id:
            source_frame = self.latest_pointcloud.header.frame_id

        map_x, map_y, map_z = self.transform_to_map_frame(
            camera_x, camera_y, camera_z, source_frame=source_frame
        )
        if map_x is None:
            return

        payload = {
            'class_name': class_name,
            'distance': round(float(distance), 3),
            'confidence': round(float(confidence), 4),
            'map_x': float(map_x),
            'map_y': float(map_y),
            'map_z': float(map_z),
        }
        out = String()
        out.data = json.dumps(payload)
        self.parking_detail_publisher.publish(out)

    def color_image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            self.annotated_image = cv_image

            # 1) Her karede: mevcut takipçileri güncelle (YOLO çalışmasa bile levha kaybolmaz)
            if self.trackers:
                self._update_trackers(cv_image)

            # 2) Belirli aralıklarla: YOLO ile yeni levha ara / mevcut takipçileri doğrula-düzelt
            now = time.time()
            if now - self.last_detection_time >= self.detection_interval:
                self.last_detection_time = now
                resized_image = cv2.resize(cv_image, (640, 360))
                scale_x = cv_image.shape[1] / 640
                scale_y = cv_image.shape[0] / 360
                self._sync_trackers_with_detections(resized_image, scale_x, scale_y)

            sign_data = {}
            for class_name, info in self.trackers.items():
                x1, y1, x2, y2 = info['bbox']
                confidence = info['confidence']
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                # Tek pointcloud sorgusu: hem mesafe hem (gerekirse) konum için kullanılır
                camera_x, camera_y, camera_z = self.get_point_from_pointcloud(center_x, center_y)
                if camera_x is not None:
                    distance = math.sqrt(camera_x**2 + camera_y**2 + camera_z**2)
                else:
                    distance = self.calculate_distance(x1, y1, x2, y2)

                if distance < 0.7 or distance > 25.0:
                    continue

                self._draw_box(x1, y1, x2, y2, class_name, distance, confidence)
                sign_data[class_name] = round(distance, 2)

                # PARK kararını bu node VERMEZ. Sadece park manager'a detay yayınlar.
                normalized_name = self.normalize_sign_class_name(class_name)
                is_no_parking = self.is_no_parking_class(class_name)
                is_parking = ("park" in normalized_name) and not is_no_parking
                if is_parking or is_no_parking:
                    self.publish_parking_detail(
                        class_name,
                        distance,
                        confidence,
                        camera_x,
                        camera_y,
                        camera_z,
                    )

                if class_name.lower() == "kirmizi":
                    if distance <= 7.0:
                        publish_data = {class_name: round(distance, 2)}
                        msg = String()
                        msg.data = json.dumps(publish_data)
                        self.sign_publisher.publish(msg)
                        self.get_logger().info(f"Published (kirmizi levha): {msg.data}")
                    
                if (class_name.lower() == "durak" and distance <= 12.0) or (class_name.lower() != "durak" and distance <= 7.0):
                    publish_data = {class_name: round(distance, 2)}
                    msg = String()
                    msg.data = json.dumps(publish_data)
                    self.sign_publisher.publish(msg)
                    self.get_logger().info(f"Published: {msg.data}")

            if sign_data:
                msg = String()
                msg.data = json.dumps(sign_data)
                self.sign_publisher.publish(msg)

            if self.annotated_image.shape[0] > 0:
                small_image = cv2.resize(self.annotated_image, (self.annotated_image.shape[1]//2, self.annotated_image.shape[0]//2))
                cv2.imshow("Levha Tespiti", small_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

    def calculate_distance(self, x1, y1, x2, y2):
        real_width = 0.5
        bbox_width = max(x2 - x1, 1)
        return (real_width * self.fx) / bbox_width * 1.7

    def _draw_box(self, x1, y1, x2, y2, class_name, distance, confidence):
        color = (0, 0, 255) if "park" == class_name.lower() or "durak" in class_name.lower() else (0, 255, 0)
        cv2.rectangle(self.annotated_image, (x1, y1), (x2, y2), color, 2)
        label = f"{class_name}: {distance:.2f}m ({confidence:.2f})"
        cv2.putText(self.annotated_image, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

def main(args=None):
    rclpy.init(args=args)
    node = SignDetectorWithNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
