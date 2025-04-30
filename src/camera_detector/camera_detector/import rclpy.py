import rclpy
from rclpy.node import Node
import threading
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import numpy as np
import time

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Point, Quaternion
from cv_bridge import CvBridge
import cv2
import math

from ultralytics import YOLO

class Detector(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node has been started.')

        # === Mappa ===
        self.max_range = 30.0
        self.resolution = 0.1
        self.map_width = int(2 * self.max_range / self.resolution) + 1
        self.map_height = self.map_width
        self.origin_x = - (self.map_width * self.resolution) / 2.0
        self.origin_y = self.origin_x
        self.map = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.map_lock = threading.Lock()

        # === Camera ===
        self.bridge = CvBridge()
        self.model = YOLO("model/yolov8n.pt")
        self.class_names = self.model.names
        self.camera_fov_deg = 90
        self._init_camera()
        self.detected_objects = []
        self.detected_objects_lock = threading.Lock()

        # === LiDAR ===
        self.latest_scan_data = None
        self.lidar_data_lock = threading.Lock()

        # === Threads e Timer ===
        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.camera_thread.start()
        
        self.create_timer(0.1, self._update_costmap)
        self.create_timer(0.1, self._publish_costmap)

        # === Publisher/Subscriber ===
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/my_cost_map', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self._lidar_callback, 10)

    def _init_camera(self):
        self.cap = cv2.VideoCapture('/dev/video0')
        if not self.cap.isOpened():
            self.get_logger().error("Camera initialization failed")
            return
        self.camera_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.get_logger().info(f"Camera initialized: {self.camera_width}x{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")

    def _camera_loop(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Frame acquisition failed", throttle_duration_sec=5)
                continue
            
            processed_frame = self._process_frame(frame)
            try:
                img_msg = self.bridge.cv2_to_imgmsg(processed_frame, "bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                self.image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Image publishing failed: {str(e)}")

    def _process_frame(self, frame):
        results = self.model.predict(frame, verbose=False)[0]
        detections = []
        
        for box in results.boxes:
            cls_id = int(box.cls[0])
            label = f"{self.class_names[cls_id]} {box.conf[0]:.2f}"
            
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
            
            if self.class_names[cls_id] == 'person':
                detections.append({
                    'x1': x1,
                    'x2': x2,
                    'timestamp': self.get_clock().now().nanoseconds
                })

        with self.detected_objects_lock:
            self.detected_objects = detections
            
        return frame

    def _lidar_callback(self, msg):
        with self.lidar_data_lock:
            angles = []
            ranges = []
            
            for i, dist in enumerate(msg.ranges):
                if not (math.isfinite(dist) and msg.range_min <= dist <= msg.range_max):
                    continue
                
                angle = msg.angle_min + (i * msg.angle_increment)
                angles.append(angle)
                ranges.append(dist)
            
            self.latest_scan_data = {
                'angles': np.array(angles),
                'ranges': np.array(ranges),
                'angle_increment': msg.angle_increment
            }

    def _update_costmap(self):
        # Acquisizione dati sincronizzata
        with self.lidar_data_lock:
            if self.latest_scan_data is None:
                return
            scan_data = self.latest_scan_data.copy()
        
        with self.detected_objects_lock:
            detections = self.detected_objects.copy()
        
        # Calcolo efficiente con numpy
        angles = scan_data['angles']
        ranges = scan_data['ranges']
        valid = (ranges > 0) & (ranges <= self.max_range)
        
        x_coords = (ranges[valid] * np.cos(angles[valid]) - self.origin_x) / self.resolution
        y_coords = (ranges[valid] * np.sin(angles[valid]) - self.origin_y) / self.resolution
        
        grid_x = x_coords.astype(int)
        grid_y = y_coords.astype(int)
        
        # Applicazione dei rilevamenti della camera
        for det in detections:
            center_x = (det['x1'] + det['x2']) / 2
            angle_center = np.deg2rad((center_x / self.camera_width) * self.camera_fov_deg - self.camera_fov_deg/2)
            
            # Filtraggio punti LiDAR nell'area di interesse
            angle_mask = (angles > angle_center - np.deg2rad(5)) & (angles < angle_center + np.deg2rad(5))
            grid_x = grid_x[~angle_mask]
            grid_y = grid_y[~angle_mask]

        # Aggiornamento mappa sicuro
        with self.map_lock:
            self.map.fill(0)
            valid_cells = (grid_x >= 0) & (grid_x < self.map_width) & (grid_y >= 0) & (grid_y < self.map_height)
            np.add.at(self.map, (grid_y[valid_cells], grid_x[valid_cells]), 100)

    def _publish_costmap(self):
        og = OccupancyGrid()
        og.header.stamp = self.get_clock().now().to_msg()
        og.header.frame_id = 'laser'
        og.info.resolution = self.resolution
        og.info.width = self.map_width
        og.info.height = self.map_height
        og.info.origin.position = Point(x=self.origin_x, y=self.origin_y, z=0.0)
        
        with self.map_lock:
            og.data = self.map.flatten().astype(int).tolist()
        
        self.costmap_pub.publish(og)

def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()