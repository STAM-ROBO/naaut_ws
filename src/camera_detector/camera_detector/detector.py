import rclpy
from rclpy.node import Node
import threading
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import numpy as np

from sensor_msgs.msg import Image,LaserScan
from geometry_msgs.msg import Point
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

        # === Model ===
        self.model = YOLO("camera_detector/camera_detector/model/yolov8n.pt") #use naaut_model.pt
        self.class_names = self.model.names # {0: 'ship', 1: 'buoy', 2: 'fishnet buoy', 3: 'lighthouse', 4: 'wind farm'}
        self.get_logger().info(f"Class names: {self.class_names}")

        # === Camera ===
        self.bridge = CvBridge()
        self.camera_fov_deg = 80.80
        self.cap = None
        self.camera_width = 1280
        self.camera_height = 720
        self._init_camera()
        self.detected_objects = []
        self.detected_objects_lock = threading.Lock()
        
        # === LiDAR ===
        self.latest_scan_data = None
        self.lidar_data_lock = threading.Lock()

        # === THREAD e Timer ===
        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.camera_thread.start()
        self.create_timer(0.05, self._update_costmap)
        self.create_timer(0.05, self._publish_costmap)


        # === Publisher/Subscriber ===
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/my_cost_map', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self._lidar_callback, 10)

    def _init_camera(self):
        self.cap = cv2.VideoCapture('/dev/video0')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        if not self.cap.isOpened():
            self.get_logger().error("Camera initialization failed")
            return
        self.camera_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.get_logger().info(f"Camera initialized: {self.camera_width}x{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")

    def _camera_loop(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warning("Failed to capture image.")
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
            confidence = box.conf[0]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            label = f"{self.class_names[cls_id]} conf:{confidence:.2f}"

            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            detections.append({
                'cls': self.class_names[cls_id],
                'conf': float(confidence),
                'x1': x1,
                'y1': y1,
                'x2': x2,
                'y2': y2,
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
                'timestamp': self.get_clock().now().to_msg(),
                'angles': np.array(angles),      # Array di angoli in radianti
                'ranges': np.array(ranges),      # Array di distanze corrispondenti
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
            }
            
    def _update_costmap(self):

        self.map.fill(0)
        W = self.camera_width

        with self.lidar_data_lock:
            if self.latest_scan_data is None:
                self.get_logger().warning("No LiDAR data available.")
                return
            scan_data = self.latest_scan_data.copy()
        
        with self.detected_objects_lock:
            if not self.detected_objects:
                self.get_logger().warning("No detected objects available.")
                return
            detections = self.detected_objects.copy()
        
        angles = scan_data['angles']
        ranges = scan_data['ranges']
        
        for detection in detections:
        
            if detection['cls'] == 'person': #ONLY FOR DEBUG WITH yolo8n.pt TODO eliminare
                x1 = detection['x1']
                x2 = detection['x2']
                center_x = (x1 + x2) / 2 

                center_x_deg = (center_x / W) * self.camera_fov_deg - (self.camera_fov_deg / 2)
                left_bound_angle = (x1 / W) * self.camera_fov_deg - (self.camera_fov_deg / 2)
                right_bound_angle = (x2 / W) * self.camera_fov_deg - (self.camera_fov_deg / 2)

                # Conversione angoli in radianti    
                left_bound_angle_rad = math.radians(left_bound_angle)
                right_bound_angle_rad = math.radians(right_bound_angle)

                left_bound_lidar_rad = (-math.pi - left_bound_angle_rad) if left_bound_angle_rad < 0 else (math.pi - left_bound_angle_rad)
                right_bound_lidar_rad = (-math.pi - right_bound_angle_rad) if right_bound_angle_rad < 0 else (math.pi - right_bound_angle_rad)
                
                self.get_logger().info(
                    f"Detected {detection['cls']} | "
                    f"Camera: center={center_x_deg:.2f}° ({center_x}px) | "
                    f"Bounds: [{left_bound_angle:.2f}°, {right_bound_angle:.2f}°] | "
                    f"Rad: [{left_bound_angle_rad:.3f}, {right_bound_angle_rad:.3f}] | "
                    f"LiDAR: [{left_bound_lidar_rad:.3f}, {right_bound_lidar_rad:.3f}]"
                )

                if left_bound_lidar_rad < 0 and right_bound_lidar_rad >0:
                    filt_angles = (angles > right_bound_lidar_rad) | (angles < left_bound_lidar_rad)
                else:
                    filt_angles = (angles > right_bound_lidar_rad) & (angles < left_bound_lidar_rad)

                filtered_angles = angles[filt_angles]
                filtered_ranges = ranges[filt_angles]

                if filtered_angles.size > 0:
                    for angle, range in zip(filtered_angles, filtered_ranges):
                        range = min(filtered_ranges) #TODO decidere se lasciare range oppure range min
                        
                        x_lidar = range * math.cos(angle)
                        y_lidar = range * math.sin(angle)

                        # Converti in coordinate della griglia
                        grid_x = int((x_lidar - self.origin_x) / self.resolution)
                        grid_y = int((y_lidar - self.origin_y) / self.resolution)

                        # Controlla i limiti della griglia
                        if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                            self.map[grid_y, grid_x] = 100
               
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
        node.get_logger().info('Keyboard interrupt received. Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
