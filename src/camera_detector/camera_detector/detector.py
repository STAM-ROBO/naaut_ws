import rclpy
from rclpy.node import Node
import threading
from nav_msgs.msg import OccupancyGrid
import numpy as np

from collections import deque

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
        self.origin_x = - (self.map_width * self.resolution) / 2.
        self.origin_y = self.origin_x
        self.map = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.map_lock = threading.Lock()

        # === Model ===
        self.model_name = "naaut_model.pt" # use naaut_model.pt, use yolov8n.pt for debug
        self.model = YOLO(f"src/camera_detector/camera_detector/model/{self.model_name}")
        self.class_names = self.model.names # {0: 'ship', 1: 'buoy', 2: 'fishnet buoy', 3: 'lighthouse', 4: 'wind farm'}
        self.get_logger().info(f"Class names: {self.class_names}")

        # === Camera ===
        self.bridge = CvBridge()
        self.camera_fov_deg = 80.80
        self.cx = 659
        self.fx = 752 #horizontal focal length in px
        self.camera_width = 1280
        self.camera_height = 720
        self.cap = None
        self._init_camera()
        self.detected_objects = []
        self.detected_objects_lock = threading.Lock()

        if self.model_name == "yolov8n.pt": #TODO solo per DEBUG
            self.default_size =  {
                0: {'name': 'person', 'width': 0.7, 'height': 1.8},
                1:  {'name': 'bicycle', 'width': 1.5, 'height': 1},
                2: {'name': 'car', 'width': 4.5, 'height': 1.7},
                    }  
            
        elif self.model_name == "naaut_model.pt":
            self.default_size = {
                0: {'name': 'ship', 'width': 15.0, 'height': 5.0},  # Nave
                1: {'name': 'buoy', 'width': 0.5, 'height': 1.5},   # Boa standard
                2: {'name': 'fishnet buoy', 'width': 0.5, 'height': 0.8},  # Boa per reti da pesca
                # 3: {'name': 'lighthouse', 'width': 6.0, 'height': 20.0},   # Faro 
                # 4: {'name': 'wind farm', 'width': 8.0, 'height': 8.0}
                    }  

        # === LiDAR ===
        self.latest_scan_data = None
        self.buffer_size = 50
        self.lidar_x_frame = int(self.camera_height / 2) + 50 #valido per risoluzione 1280x720
        self.lidar_upper_limit  = self.lidar_x_frame - 50
        self.lidar_lower_limit = self.lidar_x_frame + 50
        self.lidar_data_buffer = deque(maxlen=self.buffer_size)
        self.lidar_data_lock = threading.Lock()

        # === THREAD e Timer ===
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
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        if not self.cap.isOpened():
            self.get_logger().error("Camera initialization failed")
            return
        
        if not int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) == self.camera_width:
                self.get_logger().error("Camera width mismatch")
                return
        self.get_logger().info(f"Camera initialized: {self.camera_width}x{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")

    def _camera_loop(self):

        frame_count = 0
        skip_rate = 10 #TODO everificare quanto è necessario in prod

        while True:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warning("Failed to capture image.")
                continue
            
            frame_ts = self.get_clock().now().to_msg()
            frame_count += 1

            if frame_count % skip_rate != 0:
                continue

            processed_frame = self._process_frame(frame, frame_ts)

            cv2.line(processed_frame, (0, self.lidar_lower_limit), (self.camera_width, self.lidar_lower_limit), (0, 0, 255), 2)
            cv2.line(processed_frame, (0, self.lidar_upper_limit), (self.camera_width, self.lidar_upper_limit), (0, 255, 0), 2)

            try:
                img_msg = self.bridge.cv2_to_imgmsg(processed_frame, "bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                self.image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Image publishing failed: {str(e)}")

    def _process_frame(self, frame, frame_ts):

        with self.detected_objects_lock:
            self.detected_objects = []
        
        results = self.model.predict(frame, verbose=False)[0]
        detections = []

        for box in results.boxes:
            cls_id = int(box.cls[0])
            confidence = box.conf[0]
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            if (self.lidar_upper_limit <= y1 <= self.lidar_lower_limit ) or (self.lidar_upper_limit <= y2 <= self.lidar_lower_limit ) or ( y2 >= self.lidar_lower_limit and y1 <= self.lidar_upper_limit):
                use_lidar = True
            else:
                use_lidar = False

            #if self.class_names[cls_id] in ['person', 'car', 'bicycle']: #ONLY FOR DEBUG WITH yolo8n.pt TODO eliminare!
            if self.class_names[cls_id]  in ['ship', 'buoy', 'fishnet buoy']: 
                
                label = f"{self.class_names[cls_id]} conf:{confidence:.2f}"
                self.get_logger().info(f"Processing Frame: Detected {label} at y1:{y1}, y2:{y2}")

                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

                detections.append({
                    'cls': self.class_names[cls_id],
                    'id': cls_id,
                    'conf': float(confidence),
                    'x1': x1,
                    'y1': y1,
                    'x2': x2,
                    'y2': y2,
                    'use_lidar_range': use_lidar,
                    'timestamp': frame_ts
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

            self.lidar_data_buffer.append(self.latest_scan_data)
                
    def _estimate_distance(self, cls_id, pixel_width):
        """
        Stima la distanza usando la dimensione nota dell'oggetto, la larghezza in pixel e la lunghezza focale della camera.
        
        Args:
            cls_id (int): cls_id dell'oggetto 
            pixel_width (float): Larghezza del bounding box in pixel
        
        Returns:
            float: Distanza stimata in metri
        """

        if cls_id not in self.default_size:
            self.get_logger().warn(f"Unknown object {cls_id}")
            return float('inf')
        
        actual_width = self.default_size[cls_id]['width']
        focal_length = self.fx 
        
        if pixel_width <= 0:
            self.get_logger().error("Invalid pixel width. Cannot estimate distance.")
            return float('inf')
        
        return (actual_width * focal_length) / pixel_width

    def _time_to_float(self, t):
        return t.sec + t.nanosec * 1e-9
    
    def _synchronize_lidar_and_camera(self):

        with self.lidar_data_lock:
            if self.lidar_data_buffer is None:
                self.get_logger().warning("No LiDAR data available.")
                return None, None
            buffer_data = self.lidar_data_buffer.copy()

        with self.detected_objects_lock:
            if not self.detected_objects:
                return None, None
            
            detections = self.detected_objects.copy()
            ts_frame = detections[0]['timestamp']

        min_diff = float('inf')
        best_match = None
        for scan in buffer_data:
            diff = abs(self._time_to_float(scan['timestamp']) - self._time_to_float(ts_frame))
            if diff < min_diff:
                min_diff = diff
                best_match = scan
        
        if best_match and min_diff <= 0.1:
            return best_match, detections
        
        self.get_logger().warning(
        f"Sincronizzazione Camera-LiDAR: Nessuna scansione LiDAR entro la soglia di {0.1}s. "
        f"Delay minimo trovato = {min_diff:.3f}s. Riavviare il LiDAR se necessario.")
        return None, None
            
    def _update_costmap(self):
        self.map.fill(0)
        W = self.camera_width
        scan_data, detections = self._synchronize_lidar_and_camera()
        if scan_data is None or detections is None:
            return
    
        angles = scan_data['angles']
        ranges = scan_data['ranges']
        
        for detection in detections:
            #if detection['cls'] in ['person', 'car', 'bicycle']: #row ONLY FOR DEBUG WITH yolo8n.pt TODO eliminare!
            if detection['cls']  in ['ship', 'buoy', 'fishnet buoy']: 
                x1 = detection['x1']
                x2 = detection['x2']

                left_bound_angle = (x1 / W) * self.camera_fov_deg - (self.camera_fov_deg / 2)
                right_bound_angle = (x2 / W) * self.camera_fov_deg - (self.camera_fov_deg / 2)

                left_bound_angle_rad = math.radians(left_bound_angle)
                right_bound_angle_rad = math.radians(right_bound_angle)

                #conversioni da angoli camera ad angoli lidar
                left_bound_lidar_rad = (-math.pi - left_bound_angle_rad) if left_bound_angle_rad < 0 else (math.pi - left_bound_angle_rad)
                right_bound_lidar_rad = (-math.pi - right_bound_angle_rad) if right_bound_angle_rad < 0 else (math.pi - right_bound_angle_rad)
                
                if left_bound_lidar_rad < 0 and right_bound_lidar_rad >0:
                    # Se la bbox attraversa il centro della camera
                    filt_angles = (angles > right_bound_lidar_rad) | (angles < left_bound_lidar_rad)
                else:
                    # Se la bbox è interamente a sinistra o a destra
                    filt_angles = (angles > right_bound_lidar_rad) & (angles < left_bound_lidar_rad)

                filtered_angles = angles[filt_angles]
                filtered_ranges = ranges[filt_angles]

                if filtered_ranges.any():
                    if not detection['use_lidar_range']:
                        # La bounding box non è all'altezza del LiDAR: usa metodo di stima
                        width_bb = x2 - x1
                        range = self._estimate_distance(detection['id'], width_bb)
                        self.get_logger().warning(f"Using Estimation Method, Detected {detection['cls']} at range: {range:.2f} meters")

                        if range > self.max_range:
                            self.get_logger().warning(f"Estimated range {range:.2f} exceeds max range {self.max_range:.2f}.")
                            return
                        
                    else:
                        range = min(filtered_ranges)
                        self.get_logger().info(f"Detected {detection['cls']} at range: {range:.2f} meters")
                #     f"Camera: center={center_x_deg:.2f}° ({center_x}px) | "
                #     f"Bounds: [{left_bound_angle:.2f}°, {right_bound_angle:.2f}°] | "
                #     f"Rad: [{left_bound_angle_rad:.3f}, {right_bound_angle_rad:.3f}] | "
                #     f"LiDAR: [{left_bound_lidar_rad:.3f}, {right_bound_lidar_rad:.3f}]"

                else:
                    self.get_logger().info(f"No available LiDAR data within the bounding box range for detected object: {detection['cls']}.")
                    return
                
                for angle in filtered_angles:
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

        try:
            self.costmap_pub.publish(og)
            #self.get_logger().info("Costmap published.")

        except Exception as e:
            self.get_logger().error(f"Failed to publish costmap: {str(e)}")
     

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

