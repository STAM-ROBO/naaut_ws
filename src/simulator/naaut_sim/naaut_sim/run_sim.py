import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
import time
import threading
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose
from pyproj import CRS, Proj, Transformer

class naaut_simulator(Node):
    def __init__(self):
        super().__init__('naaut_simulator')       
        
        self.declare_parameter('orientation_topic','/imu_gnss')
        self.declare_parameter('imu_link','imu_link')
        self.declare_parameter('fix_topic','/gps/fix')
        self.declare_parameter('gnss_link','gps_link')
        self.declare_parameter('global_odometry_topic','/odom/gps')
        self.declare_parameter('local_odometry_topic','/odom')
        self.orientation_topic = self.get_parameter('orientation_topic').get_parameter_value().string_value
        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
        self.fix_topic = self.get_parameter('fix_topic').get_parameter_value().string_value
        self.gnss_link = self.get_parameter('gnss_link').get_parameter_value().string_value
        self.global_odometry_topic = self.get_parameter('global_odometry_topic').get_parameter_value().string_value
        self.local_odometry_topic = self.get_parameter('local_odometry_topic').get_parameter_value().string_value
        
        self.datum_lat=44.393413
        self.datum_lon=8.947172
        
        #sim params
        self.time_step = 0.05
        self.mass = 80.0

        # 1/2 * water_density * area_in_water * Cd
        self.drag_coefficient_x = 0.5 * 1000 * (3.14 * 0.26**2) * 0.2
        self.drag_coefficient_y = 0.5 * 1000 * (3.14 * 0.26**2) * 5.0
        self.drag_coefficient = np.array([self.drag_coefficient_x,self.drag_coefficient_y])
        
        #rectangular vessel
        self.rot_drag=0.10
        self.vessel_inertia = (1 / 12) * self.mass * (2**2 + 2**2)
        
        self.roll_freq=0.3   
        self.roll_amp=np.radians(1.5)      
        self.pitch_freq=0.1   
        self.pitch_amp=np.radians(0.9)
        
        self.orientation_pub  = self.create_publisher(Imu, self.orientation_topic,  10)
        self.fix_pub        = self.create_publisher(NavSatFix, self.fix_topic,  10)
        self.cmd_vel_sub = self.create_subscription(Twist,'cmd_vel', self._get_cmd_vel, 10)
        self.global_odometry_pub  = self.create_publisher(Odometry,  self.global_odometry_topic,  10)      
        self.local_odometry_pub  = self.create_publisher(Odometry,  self.local_odometry_topic,  10)                
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x_vel=0.0
        self.rz_vel=0.0            
        run_sim_loop_thread = threading.Thread(target=self._run_sim_loop, daemon=True)
        run_sim_loop_thread.start()
    
    def _get_cmd_vel(self,msg):
        self.x_vel=msg.linear.x * 15.0
        self.rz_vel=msg.angular.z  * 40.0
               
    def _run_sim_loop(self):
        self.get_logger().info("Starting simu loop")
        lin_vel = np.array([0.0, 0.0])
        lin_pos = np.array([0.0, 0.0])
        heading_dot= 0.0
        heading = 0.0     
        pitch=0.0
        roll= 0.0
        timestamp=0.0
        thr_filt = 0.0
                       
        while rclpy.ok():  
            torque = self.rz_vel  
            thrust = self.x_vel               
            angular_acceleration = torque / self.vessel_inertia
            heading_dot += angular_acceleration * self.time_step - heading_dot * self.rot_drag
            heading += heading_dot * self.time_step              
            
            thrust_vect=  np.array([np.cos(heading), np.sin(heading)]) * thrust

            drag_force = self.drag_coefficient[0] * lin_vel
            total_force = thrust_vect - drag_force
            act_accel = total_force / self.mass

            # integrate accel
            lin_vel += act_accel * self.time_step

            # integrate velocity
            lin_pos += lin_vel * self.time_step
            
            alpha = 0.05
            thr_filt = alpha * thrust + (1-alpha)* thr_filt
            
            roll = self.roll_amp*math.sin(2 * math.pi * self.roll_freq * timestamp)
            pitch = np.clip(self.pitch_amp*math.sin(2 * math.pi * self.pitch_freq * timestamp) - thr_filt * 0.01, -0.3, 0.3)
            
            self._pub_simu_results(heading, pitch, roll, lin_pos, lin_vel)
            self._pub_gnss_simu_results(heading, pitch, roll, lin_pos, lin_vel)    
            timestamp+=self.time_step            
            time.sleep(self.time_step)

        self.get_logger().info("Finished sim")
        
    def _pub_gnss_simu_results(self, heading, pitch, roll,  xy_location, xy_velocity): 
        this_time = self.get_clock().now().to_msg()
        quaternion = quaternion_from_euler(roll, pitch, heading)     
        
        #local to wgs84
        lon, lat = self._local_odom_to_wgs84(xy_location, self.datum_lat, self.datum_lon)
        
        lon += np.random.rand()*0.00002
        lat += np.random.rand()*0.00002
        elevation = 0.0
                    
        #Publish GPS Fix Data
        fix_msg = NavSatFix()
        fix_msg.header.stamp = this_time
        fix_msg.header.frame_id = self.gnss_link
        fix_msg.latitude = lat
        fix_msg.longitude = lon
        fix_msg.altitude = elevation
        fix_msg.position_covariance[0] = float(0.01)**2
        fix_msg.position_covariance[4] = float(0.01)**2
        fix_msg.position_covariance[8] = float(0.01)**2
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            
        #publish odometry with gps and UTM

        coord_transformer = self._wgs84_to_UTM(lat, lon)
        utm_x, utm_y = coord_transformer.transform(lat, lon)

        global_odom_msg = Odometry()
        global_odom_msg.header.stamp = this_time
        global_odom_msg.header.frame_id = 'wgs84'
        global_odom_msg.child_frame_id  = self.gnss_link
        global_odom_msg.pose.pose.position.x = utm_x
        global_odom_msg.pose.pose.position.y = utm_y
        global_odom_msg.pose.pose.position.z = elevation
        global_odom_msg.pose.pose.orientation.x = quaternion[0]
        global_odom_msg.pose.pose.orientation.y = quaternion[1]
        global_odom_msg.pose.pose.orientation.z = quaternion[2]
        global_odom_msg.pose.pose.orientation.w = quaternion[3]
        global_odom_msg.pose.covariance         = [0.0] * 36
        global_odom_msg.pose.covariance[21]     = 0.1
        global_odom_msg.pose.covariance[28]     = 0.1
        global_odom_msg.pose.covariance[35]     = 0.1
        #global_odom_msg.twist.twist.linear.x    = xy_velocity[0]
        #global_odom_msg.twist.twist.linear.y    = xy_velocity[1]
        #global_odom_msg.twist.twist.linear.z    = 0.0
        #global_odom_msg.twist.covariance        = [0.0] * 36
            
        self.global_odometry_pub.publish(global_odom_msg)
        self.fix_pub.publish(fix_msg)
        
    
    def _pub_simu_results(self, heading, pitch, roll,  xy_location, xy_velocity):          
        this_time = self.get_clock().now().to_msg()
        quaternion = quaternion_from_euler(roll, pitch, heading)     
        
        #Publish emulated IMU Data
        imu_message = Imu()
        imu_message.header.frame_id = self.imu_link
        imu_message.header.stamp = self.get_clock().now().to_msg()    
        imu_message.orientation.x = quaternion[0]
        imu_message.orientation.y = quaternion[1]
        imu_message.orientation.z = quaternion[2]
        imu_message.orientation.w = quaternion[3]        
        imu_message.orientation_covariance    = [0.0] * 9
            
        #publish odometry with gps and UTM
        local_odom_msg = Odometry()
        local_odom_msg.header.stamp = this_time
        local_odom_msg.header.frame_id = 'odom'
        local_odom_msg.child_frame_id  = 'base_link'
        local_odom_msg.pose.pose.position.x = xy_location[0]
        local_odom_msg.pose.pose.position.y = xy_location[1]
        local_odom_msg.pose.pose.position.z = 0.0
        local_odom_msg.pose.pose.orientation.x = quaternion[0]
        local_odom_msg.pose.pose.orientation.y = quaternion[1]
        local_odom_msg.pose.pose.orientation.z = quaternion[2]
        local_odom_msg.pose.pose.orientation.w = quaternion[3]
        local_odom_msg.pose.covariance         = [0.0] * 36
        local_odom_msg.pose.covariance[21]     = 0.1
        local_odom_msg.pose.covariance[28]     = 0.1
        local_odom_msg.pose.covariance[35]     = 0.1
        local_odom_msg.twist.twist.linear.x    = xy_velocity[0]
        local_odom_msg.twist.twist.linear.y    = xy_velocity[1]
        local_odom_msg.twist.twist.linear.z    = 0.0
        local_odom_msg.twist.covariance        = [0.0] * 36
        
        t = TransformStamped()
        t.header.stamp = this_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = local_odom_msg.pose.pose.position.x
        t.transform.translation.y = local_odom_msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = local_odom_msg.pose.pose.orientation.x
        t.transform.rotation.y = local_odom_msg.pose.pose.orientation.y
        t.transform.rotation.z = local_odom_msg.pose.pose.orientation.z
        t.transform.rotation.w = local_odom_msg.pose.pose.orientation.w
        
        #publish all messages
        #self.global_odometry_pub.publish(global_odom_msg)
        self.local_odometry_pub.publish(local_odom_msg)
        #self.fix_pub.publish(fix_msg)
        self.tf_broadcaster.sendTransform(t)   
        
    def _wgs84_to_UTM(self, lat, lon):
        #calcolo zona UTM
        zone_number = int((lon + 180) / 6) + 1
        #are we in norther hemisphere?
        isnorth = lat >= 0
        #wgs84 system EPSG code
        wgs84_crs = CRS("epsg:4326")
        #make the appropriate UTM EPSG code depending on whether you are in the Northern Hemisphere
        utm_crs_str = f"epsg:326{zone_number}" if isnorth else f"epsg:327{zone_number}"
        utm_crs     = CRS(utm_crs_str)
        #create a coordinate converter
        transformer = Transformer.from_crs(wgs84_crs, utm_crs, always_xy=True)
        return transformer

    def _local_odom_to_wgs84(self, xy_location, datum_lat, datum_lon):
        #Define the WGS84 geodetic CRS and a local projected CRS centered on the initial datum
        wgs84 = Proj(proj="latlong", datum="WGS84")
        local_proj = Proj(proj="tmerc", lat_0=datum_lat, lon_0=datum_lon, datum="WGS84")
        #Create a transformer for local to global conversion
        transformer = Transformer.from_proj(local_proj, wgs84)
        #Transform local coordinates to WGS84
        longitude, latitude = transformer.transform(xy_location[0], xy_location[1]) 
        return longitude, latitude       


    def shutdown(self):
        self.get_logger().info("Shutting down node.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node=naaut_simulator()
    
    try:
        rclpy.spin(node)  # Keep node running to handle callbacks
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.")

if __name__ == '__main__':
    main()