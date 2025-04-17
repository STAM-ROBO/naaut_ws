import sys
import math
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from .UM982NtripDriver import UM982NtripDriver
from tf2_ros import TransformBroadcaster
import tf_transformations
from geometry_msgs.msg import TransformStamped

class UM982DriverROS2(Node):
    def __init__(self) -> None:
        super().__init__('um982_serial_driver') 
        self.declare_parameter('port', '/dev/serial/by-path/platform-3610000.usb-usb-0:2.1:1.0-port0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('caster_host', "euref-ip.asi.it")
        self.declare_parameter('caster_port',  2101)
        self.declare_parameter('mountpoint', "GENO00ITA0")
        self.declare_parameter('username', "ddigloria")
        self.declare_parameter('password', "cogo-2023")
        
        self.declare_parameter('orientation_topic','/imu_gnss')
        self.declare_parameter('imu_link','imu_link')
        self.declare_parameter('fix_topic','/gps/fix')
        self.declare_parameter('gnss_link','gps_link')
        self.declare_parameter('odometry_topic','/odometry/gps')
    
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.caster_host = self.get_parameter('caster_host').get_parameter_value().string_value
        self.caster_port = self.get_parameter('caster_port').get_parameter_value().integer_value
        self.mountpoint = self.get_parameter('mountpoint').get_parameter_value().string_value
        self.username = self.get_parameter('username').get_parameter_value().string_value
        self.password = self.get_parameter('password').get_parameter_value().string_value
        self.orientation_topic = self.get_parameter('orientation_topic').get_parameter_value().string_value
        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
        self.fix_topic = self.get_parameter('fix_topic').get_parameter_value().string_value
        self.gnss_link = self.get_parameter('gnss_link').get_parameter_value().string_value
        self.odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        
        self.orientation_pub  = self.create_publisher(Imu, self.orientation_topic,  10)
        self.fix_pub        = self.create_publisher(NavSatFix, self.fix_topic,  10)
        self.ntrip_sta_pub  = self.create_publisher(String,  '/caster_status',  10)
        self.fix_sta_pub  = self.create_publisher(String,  '/fix_status',  10)
        self.odometry_topic_pub  = self.create_publisher(Odometry,  self.odometry_topic,  10)                
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.utm_datum_is_set=0
        self.utm_datumx=0
        self.utm_datumy=0
    
    def pub_sys_status(self):
        msg=String()
        fix_sta_msg=String()
        
        msg.data=self.um982.rtcm_status
        fix_sta_msg.data=self.um982.fix_type
        
        self.fix_sta_pub.publish(fix_sta_msg)
        self.ntrip_sta_pub.publish(msg)

    def gnss_pub_task(self):
        if self.um982.fix is not None and self.um982.vel is not None and self.um982.orientation is not None:  
            bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = self.um982.fix
            utm_x, utm_y = self.um982.utmpos
            vel_east, vel_north, vel_ver, vel_east_std, vel_north_std, vel_ver_std = self.um982.vel
            heading, pitch, roll = self.um982.orientation
            this_time = self.get_clock().now().to_msg()
            
            if not self.utm_datum_is_set:
                self.utm_datumx=utm_x
                self.utm_datumy=utm_y
                self.utm_datum_is_set=1
                        
            #Publish GPS Fix Data
            fix_msg = NavSatFix()
            fix_msg.header.stamp = this_time
            fix_msg.header.frame_id = self.gnss_link
            fix_msg.latitude = bestpos_lat
            fix_msg.longitude = bestpos_lon
            fix_msg.altitude = bestpos_hgt
            fix_msg.position_covariance[0] = float(bestpos_latstd)**2
            fix_msg.position_covariance[4] = float(bestpos_lonstd)**2
            fix_msg.position_covariance[8] = float(bestpos_hgtstd)**2
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            
            #Publish emulated IMU Data
            imu_message = Imu()
            imu_message.header.frame_id = self.imu_link
            imu_message.header.stamp = self.get_clock().now().to_msg()    
            quaternion = quaternion_from_euler(math.radians(0), math.radians(pitch), math.radians(heading))
            imu_message.orientation.x = quaternion[0]
            imu_message.orientation.y = quaternion[1]
            imu_message.orientation.z = quaternion[2]
            imu_message.orientation.w = quaternion[3]        
            imu_message.orientation_covariance         = [0.0] * 9
            imu_message.orientation_covariance[0]      = float(bestpos_latstd)**2
            imu_message.orientation_covariance[4]      = float(bestpos_lonstd)**2
            imu_message.orientation_covariance[8]     = float(bestpos_hgtstd)**2    
                
            #publish odometry with gps and UTM
            odom_msg = Odometry()
            odom_msg.header.stamp = this_time
            odom_msg.header.frame_id = 'map'
            odom_msg.child_frame_id  = 'gps_link'
            if self.utm_datum_is_set:
                odom_msg.pose.pose.position.x = utm_x-self.utm_datumx
                odom_msg.pose.pose.position.y = utm_y-self.utm_datumy
            else:
                odom_msg.pose.pose.position.x = 0.0
                odom_msg.pose.pose.position.y = 0.0
            odom_msg.pose.pose.position.z = 0.0
            quaternion = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(heading))
            odom_msg.pose.pose.orientation.x = quaternion[0]
            odom_msg.pose.pose.orientation.y = quaternion[1]
            odom_msg.pose.pose.orientation.z = quaternion[2]
            odom_msg.pose.pose.orientation.w = quaternion[3]
            odom_msg.pose.covariance         = [0.0] * 36
            odom_msg.pose.covariance[0]      = float(bestpos_latstd)**2
            odom_msg.pose.covariance[7]      = float(bestpos_lonstd)**2
            odom_msg.pose.covariance[14]     = float(bestpos_hgtstd)**2
            odom_msg.pose.covariance[21]     = 0.1
            odom_msg.pose.covariance[28]     = 0.1
            odom_msg.pose.covariance[35]     = 0.1
            odom_msg.twist.twist.linear.x    = vel_east
            odom_msg.twist.twist.linear.y    = vel_north
            odom_msg.twist.twist.linear.z    = vel_ver
            odom_msg.twist.covariance        = [0.0] * 36
            odom_msg.twist.covariance[0]     = float(vel_east_std)**2
            odom_msg.twist.covariance[7]     = float(vel_north_std)**2
            odom_msg.twist.covariance[14]    = float(vel_ver_std)**2
                        
            #publish all messages
            self.fix_pub.publish(fix_msg)
            self.odometry_topic_pub.publish(odom_msg)
            self.publish_transform_from_odometry(odom_msg)
    
    def publish_transform_from_odometry(self, odom_msg):
        t = TransformStamped()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = odom_msg.pose.pose.orientation.x
        t.transform.rotation.y = odom_msg.pose.pose.orientation.y
        t.transform.rotation.z = odom_msg.pose.pose.orientation.z
        t.transform.rotation.w = odom_msg.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)
        
    def run(self):
        self._ros_log_info( "Started")
     
        try:
            self.um982 = UM982NtripDriver(self.port, self.baudrate)  
        except Exception as ex:
            self._ros_log_error(ex)
            return
        else:
            if self.um982.set_caster(self.caster_host, self.caster_port, self.mountpoint, self.username, self.password):
                self._ros_log_info("NTRIP enabled")
            else:
                self._ros_log_info("NTRIP disabled")
        
            while rclpy.ok():
                self.um982.loop()
                self.gnss_pub_task()
                self.pub_sys_status()
                time.sleep(0.005)    
    
    def _ros_log_debug(self, log_data):
        self.get_logger().debug(str(log_data), throttle_duration_sec=1)

    def _ros_log_info(self, log_data):
        self.get_logger().info(str(log_data), throttle_duration_sec=1)

    def _ros_log_warn(self, log_data):
        self.get_logger().warn(str(log_data), throttle_duration_sec=1)

    def _ros_log_error(self, log_data):
        self.get_logger().error(str(log_data), throttle_duration_sec=1)


def main(args=None):
    rclpy.init(args=args)
    um982_driver = UM982DriverROS2()
    try:
        um982_driver.run()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
