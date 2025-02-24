import sys
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from .UM982NtripDriver import UM982NtripDriver

class UM982DriverROS2(Node):
    def __init__(self) -> None:
        super().__init__('um982_serial_driver') 
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('update_frequency', 10.0)  
        
        self.declare_parameter('caster_host', "euref-ip.asi.it")
        self.declare_parameter('caster_port',  2101)
        self.declare_parameter('mountpoint', "GENO00ITA0")
        self.declare_parameter('username', "ddigloria")
        self.declare_parameter('password', "cogo-2023")
    
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.update_frequency = self.get_parameter('update_frequency').get_parameter_value().double_value
        self.caster_host = self.get_parameter('caster_host').get_parameter_value().string_value
        self.caster_port = self.get_parameter('caster_port').get_parameter_value().integer_value
        self.mountpoint = self.get_parameter('mountpoint').get_parameter_value().string_value
        self.username = self.get_parameter('username').get_parameter_value().string_value
        self.password = self.get_parameter('password').get_parameter_value().string_value
        self.fix_pub        = self.create_publisher(NavSatFix, '/gps/fix',     10)
        self.utm_pub        = self.create_publisher(Odometry,  '/gps/utmpos',  10)
        self.ntrip_sta_pub  = self.create_publisher(String,  '/caster_status',  10)
                    
    
    def ntrip_status(self):
        if self.um982.rtcm_status is not None:
            msg=String()
            msg.data=self.um982.rtcm_status
            self.ntrip_sta_pub.publish(msg)

    def gnss_pub_task(self):
        if self.um982.fix is not None and self.um982.fix is not None and self.um982.vel is not None and self.um982.orientation is not None:  
            bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = self.um982.fix
            utm_x, utm_y = self.um982.utmpos
            vel_east, vel_north, vel_ver, vel_east_std, vel_north_std, vel_ver_std = self.um982.vel
            heading, pitch, roll = self.um982.orientation
            this_time = self.get_clock().now().to_msg()

            # Step 1: Publish GPS Fix Data
            fix_msg = NavSatFix()
            fix_msg.header.stamp = this_time
            fix_msg.header.frame_id = 'gps'
            fix_msg.latitude = bestpos_lat
            fix_msg.longitude = bestpos_lon
            fix_msg.altitude = bestpos_hgt
            fix_msg.position_covariance[0] = float(bestpos_latstd)**2
            fix_msg.position_covariance[4] = float(bestpos_lonstd)**2
            fix_msg.position_covariance[8] = float(bestpos_hgtstd)**2
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.fix_pub.publish(fix_msg)

            # Step 2: Publish UTM Position Data
            odom_msg = Odometry()
            odom_msg.header.stamp = this_time
            odom_msg.header.frame_id = 'earth'
            odom_msg.child_frame_id  = 'base_link'
            odom_msg.pose.pose.position.x = utm_x
            odom_msg.pose.pose.position.y = utm_y
            odom_msg.pose.pose.position.z = bestpos_hgt
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
            self.utm_pub.publish(odom_msg)

            #self._ros_log_info(f"{heading}")
        
    def run(self):
        try:       
            self.um982 = UM982NtripDriver(self.port, self.baudrate)  
            self.um982.set_caster(self.caster_host, self.caster_port, self.mountpoint, self.username, self.password)  
            self.pub_timer      = self.create_timer(1/self.update_frequency, self.gnss_pub_task)
            self.ntrip_pub_timer= self.create_timer(1, self.ntrip_status)
            while rclpy.ok():
                self.um982.loop()
                rclpy.spin(self)
                time.sleep(0.001)
        except Exception as e:
            self._ros_log_error(e)

    def stop(self):
        self.um982.stop()
        self.pub_timer.cancel()
        
    def _ros_log_debug(self, log_data):
        self.get_logger().debug(str(log_data))

    def _ros_log_info(self, log_data):
        self.get_logger().info(str(log_data))

    def _ros_log_warn(self, log_data):
        self.get_logger().warn(str(log_data))

    def _ros_log_error(self, log_data):
        self.get_logger().error(str(log_data))


import time
import signal

def signal_handler(sig, frame):
    um982_driver.stop()
    time.sleep(0.1)
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
rclpy.init()
um982_driver = UM982DriverROS2()

def main():
    um982_driver.run()

if __name__ == "__main__":
    main()
