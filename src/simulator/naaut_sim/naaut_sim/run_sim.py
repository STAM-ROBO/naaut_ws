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

class naaut_simulator(Node):
    def __init__(self):
        super().__init__('naaut_simulator')       
        
        self.declare_parameter('orientation_topic','/imu_gnss')
        self.declare_parameter('imu_link','imu_link')
        self.declare_parameter('fix_topic','/gps/fix')
        self.declare_parameter('gnss_link','gps_link')
        self.declare_parameter('odometry_topic','/odom')
        self.orientation_topic = self.get_parameter('orientation_topic').get_parameter_value().string_value
        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
        self.fix_topic = self.get_parameter('fix_topic').get_parameter_value().string_value
        self.gnss_link = self.get_parameter('gnss_link').get_parameter_value().string_value
        self.odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
    
        
        #sim params
        self.time_step = 0.02
        self.mass = 80.0

        # 1/2 * water_density * area_in_water * Cd
        self.drag_coefficient_x = 0.5 * 1000 * (3.14 * 0.26**2) * 0.2
        self.drag_coefficient_y = 0.5 * 1000 * (3.14 * 0.26**2) * 5.0
        self.drag_coefficient = np.array([self.drag_coefficient_x,self.drag_coefficient_y])
        
        #rectangular vessel
        self.rot_drag=0.15
        self.vessel_inertia = (1 / 12) * self.mass * (2**2 + 2**2)
        
        self.x_vel=0.0
        self.rz_vel=0.0     
        
        self.roll_freq=0.3   
        self.roll_amp=np.radians(1.5)
        
        self.pitch_freq=0.1   
        self.pitch_amp=np.radians(0.9)
        
        self.orientation_pub  = self.create_publisher(Imu, self.orientation_topic,  10)
        self.fix_pub        = self.create_publisher(NavSatFix, self.fix_topic,  10)
        self.cmd_vel_sub = self.create_subscription(Twist,'cmd_vel', self.get_cmd_vel, 10)
        self.odometry_topic_pub  = self.create_publisher(Odometry,  self.odometry_topic,  10)                
        self.tf_broadcaster = TransformBroadcaster(self)
       
        run_sim_loop_thread = threading.Thread(target=self.run_sim_loop, daemon=True)
        run_sim_loop_thread.start()
    
    def get_cmd_vel(self,msg):
        self.x_vel=msg.linear.x * 5.0
        self.rz_vel=msg.angular.z  * 15.0
               
    def run_sim_loop(self):
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
            
            # drag orientato come il vettore velocità
            drag_force = self.drag_coefficient[0] * lin_vel

            net_force = thrust_vect - drag_force
            acceleration = net_force / self.mass
            lin_vel += acceleration * self.time_step
            lin_pos += lin_vel * self.time_step
            
            alpha = 0.05
            thr_filt = alpha * thrust + (1-alpha)* thr_filt
            
            roll = self.roll_amp*math.sin(2 * math.pi * self.roll_freq * timestamp)
            pitch = self.pitch_amp*math.sin(2 * math.pi * self.pitch_freq * timestamp) - thr_filt * 0.01
                        
            self.pub_simu_results(heading, pitch, roll, lin_pos, lin_vel, thrust_vect)
            timestamp+=self.time_step
            
            time.sleep(self.time_step)           
        self.get_logger().info("Finished sim")
    
    def pub_simu_results(self, heading, pitch, roll,  xy_location, xy_velocity, thrust):          
        this_time = self.get_clock().now().to_msg()
        
        quaternion = quaternion_from_euler(roll, pitch, heading)
            
        #Publish GPS Fix Data
        #fix_msg = NavSatFix()
        #fix_msg.header.stamp = this_time
        #fix_msg.header.frame_id = self.gnss_link
        #fix_msg.latitude = bestpos_lat
        #fix_msg.longitude = bestpos_lon
        #fix_msg.altitude = bestpos_hgt
        #fix_msg.position_covariance[0] = float(bestpos_latstd)**2
        #fix_msg.position_covariance[4] = float(bestpos_lonstd)**2
        #fix_msg.position_covariance[8] = float(bestpos_hgtstd)**2
        #fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
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
        odom_msg = Odometry()
        odom_msg.header.stamp = this_time
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id  = 'gps_link'
        odom_msg.pose.pose.position.x = xy_location[0]
        odom_msg.pose.pose.position.y = xy_location[1]
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]
        odom_msg.pose.covariance         = [0.0] * 36
        odom_msg.pose.covariance[21]     = 0.1
        odom_msg.pose.covariance[28]     = 0.1
        odom_msg.pose.covariance[35]     = 0.1
        odom_msg.twist.twist.linear.x    = xy_velocity[0]
        odom_msg.twist.twist.linear.y    = xy_velocity[1]
        odom_msg.twist.twist.linear.z    = 0.0
        odom_msg.twist.covariance        = [0.0] * 36
       
        #publish all messages
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