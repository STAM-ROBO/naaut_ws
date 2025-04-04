import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Quaternion
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import tf_transformations
from nav_msgs.msg import Odometry
from diff_motor_msgs.msg import MotorFeedback,MotorCommand

class diff_propeller_controller(Node):

    def __init__(self):
        super().__init__('diff_propeller_controller')
        self.get_logger().info('Motor Controller started')
        self.declare_parameter('wheel_diameter',0.17)
        self.declare_parameter('wheels_base',0.037)
        self.declare_parameter('encoder_ticks_per_rev',90)  
        self.declare_parameter('fixed_frame_id', 'odom')
        self.declare_parameter('robot_frame_id', 'base_footprint')      
        self.declare_parameter('publish_tf',True)
        self.declare_parameter('odom_topic','/odom')
        
        self.frame_id = self.get_parameter('fixed_frame_id').value
        self.child_frame_id = self.get_parameter('robot_frame_id').value        
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        self.wheels_base = self.get_parameter('wheels_base').get_parameter_value().double_value        
        self.round_ticks = self.get_parameter('encoder_ticks_per_rev').get_parameter_value().integer_value   
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        
        self.command_publisher = self.create_publisher(MotorCommand,'/motor_command',5)
        self.odom_publisher = self.create_publisher(Odometry,self.odom_topic,5)
        self.transform_subscriber = self.create_subscription(Twist,'cmd_vel', self.relay_vel, 5)
        self.odometry_subscriber = self.create_subscription(MotorFeedback,'/motor_feedback',self.update_odometry,5)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = None
        self.theta = 0
        self.x = 0
        self.y = 0
        self.cur_ticks_right= -1
        self.cur_ticks_left = -1
        self.wheel_circumference = np.pi*self.wheel_diameter
        

    def get_diff_vel(self,t,r):  
        rSpd = (np.int16)(((t.x + (r.z * self.wheels_base * 0.5))) / self.wheel_circumference *60)
        lSpd = (np.int16)(((t.x - (r.z * self.wheels_base * 0.5))) / self.wheel_circumference *60)
        return rSpd,lSpd 

    def relay_vel(self, msg):
        vel_right,vel_left = self.get_diff_vel(msg.linear,msg.angular)
        msg_cmd = MotorCommand()
        msg_cmd.left_rpm = (int)(vel_left)
        msg_cmd.right_rpm = (int)(vel_right)
        #print(msg_cmd)
        self.command_publisher.publish(msg_cmd)
        #self.hoverboard.update_vels(vel_right,vel_left)

    def update_odometry(self,feedback_msg):
        if(self.last_time is None):
            self.last_time = self.get_clock().now()
            return
        
        delta_t = (self.get_clock().now()-self.last_time).nanoseconds*1e-9
        self.last_time = self.get_clock().now()
        
        r_ticks,l_ticks = feedback_msg.right_ticks_delta,feedback_msg.left_ticks_delta #self.hoverboard.get_curr_speed_r_l()
        r_dist = (self.wheel_circumference * r_ticks)/self.round_ticks
        l_dist = (self.wheel_circumference * l_ticks)/self.round_ticks
        dist_delta = (r_dist+l_dist)/2
        self.theta_delta = (r_dist-l_dist)/self.wheels_base
   
        #velocities im m/s and rad/s
        self.vel_linear = dist_delta/delta_t
        self.vel_rotational = self.theta_delta/delta_t

        # Update pose
        self.delta_x = dist_delta * math.cos((self.theta_delta/2) + self.theta)
        self.x += self.delta_x
        self.delta_y = dist_delta * math.sin((self.theta_delta/2) + self.theta)
        self.y += self.delta_y
        self.theta += self.theta_delta

        # Publish the transform
        self.publish_transform()
        
    def publish_transform(self):
        q_sum = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        q_delta = tf_transformations.quaternion_from_euler(0, 0, self.theta_delta)
      
        """Prepare the transform from odom to base_link."""
        t = TransformStamped()
        
        # Fill in the header       
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id

        # Set translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Set rotation (convert theta to quaternion)    
        t.transform.rotation.x = q_sum[0]
        t.transform.rotation.y = q_sum[1]
        t.transform.rotation.z = q_sum[2]
        t.transform.rotation.w = q_sum[3]
        
        """Prepare the nav_msgs/Odometry message"""
        quat = Quaternion()
        quat.x = q_sum[0]
        quat.y = q_sum[1]
        quat.z = q_sum[2]
        quat.w = q_sum[3]
        odom_msg = Odometry()
        
        odom_msg.header.frame_id = self.frame_id   
        odom_msg.child_frame_id = self.child_frame_id
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quat
        odom_msg.twist.twist.linear.x = self.vel_linear
        odom_msg.twist.twist.linear.y =0.0
        odom_msg.twist.twist.linear.z =0.0
        odom_msg.twist.twist.angular.x =0.0
        odom_msg.twist.twist.angular.y =0.0
        odom_msg.twist.twist.angular.z =self.vel_rotational
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        t.header.stamp = self.get_clock().now().to_msg()
        
        self.odom_publisher.publish(odom_msg)
        if(self.publish_tf):
            self.tf_broadcaster.sendTransform(t)
        

def main(args=None):
    rclpy.init(args=args)
    motor = diff_propeller_controller()
    rclpy.spin(motor)

if __name__ == '__main__':
    main()