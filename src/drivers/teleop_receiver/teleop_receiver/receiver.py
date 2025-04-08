import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import threading
import numpy as np

class TeleopReceiver(Node):
    def __init__(self):
        super().__init__('teleop_receiver')
        self.declare_parameter('serial_port','')
        self.declare_parameter('baudrate',115200)
        self.declare_parameter('velocity_topic','/cmd_vel_joy')
        
        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.velocity_topic = self.get_parameter('velocity_topic').get_parameter_value().string_value
    
        self.cmd_vel_msg = Twist()
        self.cmd_vel_publisher = self.create_publisher(Twist,self.velocity_topic,10)
                
        
    def _parse_serial_data(self, serial_data):
        try:
            parts = serial_data.strip().split(',')   
            if len(parts) == 3 and parts[0] == "START":
                command = parts[0]
                value1 = int(parts[1])
                value2 = int(parts[2])
                return [command, value1, value2]
            else:   
                raise ValueError("Invalid data format")
        except Exception as e:
            return None
    
    def serial_comm_loop(self):
        with serial.Serial(port = self.serial_port_name, baudrate = self.baudrate) as ser:   
            self.get_logger().info(f'{self.serial_port_name} port opened successfully')         
            while rclpy.ok():
                # Read a line from the serial port
                if ser.in_waiting > 0:
                    serial_data = ser.readline().decode('utf-8')
                    parsed_data = self._parse_serial_data(serial_data)                 
                    if parsed_data:
                        x_raw = np.clip(pow(parsed_data[2] / 2100,3),-1.0,1.0)
                        rz_raw= np.clip(pow(parsed_data[1] / 2000,3),-1.0,1.0)
                        #self.get_logger().info(f'x={parsed_data[2]} rz={parsed_data[1]}')
                        x_vel = -x_raw*0.4;
                        rz_vel = -rz_raw*1.0;
                        self.cmd_vel_msg.linear.x = x_vel  
                        self.cmd_vel_msg.angular.z = rz_vel 
                        self.cmd_vel_publisher.publish(self.cmd_vel_msg)                    


def main(args=None):
    rclpy.init(args=args)
    teleop_receiver = TeleopReceiver()
    try:
        teleop_receiver.serial_comm_loop()
    except KeyboardInterrupt:
        pass  

if __name__ == '__main__':
    main()