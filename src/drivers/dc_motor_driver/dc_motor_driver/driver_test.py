
import rclpy
from rclpy.node import Node
import serial
import numpy as np
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
import struct
import math
import threading
import time
from geometry_msgs.msg import Twist

class dc_driver_node(Node):
    def __init__(self):
        super().__init__('minolo_motor_interface')
        self.cur_speed_r = 0
        self.cur_speed_l = 0
        self.cmd_speed_r = 0
        self.cmd_speed_l =0

        self.declare_parameter('serial_port','/dev/ttyACM0')
        self.declare_parameter('baudrate',115200)
        
        self.wheel_diameter = 0.2
        self.wheels_base = 1.5   
        self.battery_volt= 0

        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value      
        
        self.cmd_vel_sub = self.create_subscription(Twist,'cmd_vel', self.get_diff_vel, 10)
        self.battery_status_publisher = self.create_publisher(String,'/battery_state',20)
        self.battery_timer = self.create_timer(1,self.publish_battery_status)
     
        serial_comm_thread = threading.Thread(target=self.serial_comm_loop, daemon=True)
        serial_comm_thread.start()

    
    def get_diff_vel(self,msg):
        t=msg.linear
        r=msg.angular  
        self.cmd_speed_r = (np.int16)(((t.x + (r.z * self.wheels_base * 0.5))) / (np.pi*self.wheel_diameter) *60)
        self.cmd_speed_l = (np.int16)(((t.x - (r.z * self.wheels_base * 0.5))) / (np.pi*self.wheel_diameter) *60)
        
    def serial_comm_loop(self):
        try:
            with serial.Serial(port = self.serial_port_name, baudrate = self.baudrate) as ser:   
                while rclpy.ok():  
                    serial_packet = ser.read()
                    recv_unpacked_data = struct.unpack('B',serial_packet)
                    self.battery_volt=float(recv_unpacked_data[0])/10.0/1.024
                    time.sleep(0.01)                          
                    cmd_speed_l=np.clip(self.cmd_speed_l, -127, 127)
                    cmd_speed_r=np.clip(self.cmd_speed_r, -127, 127)
                    print(f"A:{cmd_speed_l}, B:{cmd_speed_r}")  
                    serial_command = struct.pack('bb',cmd_speed_l, cmd_speed_r)
                    ser.write(serial_command)
        except Exception as ex:
            self.get_logger().error(ex)

    def publish_battery_status(self):
        msg = String()      
        msg.data = f"voltage: {self.battery_volt:.1f}V"
        self.battery_status_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    hover = dc_driver_node()
    try:
        rclpy.spin(hover)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()