import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from rclpy.qos import QoSProfile, DurabilityPolicy

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')
        
        qos_profile = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.publisher_ = self.create_publisher(OccupancyGrid, 'my_map', qos_profile)
        
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.publish_occupancy_grid)    
        self.toggle=0

    def publish_occupancy_grid(self):
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'base_link'
        grid_msg.info.map_load_time = self.get_clock().now().to_msg()
        grid_msg.info.resolution = 0.5 
        grid_msg.info.width = 50      
        grid_msg.info.height = 50     
        grid_msg.info.origin = Pose()
        grid_msg.info.origin.position.x = -(grid_msg.info.width*grid_msg.info.resolution)/2
        grid_msg.info.origin.position.y = -(grid_msg.info.height*grid_msg.info.resolution)/2
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Grid data (-1: unknown, 0: free, 100: occupied)
        grid_data = [0] * (grid_msg.info.width * grid_msg.info.height)

        if self.toggle:
            self.toggle=0
            for i in range(40, 50):
                for j in range(10, 15):
                    grid_data[i * grid_msg.info.width + j] = 100  # Occupied
        else:
            self.toggle=1
            for i in range(40, 50):
                for j in range(20, 26):
                    grid_data[i * grid_msg.info.width + j] =0

        grid_msg.data = grid_data
        self.publisher_.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridPublisher()

    try:
        node.get_logger().info('occupancy_grid_publisher started')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('occupancy_grid_publisher stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
