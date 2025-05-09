import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped
from naaut_base.utils.gps_utils import *
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import time

class InteractiveGpsWpCommander(Node):
    """
    ROS2 node to send gps waypoints to nav2 received from mapviz's point click publisher
    """

    def __init__(self):
        super().__init__(node_name="gps_wp_commander")
        self.navigator = BasicNavigator("basic_navigator")
        self.mapviz_wp_sub = self.create_subscription(PointStamped, "/clicked_point", self.mapviz_wp_cb, 1)       
        
        goalPose=Point()
        goalPose.x=40.0 
        goalPose.y=40.0        
        goalPose.z=0.0
        self.resp = PoseStamped()
        self.resp.header.frame_id = 'map'
        self.resp.header.stamp = self.get_clock().now().to_msg()
        self.resp.pose.position = goalPose
        self.navigator.goToPose(self.resp)

        self.get_logger().info('Ready for waypoints...')

    def mapviz_wp_cb(self, msg: PointStamped):
        """
        clicked point callback, sends received point to nav2 gps waypoint follower if its a geographic point
        """
        if msg.header.frame_id != "wgs84":
            self.get_logger().warning("Received point from mapviz that ist not in wgs84 frame. This is not a gps point and wont be followed")
            return

        #convert from WGS84 to UTM
        lat = msg.point.x
        lon = msg.point.y        
        coord_transformer = wgs84_to_UTM(lat, lon)
        utm_x, utm_y = coord_transformer.transform(lat, lon)
        
        #il goalpose deve aver origine 0,0
        #TODO traslazione sul DATUM
        
        goalPose=Point()
        goalPose.x=40.0 
        goalPose.y=40.0        
        goalPose.z=0.0
        
        self.resp = PoseStamped()
        self.resp.header.frame_id = 'map'
        self.resp.header.stamp = self.get_clock().now().to_msg()
        self.resp.pose.position = goalPose
        self.navigator.goToPose(self.resp)


    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            time.sleep(0.1)

def main():
    rclpy.init()
    gps_wpf = InteractiveGpsWpCommander()
    gps_wpf.spin()


if __name__ == "__main__":
    main()
