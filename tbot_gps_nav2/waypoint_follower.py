import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped
from tbot_gps_nav2.gps_utils import latLonYaw2Geopose
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL

from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class GpsWaypointsFollower(Node):
    def __init__(self):
        super().__init__(node_name="gps_wp_commander")
        self.navigator = BasicNavigator("basic_navigator")
        self.mapviz_wp_sub = self.create_subscription( PointStamped, "/clicked_point", self.mapviz_wp_cb, 1)
        
        self.marker_pub = self.create_publisher(Marker,'/visualization_marker',10)
        self.path_pub = self.create_publisher(Path, 'waypoint_path', 10)
        
        self.localizer = self.create_client(FromLL,  '/fromLL')
        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            
        self.get_logger().info('Ready for waypoints...')
        
        self.waypoints = []
        self.points = []

    def mapviz_wp_cb(self, msg: PointStamped):
        if msg.header.frame_id != "wgs84":
            self.get_logger().warning(
                "Received point from mapviz that ist not in wgs84 frame. This is not a gps point and wont be followed")
            return
    
        wps = [latLonYaw2Geopose(msg.point.y, msg.point.x)]
        
        self.points.append(msg)
        marker = Marker()
        marker.header = msg.header
        marker.ns = "mapviz_points"
        marker.id = len(self.points)  # Unique ID for each point
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = msg.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # Scale of the sphere
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red color

        self.marker_pub.publish(marker)

        for wp in wps:
            req = FromLL.Request()
            req.ll_point.longitude = wp.position.longitude
            req.ll_point.latitude = wp.position.latitude
            req.ll_point.altitude = wp.position.altitude

            self.get_logger().info("Waypoint added to conversion queue...")
            future = self.localizer.call_async(req)
            future.add_done_callback(self.command_send_cb)


    def command_send_cb(self, future):
        try:
            response = future.result()
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position = response.map_point
            
            self.waypoints.append(pose)
            # self.get_logger().info("Following converted waypoint...")
            # self.navigator.goToPose(pose)
            
            if len(self.waypoints) > 3:
                path = Path()
                path.header.frame_id = 'map'
                path.header.stamp = self.get_clock().now().to_msg()
                path.poses = self.waypoints
                self.path_pub.publish(path)
                
                self.navigator.followWaypoints(self.waypoints)
                self.waypoints = []
                
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main():
    rclpy.init()
    node = GpsWaypointsFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
