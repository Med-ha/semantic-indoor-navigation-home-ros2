import math
import os
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class GoToNamedPlaceNode(Node):
    def __init__(self):
        super().__init__('go_to_named_place_node')

        self.declare_parameter(
            'places_file',
            os.path.expanduser('~/semantic_nav_ws/src/semantic_nav_bringup/config/named_places.yaml')
        )

        self.places_file = os.path.expanduser(self.get_parameter('places_file').value)

        self.cmd_sub = self.create_subscription(
            String,
            '/go_to_place',
            self.command_callback,
            10
        )

        self.pose_pub = self.create_publisher(PoseStamped, '/semantic_target_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/semantic_target_marker', 10)

        self.get_logger().info("Go-to-named-place node started. Waiting for /go_to_place commands.")

    def load_places(self):
        if not os.path.exists(self.places_file):
            return {}

        with open(self.places_file, 'r') as f:
            data = yaml.safe_load(f)

        if not isinstance(data, dict):
            return {}

        return data.get('places', {})

    def yaw_to_quaternion(self, yaw: float):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def command_callback(self, msg: String):
        place_name = msg.data.strip()
        places = self.load_places()

        if place_name not in places:
            self.get_logger().warn(f"Place '{place_name}' not found in {self.places_file}")
            self.get_logger().info(f"Available places: {list(places.keys())}")
            return

        place = places[place_name]
        x = float(place['x'])
        y = float(place['y'])
        yaw = float(place.get('yaw', 0.0))
        qz, qw = self.yaw_to_quaternion(yaw)

        now = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = now
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.pose_pub.publish(pose)

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = now
        marker.ns = 'named_place_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.28
        marker.scale.y = 0.28
        marker.scale.z = 0.28
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

        self.get_logger().info(f"Published target for place '{place_name}' at x={x:.2f}, y={y:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = GoToNamedPlaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
