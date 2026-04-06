import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker


class ClickedPointTargetNode(Node):
    def __init__(self):
        super().__init__('clicked_point_target_node')

        self.clicked_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_callback,
            10
        )

        self.pose_pub = self.create_publisher(PoseStamped, '/semantic_target_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/semantic_target_marker', 10)

        self.get_logger().info('Clicked point target node started. Use RViz Publish Point.')

    def clicked_callback(self, msg: PointStamped):
        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)

        marker = Marker()
        marker.header.frame_id = pose.header.frame_id
        marker.header.stamp = pose.header.stamp
        marker.ns = 'semantic_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)

        self.get_logger().info(
            f'New clicked target: x={msg.point.x:.2f}, y={msg.point.y:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ClickedPointTargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
