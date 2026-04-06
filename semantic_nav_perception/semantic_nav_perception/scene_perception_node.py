import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class ScenePerceptionNode(Node):
    def __init__(self):
        super().__init__('scene_perception_node')

        self.pose_pub = self.create_publisher(PoseStamped, '/semantic_target_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/semantic_target_marker', 10)

        self.declare_parameter('target_x', 2.0)
        self.declare_parameter('target_y', -1.0)
        self.declare_parameter('target_yaw', 0.0)

        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.target_yaw = float(self.get_parameter('target_yaw').value)

        self.timer = self.create_timer(1.0, self.publish_target)
        self.get_logger().info('Scene perception node started. Publishing fixed target pose.')

    def yaw_to_quaternion(self, yaw: float):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def publish_target(self):
        now = self.get_clock().now().to_msg()
        qz, qw = self.yaw_to_quaternion(self.target_yaw)

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = now
        pose_msg.pose.position.x = self.target_x
        pose_msg.pose.position.y = self.target_y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = now
        marker.ns = 'semantic_target'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose_msg.pose
        marker.scale.x = 0.6
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ScenePerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
