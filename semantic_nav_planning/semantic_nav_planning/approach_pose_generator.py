import math

import rclpy
from rclpy.node import Node

#from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray


class ApproachPoseGenerator(Node):
    def __init__(self):
        super().__init__('approach_pose_generator')

        self.target_sub = self.create_subscription(
            PoseStamped,
            '/semantic_target_pose',
            self.target_callback,
            10
        )

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )

        self.candidate_pub = self.create_publisher(MarkerArray, '/candidate_poses', 10)
        self.selected_pose_pub = self.create_publisher(PoseStamped, '/selected_approach_pose', 10)
        self.selected_marker_pub = self.create_publisher(Marker, '/selected_approach_marker', 10)
        self.candidate_scores_pub = self.create_publisher(Float32MultiArray, '/candidate_scores', 10)
        self.ranked_pose_pub = self.create_publisher(PoseArray, '/ranked_approach_poses', 10)

        self.declare_parameter('radius', 0.7)
        self.declare_parameter('num_candidates', 8)
        self.declare_parameter('clearance_samples', 8)
        self.declare_parameter('clearance_radius', 0.22)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('min_robot_goal_distance', 0.30)
        self.min_robot_goal_distance = float(self.get_parameter('min_robot_goal_distance').value)

        self.radius = float(self.get_parameter('radius').value)
        self.num_candidates = int(self.get_parameter('num_candidates').value)
        self.clearance_samples = int(self.get_parameter('clearance_samples').value)
        self.clearance_radius = float(self.get_parameter('clearance_radius').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)

        self.robot_x = None
        self.robot_y = None

        self.costmap = None
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None

        self.get_logger().info('Approach pose generator started.')

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

    def yaw_to_quaternion(self, yaw: float):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def world_to_map(self, x, y):
        if self.costmap is None:
            return None

        mx = int((x - self.map_origin_x) / self.map_resolution)
        my = int((y - self.map_origin_y) / self.map_resolution)

        if mx < 0 or my < 0 or mx >= self.map_width or my >= self.map_height:
            return None

        return mx, my

    def get_cost(self, x, y):
        cell = self.world_to_map(x, y)
        if cell is None:
            return 100

        mx, my = cell
        index = my * self.map_width + mx
        return self.costmap[index]

    def is_occupied(self, x, y):
        cost = self.get_cost(x, y)
        return cost >= self.occupied_threshold

    def estimate_clearance_score(self, x, y):
        free_count = 0

        for i in range(self.clearance_samples):
            theta = (2.0 * math.pi / self.clearance_samples) * i
            sx = x + self.clearance_radius * math.cos(theta)
            sy = y + self.clearance_radius * math.sin(theta)

            if not self.is_occupied(sx, sy):
                free_count += 1

        return free_count / float(self.clearance_samples)

    def compute_heading_score(self, candidate_yaw, cand_x, cand_y, target_x, target_y):
        desired_yaw = math.atan2(target_y - cand_y, target_x - cand_x)
        diff = abs(math.atan2(math.sin(desired_yaw - candidate_yaw), math.cos(desired_yaw - candidate_yaw)))
        return 1.0 - min(diff / math.pi, 1.0)

    def target_callback(self, msg: PoseStamped):
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        valid_candidates = []
        score_msg = Float32MultiArray()

        for i in range(self.num_candidates):
            theta = (2.0 * math.pi / self.num_candidates) * i

            cand_x = target_x + self.radius * math.cos(theta)
            cand_y = target_y + self.radius * math.sin(theta)

            yaw = math.atan2(target_y - cand_y, target_x - cand_x)
            qz, qw = self.yaw_to_quaternion(yaw)

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = now
            pose.pose.position.x = cand_x
            pose.pose.position.y = cand_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            occupied = self.is_occupied(cand_x, cand_y)
            clearance_score = self.estimate_clearance_score(cand_x, cand_y)
            heading_score = self.compute_heading_score(yaw, cand_x, cand_y, target_x, target_y)

            if self.robot_x is not None and self.robot_y is not None:
                dist = self.distance(self.robot_x, self.robot_y, cand_x, cand_y)
                distance_score = 1.0 / (1.0 + dist)
            else:
                dist = 999.0
                distance_score = 0.0

            if dist < self.min_robot_goal_distance:
                occupied = True

            total_score = (
                1.5 * clearance_score +
                1.0 * heading_score +
                1.0 * distance_score
            )

            score_msg.data.append(float(total_score))

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = now
            marker.ns = 'candidate_poses'
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x = 0.45
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            marker.color.a = 0.9

#            if occupied:
#                marker.color.r = 0.5
#                marker.color.g = 0.5
#                marker.color.b = 0.5
#            else:
#                marker.color.r = 0.2
#                marker.color.g = 0.5
#                marker.color.b = 1.0
#                valid_candidates.append((total_score, pose))

            if occupied or clearance_score < 0.6:
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
            else:
                marker.color.r = 0.2
                marker.color.g = 0.5
                marker.color.b = 1.0
                valid_candidates.append((total_score, pose))

            marker_array.markers.append(marker)

        self.candidate_pub.publish(marker_array)
        self.candidate_scores_pub.publish(score_msg)

        if not valid_candidates:
            self.get_logger().warn('No valid approach candidates found.')
            return

        valid_candidates.sort(key=lambda x: x[0], reverse=True)

        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        pose_array.header.stamp = now
        pose_array.poses = [item[1].pose for item in valid_candidates[:3]]
        self.ranked_pose_pub.publish(pose_array)

        best_pose = valid_candidates[0][1]
        self.selected_pose_pub.publish(best_pose)

        best_marker = Marker()
        best_marker.header.frame_id = 'map'
        best_marker.header.stamp = now
        best_marker.ns = 'selected_approach_pose'
        best_marker.id = 999
        best_marker.type = Marker.ARROW
        best_marker.action = Marker.ADD
        best_marker.pose = best_pose.pose
        best_marker.scale.x = 0.6
        best_marker.scale.y = 0.12
        best_marker.scale.z = 0.12
        best_marker.color.a = 1.0
        best_marker.color.r = 1.0
        best_marker.color.g = 0.2
        best_marker.color.b = 0.2
        self.selected_marker_pub.publish(best_marker)

        self.get_logger().info(
            f'Selected best approach pose at x={best_pose.pose.position.x:.2f}, y={best_pose.pose.position.y:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ApproachPoseGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
