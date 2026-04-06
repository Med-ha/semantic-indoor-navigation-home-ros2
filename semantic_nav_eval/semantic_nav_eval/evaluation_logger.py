import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String


class EvaluationLogger(Node):
    def __init__(self):
        super().__init__('evaluation_logger')

        self.declare_parameter(
            'output_csv',
            os.path.expanduser('~/semantic_nav_ws/results/eval_log.csv')
        )

        self.output_csv = os.path.expanduser(
            self.get_parameter('output_csv').value
        )

        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)

        self.target_sub = self.create_subscription(
            PoseStamped,
            '/semantic_target_pose',
            self.target_callback,
            10
        )

        self.ranked_sub = self.create_subscription(
            PoseArray,
            '/ranked_approach_poses',
            self.ranked_callback,
            10
        )

        self.active_goal_sub = self.create_subscription(
            PoseStamped,
            '/nav_eval_active_goal',
            self.active_goal_callback,
            10
        )

        self.result_sub = self.create_subscription(
            String,
            '/nav_eval_result',
            self.result_callback,
            10
        )

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        self.current_target = None
        self.current_goal = None
        self.current_ranked_count = 0
        self.robot_x = None
        self.robot_y = None
        self.run_start_time = None

        self.ensure_csv_header()
        self.get_logger().info(f'Logging evaluation rows to: {self.output_csv}')

    def ensure_csv_header(self):
        file_exists = os.path.exists(self.output_csv)
        if not file_exists or os.path.getsize(self.output_csv) == 0:
            with open(self.output_csv, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp',
                    'target_x',
                    'target_y',
                    'goal_x',
                    'goal_y',
                    'ranked_pose_count',
                    'candidate_used',
                    'status',
                    'nav_time_sec',
                    'final_pose_error_m'
                ])

    def target_callback(self, msg: PoseStamped):
        self.current_target = msg
        self.run_start_time = self.get_clock().now()

    def ranked_callback(self, msg: PoseArray):
        self.current_ranked_count = len(msg.poses)

    def active_goal_callback(self, msg: PoseStamped):
        self.current_goal = msg

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def parse_result(self, text):
        result = {}
        parts = text.split(',')
        for part in parts:
            if '=' in part:
                k, v = part.split('=', 1)
                result[k.strip()] = v.strip()
        return result

    def compute_final_pose_error(self):
        if self.current_goal is None or self.robot_x is None or self.robot_y is None:
            return -1.0

        gx = self.current_goal.pose.position.x
        gy = self.current_goal.pose.position.y
        return math.sqrt((gx - self.robot_x) ** 2 + (gy - self.robot_y) ** 2)

    def result_callback(self, msg: String):
        parsed = self.parse_result(msg.data)

        if self.current_target is None or self.current_goal is None or self.run_start_time is None:
            self.get_logger().warn('Skipping log row because target/goal/start time is missing.')
            return

        now = self.get_clock().now()
        nav_time_sec = (now - self.run_start_time).nanoseconds / 1e9
        final_error = self.compute_final_pose_error()

        row = [
            datetime.now().isoformat(timespec='seconds'),
            self.current_target.pose.position.x,
            self.current_target.pose.position.y,
            self.current_goal.pose.position.x,
            self.current_goal.pose.position.y,
            self.current_ranked_count,
            parsed.get('candidate_index', 'unknown'),
            parsed.get('status', 'unknown'),
            round(nav_time_sec, 3),
            round(final_error, 3)
        ]

        with open(self.output_csv, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)

        self.get_logger().info(f'Logged row: {row}')


def main(args=None):
    rclpy.init(args=args)
    node = EvaluationLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
