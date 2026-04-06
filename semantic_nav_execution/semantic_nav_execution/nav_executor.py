import math
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class NavExecutor(Node):
    def __init__(self):
        super().__init__('nav_executor')
        
        self.eval_result_pub = self.create_publisher(String, '/nav_eval_result', 10)
        self.eval_active_goal_pub = self.create_publisher(PoseStamped, '/nav_eval_active_goal', 10)
        
        self.goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')


        self.ranked_sub = self.create_subscription(
            PoseArray,
            '/ranked_approach_poses',
            self.ranked_callback,
            10
        )

        self.goal_active = False
        self.current_candidates = []
        self.current_goal_index = 0
        self.last_goal_xy = None
        self.goal_resend_threshold = 0.05

        self.current_frame_id = 'map'
        self.current_total_candidates = 0

        self.get_logger().info('Nav executor started. Waiting for /ranked_approach_poses')

    def pose_distance_xy(self, pose, last_xy):
        dx = pose.position.x - last_xy[0]
        dy = pose.position.y - last_xy[1]
        return math.sqrt(dx * dx + dy * dy)

#    def ranked_callback(self, msg: PoseArray):
#        if self.goal_active:
#            return

#        if len(msg.poses) == 0:
#            self.get_logger().warn('Received empty ranked pose list.')
#            return

#        self.current_candidates = msg.poses
#        self.current_goal_index = 0

#        self.send_current_goal(msg.header.frame_id, msg.header.stamp)

    def ranked_callback(self, msg: PoseArray):
        if self.goal_active:
            return

        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty ranked pose list.')
            return

        self.current_candidates = msg.poses
        self.current_goal_index = 0
        self.current_frame_id = msg.header.frame_id if msg.header.frame_id else 'map'
        self.current_total_candidates = len(msg.poses)

        self.send_current_goal(msg.header.frame_id, msg.header.stamp)

    def send_current_goal(self, frame_id, stamp):
        if self.current_goal_index >= len(self.current_candidates):
            self.get_logger().warn('No more fallback candidates left.')
            return

        pose = self.current_candidates[self.current_goal_index]

        current_xy = (pose.position.x, pose.position.y)

        if self.last_goal_xy is not None:
            if self.pose_distance_xy(pose, self.last_goal_xy) < self.goal_resend_threshold:
                self.get_logger().info('Goal too close to previous one. Skipping resend.')
                return

        if not self.goal_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('NavigateToPose action server not available yet.')
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id if frame_id else 'map'
        goal_pose.header.stamp = stamp
        goal_pose.pose = pose

        self.eval_active_goal_pub.publish(goal_pose)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.goal_active = True
        self.last_goal_xy = current_xy

        self.get_logger().info(
            f'Sending ranked goal {self.current_goal_index + 1}/{len(self.current_candidates)}: '
            f'x={pose.position.x:.2f}, y={pose.position.y:.2f}'
        )

        self.send_goal_future = self.goal_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)


    def publish_eval_result(self, status_text):
        msg = String()
        msg.data = (
            f'status={status_text},'
            f'candidate_index={self.current_goal_index + 1},'
            f'total_candidates={self.current_total_candidates}'
        )
        self.eval_result_pub.publish(msg)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            self.goal_active = False
            self.try_next_candidate()
            return

        self.get_logger().info('Goal accepted by Nav2.')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f} m'
        )

#    def result_callback(self, future):
#        result = future.result()
#        status = result.status

#        if status == GoalStatus.STATUS_SUCCEEDED:
#            self.get_logger().info('Navigation succeeded.')
#            self.goal_active = False
#            self.current_candidates = []
#            self.current_goal_index = 0
#            return

#        if status == GoalStatus.STATUS_ABORTED:
#            self.get_logger().warn('Navigation aborted.')
#        elif status == GoalStatus.STATUS_CANCELED:
#            self.get_logger().warn('Navigation canceled.')
#        else:
#            self.get_logger().warn(f'Navigation ended with status code: {status}')

#        self.goal_active = False
#        self.try_next_candidate()

    def result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded.')
            self.publish_eval_result('success')
            self.goal_active = False
            self.current_candidates = []
            self.current_goal_index = 0
            return

        if status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Navigation aborted.')
            self.publish_eval_result('aborted')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation canceled.')
            self.publish_eval_result('canceled')
        else:
            self.get_logger().warn(f'Navigation ended with status code: {status}')
            self.publish_eval_result(f'status_{status}')

        self.goal_active = False
        self.try_next_candidate()

    def try_next_candidate(self):
        self.current_goal_index += 1

        if self.current_goal_index >= len(self.current_candidates):
            self.get_logger().warn('All ranked candidates failed.')
            self.publish_eval_result('all_failed')
            return

        self.get_logger().info(
            f'Trying fallback candidate {self.current_goal_index + 1}/{len(self.current_candidates)}'
        )

        now = self.get_clock().now().to_msg()
#        self.send_current_goal('map', now)
        self.send_current_goal(self.current_frame_id, now)

def main(args=None):
    rclpy.init(args=args)
    node = NavExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
