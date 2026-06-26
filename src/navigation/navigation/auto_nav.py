import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
import time
import math

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.original_waypoints = []
        self.waypoints          = []
        self.current_idx        = 0

        # object 처리용 별도 경로 [object_pos, home_pos]
        self.object_waypoints   = []
        self.object_idx         = 0

        self.is_running         = False
        self.object_found       = False
        self.home_x             = None
        self.home_y             = None
        self.center_x = None
        self.center_y = None
        self.current_handle     = None

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.create_subscription(Path, '/coverage_path', self.path_callback, latched_qos)
        self.create_subscription(PoseStamped, '/object_pose', self.object_callback, latched_qos)
        self.get_logger().info('AutoNav Ready.')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except:
            return self.home_x, self.home_y  # fallback

    def path_callback(self, msg):
        if self.is_running:
            self.get_logger().warn('Already navigating, ignoring new path')
            return

        self.original_waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.waypoints = list(self.original_waypoints)
        self.home_x = self.waypoints[-1][0]
        self.home_y = self.waypoints[-1][1]
        self.center_x = self.waypoints[2][0]
        self.center_y = self.waypoints[2][1]

        self.current_idx = 0
        self.is_running = True
        self.get_logger().info(f'Received {len(self.waypoints)} waypoints')
        self.send_next_goal()

    def object_callback(self, msg):
        if self.object_found:
            return

        self.object_found = True
        obj_x = msg.pose.position.x
        obj_y = msg.pose.position.y
        self.get_logger().info(f'🎯 Object detected! Heading to object then home...')

        resume_x, resume_y = self.waypoints[self.current_idx]

        if self.current_idx in (5, 6, 7):
            self.object_waypoints = [
                (obj_x, obj_y),
                (self.center_x, self.center_y),
                (self.home_x, self.home_y),
                (self.center_x, self.center_y),
                (resume_x, resume_y),          # 복귀 목적지
            ]
        else:
            self.object_waypoints = [
                (obj_x, obj_y),
                (self.home_x, self.home_y),
                (resume_x, resume_y),          # 복귀 목적지
            ]

        self.object_idx = 0
        
        if self.current_handle is not None:
            self.current_handle.cancel_goal_async()

    def send_next_goal(self):
        if self.object_found:
            # object 처리 경로 주행
            if self.object_idx >= len(self.object_waypoints):
                # home까지 도착 완료 → 원래 경로 current_idx부터 재개
                self.get_logger().info(f'✅ Object handling done. Resuming patrol from waypoint [{self.current_idx}/{len(self.waypoints)}]')
                self.object_found = False
                self.object_waypoints = []
                self.object_idx = 0
                self.current_idx += 1
            else:
                x, y = self.object_waypoints[self.object_idx]
                label = '[OBJECT]' if self.object_idx == 0 else '[HOME]'
                self.get_logger().info(f'Navigating to {label} ({x:.2f}, {y:.2f})')
                self.send_goal(x, y)
                return

        # 정상 순찰 경로 반복
        # if self.current_idx >= len(self.waypoints):
        #     self.get_logger().info('🔄 [LOOP] Arrived at HOME. Restarting patrol!')
        #     self.current_idx = 0
        #     self.waypoints = list(self.original_waypoints)
        #     time.sleep(1.0)

        # 한번 돌고 멈추기
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('🏁 Patrol finished. Shutting down...')
            self.destroy_node()
            rclpy.shutdown()
            return

        x, y = self.waypoints[self.current_idx]
        total = len(self.waypoints)
        label = '[HOME]' if self.current_idx == total - 1 else f'[{self.current_idx + 1}/{total}]'
        self.get_logger().info(f'Navigating to {label} ({x:.2f}, {y:.2f})')
        self.send_goal(x, y)

    def send_goal(self, x, y):
        cur_x, cur_y = self.get_current_pose()
        dx = x - cur_x
        dy = y - cur_y
        yaw = math.atan2(dy, dx)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)

        goal_msg      = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected! Skipping.')
            if self.object_found:
                self.object_idx += 1
            else:
                self.current_idx += 1
            self.send_next_goal()
            return

        self.current_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        if self.object_found:
            x, y = self.object_waypoints[self.object_idx]
            self.get_logger().info(f'✅ Reached object waypoint ({x:.2f}, {y:.2f})')
            self.object_idx += 1
        else:
            x, y = self.waypoints[self.current_idx]
            self.get_logger().info(f'✅ Reached ({x:.2f}, {y:.2f})')
            self.current_idx += 1

        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'  Distance remaining: {dist:.2f}m', throttle_duration_sec=3.0)


def main(args=None):
    rclpy.init(args=args)
    node = AutoNav()
    rclpy.spin(node)

if __name__ == '__main__':
    main()