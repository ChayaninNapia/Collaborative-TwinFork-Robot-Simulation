#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String  # เพิ่ม import ด้านบน

class PController(Node):
    def __init__(self):
        super().__init__('p_controller_ai_daeng')

        # parameters: P gains, loop rate, thresholds, max velocities
        self.declare_parameter('kp_lin', 1.0)
        self.declare_parameter('kp_ang', 5.0)
        self.declare_parameter('loop_rate', 10.0)      # Hz
        self.declare_parameter('dist_thresh', 0.05)
        self.declare_parameter('yaw_thresh', 0.01)    # rad
        self.declare_parameter('max_lin_vel', 1.0)    # m/s
        self.declare_parameter('max_ang_vel', 1.5)    # rad/s

        p = self.get_parameter
        self.kp_lin     = p('kp_lin').value
        self.kp_ang     = p('kp_ang').value
        loop_rate       = p('loop_rate').value
        self.dist_thresh= p('dist_thresh').value
        self.yaw_thresh = p('yaw_thresh').value
        self.max_lin    = p('max_lin_vel').value
        self.max_ang    = p('max_ang_vel').value

        # state: 'move' then 'rotate'
        self.state = 'move'
        self.goal = None
        self.current = None

        # publishers & subscribers
        self.status_pub = self.create_publisher(String, '/mission_status/ai_daeng', 10)
        self.pub = self.create_publisher(Twist, '/Ai_Daeng/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/Ai_Daeng/goal_pose', self.goal_cb, 10)
        self.create_subscription(Odometry, '/Ai_Daeng/odom', self.odom_cb, 10)

        # control loop timer
        period = 1.0 / loop_rate if loop_rate > 0 else 0.1
        self.timer = self.create_timer(period, self.control_loop)

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def goal_cb(self, msg: PoseStamped):
        self.goal = msg
        self.state = 'move'
        
        # ส่งสถานะเริ่มทำงาน
        self.status_pub.publish(String(data='working'))
        
        yaw_target = 2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        self.get_logger().info(
            f"[New Goal] x: {msg.pose.position.x:.3f}, y: {msg.pose.position.y:.3f}, "
            f"yaw_target: {yaw_target:.3f}"
        )

    def odom_cb(self, msg: Odometry):
        self.current = msg.pose.pose

    def control_loop(self):
        if self.goal is None or self.current is None:
            return

        # current pose
        px = self.current.position.x
        py = self.current.position.y
        yaw_cur = 2.0 * math.atan2(self.current.orientation.z,
                                   self.current.orientation.w)

        # goal pose
        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y
        yaw_goal_pose = 2.0 * math.atan2(self.goal.pose.orientation.z,
                                         self.goal.pose.orientation.w)

        # vector and distance
        dx = gx - px
        dy = gy - py
        err_dist = math.hypot(dx, dy)

        cmd = Twist()

        if self.state == 'move':
            # desired heading to goal
            yaw_heading = math.atan2(dy, dx)
            err_ang = math.atan2(math.sin(yaw_heading - yaw_cur),
                                  math.cos(yaw_heading - yaw_cur))
            # projected forward error
            err_lin = err_dist * math.cos(err_ang)

            # debug log
            self.get_logger().info(
                f"[Move] dist_err: {err_lin:.3f}, ang_err: {err_ang:.3f}"
            )

            # P controller
            lin = self.kp_lin * err_lin
            ang = self.kp_ang * err_ang

            # clamp
            cmd.linear.x  = self.clamp(lin, -self.max_lin, self.max_lin)
            cmd.angular.z = self.clamp(ang, -self.max_ang, self.max_ang)

            self.pub.publish(cmd)

            # transition to rotate state
            if err_dist < self.dist_thresh:
                self.state = 'rotate'
                self.get_logger().info("Switched to rotate state.")

        elif self.state == 'rotate':
            # rotation error to goal orientation
            err_yaw = math.atan2(math.sin(yaw_goal_pose - yaw_cur),
                                  math.cos(yaw_goal_pose - yaw_cur))

            # debug log
            self.get_logger().info(f"[Rotate] yaw_err: {err_yaw:.3f}")

            # P controller for rotate only
            ang = self.kp_ang * err_yaw
            cmd.linear.x  = 0.0
            cmd.angular.z = self.clamp(ang, -self.max_ang, self.max_ang)

            self.pub.publish(cmd)

            # stop when oriented
            if abs(err_yaw) < self.yaw_thresh:
                cmd.angular.z = 0.0
                self.pub.publish(cmd)
                self.get_logger().info("Goal reached and oriented. Stopping.")
                self.state = 'done'
                
                # ส่งสถานะเสร็จสิ้น
                self.status_pub.publish(String(data='success'))

def main(args=None):
    rclpy.init(args=args)
    node = PController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
