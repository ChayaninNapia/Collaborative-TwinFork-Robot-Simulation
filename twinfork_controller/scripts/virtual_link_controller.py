#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion
from builtin_interfaces.msg import Duration
from std_msgs.msg import String

class VirtualLinkPController(Node):
    def __init__(self):
        super().__init__('virtual_link_p_controller')

        self.declare_parameter('kp_lin', 1.0)
        self.declare_parameter('kp_ang', 2.0)
        self.declare_parameter('lin_thresh', 0.01)
        self.declare_parameter('ang_thresh', 0.01)
        self.declare_parameter('wheel_base', 0.4)
        self.declare_parameter('loop_rate', 10.0)
        self.declare_parameter('max_lin_vel', 1.0)
        self.declare_parameter('max_ang_vel', 1.0)

        self.kp_lin = self.get_parameter('kp_lin').value
        self.kp_ang = self.get_parameter('kp_ang').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.lin_thresh = self.get_parameter('lin_thresh').value
        self.ang_thresh = self.get_parameter('ang_thresh').value
        self.max_lin_vel = self.get_parameter('max_lin_vel').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value

        self.goal = None
        self.state = 'idle'

        self.tf_buffer = Buffer(cache_time=Duration(sec=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_goal = self.create_subscription(
            PoseStamped,
            '/Virtual/goal_pose',
            self.goal_cb,
            10
        )

        self.pub_daeng = self.create_publisher(Twist, '/Ai_Daeng/cmd_vel', 10)
        self.pub_khieow = self.create_publisher(Twist, '/Ai_Khieow/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission_status/virtual', 10)


        self.timer = self.create_timer(1.0 / self.get_parameter('loop_rate').value, self.control_loop)

        self.get_logger().info("VirtualLinkPController node started.")

    def goal_cb(self, msg):
        self.goal = msg.pose
        self.state = 'move'
        self.get_logger().info(f"[New Goal] x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, yaw={2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w):.2f} rad")
        self.status_pub.publish(String(data='working'))


    def control_loop(self):
        if not self.goal:
            return

        try:
            tf = self.tf_buffer.lookup_transform('map', 'virtual_forklift', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        dx = self.goal.position.x - x
        dy = self.goal.position.y - y
        dist = math.hypot(dx, dy)

        target_yaw = math.atan2(dy, dx)
        dtheta = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
        goal_yaw = 2.0 * math.atan2(self.goal.orientation.z, self.goal.orientation.w)
        final_yaw_error = math.atan2(math.sin(goal_yaw - yaw), math.cos(goal_yaw - yaw))

        cmd = Twist()

        if self.state == 'move':
            if dist > self.lin_thresh:
                cmd.linear.x = self.clamp(self.kp_lin * dist, -self.max_lin_vel, self.max_lin_vel)
                cmd.angular.z = self.clamp(self.kp_ang * dtheta, -self.max_ang_vel, self.max_ang_vel)
                self.get_logger().info(f"[Move] dist={dist:.2f}, dtheta={math.degrees(dtheta):.2f}°")
            else:
                self.state = 'rotate'
                self.get_logger().info("Switched to [rotate] state.")

        elif self.state == 'rotate':
            if abs(final_yaw_error) > self.ang_thresh:
                cmd.angular.z = self.clamp(self.kp_ang * final_yaw_error, -self.max_ang_vel, self.max_ang_vel)
                self.get_logger().info(f"[Rotate] final_yaw_error={math.degrees(final_yaw_error):.2f}°")
            else:
                self.state = 'idle'
                cmd = Twist()  # stop
                self.get_logger().info("[Idle] Goal reached. All motion stopped.")
                self.status_pub.publish(String(data='success'))


        if self.state != 'idle':
            v_l = cmd.linear.x - cmd.angular.z * self.wheel_base / 2.0
            v_r = cmd.linear.x + cmd.angular.z * self.wheel_base / 2.0

            msg_l = Twist()
            msg_l.linear.x = v_l

            msg_r = Twist()
            msg_r.linear.x = v_r

            self.pub_daeng.publish(msg_l)
            self.pub_khieow.publish(msg_r)

            self.get_logger().info(
                f"[Control] L={v_l:.2f} m/s, R={v_r:.2f} m/s | lin={cmd.linear.x:.2f}, ang={cmd.angular.z:.2f}"
            )

    def clamp(self, value, min_val, max_val):
        return max(min_val, min(max_val, value))

def main(args=None):
    rclpy.init(args=args)
    node = VirtualLinkPController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
