#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DiffCmdVelRelay(Node):
    def __init__(self):
        super().__init__('diff_cmd_vel_relay')

        # Declare track width (meter) ระยะห่างระหว่างสองหุ่น
        self.declare_parameter('wheel_base', 0.4)
        self.wheel_base = self.get_parameter('wheel_base').value

        # Subscriber to the generic /cmd_vel topic
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_cb,
            10
        )

        # Publishers in each namespace
        self.pub_daeng = self.create_publisher(
            Twist,
            '/Ai_Daeng/cmd_vel',
            10
        )
        self.pub_khieow = self.create_publisher(
            Twist,
            '/Ai_Khieow/cmd_vel',
            10
        )

        self.get_logger().info(
            f'DiffCmdVelRelay started (wheel_base={self.wheel_base:.2f} m)'
        )

    def cmd_vel_cb(self, msg: Twist):
        # Extract commanded linear and angular velocity
        v = msg.linear.x
        w = msg.angular.z
        b = self.wheel_base

        # Compute left/right wheel velocities
        v_l = v - (w * b / 2.0)
        v_r = v + (w * b / 2.0)

        # Build Twist messages for each "wheel"
        twist_l = Twist()
        twist_l.linear.x = v_l
        # leave other fields zero (no angular.z on each wheel)

        twist_r = Twist()
        twist_r.linear.x = v_r

        # Publish
        self.pub_daeng.publish(twist_l)
        self.pub_khieow.publish(twist_r)

        self.get_logger().debug(
            f"cmd_vel→ v={v:.3f}, w={w:.3f} ⇒ v_L={v_l:.3f}, v_R={v_r:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DiffCmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
