#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import math


class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')

        # Parameters
        self.declare_parameter('align_offset_x_daeng', -0.18)
        self.declare_parameter('align_offset_x_khieow', 0.18)
        self.declare_parameter('align_offset_y', -2.0)

        self.offset_x_daeng = self.get_parameter('align_offset_x_daeng').value
        self.offset_x_khieow = self.get_parameter('align_offset_x_khieow').value
        self.offset_y = self.get_parameter('align_offset_y').value

        # Status tracking
        self.status_daeng = 'idle'
        self.align_goal_daeng = None
        self.dock_goal_daeng = None

        # Publishers
        self.pub_goal_daeng = self.create_publisher(PoseStamped, '/Ai_Daeng/goal_pose', 10)
        self.pub_goal_khieow = self.create_publisher(PoseStamped, '/Ai_Khieow/goal_pose', 10)
        self.status_pub_daeng_gui = self.create_publisher(String, '/status/ai_daeng', 10)

        # Subscriptions
        self.create_subscription(String, 'pallet_command', self.pallet_command_cb, 10)
        self.create_subscription(String, '/mission_status/ai_daeng', self.status_daeng_cb, 10)
        self.create_subscription(String, '/mission_status/ai_khieow', self.status_khieow_cb, 10)

        # Startup message
        self.get_logger().info('🚀 Mission Planner Node started')
        self.status_pub_daeng_gui.publish(String(data='idle'))

    def apply_offset(self, x, y, theta, dx, dy):
        # ปรับมุม: เพราะ local x หุ่นหันไปทาง +y ของโลก
        adjusted_theta = theta - math.pi / 2
        x_offset = math.cos(adjusted_theta) * dx - math.sin(adjusted_theta) * dy
        y_offset = math.sin(adjusted_theta) * dx + math.cos(adjusted_theta) * dy
        return x + x_offset, y + y_offset

    def pallet_command_cb(self, msg: String):
        self.get_logger().info(f"📦 Received pallet_command: {msg.data}")
        tag, pose = msg.data.split(':')

        if tag != 'lift':
            self.get_logger().warn(f"⚠️ Unsupported tag '{tag}' → Ignored.")
            return

        x_str, y_str, theta_str = pose.split(',')
        x = float(x_str)
        y = float(y_str)
        theta = float(theta_str)

        # ========== Ai_Daeng Goals ==========
        x_align_d, y_align_d = self.apply_offset(x, y, theta, self.offset_x_daeng, self.offset_y)
        x_dock_d, y_dock_d = self.apply_offset(x, y, theta, self.offset_x_daeng, 0.0)

        self.align_goal_daeng = PoseStamped()
        self.align_goal_daeng.header.frame_id = 'map'
        self.align_goal_daeng.header.stamp = self.get_clock().now().to_msg()
        self.align_goal_daeng.pose.position.x = x_align_d 
        self.align_goal_daeng.pose.position.y = y_align_d
        self.align_goal_daeng.pose.orientation.z = math.sin(theta / 2.0)
        self.align_goal_daeng.pose.orientation.w = math.cos(theta / 2.0)

        self.dock_goal_daeng = PoseStamped()
        self.dock_goal_daeng.header.frame_id = 'map'
        self.dock_goal_daeng.header.stamp = self.get_clock().now().to_msg()
        self.dock_goal_daeng.pose.position.x = x_dock_d
        self.dock_goal_daeng.pose.position.y = y_dock_d
        self.dock_goal_daeng.pose.orientation.z = math.sin(theta / 2.0)
        self.dock_goal_daeng.pose.orientation.w = math.cos(theta / 2.0)

        # ========== Ai_Khieow Goals ==========
        x_align_k, y_align_k = self.apply_offset(x, y, theta, self.offset_x_khieow, self.offset_y)
        x_dock_k, y_dock_k = self.apply_offset(x, y, theta, self.offset_x_khieow, 0.0)

        self.align_goal_khieow = PoseStamped()
        self.align_goal_khieow.header.frame_id = 'map'
        self.align_goal_khieow.header.stamp = self.get_clock().now().to_msg()
        self.align_goal_khieow.pose.position.x = x_align_k
        self.align_goal_khieow.pose.position.y = y_align_k
        self.align_goal_khieow.pose.orientation.z = math.sin(theta / 2.0)
        self.align_goal_khieow.pose.orientation.w = math.cos(theta / 2.0)

        self.dock_goal_khieow = PoseStamped()
        self.dock_goal_khieow.header.frame_id = 'map'
        self.dock_goal_khieow.header.stamp = self.get_clock().now().to_msg()
        self.dock_goal_khieow.pose.position.x = x_dock_k
        self.dock_goal_khieow.pose.position.y = y_dock_k
        self.dock_goal_khieow.pose.orientation.z = math.sin(theta / 2.0)
        self.dock_goal_khieow.pose.orientation.w = math.cos(theta / 2.0)

        # ส่ง goal ให้ Ai_Daeng เริ่ม
        self.pub_goal_daeng.publish(self.align_goal_daeng)
        self.status_daeng = 'aligning'
        self.status_pub_daeng_gui.publish(String(data=self.status_daeng))
        self.get_logger().info("🟥 Sent align goal to Ai_Daeng")

    def status_daeng_cb(self, msg: String):
        self.get_logger().info(f"📨 /mission_status/ai_daeng: {msg.data}")
        if msg.data == 'success':
            if self.status_daeng == 'aligning':
                self.status_daeng = 'docking'
                self.status_pub_daeng_gui.publish(String(data=self.status_daeng))
                self.pub_goal_daeng.publish(self.dock_goal_daeng)
                self.get_logger().info("🟥 Alignment done → Sending docking goal to Ai_Daeng")
            elif self.status_daeng == 'docking':
                self.status_daeng = 'idle'
                self.status_pub_daeng_gui.publish(String(data=self.status_daeng))
                self.get_logger().info("🟥 Docking done → Ai_Daeng is idle")

    def status_khieow_cb(self, msg: String):
        self.get_logger().info(f"📨 /mission_status/ai_khieow: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
