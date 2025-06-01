#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
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
        self.status_khieow = 'idle'
        self.align_goal_daeng = None
        self.dock_goal_daeng = None

        # Publishers
        self.pub_goal_daeng = self.create_publisher(PoseStamped, '/Ai_Daeng/goal_pose', 10)
        self.pub_goal_khieow = self.create_publisher(PoseStamped, '/Ai_Khieow/goal_pose', 10)
        self.publisher_ = self.create_publisher(PoseStamped, '/Virtual/goal_pose', 10)
        self.status_pub_daeng_gui = self.create_publisher(String, '/status/ai_daeng', 10)
        self.status_pub_khieow_gui = self.create_publisher(String, '/status/ai_khieow', 10)
         # publishers สำหรับ effort
        self.pub_daeng_effort = self.create_publisher(
            Float64MultiArray, '/Ai_Daeng/effort_controller/commands', 10)
        self.pub_khieow_effort = self.create_publisher(
            Float64MultiArray, '/Ai_Khieow/effort_controller/commands', 10)

        # Subscriptions
        self.create_subscription(String, 'pallet_command', self.pallet_command_cb, 10)
        self.create_subscription(String, '/mission_status/ai_daeng', self.status_daeng_cb, 10)
        self.create_subscription(String, '/mission_status/ai_khieow', self.status_khieow_cb, 10)
        self.create_subscription(String, '/mission_status/virtual', self.status_virtual_cb, 10)

        # Startup message
        self.get_logger().info('🚀 Mission Planner Node started')
        self.status_pub_daeng_gui.publish(String(data='idle'))
        self.status_pub_khieow_gui.publish(String(data='idle'))

    def apply_offset(self, x, y, theta, dx, dy):
        # ปรับมุม: เพราะ local x หุ่นหันไปทาง +y ของโลก
        adjusted_theta = theta - math.pi / 2
        x_offset = math.cos(adjusted_theta) * dx - math.sin(adjusted_theta) * dy
        y_offset = math.sin(adjusted_theta) * dx + math.cos(adjusted_theta) * dy
        return x + x_offset, y + y_offset

    def pallet_command_cb(self, msg: String):
        self.get_logger().info(f"📦 Received pallet_command: {msg.data}")
        tag, pose = msg.data.split(':')

        x_str, y_str, theta_str = pose.split(',')
        x = float(x_str)
        y = float(y_str)
        theta = float(theta_str)
        
        if tag == 'lift':
            self.get_logger().info(f"📦 Lifting at x: {x}, y: {y}, theta: {theta}")
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
            
        elif tag == 'drop':
            self.get_logger().info(f"📦 Dropping at x: {x}, y: {y}, theta: {theta}")
            
            # # กำหนด Home Pose สำหรับแต่ละตัว
            # home_pose_daeng = (0.0, 0.0, 1.57)         # (x, y, yaw)
            # home_pose_khieow = (1.0, 0.0, 1.57)  # (x, y, yaw)

            # # คำนวณ quaternion สำหรับ Daeng
            # qz_d = math.sin(home_pose_daeng[2] / 2.0)
            # qw_d = math.cos(home_pose_daeng[2] / 2.0)

            # self.backoff_goal_daeng = PoseStamped()
            # self.backoff_goal_daeng.header.frame_id = 'map'
            # self.backoff_goal_daeng.header.stamp = self.get_clock().now().to_msg()
            # self.backoff_goal_daeng.pose.position.x = home_pose_daeng[0]
            # self.backoff_goal_daeng.pose.position.y = home_pose_daeng[1]
            # self.backoff_goal_daeng.pose.orientation.x = 0.0
            # self.backoff_goal_daeng.pose.orientation.y = 0.0
            # self.backoff_goal_daeng.pose.orientation.z = qz_d
            # self.backoff_goal_daeng.pose.orientation.w = qw_d

            # # คำนวณ quaternion สำหรับ Khieow
            # qz_k = math.sin(home_pose_khieow[2] / 2.0)
            # qw_k = math.cos(home_pose_khieow[2] / 2.0)

            # self.backoff_goal_khieow = PoseStamped()
            # self.backoff_goal_khieow.header.frame_id = 'map'
            # self.backoff_goal_khieow.header.stamp = self.get_clock().now().to_msg()
            # self.backoff_goal_khieow.pose.position.x = home_pose_khieow[0]
            # self.backoff_goal_khieow.pose.position.y = home_pose_khieow[1]
            # self.backoff_goal_khieow.pose.orientation.x = 0.0
            # self.backoff_goal_khieow.pose.orientation.y = 0.0
            # self.backoff_goal_khieow.pose.orientation.z = qz_k
            # self.backoff_goal_khieow.pose.orientation.w = qw_k
            
            #######################################################################################
            
            x_align_d, y_align_d = self.apply_offset(x, y, theta, -1.0, -3.0)

            self.backoff_goal_daeng = PoseStamped()
            self.backoff_goal_daeng.header.frame_id = 'map'
            self.backoff_goal_daeng.header.stamp = self.get_clock().now().to_msg()
            self.backoff_goal_daeng.pose.position.x = x_align_d
            self.backoff_goal_daeng.pose.position.y = y_align_d
            self.backoff_goal_daeng.pose.orientation.x = 0.0
            self.backoff_goal_daeng.pose.orientation.y = 0.0
            self.backoff_goal_daeng.pose.orientation.z = math.sin(theta / 2.0)
            self.backoff_goal_daeng.pose.orientation.w = math.cos(theta / 2.0)

            x_align_k, y_align_k = self.apply_offset(x, y, theta, 1.0, -3.0)

            self.backoff_goal_khieow = PoseStamped()
            self.backoff_goal_khieow.header.frame_id = 'map'
            self.backoff_goal_khieow.header.stamp = self.get_clock().now().to_msg()
            self.backoff_goal_khieow.pose.position.x = x_align_k
            self.backoff_goal_khieow.pose.position.y = y_align_k
            self.backoff_goal_khieow.pose.orientation.x = 0.0
            self.backoff_goal_khieow.pose.orientation.y = 0.0
            self.backoff_goal_khieow.pose.orientation.z = math.sin(theta / 2.0)
            self.backoff_goal_khieow.pose.orientation.w = math.cos(theta / 2.0)


            virtual_goal = PoseStamped()
            virtual_goal.header.stamp = self.get_clock().now().to_msg()
            virtual_goal.header.frame_id = 'map'
            virtual_goal.pose.position.x = x
            virtual_goal.pose.position.y = y
            virtual_goal.pose.orientation.z = math.sin(theta / 2.0)
            virtual_goal.pose.orientation.w = math.cos(theta / 2.0)

            self.publisher_.publish(virtual_goal)
            self.status_daeng = 'Transporting'
            self.status_khieow = 'Transporting'
            self.status_pub_khieow_gui.publish(String(data=self.status_khieow))
            self.status_pub_daeng_gui.publish(String(data=self.status_daeng))
            self.get_logger().info(f"📨 Sent virtual goal: x={x:.2f}, y={y:.2f}, theta={theta:.1f}°")

            
            

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
                
                self.pub_goal_khieow.publish(self.align_goal_khieow)
                self.status_khieow = 'aligning'
                self.status_pub_khieow_gui.publish(String(data=self.status_khieow))
                self.get_logger().info("🟩 Sent align goal to Ai_Khieow")
                
            elif self.status_daeng == 'backing off':
                self.status_daeng = 'idle'
                self.status_pub_daeng_gui.publish(String(data=self.status_daeng))
                self.get_logger().info("🟥 Backing off done → 🟥 Ai_Daeng is idle")
                
                self.pub_goal_khieow.publish(self.backoff_goal_khieow)
                self.get_logger().info("🟩 Sent backoff goal to Ai_Khieow")
                

    def status_khieow_cb(self, msg: String):
        self.get_logger().info(f"📨 /mission_status/ai_khieow: {msg.data}")
        if msg.data == 'success':
            if self.status_khieow == 'aligning':
                self.status_khieow = 'docking'
                self.status_pub_khieow_gui.publish(String(data=self.status_khieow))
                self.pub_goal_khieow.publish(self.dock_goal_khieow)
                self.get_logger().info("🟩 Alignment done → Sending docking goal to Ai_Khieow")
            elif self.status_khieow == 'docking':
                self.status_khieow = 'Lifting'
                self.status_daeng = 'Lifting'
                self.status_pub_khieow_gui.publish(String(data=self.status_khieow))
                self.status_pub_daeng_gui.publish(String(data=self.status_daeng))
                self.get_logger().info("🟩 Docking done → 🟥🟩 Twinfork is Lifting")
                # ส่ง effort พร้อมกันทั้งสองตัว
                effort_value = 800.0

                msg_d = Float64MultiArray()
                msg_d.data = [effort_value]
                self.pub_daeng_effort.publish(msg_d)

                msg_k = Float64MultiArray()
                msg_k.data = [effort_value]
                self.pub_khieow_effort.publish(msg_k)

                self.get_logger().info(f'Effort → Both: [{effort_value}]')
                
            elif self.status_khieow == 'backing off':
                self.status_khieow = 'idle'
                self.status_pub_khieow_gui.publish(String(data=self.status_khieow))
                self.get_logger().info("🟩 Backing off done → 🟩 Ai_Khieow is idle")
    
    def status_virtual_cb(self, msg: String):
        self.get_logger().info(f"📨 /mission_status/virtual: {msg.data}")
        if msg.data == 'success':
            if self.status_khieow == 'Transporting':
                self.status_khieow = 'backing off'
                self.status_daeng = 'backing off'
                self.status_pub_khieow_gui.publish(String(data=self.status_khieow))
                self.status_pub_daeng_gui.publish(String(data=self.status_daeng))
                self.get_logger().info("🟩 Transporting done → 🟥🟩 Twinfork is idle")

                msg_d = Float64MultiArray()
                msg_d.data = [0.0]
                self.pub_daeng_effort.publish(msg_d)

                msg_k = Float64MultiArray()
                msg_k.data = [0.0]
                self.pub_khieow_effort.publish(msg_k)

                self.get_logger().info(f'Effort → Both: [0.0]')
                
                self.backoff_timer = self.create_timer(3.0, self.send_backoff_goals_once)
                
    def send_backoff_goals_once(self):
        self.pub_goal_daeng.publish(self.backoff_goal_daeng)
        self.get_logger().info("🟥 Sent backoff goal to Ai_Daeng")

        # ยกเลิก timer ทันที
        self.backoff_timer.cancel()


                
            
            


                


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
