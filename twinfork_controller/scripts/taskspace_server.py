#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication
from taskspace_gui import TaskspaceGUI

class TaskspaceServer(Node):
    def __init__(self):
        super().__init__('taskspace_server')
        self.publisher_ = self.create_publisher(String, 'pallet_command', 10)
        self.create_subscription(
            String, '/status/ai_daeng', self.update_ai_daeng_status, 10
        )
        self.create_subscription(
            String, '/status/ai_khieow', self.update_ai_khieow_status, 10
        )

        self.get_logger().info('Taskspace Server Node Started')

    def send_command(self, tag, x, y, theta):
        theta_offset = 1.57079632679  # ปรับมุมหุ่นให้หันไปทาง +y ของโลก
        theta += theta_offset
        msg = String()
        msg.data = f"{tag}:{x:.2f},{y:.2f},{theta:.2f}"
        self.get_logger().info(f"📤 ส่งคำสั่งไปยัง topic: {msg.data}")
        self.publisher_.publish(msg)

    def update_ai_daeng_status(self, msg):
        log_msg = f"[Ai_Daeng] 🔴 Received status: {msg.data}"
        self.get_logger().info(log_msg)
        self.gui.status_bridge.update_status_signal.emit('ai_daeng', msg.data)

    def update_ai_khieow_status(self, msg):
        log_msg = f"[Ai_Khieow] 🟢 Received status: {msg.data}"
        self.get_logger().info(log_msg)
        self.gui.status_bridge.update_status_signal.emit('ai_khieow', msg.data)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    taskspace_node = TaskspaceServer()
    gui = TaskspaceGUI(pose_callback=taskspace_node.send_command)
    taskspace_node.gui = gui

    gui.show()

    # รัน PyQt5 loop + rclpy spin แบบ parallel
    from threading import Thread
    ros_spin_thread = Thread(target=rclpy.spin, args=(taskspace_node,), daemon=True)
    ros_spin_thread.start()

    app.exec_()
    taskspace_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
