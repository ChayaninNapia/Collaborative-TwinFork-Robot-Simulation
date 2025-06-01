#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tkinter as tk
from tkinter import ttk

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher_gui')
        # เตรียม publishers สองตัว
        self.pub_daeng = self.create_publisher(PoseStamped, '/Ai_Daeng/goal_pose', 10)
        self.pub_khieow = self.create_publisher(PoseStamped, '/Ai_Khieow/goal_pose', 10)

    def publish(self, ns, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        # แปลง yaw → quaternion (z,w)
        qz = math.sin(yaw/2.0)
        qw = math.cos(yaw/2.0)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        if ns == 'Ai_Daeng':
            self.pub_daeng.publish(msg)
            self.get_logger().info(f'Published to /Ai_Daeng/goal_pose: ({x}, {y}, {yaw})')
        else:
            self.pub_khieow.publish(msg)
            self.get_logger().info(f'Published to /Ai_Khieow/goal_pose: ({x}, {y}, {yaw})')

def start_gui(node: GoalPublisher):
    root = tk.Tk()
    root.title('Twinfork Goal Publisher')
    frm = ttk.Frame(root, padding=10)
    frm.grid()

    # Entry fields
    ttk.Label(frm, text='X:').grid(column=0, row=0, sticky='e')
    ent_x = ttk.Entry(frm, width=10); ent_x.grid(column=1, row=0)
    ttk.Label(frm, text='Y:').grid(column=0, row=1, sticky='e')
    ent_y = ttk.Entry(frm, width=10); ent_y.grid(column=1, row=1)
    ttk.Label(frm, text='Yaw (rad):').grid(column=0, row=2, sticky='e')
    ent_yaw = ttk.Entry(frm, width=10); ent_yaw.grid(column=1, row=2)

    # Callback function
    def on_publish(ns):
        try:
            x = float(ent_x.get())
            y = float(ent_y.get())
            yaw = float(ent_yaw.get())
        except ValueError:
            tk.messagebox.showerror('Error', 'กรุณาใส่ตัวเลขให้ถูกต้อง')
            return
        node.publish(ns, x, y, yaw)

    # Buttons
    btn_daeng = ttk.Button(frm, text='Publish to Ai_Daeng', 
                           command=lambda: on_publish('Ai_Daeng'))
    btn_daeng.grid(column=0, row=3, columnspan=2, pady=(10, 0))
    btn_khieow = ttk.Button(frm, text='Publish to Ai_Khieow', 
                            command=lambda: on_publish('Ai_Khieow'))
    btn_khieow.grid(column=0, row=4, columnspan=2, pady=(5, 0))

    root.mainloop()

def main():
    rclpy.init()
    node = GoalPublisher()
    # รัน GUI ใน thread หลัก และ spin ROS ใน background
    import threading
    thr = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    thr.start()
    start_gui(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
