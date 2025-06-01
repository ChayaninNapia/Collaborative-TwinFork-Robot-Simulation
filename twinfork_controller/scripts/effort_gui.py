#!/usr/bin/env python3
import math
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk, messagebox

class TwinPublisher(Node):
    def __init__(self):
        super().__init__('twin_publisher_gui')
        # publishers สำหรับ goal
        self.pub_daeng_goal = self.create_publisher(
            PoseStamped, '/Ai_Daeng/goal_pose', 10)
        self.pub_khieow_goal = self.create_publisher(
            PoseStamped, '/Ai_Khieow/goal_pose', 10)
        # publishers สำหรับ effort
        self.pub_daeng_effort = self.create_publisher(
            Float64MultiArray, '/Ai_Daeng/effort_controller/commands', 10)
        self.pub_khieow_effort = self.create_publisher(
            Float64MultiArray, '/Ai_Khieow/effort_controller/commands', 10)

    def publish_goal(self, ns, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        if ns == 'Ai_Daeng':
            self.pub_daeng_goal.publish(msg)
        else:
            self.pub_khieow_goal.publish(msg)
        self.get_logger().info(f'Goal → {ns}: ({x}, {y}, {yaw})')

    def publish_effort(self, ns, efforts):
        msg = Float64MultiArray()
        msg.data = [float(e) for e in efforts]
        if ns == 'Ai_Daeng':
            self.pub_daeng_effort.publish(msg)
        else:
            self.pub_khieow_effort.publish(msg)
        self.get_logger().info(f'Effort → {ns}: {msg.data}')

    def publish_effort_both(self, efforts):
        # ส่ง effort พร้อมกันทั้งสองตัว
        msg = Float64MultiArray()
        msg.data = [float(e) for e in efforts]
        self.pub_daeng_effort.publish(msg)
        self.pub_khieow_effort.publish(msg)
        self.get_logger().info(f'Effort → Both: {msg.data}')

def start_gui(node: TwinPublisher):
    root = tk.Tk()
    root.title('Twinfork Goal & Effort Publisher')

    # สร้างสองเฟรม ซ้าย: Goal, ขวา: Effort
    frm_goal = ttk.Labelframe(root, text='Publish Goal', padding=8)
    frm_goal.grid(column=0, row=0, padx=10, pady=10, sticky='nsew')
    frm_eff = ttk.Labelframe(root, text='Publish Effort', padding=8)
    frm_eff.grid(column=1, row=0, padx=10, pady=10, sticky='nsew')

    # ---- Goal frame ----
    ttk.Label(frm_goal, text='X:').grid(column=0, row=0, sticky='e')
    ent_x = ttk.Entry(frm_goal, width=8); ent_x.grid(column=1, row=0)
    ttk.Label(frm_goal, text='Y:').grid(column=0, row=1, sticky='e')
    ent_y = ttk.Entry(frm_goal, width=8); ent_y.grid(column=1, row=1)
    ttk.Label(frm_goal, text='Yaw (rad):').grid(column=0, row=2, sticky='e')
    ent_yaw = ttk.Entry(frm_goal, width=8); ent_yaw.grid(column=1, row=2)

    def on_goal(ns):
        try:
            x = float(ent_x.get())
            y = float(ent_y.get())
            yaw = float(ent_yaw.get())
        except ValueError:
            messagebox.showerror('Error','กรุณาใส่ตัวเลข Goal ให้ถูกต้อง')
            return
        node.publish_goal(ns, x, y, yaw)

    ttk.Button(frm_goal, text='Send to Ai_Daeng',
               command=lambda: on_goal('Ai_Daeng'))\
        .grid(column=0, row=3, columnspan=2, pady=5, sticky='ew')
    ttk.Button(frm_goal, text='Send to Ai_Khieow',
               command=lambda: on_goal('Ai_Khieow'))\
        .grid(column=0, row=4, columnspan=2, pady=5, sticky='ew')

    # ---- Effort frame ----
    ttk.Label(frm_eff, text='Efforts (comma-separated):')\
        .grid(column=0, row=0, sticky='w')
    ent_eff = ttk.Entry(frm_eff, width=25); ent_eff.grid(column=0, row=1, pady=5)

    def parse_efforts():
        text = ent_eff.get()
        arr = [t.strip() for t in text.split(',') if t.strip()!='']
        if not arr:
            raise ValueError
        return arr

    def on_effort(ns):
        try:
            arr = parse_efforts()
        except ValueError:
            messagebox.showerror('Error',
                'กรุณาใส่ตัวเลข Effort ให้ถูกต้อง (คั่นด้วยคอมม่า)')
            return
        node.publish_effort(ns, arr)

    def on_effort_both():
        try:
            arr = parse_efforts()
        except ValueError:
            messagebox.showerror('Error',
                'กรุณาใส่ตัวเลข Effort ให้ถูกต้อง (คั่นด้วยคอมม่า)')
            return
        node.publish_effort_both(arr)

    ttk.Button(frm_eff, text='Send to Ai_Daeng',
               command=lambda: on_effort('Ai_Daeng'))\
        .grid(column=0, row=2, pady=5, sticky='ew')
    ttk.Button(frm_eff, text='Send to Ai_Khieow',
               command=lambda: on_effort('Ai_Khieow'))\
        .grid(column=0, row=3, pady=5, sticky='ew')
    ttk.Button(frm_eff, text='Send to Both',
               command=on_effort_both)\
        .grid(column=0, row=4, pady=5, sticky='ew')

    root.mainloop()

def main():
    rclpy.init()
    node = TwinPublisher()
    th = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    th.start()
    start_gui(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
