import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tkinter as tk
from tkinter import messagebox
import math

class VirtualGoalPublisher(Node):
    def __init__(self):
        super().__init__('virtual_goal_publisher')

        self.publisher_ = self.create_publisher(PoseStamped, '/Virtual/goal_pose', 10)
        self.frame_id = 'map'  # default frame_id

        # Start GUI
        self.root = tk.Tk()
        self.root.title("Virtual Goal Pose Sender")

        tk.Label(self.root, text="x:").grid(row=0, column=0)
        tk.Label(self.root, text="y:").grid(row=1, column=0)
        tk.Label(self.root, text="theta (deg):").grid(row=2, column=0)

        self.x_entry = tk.Entry(self.root)
        self.y_entry = tk.Entry(self.root)
        self.theta_entry = tk.Entry(self.root)

        self.x_entry.grid(row=0, column=1)
        self.y_entry.grid(row=1, column=1)
        self.theta_entry.grid(row=2, column=1)

        send_btn = tk.Button(self.root, text="Send Goal", command=self.send_goal)
        send_btn.grid(row=3, column=0, columnspan=2, pady=10)

    def send_goal(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            theta_deg = float(self.theta_entry.get())
            theta_rad = math.radians(theta_deg)

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.orientation.z = math.sin(theta_rad / 2.0)
            msg.pose.orientation.w = math.cos(theta_rad / 2.0)

            self.publisher_.publish(msg)
            self.get_logger().info(f"Sent goal: x={x}, y={y}, theta={theta_deg}°")
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numbers for x, y, and theta.")

    def run_gui(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = VirtualGoalPublisher()

    try:
        node.run_gui()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
