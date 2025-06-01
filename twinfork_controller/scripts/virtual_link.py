#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_ros

class VirtualForkliftBroadcaster(Node):
    def __init__(self):
        super().__init__('virtual_forklift_broadcaster')

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Loop timer
        self.timer = self.create_timer(0.1, self.broadcast_virtual_forklift)

    def broadcast_virtual_forklift(self):
        try:
            # lookup Ai_Daeng and Ai_Khieow base_footprint in map frame
            t_daeng = self.tf_buffer.lookup_transform(
                'map', 'Ai_Daeng/base_footprint', rclpy.time.Time())
            t_khieow = self.tf_buffer.lookup_transform(
                'map', 'Ai_Khieow/base_footprint', rclpy.time.Time())

            # average position
            x_avg = (t_daeng.transform.translation.x + t_khieow.transform.translation.x) / 2.0
            y_avg = (t_daeng.transform.translation.y + t_khieow.transform.translation.y) / 2.0

            # average orientation (naive average of z,w)
            z_avg = (t_daeng.transform.rotation.z + t_khieow.transform.rotation.z) / 2.0
            w_avg = (t_daeng.transform.rotation.w + t_khieow.transform.rotation.w) / 2.0

            # broadcast
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'virtual_forklift'
            t.transform.translation.x = x_avg
            t.transform.translation.y = y_avg
            t.transform.translation.z = 0.0
            t.transform.rotation.z = z_avg
            t.transform.rotation.w = w_avg

            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

def main():
    rclpy.init()
    node = VirtualForkliftBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
