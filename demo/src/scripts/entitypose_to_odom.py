#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry

class PoseArrayToOdom(Node):
    def __init__(self):
        super().__init__('posearray_to_odom')
        # Parámetros
        self.declare_parameter('pose_topic', '/model/demo/pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.pose_topic = self.get_parameter('pose_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.sub = self.create_subscription(PoseArray, self.pose_topic, self.cb, 10)
        self.pub = self.create_publisher(Odometry, self.odom_topic, 10)

        self.get_logger().info(
            f"Convirtiendo {self.pose_topic} -> {self.odom_topic} "
            f"({self.odom_frame}->{self.base_frame})"
        )

    def cb(self, msg: PoseArray):
        if not msg.poses:
            return
        # Heurística: tomamos la primera pose como la del cuerpo del robot
        p = msg.poses[0]

        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose = p
        # Velocidades a 0 (para tu gait da igual)
        self.pub.publish(od)

def main():
    rclpy.init()
    rclpy.spin(PoseArrayToOdom())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
