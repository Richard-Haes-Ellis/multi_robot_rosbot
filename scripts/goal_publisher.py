#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.node import Node
import time

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')
        
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = 10.0
        self.goal_pose.pose.position.y = -2.0
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = 0.0
        self.goal_pose.pose.orientation.w = 1.0

    def pub(self):
        self.publisher_.publish(self.goal_pose)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    
    print("Publishing goal pose")
    
    time.sleep(5)

    minimal_publisher.pub()

    # rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()