import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from nav_msgs.msg._odometry import Odometry
from std_msgs.msg import String
import time
import math
import numpy as np




class Control(Node):

    def __init__(self):
        super().__init__('control')

        self.subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goalPoseUpdate,
            10)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Odometry,
            '/rosbot1/odom',
            self.posUpdate,
            10)
        self.subscription  # prevent unused variable warning

        self.rx = 0.0
        self.ry = 0.0
        self.rz = 0.0

        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.goalX=-1
        self.goalY=-1
        self.goalZ=-1


        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.controller)


    def euler_from_quaternion(self,x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def goalPoseUpdate(self, msg):
        self.goalX=msg.pose.position.x
        self.goalY=msg.pose.position.y
        self.goalZ=msg.pose.position.z

    def posUpdate(self,msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        self.rz = msg.pose.pose.position.z
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        self.roll,self.pitch,self.yaw = self.euler_from_quaternion(x,y,z,w)

    def controller(self):
        if(self.goalX == -1 and self.goalY ==-1 and self.goalZ == -1):
            pass
        else:

            print("NUEVO GOAL")


def main(args=None):
    rclpy.init(args=args)
    control = Control()
    rclpy.spin(control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
