'''
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist



class FollowMe(Node):

    def __init__(self):
        super().__init__('follow_me')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)

    def listener_callback(self, msg):
        twist_msg = Twist()
        for i in range (344, 359):
          if 0.45 < msg.ranges[i] < 0.55:
            print("Personne repérée!")
            twist_msg.linear.x = 0.0
            twist_msg.linear.z = 0.0
          else:
             print("Rien en vue.")
             if msg.ranges[i] < 0.45:
                twist_msg.linear.x = -0.2
                twist_msg.linear.z = 0.2
             elif 0.55 < msg.ranges[i] < 1.0:
                twist_msg = Twist()
                twist_msg.linear.x = 0.2
                twist_msg.linear.z = 0.2
        self.publisher_.publish(twist_msg)

        for i in range (0, 14):
          if 0.45 < msg.ranges[i] < 0.55:
            print("Personne repérée!")
            twist_msg.linear.x = 0.0
            twist_msg.linear.z = 0.0
          else:
             print("Rien en vue.")
             if msg.ranges[i] < 0.45:
                twist_msg.linear.x = -0.2
                twist_msg.linear.z = 0.2
             elif 0.55 < msg.ranges[i] < 1.0:
                twist_msg.linear.x = 0.2
                twist_msg.linear.z = 0.2
        self.publisher_.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    follow_me = FollowMe()

    rclpy.spin(follow_me)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    follow_me.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
'''

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class FollowMe(Node):

    def __init__(self):
        super().__init__('follow_me')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)

    def listener_callback(self, msg):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.z = 0.2
        self.publisher_.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    follow_me = FollowMe()

    rclpy.spin(follow_me)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    follow_me.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()