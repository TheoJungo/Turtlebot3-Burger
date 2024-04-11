# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from math import atan2


# class TurtleBotPosition(Node):

#     def __init__(self):
#         super().__init__('turtlebot_position')
#         self.subscription = self.create_subscription(
#             Odometry, 'odom', self.odom_callback, 10)

#     def odom_callback(self, msg):
#         # Extract position
#         x = msg.pose.pose.position.x
#         y = msg.pose.pose.position.y

#         # Extract orientation
#         orientation_quat = msg.pose.pose.orientation
#         q0 = orientation_quat.w
#         q1 = orientation_quat.x
#         q2 = orientation_quat.y
#         q3 = orientation_quat.z

#         # Convert quaternion to yaw (theta)
#         theta = atan2(2.0 * (q0*q3 + q1*q2), 1.0 - 2.0 * (q2*q2 + q3*q3))

#         # Print position and orientation
#         self.get_logger().info(f'TurtleBot position: x={x}, y={y}, theta={theta}')


# def main(args=None):
#     rclpy.init(args=args)
#     turtlebot_position = TurtleBotPosition()
#     rclpy.spin(turtlebot_position)
#     turtlebot_position.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class PoseEstimationSubscriber(Node):

    def __init__(self):
        super().__init__('pose_estimation_subscriber')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_estimation_callback,
            10)

    def pose_estimation_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        x = position.x
        y = position.y
        z = position.z
        roll, pitch, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
        print(f"Current Pose Estimate (x, y, z, yaw): ({x}, {y}, {z}, {yaw})")

    def quaternion_to_euler(self, x, y, z, w):
        import math
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    pose_estimation_subscriber = PoseEstimationSubscriber()
    rclpy.spin(pose_estimation_subscriber)
    pose_estimation_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
