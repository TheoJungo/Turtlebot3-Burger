import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from robot_navigator import BasicNavigator, NavigationResult
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time


class FollowMe(Node):

    def __init__(self):
        super().__init__('follow_me')
        self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_estimation_callback,
            10)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=30
        )
        self.create_subscription(LaserScan, 'scan', self.listener_callback, qos_profile)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.initial_position = None
        self.initial_orientation = None
        self.last_movement_time = None
        self.follow_enabled = False  # Flag to indicate if "Follow Me" behavior is enabled
        self.time_threshold = 3  # Time threshold for no movement in seconds
        self.position_threshold = 0.1  # Position threshold for no movement in meters
        self.angle_treshold = 2
        self.returning_to_initial_position = False  # Flag to indicate if robot is returning to initial position
        self.init_follow_me = 0
        self.taille_personne = 0

    def pose_estimation_callback(self, msg):
        if self.initial_position is None:
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            x = position.x
            y = position.y
            z = position.z
            roll, pitch, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
            print(f"Initial Pose Estimate (x, y, z, yaw): ({x}, {y}, {z}, {yaw})")

            # Set initial position and orientation
            self.initial_position = (x, y, z)
            self.initial_orientation = yaw

            # Start the "Follow Me" behavior
            self.start_following()

    def start_following(self):
        # Implement the "Follow Me" behavior here
        print("Follow Me behavior started!")
        self.follow_enabled = True

    def quaternion_to_euler(self, x, y, z, w):
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

    def listener_callback(self, msg):
        twist_msg = Twist()
        angle_width = 30
        dist_min = 0.25
        dist_max = 0.8
        dist_opti = 0.5
        nb_points = len(msg.ranges)
        nb_valid_points = 0
        somme_dist = 0
        somme_ang = []
        somme_ang_val = 0
        nb_somme_ang_final = 0
        vitesse_linear_max = 1.0
        vitesse_angular_max = 0.05

        # Check if the robot is currently returning to the initial position
        if self.returning_to_initial_position:
            # Stop all movements
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.publisher_.publish(twist_msg)
            return

        for i in range(nb_points):
            if 0 <= i < angle_width/2 or (nb_points - angle_width/2) < i:
                # Le point est dans le cone
                if dist_min < msg.ranges[i] < dist_max:
                    # Le point est aussi à la bonne distance
                    if self.init_follow_me == 0:
                        self.taille_personne += 1
                    if i > 180:
                        ang = i - 360
                    else:
                        ang = i
                    dist = msg.ranges[i]
                    nb_valid_points += 1
                    somme_dist += dist
                    somme_ang.append(ang)
                    nb_somme_ang = len(somme_ang)
                    milieu_somme_ang = nb_somme_ang // 2
                    valeurs_du_milieu = somme_ang[milieu_somme_ang - (self.taille_personne // 2) : milieu_somme_ang + (self.taille_personne // 2)]
                    nb_somme_ang_final = len(valeurs_du_milieu)
                    self.get_logger().info(f"taille_personne= {self.taille_personne}, valeurs_du_milieu= {len(valeurs_du_milieu)}")

        if self.taille_personne != 0:
          self.init_follow_me = 1

        for i in range(nb_somme_ang_final):
          somme_ang_val += valeurs_du_milieu[i]

        # Ceci est notre meilleure estimation d'où est la personne
        if nb_valid_points != 0:
            moy_dist = somme_dist / nb_valid_points
            moy_ang = somme_ang_val / nb_somme_ang_final
            self.get_logger().info(f"moy_dist = {moy_dist}, moy_ang = {moy_ang}")
        else:
            # IDEE? ajouter la fonction où il faut suivre la dernière postion valide avant que le point sorte du cone
            moy_dist = dist_opti
            moy_ang = 0
            self.get_logger().info("Pas de point détecté")

        # Vérifier si le robot est en mouvement
        if abs(moy_dist - dist_opti) > self.position_threshold or abs(moy_ang) > self.angle_treshold:            
            self.last_movement_time = time.time()
            self.returning_to_initial_position = False  # Reset flag when movement detected

        # Vérifier si le robot doit retourner à la position initiale
        if self.last_movement_time is not None:
            time_since_last_movement = time.time() - self.last_movement_time
            if time_since_last_movement > self.time_threshold and self.follow_enabled:
                self.return_to_initial_position()
                self.last_movement_time = None
                self.follow_enabled = False  # Disable "Follow Me" behavior permanently
                self.returning_to_initial_position = True  # Set flag to indicate returning to initial position

        # Ceci sont les vitesses linéaire et angulaire à appliquer au robot pour qu'il se déplace à notre meilleure estimation de la personne
        twist_msg.linear.x = (moy_dist - dist_opti) * vitesse_linear_max
        twist_msg.angular.z = moy_ang * vitesse_angular_max
        self.publisher_.publish(twist_msg)

    def return_to_initial_position(self):
        if self.initial_position is not None and self.initial_orientation is not None:
            # Convert yaw to quaternion
            yaw = self.initial_orientation
            qx, qy, qz, qw = self.euler_to_quaternion(0, 0, yaw)

            # Create a BasicNavigator instance
            navigator = BasicNavigator()

            # Wait until the navigation2 stack is active
            navigator.waitUntilNav2Active()

            # Define the goal pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z = self.initial_position
            goal_pose.pose.orientation.x = qx
            goal_pose.pose.orientation.y = qy
            goal_pose.pose.orientation.z = qz
            goal_pose.pose.orientation.w = qw

            # Navigate the robot to the initial position
            navigator.goToPose(goal_pose)

            # Keep track of navigation progress
            while not navigator.isNavComplete():
                feedback = navigator.getFeedback()
                if feedback:
                    # Handle navigation feedback
                    pass
                # Add logic for timeout or preemption if needed

            # Get the navigation result
            result = navigator.getResult()

            # Shutdown the ROS 2 navigation stack
            navigator.lifecycleShutdown()

            # Handle the navigation result
            if result == NavigationResult.SUCCEEDED:
                print('Returned to initial position successfully!')
            elif result == NavigationResult.CANCELED:
                print('Navigation to initial position was canceled!')
            elif result == NavigationResult.FAILED:
                print('Failed to return to initial position!')
            else:
                print('Invalid navigation result!')

            self.follow_enabled = False  # Disable "Follow Me" behavior while returning to initial position

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr

        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    follow_me = FollowMe()
    rclpy.spin(follow_me)
    follow_me.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
