"""Static point-to-point velocity controller.\n
Benefits from direct movement, fast decisions, and bi-directional movements."""

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

import math

P2P_VEL_SUB_TOPIC = 'p2p_pose'
P2P_PUB_TOPIC = 'cmd_vel'

MIN_SPD = 0.3 # Minimum speed to stay productive
VEL_SCALE = 4 # Scale the velocity to be more practical
NORM_WHEEL_SPD = 1 # Maximum speed of a single wheel
WHEEL_OVERSPEED_MAX = 0.3 # Wheel overspeed headroom
ANGULAR_OVERSPEED_THRESHOLD = 0.5 # Angular speed threshold for overspeed
# TODO turn boost scaling (or not)

class StaticP2PVel(Node):
    
    def __init__(self):
        super().__init__('static_p2p_vel')
        self.subscription = self.create_subscription(
            Pose2D,
            P2P_VEL_SUB_TOPIC,
            self.vector_callback,
            10)
        self.publisher = self.create_publisher(Twist, P2P_PUB_TOPIC, 10)

    def vector_callback(self, msg: Pose2D):
        # self.get_logger().info('Received vector: %f %f' % (msg.x, msg.y))
        # x -> forward, y -> left

        if msg.x == msg.y == 0.0:
            # At destination - stop robot
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            # self.get_logger().info('No movement')
            return

        # Find the slope off the standard vector field f(x,y) = y/x -> f(msg.y, msg.x) = msg.x/msg.y
        desired_slope = float('inf') # Straight line by default
        if msg.y != 0:
            desired_slope = msg.x/msg.y # only calculate if y != 0

        distance = math.sqrt(msg.x**2 + msg.y**2)
        
        twist = Twist()

        # Note LOWER slope magnitude = HIGHER twist magnitude
        # Quick note: the soft limit is across +/- infinities, not 0, so the "hard" limit of 0 doesn't apply
        if math.copysign(1, desired_slope) == 1:
            # Rotate counter-clockwise for +0.0 <= slope <= +inf
            w = -1/(1 + desired_slope) # 0 <= w <= -1
        else:
            # Rotate clockwise for -inf <= slope <= -0.0
            w = 1/(1 - desired_slope) # 0 <= w <= 1

        # Find the speed
        v = min(1, MIN_SPD + distance/VEL_SCALE) # 0 <= v <= 1

        # Find out if we need to go backwards. Slope was already adjusted.
        if msg.x < 0:
            v = -v

        # float catches integer cases
        twist.linear.x = float(v)  # RUUUUUUUN
        twist.angular.z = float(w) # SPIIIIIIN

        # Speed up the bot as it gets straighter
        max_wheel_speed = NORM_WHEEL_SPD + WHEEL_OVERSPEED_MAX*max(0, 1 - abs(w)/ANGULAR_OVERSPEED_THRESHOLD)

        # Normalize the twist to avoid impractically high values
        twist.linear.x, twist.angular.z = normalize(twist.linear.x, twist.angular.z)
        twist.linear.x *= max_wheel_speed
        twist.angular.z *= max_wheel_speed

        #TODO why backwards
        twist.angular.z *= -1

        self.publisher.publish(twist)
        # self.get_logger().info('moving at %f m/s and %f rad/s' % (twist.linear.x, twist.angular.z))
        # self.get_logger().info('v_left: %f, v_right: %f' % (twist.linear.x + twist.angular.z, twist.linear.x - twist.angular.z))

def normalize(n1:float, n2:float) -> tuple[float, float]:
    """Generic normalize function for two numbers"""
    norm = abs(n1) + abs(n2)
    return n1/norm, n2/norm

def main(args=None):
    rclpy.init(args=args)
    static_p2p_vel = StaticP2PVel()
    rclpy.spin(static_p2p_vel)
    static_p2p_vel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
