import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from robot_interfaces.msg import Pose2DArray
#from brian_node import distance2d

EARTH_RADIUS_METERS = 6_378_137
#Right now assuming if front of heimdall is north, pixhawk says 0
COMPASS_OFFSET = 0
LOCAL_PATH_VEL_PUB_TOPIC = 'vel_pose'

class localPathPlanner(Node):

    def __init__(self):
        super().__init__('local_path_planner')

        self.localSubscription = self.create_subscription(
            Pose2D,
            "next_local",
            self.local_listener_callback,
            10
        )

        self.gnssSubscription = self.create_subscription(
            Pose2DArray,
            # Not sure if this is the right channel name
            'next_gnss',
            self.gnss_listener_callback,
            10
        )

        #use autonomy/p2p_Vel
        self.vel_publisher = self.create_publisher(
            Pose2D,
            LOCAL_PATH_VEL_PUB_TOPIC,
            10
        )


    def local_listener_callback(self, msg):
        # self.get_logger().info(f'X: {msg.x}  Y: {msg.y}  Theta: {msg.theta}')
        # self.get_logger().info('X: %f  Y: %f  Theta: %f' % (msg.x, msg.y, msg.theta))
        self.vel_publisher.publish(msg)

    def gnss_listener_callback(self, msg):
        # For now just publish when something comes in
        #Need to slice it up into smaller chunks at some point so obstacle avoidance can happen

        current  = msg.poses[0]
        goal = msg.poses[1]

        localDist = global_To_Local_Dist(current, goal)
        localAng = gps_To_Ang(current, goal)
        localX = localDist * math.cos(localAng)
        localY = localDist * math.sin(localAng)

        #I don't belive theta actually matters in sending data to the controller, so it's -1 for now
        localConversion = Pose2D()
        localConversion.x = localX
        localConversion.y = localY
        localConversion.theta = -1.0

        self.get_logger().info(f'Local Conversion X: {localConversion.x}  Y: {localConversion.y}')

        # self.vel_publisher.publish(localConversion)

        

def global_To_Local_Dist(current: Pose2D, goal: Pose2D):
    phi1 = math.radians(current.x)
    phi2 = math.radians(goal.x)
    delta_phi = math.radians(goal.x - current.x)
    delta_lambda = math.radians(goal.y - current.y)
    a = math.sin(delta_phi / 2.0) ** 2 + \
    math.cos(phi1) * math.cos(phi2) * \
    math.sin(delta_lambda / 2.0) ** 2

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS_METERS * c
    

def gps_To_Ang(current: Pose2D, goal: Pose2D):
    #Creating a gps cooridinate with the same y as current to obtain the distance purely in the x direction
    modified_y_goal = Pose2D()
    modified_y_goal.x = goal.x
    modified_y_goal.y = current.y

    delta_x = global_To_Local_Dist(current, modified_y_goal)

    #Delta x will always be positive initially. To get an accurate angle I need to determine whether the goal is north (delta x is positive) or south (negative) of the current location
    if current.x > goal.x:
         delta_x = -delta_x

    #Creating a gps with the same x as current to obtain the distance purely in the y direction
    modified_x_goal = Pose2D()
    modified_x_goal.y = goal.y
    modified_x_goal.x = current.x

    delta_y = global_To_Local_Dist(current, modified_x_goal)

    #Assuming positive x is north, then positive y would be west, however, longitude is the opposite of that, hence the current < goal for negative
    if current.y < goal.y: 
         delta_y = -delta_y

    theta_coords = math.atan2(delta_y, delta_x)

    #this is from the pixhawk and it's given in degrees
    theta_orientation = current.theta

    # The angle of the point in Heimdall's frame relative to the x axis and positive being towards positive y
    theta_local = theta_coords - math.radians(theta_orientation)

    return theta_local #in radians


def main(args=None):
    rclpy.init(args=args)
    local_path_planner = localPathPlanner()
    rclpy.spin(local_path_planner)

    local_path_planner.destroy_node()
    rclpy.shutdown()    

if ( __name__ == 'main'):
        main()


