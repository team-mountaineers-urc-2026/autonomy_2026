import rclpy
import math
import time
from rclpy.node import Node
from autonomy_2026.autonomy_targets import AutonomyTargets as Targets
from autonomy_2026.decision_status import DecisionStatus as Status

from std_msgs.msg import String, Int32, Empty, Bool
from robot_interfaces.msg import Waypoint
from geometry_msgs.msg import Pose2D, Vector3
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

class WaypointMangaer(Node):

    def __init__(self):
        super().__init__('waypoint_manager')
        
        self.declare_parameter("publish_freq", 5.0)

        self.publish_period = 1.0 / self.get_parameter("publish_freq").get_parameter_value().double_value

        # Subscription to the GUI to get the waypoints
        self.waypoint_sub = self.create_subscription(Waypoint, "waypoint_create", self.create_waypoint_callback, 10)
        self.append_sub = self.create_subscription(Waypoint, "waypoint_append", self.add_waypoint_callback, 10)
        self.start_waypoint = self.create_subscription(Empty, "waypoint_pop", self.set_current_waypoint, 10)
        self.start_waypoint = self.create_subscription(Empty, "waypoint_clear", self.clear_waypoints_callback, 10)
        self.led_pub    = self.create_publisher(Bool, 'goal_alert', 10)
        
        # Status Update from Brian
        self.status_sub = self.create_subscription(Int32, "decision_status", self.status_callback, 10)

        # Publisher to BRIAN 
        self.next_pub = self.create_publisher(Pose2D, "next_waypoint", 10)
            # Output to the brian
            # x is latitude
            # y is longitude
            # theta is what we are looking for (see AutonomyTargets Enum Values)
        

        # Waypoint timer for repeatedly publishing waypoint
        self.publish_timer = self.create_timer(self.publish_period, self.timer_callback)


        #List of past, present, and future waypoints
        self.past_waypoints = []
        self.current_waypoint = None
        self.future_waypoints = []

        ## Examples of old way:
        # self.current_waypoint = Waypoint(latitude=39.64610, longitude=-79.97038, point_type=Targets.GPS.value)
        # self.future_waypoints.append(Waypoint(latitude=39.64602, longitude=-79.97008, point_type=Targets.GPS.value))

        # Stop Message, Don't Move 
        self.stop_msg = Pose2D(theta=float(Targets.STOP.value))
        self.empty_msg = Pose2D(theta=float(Targets.EMPTY.value))
        self.stop_point = Waypoint(point_type=Targets.STOP.value)

        # TIMER FOR LED FIX
        self.led_timer = self.create_timer(
            12.0,
            self.switch_pt2
        )

        self.led_timer.cancel()

    def status_callback(self, msg):
        
        match msg.data:

            case Status.NEXT_POINT.value:
                self.get_logger().info("Moving to Next Waypoint")
                self.past_waypoints.append(self.current_waypoint)
                if len(self.future_waypoints) > 0:
                    self.current_waypoint = self.future_waypoints.pop()
                else:
                    self.get_logger().info("Future list was empty :(")
                    self.current_waypoint = None

            case Status.BACKTRACK.value:
                self.get_logger().info("Backtracking Not Implemented")
                # self.past_waypoints.append(self.current_waypoint)
                self.next_pub.publish(self.stop_msg)
            
            case Status.ARRIVED.value:
                # self.get_logger().info("Arrived")

                # If we actually have a waypoint to append, append it
                if self.current_waypoint != None:
                    self.past_waypoints.append(self.current_waypoint)
                self.current_waypoint = None
                self.next_pub.publish(self.stop_msg)


    def timer_callback(self):
        # self.get_logger().info(f'CURRENT WAYPOINT : {self.current_waypoint}')
        if self.current_waypoint:
            x = self.current_waypoint.latitude
            y = self.current_waypoint.longitude
            theta = float(self.current_waypoint.point_type)
            self.next_pub.publish(Pose2D(x=x, y=y, theta=theta))
        else:
            self.get_logger().info('No Waypoint to Currently Travel To')
            self.next_pub.publish(self.empty_msg)


    def create_waypoint_callback(self, msg):
        # ROS Message to send to set waypoint: 
        # ros2 topic pub /autonomy/waypoint_create robot_interfaces/msg/Waypoint '{latitude: 39.64610, longitude: -79.97038, point_type: 2}' --once
        # Point Type:  STOP = 0 ||  GPS = 1 ||  WAYPOINT = 2 || ARUCO_0 = 3 || ARUCO_1 = 4 || ARUCO_2 = 5 ||  ARUCO_3 = 6 || BOTTLE = 7 || HAMMER = 8
        
        self.current_waypoint = Waypoint(latitude= msg.latitude, longitude= msg.longitude, point_type = msg.point_type)
        self.get_logger().info(f'====================CURRENT WAYPOINT : {self.current_waypoint}')
    
    def set_current_waypoint(self,msg):
        
        # not at goal publisher
        self.current_waypoint = self.stop_point
        self.get_logger().info("First half fired")
        self.led_pub.publish(Bool(data=False))
        self.led_pub.publish(Bool(data=False))
        self.led_pub.publish(Bool(data=False))
        self.led_pub.publish(Bool(data=False))
        self.led_pub.publish(Bool(data=False))
        self.led_timer.reset()

    def switch_pt2(self):
        if len(self.future_waypoints) != 0:
            self.current_waypoint = self.future_waypoints.pop()
            self.get_logger().info(f'=================CURRENT WAYPOINT SET: {self.current_waypoint}')
        else:
            self.get_logger().info(f'LIST WAS EMPTY')

        self.get_logger().info("Second half fired")

        self.led_timer.cancel()

    def add_waypoint_callback(self, msg):
        # ROS Message to send to set waypoint: 
        # ros2 topic pub /autonomy/waypoint_append robot_interfaces/msg/Waypoint '{latitude: 39.64610, longitude: -79.97038, point_type: 2}' --once
        # Point Type:  STOP = 0 ||  GPS = 1 ||  WAYPOINT = 2 || ARUCO_0 = 3 || ARUCO_1 = 4 || ARUCO_2 = 5 ||  ARUCO_3 = 6 || BOTTLE = 7 || HAMMER = 8
        
        self.future_waypoints.insert(0 , Waypoint(latitude= msg.latitude, longitude= msg.longitude, point_type = msg.point_type))
        self.get_logger().info(f'====================ADDED WAYPOINT : {self.future_waypoints}')

    def clear_waypoints_callback(self,msg):
        self.future_waypoints = []
        self.current_waypoint = None
        self.get_logger().info(f'=====================CLEARED WAYPOINTS ')



def main(args=None):
    rclpy.init(args=args)

    waypoint_manager = WaypointMangaer()

    rclpy.spin(waypoint_manager)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoint_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
