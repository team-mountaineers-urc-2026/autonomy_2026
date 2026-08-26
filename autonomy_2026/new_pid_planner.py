import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from time import time
from geometry_msgs.msg import Pose2D, Twist

from math import atan2, pi, sqrt, asin, radians


################## HELPER FUNCTIONS ###################
def clamp(value: float, lower: float, upper: float) -> float:
	return float(min(upper, max(value, lower)))


def npi_to_pi_angle(angle):
	if angle > pi:
		return angle - 2*pi
	elif angle < -pi:
		return angle + 2*pi
	return angle


#####################  PID_CONTROLLER_LIB  ##########################
class PID_Controller():

	def __init__(self, prop_gain, kp, kd, ki):
		self.prop_gain = prop_gain
		self.kp = kp
		self.kd = kd
		self.ki = ki

		self.last_time = time()
		self.previous_error = 0
		self.previous_error_sum = 0

		self.val = 0

	def update(self, current_error: float) -> float:
		# proportional contribution
		result_p = self.kp * current_error

		# derivative contribution
		curr_time = time()
		result_d = self.kd * ((current_error - self.previous_error) / (curr_time - self.last_time))

		# integral contribution
		self.previous_error_sum += self.previous_error
		result_i = self.ki * self.previous_error_sum

		# update error history
		self.previous_error = current_error
		self.last_time = curr_time

		result = result_p + result_d + result_i
		self.val = result * self.prop_gain
		return self.val

	def reset(self):
		self.last_time = time()
		self.previous_error = 0
		self.previous_error_sum = 0
            

class PID_Planner(Node):

	def __init__(self):
		super().__init__('new_pid_planner')
		
        ########### DECLARE PARAMETERS #####################
		self.declare_parameter("dist_prop_gain", 0.1)
		self.declare_parameter("dist_kp", 0.04)
		self.declare_parameter("dist_kd", 0.0)
		self.declare_parameter("dist_ki", 0.0)

		self.declare_parameter("theta_prop_gain", 0.1)
		self.declare_parameter("head_kp", 0.1)
		self.declare_parameter("head_kd", 0.0)
		self.declare_parameter("head_ki", 0.0)

		self.declare_parameter("min_linear_speed", 0.0)
		self.declare_parameter("max_linear_speed", 1.0)
		self.declare_parameter("min_angular_speed", -1.0)
		self.declare_parameter("max_angular_speed", 1.0)

		self.declare_parameter("angle_threshold", 10.0)
		
		self.declare_parameter("target_pose_topic", "pid_pose")
		self.declare_parameter("cmd_vel_topic", "cmd_vel")


        ################## Setting up PID controllers for position and heading   #################################
		dist_prop_gain = self.get_parameter("dist_prop_gain").get_parameter_value().double_value
		dist_kp = self.get_parameter("dist_kp").get_parameter_value().double_value
		dist_kd = self.get_parameter("dist_kd").get_parameter_value().double_value
		dist_ki = self.get_parameter("dist_ki").get_parameter_value().double_value
		self.dist_pid_controller = PID_Controller(dist_prop_gain, dist_kp, dist_kd, dist_ki)

		theta_prop_gain = self.get_parameter("theta_prop_gain").get_parameter_value().double_value
		head_kp = self.get_parameter("head_kp").get_parameter_value().double_value
		head_kd = self.get_parameter("head_kd").get_parameter_value().double_value
		self.get_logger().info(f"KP: {head_kp}")
		self.get_logger().info(f"KD: {head_kd}")
		head_ki = self.get_parameter("head_ki").get_parameter_value().double_value
		self.head_pid_controller = PID_Controller(theta_prop_gain, head_kp, head_kd, head_ki)


		############ LOCAL VARIABLES ###########################
		self.min_linear_speed = self.get_parameter("min_linear_speed").get_parameter_value().double_value
		self.max_linear_speed = self.get_parameter("max_linear_speed").get_parameter_value().double_value
		self.min_angular_speed = self.get_parameter("min_angular_speed").get_parameter_value().double_value
		self.max_angular_speed = self.get_parameter("max_angular_speed").get_parameter_value().double_value

		self.angle_threshold = radians(self.get_parameter("angle_threshold").get_parameter_value().double_value)
		self.get_logger().info(f"ANG_THRESH: {self.angle_threshold}")


		############### ROS2 CONNECTIONS ####################
		target_pose_topic = self.get_parameter("target_pose_topic").get_parameter_value().string_value
		cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value

		qos = QoSProfile(
			depth=10,
			reliability=QoSReliabilityPolicy.BEST_EFFORT
		)

		self.target_pose_sub = self.create_subscription(Pose2D, target_pose_topic, self.target_pose_callback, 10)
		self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10) 

 
	############### CALLBACK FUNCTION #####################
	def target_pose_callback(self, msg):
		#Get the Distance and Heading
		distance_error = sqrt(msg.x**2 + msg.y**2)
		heading_error = npi_to_pi_angle(atan2(msg.y, msg.x))

		#Update PID Controller
		linear = self.dist_pid_controller.update(distance_error)
		angular = self.head_pid_controller.update(heading_error)

		#Check the heading error
		if abs(heading_error) > self.angle_threshold:
			self.get_logger().info(f"Turning: {heading_error}")
			linear = 0.0
		else:
			self.get_logger().info(f"Heading Good: {heading_error}")

		# scale cmd_vel and publish
		cmd_vel = Twist()
		cmd_vel.linear.x = clamp(linear, self.min_linear_speed, self.max_linear_speed)
		cmd_vel.angular.z = clamp(angular, self.min_angular_speed, self.max_angular_speed)
		self.cmd_vel_pub.publish(cmd_vel)



def main(args=None):
    rclpy.init(args=args)
    pid_planner = PID_Planner()
    rclpy.spin(pid_planner)

    pid_planner.destroy_node()
    rclpy.shutdown()    

if ( __name__ == 'main'):
        main()


