"""Move controller multiplexer for decision between p2p controller and PID controller.\n
True -> P2P controller.\n
False -> PID controller."""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class VelocityMux(Node):
    def __init__(self):
        super().__init__('move_controller_mux')

        self.declare_parameter('autonomy_vel_topic', '/autonomy/cmd_vel')
        self.declare_parameter('manual_vel_topic', '/base_station/cmd_vel')
        self.declare_parameter('cmd_vel_topic', '/drivetrain/cmd_vel')
        self.declare_parameter('cmd_vel_indicator', 'indicator_cmd_vel')
        self.declare_parameter('velocity_indicator', True) #defaults to manual

        #Topic Parameters
        self.auto_cmd_vel_topic = self.get_parameter('autonomy_vel_topic').get_parameter_value().string_value
        self.manual_cmd_vel_topic = self.get_parameter('manual_vel_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.indicator_topic = self.get_parameter('cmd_vel_indicator').get_parameter_value().string_value

        # Indicator Parameter
        self.velocity_indicator = self.get_parameter('velocity_indicator').get_parameter_value().bool_value

        # Subscription for listening to the autonomy command velocity
        self.auto_vel_sub_ = self.create_subscription(
            Twist,
            self.auto_cmd_vel_topic,
            self.auto_vel_received_callback,
            10
        )

        # Subscription for listening to the manual command velocity
        self.manual_vel_sub_ = self.create_subscription(
            Twist,
            self.manual_cmd_vel_topic,
            self.manual_vel_received_callback,
            10
        )

        # Subscription for listening to control mode change from GUI
        self.mode_sub_ = self.create_subscription(
            Bool,
            self.indicator_topic,
            self.mode_callback,
            10
        )

        # Publisher for command velocity to rover 
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        # Autonomy Indicator
        self.led_auto_pub = self.create_publisher(
            Bool,
            '/autonomy/is_autonomous',
            10
        )


    def auto_vel_received_callback(self, msg: Twist):
        if( not self.velocity_indicator):
            self.cmd_vel_pub.publish(msg)

    def manual_vel_received_callback(self, msg: Twist):
        if( self.velocity_indicator ):
            self.cmd_vel_pub.publish(msg)
        

    def mode_callback(self, msg: Bool):
        #Switch the mode to the current boolean. True is to run maunal mode and false is to run autonomy mode

        # Teleop
        if msg.data:
            self.get_logger().info("Rover is \033[31mAUTONOMOUS\033[0m")
            self.led_auto_pub.publish(Bool(data = False))
            self.led_auto_pub.publish(Bool(data = False))
            self.led_auto_pub.publish(Bool(data = False))
            self.led_auto_pub.publish(Bool(data = False))
        else:
            self.get_logger().info("Rover is \033[32mTELEOPERATED\033[0m")
            self.led_auto_pub.publish(Bool(data = True))
            self.led_auto_pub.publish(Bool(data = True))
            self.led_auto_pub.publish(Bool(data = True))
            self.led_auto_pub.publish(Bool(data = True))

        self.velocity_indicator = msg.data
        



def main(args=None):
    rclpy.init(args=args)
    velocity_mux = VelocityMux()
    rclpy.spin(velocity_mux)

    velocity_mux.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()