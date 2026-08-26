"""Move controller multiplexer for decision between p2p controller and PID controller.\n
True -> P2P controller.\n
False -> PID controller."""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

CONTROL_SUB_TOPIC_PARAM = 'velocity_input_topic'
P2P_TOPIC_PARAM = 'p2p_control_topic'
PID_TOPIC_PARAM = 'pid_control_topic'
MODE_TOPIC_PARAM = 'gui_mode_topic'

class MoveControllerMux(Node):
    def __init__(self):
        super().__init__('move_controller_mux')

        self.declare_parameter(CONTROL_SUB_TOPIC_PARAM, 'vel_pose')
        self.declare_parameter(P2P_TOPIC_PARAM, 'p2p_pose')
        self.declare_parameter(PID_TOPIC_PARAM, 'pid_pose')
        self.declare_parameter(MODE_TOPIC_PARAM, 'control_mux')

        self.control_sub_topic = self.get_parameter(CONTROL_SUB_TOPIC_PARAM).get_parameter_value().string_value
        self.p2p_topic = self.get_parameter(P2P_TOPIC_PARAM).get_parameter_value().string_value
        self.pid_topic = self.get_parameter(PID_TOPIC_PARAM).get_parameter_value().string_value
        self.gui_mode_change_topic = self.get_parameter(MODE_TOPIC_PARAM).get_parameter_value().string_value

        # Parent subscription for listening to the local point output
        self.vel_sub_ = self.create_subscription(
            Pose2D,
            self.control_sub_topic,
            self.vel_received_callback,
            10
        )

        # Subscription for listening to control mode change from GUI
        self.mode_sub_ = self.create_subscription(
            Bool,
            self.gui_mode_change_topic,
            self.mode_callback,
            10
        )

        self.p2p_pub_ = self.create_publisher(
            Pose2D,
            self.p2p_topic,
            10
        )

        self.pid_pub_ = self.create_publisher(
            Pose2D,
            self.pid_topic,
            10
        )

        # Currently identified publisher. Defaults to p2p.
        self.CURRENT_PUBLISHER = self.p2p_pub_

    def vel_received_callback(self, msg: Pose2D):
        self.CURRENT_PUBLISHER.publish(msg)

    def mode_callback(self, msg: Bool):
        if msg:
            self.CURRENT_PUBLISHER = self.p2p_pub_
        else:
            self.CURRENT_PUBLISHER = self.pid_pub_

def main(args=None):
    rclpy.init(args=args)
    control_mux = MoveControllerMux()
    rclpy.spin(control_mux)

    control_mux.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()