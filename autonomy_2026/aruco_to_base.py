
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from ros2_aruco_interfaces.msg import ArucoMarkers
from builtin_interfaces.msg import Time
from tf2_ros import TransformException, TransformStamped
from tf2_geometry_msgs import PoseStamped as tf2_PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import time
import numpy as np
from scipy.spatial.transform import Rotation as R

class ArucoLinkCalculator(Node):
    def __init__(self):
        super().__init__('aruco_link_calculator')

        # Parameters
        self.declare_parameter('base_link_name', 'base_link')
        self.declare_parameter('hex_aruco_ids', [49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39])
        self.declare_parameter('forget_time', 0.5)

        self.base_link = f"{self.get_namespace()}/{self.get_parameter('base_link_name').get_parameter_value().string_value}"[1:]
        self.hex_ids = self.get_parameter('hex_aruco_ids').get_parameter_value().integer_array_value
        self.forget_time = self.get_parameter('forget_time').get_parameter_value().double_value

        # Subscribers
        self.aruco_sub = self.create_subscription(ArucoMarkers, 'aruco_markers', self.aruco_callback, 10)

        # Publishers
        self.block_pub = self.create_publisher(ArucoMarkers, 'seen_hexes', 10)
        self.robot_pub = self.create_publisher(ArucoMarkers, 'seen_robots', 10)


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_times = {}

        self.back_cam_hexes     = []
        self.back_cam_ids       = []
        self.front_cam_hexes    = []
        self.front_cam_ids      = []

    def aruco_callback(self, msg: ArucoMarkers):

        camera_frame = msg.header.frame_id
        # camera_frame = camera
        # self.get_logger().info(f"{self.get_namespace()}")
        self.camera_times[camera_frame.split('/')[-1]] = time.time()

        if camera_frame.split('/')[-1] == 'front_camera':
            self.front_cam_hexes = []
            self.front_cam_ids = []

        elif camera_frame.split('/')[-1] == 'back_camera':
            self.back_cam_hexes = []
            self.back_cam_ids = []

        else:
            self.get_logger().fatal(f"{camera_frame.split('/')[-1]} unknown")


        # Tf setup
        zero = Time()
        zero.sec = 0
        zero.nanosec = 0

        # Get the base-->camera frame
        try:
            camera_tf = self.tf_buffer.lookup_transform(
                camera_frame,
                f"{self.base_link}",
                zero
            )
        except TransformException as ex:
            self.get_logger().fatal(
                f"Could not transform {camera_frame} to {self.base_link}: {ex}")
            return

        for i, marker_id in enumerate(msg.marker_ids):

            temp_pose = tf2_PoseStamped()
            temp_pose.pose = msg.poses[i]
            

            # Adjust the marker rotation for itself
            q = temp_pose.pose.orientation

            marker_quat = R.from_quat([q.x, q.y, q.z, q.w])
            apply_rot   = R.from_euler("zyx", [90.0, 0.0, -90.0], degrees=True)
            new_rot     = (marker_quat * apply_rot).as_quat()

            new_quat = Quaternion(x=new_rot[0], y=new_rot[1], z=new_rot[2], w=new_rot[3])

            temp_pose.pose.orientation = new_quat

            # Apply the camera frame transform
            temp_pose.header.frame_id = camera_frame
            temp_pose.header.stamp = camera_tf.header.stamp

            rot_aruco = self.tf_buffer.transform
            transformed_pose = self.tf_buffer.transform(temp_pose, self.base_link)

            new_pose = transformed_pose.pose

            # If block
            
            if marker_id in self.hex_ids:
                if camera_frame.split('/')[-1] == 'front_camera':
                    self.front_cam_hexes.append(new_pose)
                    self.front_cam_ids.append(marker_id)

                elif camera_frame.split('/')[-1] == 'back_camera':
                    self.back_cam_hexes.append(new_pose)
                    self.back_cam_ids.append(marker_id)


                else:
                    self.get_logger().fatal(f"{camera_frame.split('/')[-1]} unknown")


        # Forget if long ago
        if 'front_camera' in self.camera_times and self.camera_times['front_camera'] + self.forget_time < time.time():
            self.front_cam_hexes = []
            self.front_cam_ids = []
        
        if 'back_camera' in self.camera_times and self.camera_times['back_camera'] + self.forget_time < time.time():
            self.back_cam_hexes = []
            self.back_cam_ids = []
        
        # Publish
        
        hex_marker_arr = ArucoMarkers()
        hex_marker_arr.header.frame_id = self.base_link
        hex_marker_arr.marker_ids = self.front_cam_ids + self.back_cam_ids
        hex_marker_arr.poses = self.front_cam_hexes + self.back_cam_hexes

        self.block_pub.publish(hex_marker_arr)


def main(args=None):
    rclpy.init(args=args)

    aruco_link_calculator = ArucoLinkCalculator()

    rclpy.spin(aruco_link_calculator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    aruco_link_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
