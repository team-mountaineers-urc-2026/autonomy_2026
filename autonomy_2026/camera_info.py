import rclpy
from rclpy.node import Node
import rclpy.time
import yaml
from autonomy_2026.autonomy_targets import AutonomyTargets as Targets
from ros2_aruco_interfaces.msg import ArucoMarkers
from robot_interfaces.msg import ObjectPoint
from builtin_interfaces.msg import Time
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import PointStamped as tf2_PointStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
import os

cam_info_msg = CameraInfo()

class CameraInformation(Node):

    def __init__(self):
        super().__init__('camera_info')
        
        self.declare_parameter('camera_info44', '/logitech_44/camera_info')


        self.camera_info44 = self.get_parameter('camera_info44').get_parameter_value().string_value


        #Setup Camera Info Subscriber-Republisher
        self.declare_parameter('camera_img', '/image_topic')
        self.camera_img = self.get_parameter('camera_img').get_parameter_value().string_value
        self.create_subscription(
            Image,
            self.camera_img,
            self.cam_info_callback,
            10
        )
        
     
        # Set up Publishers for each camera in autonomy
        self.camera44_info_pub = self.create_publisher(
            CameraInfo,
            self.camera_info44,
            10
        )


    # Update the current list of known aruco markers
    def cam_info_callback(self, msg):

        for id in range(36,44): #Cams 27->30

            #Retrieve File info
            config_file = os.path.join('./launches/config/', f'logitech_44_info_320x180.yaml') #logitech_27_info_1080x1920
            with open (config_file,'r') as yaml_file:
                yaml_data = yaml.load(yaml_file)

            # Turn Yaml file data into CameraInfo message
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            # header.frame_id = 'my_frame'
            cam_info_msg.header = header
            cam_info_msg.width = yaml_data["image_width"]
            cam_info_msg.height = yaml_data["image_height"]
            cam_info_msg.k = yaml_data["camera_matrix"]["data"]
            cam_info_msg.d = yaml_data["distortion_coefficients"]["data"]
            cam_info_msg.r = yaml_data["rectification_matrix"]["data"]
            cam_info_msg.p = yaml_data["projection_matrix"]["data"]
            cam_info_msg.distortion_model = yaml_data["distortion_model"]

            if id == 44 or id==36 or id==41 or id==38:
                self.camera44_info_pub.publish(cam_info_msg)




def main(args=None):
    rclpy.init(args=args)

    localization_node = CameraInformation()

    rclpy.spin(localization_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    localization_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
