import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions.node import Node
from launch.actions import ExecuteProcess
from ament_index_python import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    # Front Base Link to Base Link (Static for now)
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0.377", "0", "0", "0", "0", "0", "base_link", "front_base_link"]
        ))

    # Pixhawk 1 IMU link
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["-0.135", "0", "0.09", "0", "0", "0", "base_link", "pixhawk_1_imu"]
        ))
    
    # Pixhawk 1 GPS link
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["-0.134", "0", "0.11", "0", "0", "0", "base_link", "pixhawk_1_gps"]
        ))


    # Right Internal Camera Link     
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0.127", "-0.253", "0", "0", "-1.57079632679", "-3.92699081699", "base_link", "right_internal_cam"]
        ))

    # Left Internal Camera Link
    ld.add_action(
        Node(   
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0.127", "0.253", "0", "0", "-1.57079632679", "-2.35619449019", "base_link", "left_internal_cam"]
        ))

     # Mastcam Front
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["0.01", "0", "0.327", "-1.57079632679", "0", "-1.57079632679", "base_link", "44"]
            
        ))
        
    # Mastcam Rear
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw     green , blue, red
            arguments=["-0.01", "0", "0.327", "1.57079632679", "0", "-1.57079632679", "base_link", "38"]
        ))
    
    # Mastcam Left
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0", "0.254", "0.327", "0", "0", "-1.57079632679", "base_link", "36"]
        ))
    
    # Mastcam Right
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0", "-0.2286", "0.327", "3.1415926535", "0", "-1.57079632679", "base_link", "41"]
        ))
    
    
    # Forward Cam
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0.4085", "0", "0.008", "0", "1.57079632679", "0", "front_base_link", "21"]  # forward_cam
        ))
        
    # Forward Lower Cam
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["0.371", "0", "-0.094", "0", "1.80", "0", "front_base_link", "forward_lower_cam"]
        ))
        
    # Back Lower Cam
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            # X, Y, Z, Roll, Pitch, Yaw
            arguments=["-0.097", "0", "-0.099", "0", "1.3", "3.1415926535", "base_link", "back_lower_cam"]
        ))


    # # Example Camera Link
    # ld.add_action(
    #     Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         output="screen",
    #         # X, Y, Z, Roll, Pitch, Yaw
    #         arguments=["-0.061", "0", "-0.127", "0", "-1.57079632679", "3.1415926535", "base_link", "camera"]
    #     ))

    
    
    
    # ld.add_action(
    #     Node(
    #         package="tf2_ros",
    #         executable="joint_state_publisher",
    #         output="screen",
    #         parameters=[
    #             {
    #                 # Each string in this array represents a topic name.
    #                 # For each string, create a subscription to the named topic of type sensor_msgs/msg/JointStates.
    #                 # publication to that topic will update the joints named in the message.
    #                 "source_list": ["base_link_front", "base_link_rear"],
    #                 # How much to automatically move joints during each iteration. Defaults to 0.0 of ommited.
    #                 "delta": 0,

    #             }
    #         ],
    #     ))
    return ld
