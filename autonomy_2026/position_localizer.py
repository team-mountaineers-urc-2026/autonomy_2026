import rclpy
from rclpy.node import Node
import math
from autonomy_2026.autonomy_targets import AutonomyTargets as Targets
from autonomy_2026.decision_status import DecisionStatus as Status
from ros2_aruco_interfaces.msg import ArucoMarkers
from robot_interfaces.msg import ObjectPoint
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D, Vector3, Point, Quaternion
from sensor_msgs.msg import NavSatFix
from builtin_interfaces.msg import Time
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException, TransformStamped
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import PointStamped as tf2_PointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
import copy


EARTH_RADIUS_METERS = 6_378_137

class ObjectLocalizationNode(Node):

    def __init__(self):
        super().__init__('object_localization_node')

        # QOS Profiles
        self.mavros_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        ) 

        #Timer Publish Freq
        self.declare_parameter("publish_freq", 5.0)
        self.publish_period = 1.0 / self.get_parameter("publish_freq").get_parameter_value().double_value

        # Aruco Subscribers
        self.declare_parameter('subscription_topics', ['/logitech_44/aruco_markers', '/logitech_38/aruco_markers', '/logitech_36/aruco_markers', '/logitech_41/aruco_markers',])  
        self.declare_parameter('global_origin_frame', 'base_link')
        self.subscription_topics = self.get_parameter('subscription_topics').get_parameter_value().string_array_value
        self.subscribers = []


        self.seen_objects = [ () ] * 6 # Number of object types -> 4 Aruco and 2 Object
        self.current_pose = Pose2D()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

  
        # Make a list of all the wanted subscribers
        for topic in self.subscription_topics:
            self.subscribers.append(
                self.create_subscription(
                    ArucoMarkers,
                    topic,
                    lambda msg,topic=topic: self.pose_callback_aruco(msg, topic),
                    10
                )
            )
            # self.timer =  self.create_timer(1, self.confidence_callback(topic))

        # Create Subscriber for object detection output
        self.create_subscription(
            ObjectPoint,
            'cam_object',
            self.pose_callback_object,
            10
        )

        self.current_gps_sub = self.create_subscription(
            NavSatFix, 
            '/mavros/global_position/global', 
            self.update_location_callback, 
            self.mavros_qos_profile)
        

        self.subscriber = self.create_subscription(
            Vector3,
            '/health_monitor/chassis_orientation',
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.subscriber = self.create_subscription(
            Int32,
            'decision_status',
            self.clear_seen_callback,
            10
        )
    
        # Object timer for repeatedly publishing waypoint even if not see. Recalculates position if unseen.
        self.publish_timer = self.create_timer(self.publish_period, self.timer_callback)


        self.point_pub = self.create_publisher(
            ObjectPoint,
            'object_locale',
            10
        )


    ## Depreciate all confidence levels of the found markers
    def confidence_callback(self, source):
        #The confidence will depreciate every second until 30 seconds elapses. 
        #This causes the confidence to become too low to maintain the aruco marker 

        # NOT IMPLEMENTED
        return 
    


    ## ======================  MAVROS UPDATE =================================
    def update_location_callback(self, msg: NavSatFix):
        self.current_pose.x = msg.latitude
        self.current_pose.y = msg.longitude
    # Get heading
    def imu_callback(self, msg):
        self.current_pose.theta = msg.z




    ## ==================== GPS HELPERS =====================================
    def distanceGPS2d(self, pose1 : Pose2D, pose2 : Pose2D):
        
        phi1 = math.radians(pose1.x)
        phi2 = math.radians(pose2.x)
        delta_phi = math.radians(pose2.x - pose1.x)
        delta_lambda = math.radians(pose2.y - pose1.y)

        a = math.sin(delta_phi / 2.0) ** 2 + \
            math.cos(phi1) * math.cos(phi2) * \
            math.sin(delta_lambda / 2.0) ** 2
        
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return EARTH_RADIUS_METERS * c    



    def gps_To_Ang(self, current: Pose2D, goal: Pose2D):

        # self.get_logger().info(f"CURRENT : {current.y}, {current.x}")
        # self.get_logger().info(f"GOAL_POS:     {goal.y}, {goal.x}")


        #Creating a gps cooridinate with the same y as current to obtain the distance purely in the x direction
        modified_y_goal = Pose2D()
        modified_y_goal.x = goal.x
        modified_y_goal.y = current.y

        delta_x = self.distanceGPS2d(current, modified_y_goal)

        #Delta x will always be positive initially. To get an accurate angle I need to determine whether the goal is north (delta x is positive) or south (negative) of the current location
        if current.x > goal.x:
            delta_x = -delta_x

        #Creating a gps with the same x as current to obtain the distance purely in the y direction
        modified_x_goal = Pose2D()
        modified_x_goal.y = goal.y
        modified_x_goal.x = current.x

        delta_y = self.distanceGPS2d(current, modified_x_goal)

        #As ositive y would be west, however, longitude is the opposite of that, hence the current < goal for negative
        if current.y < goal.y: 
            delta_y = -delta_y

        theta_coords = math.atan2(delta_y, delta_x)
        # self.get_logger().info(f"ABS THETA: {theta_coords}")

        #this is from the pixhawk and it's given in degrees
        theta_orientation = current.theta
        # self.get_logger().info(f"ROV THETA: {theta_orientation}")

        # The angle of the point in Heimdall's frame relative to the x axis and positive being towards positive y
        theta_local = theta_coords - theta_orientation

        while(theta_local >math.pi):
            theta_local -= 2*math.pi

        while(theta_local < -math.pi):
            theta_local += 2*math.pi

        return theta_local #in radians


    # ========================== POSTION CALLBACKS FOR OBJ AND ARUCO =============================
    # Update the current list of known aruco markers
    def pose_callback_aruco(self, msg, source):

        camera_spec_markers = []

        # Get the length of the pose array
        length = len(msg.marker_ids)
        camera_frame = msg.header.frame_id

        if( camera_frame != "44" and  camera_frame != "36" and camera_frame != "38" and camera_frame != "41"):
            return
        
        global_frame = self.get_parameter('global_origin_frame').get_parameter_value().string_value

        # Get the transform the the current camera frame
        try:
            self.tf_buffer.lookup_transform_async
            zero = Time()
            zero.sec = 0
            zero.nanosec = 0
            camera_tf = self.tf_buffer.lookup_transform(
                camera_frame,
                # self.get_parameter('global_origin_frame').get_parameter_value().string_value,
                'base_link',
                # rclpy.time.Time(0) # 
                zero
            )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {camera_frame} to {global_frame}: {ex}')
            return


        for i, marker_id in enumerate(msg.marker_ids):
            #Skip if not marker 0-3
            # self.get_logger().info(f"{i}")
            if(marker_id>= 4):
                return

            # Get the pose
            pose = msg.poses[i]
            
            #Create temporary point to use in transform
            marker_point = ObjectPoint()
            temp_point = tf2_PointStamped()

            #Create temp point from camera to object
            temp_point.point.y = pose.position.y
            temp_point.point.x = pose.position.x
            temp_point.point.z = pose.position.z
            temp_point.header.frame_id = camera_frame
            temp_point.header.stamp = camera_tf.header.stamp

            transformed_point = self.tf_buffer.transform(temp_point, 'base_link')

            #print position from camera (for testing)
            # self.get_logger().info(f"CAM X: {temp_point.point.x}")
            # self.get_logger().info(f"CAM Y: {temp_point.point.y}")
            # self.get_logger().info(f"CAM Z: {temp_point.point.z}")

            #Create pose from base to object
            marker_point.point.point.y = transformed_point.point.y
            marker_point.point.point.x = transformed_point.point.x
            marker_point.point.point.z = transformed_point.point.z
            marker_point.point.header.frame_id = camera_frame
            marker_point.point.header.stamp = camera_tf.header.stamp

            #print position from baselink (for testing)
            # self.get_logger().info(f"BASE X: {marker_point.point.point.x}")
            # self.get_logger().info(f"BASE Y: {marker_point.point.point.y}")
            # self.get_logger().info(f"BASE Z: {marker_point.point.point.z}")

            #Determine Object ID
            aruco_id = marker_id
            if(aruco_id == 0):
                marker_point.object_id = Targets.ARUCO_0.value
            elif(aruco_id == 1):
                marker_point.object_id = Targets.ARUCO_1.value
            elif(aruco_id == 2):
                marker_point.object_id = Targets.ARUCO_2.value
            elif(aruco_id == 3):
                marker_point.object_id = Targets.ARUCO_3.value

            #Alter confidence value based on number of seconds until deletion
            marker_point.confidence = 30.0


            # STORE LOCATION OF MARKER 
            # Get the mavros information at this point
            rover_GPS = copy.deepcopy(self.current_pose) # Deep copy, not by reference
            last_seen_locale = (rover_GPS, marker_point ) 

            index = -1
            count = 0

            # Append this marker to the list of ones we see right now
            for tuple in self.seen_objects:
                if( len(tuple) == 0 ):
                    continue
                elif(tuple[1].object_id == marker_point.object_id):
                    index = self.seen_objects.index(tuple)#This might be incorrect
                else:
                    count += 1
                


            # Check to see if object location already in seen objects list
            if (index == -1): 
                # If not found, add it. 
                self.seen_objects[count] = last_seen_locale
            else: 
                # Update if found
                self.seen_objects[index] = last_seen_locale

            # self.get_logger().info(f'Last Seen {last_seen_locale}')


        



    ## Update the current list of known aruco markers
    def pose_callback_object(self, msg):

        camera_frame = msg.point.header.frame_id
        
        global_frame = self.get_parameter('global_origin_frame').get_parameter_value().string_value

        # Get the transform the the current camera frame
        try:
            self.tf_buffer.lookup_transform_async
            zero = Time()
            zero.sec = 0
            zero.nanosec = 0
            camera_tf = self.tf_buffer.lookup_transform(
                camera_frame,
                global_frame,
                zero
            )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {camera_frame} to {global_frame}: {ex}')
            return


        # Get the pose
        pose = msg.point
        
        #Create temporary point to use in transform
        marker_point = ObjectPoint()
        temp_point = tf2_PointStamped()

        
        #Create temp point from camera to object
        temp_point.point.y = pose.point.y
        temp_point.point.x = pose.point.x
        temp_point.point.z = pose.point.z
        temp_point.header.frame_id = camera_frame
        temp_point.header.stamp = camera_tf.header.stamp

        transformed_point = self.tf_buffer.transform(temp_point, 'base_link')

        #Create pose from base to object
        marker_point.point.point.y = transformed_point.point.y
        marker_point.point.point.x = transformed_point.point.x
        marker_point.point.point.z = transformed_point.point.z
        marker_point.point.header.frame_id = camera_frame
        marker_point.point.header.stamp = camera_tf.header.stamp

        #Determine Object ID
        marker_point.object_id = msg.object_id

        #Alter confidence value based on number of seconds until deletion - Not Implemented
        marker_point.confidence = 30.0


        # STORE LOCATION OF MARKER 
        # Get the mavros information at this point
        rover_GPS = copy.deepcopy(self.current_pose) # Deep copy, not by reference
        last_seen_locale = (rover_GPS, marker_point ) 

        index = -1
        count = 0

        # Append this marker to the list of ones we see right now
        for tuple in self.seen_objects:
            if( len(tuple) == 0 ):
                continue
            elif(tuple[1].object_id == marker_point.object_id):
                index = self.seen_objects.index(tuple)#This might be incorrect
            else:
                count += 1
                


        # Check to see if object location already in seen objects list
        if (index == -1): 
            # If not found, add it. 
            self.seen_objects[count] = last_seen_locale
        else: 
            # Update if found
            self.seen_objects[index] = last_seen_locale

        # self.get_logger().info(f'Last Seen {last_seen_locale}')



    ## Timer for publishing updated info.
    def timer_callback(self):

        for seen_obj in self.seen_objects:
            # Check if the object has been seen
            if len(seen_obj) != 0:
          
                # Obtain current location
                current_GPS = copy.deepcopy(self.current_pose)
                prev_GPS = seen_obj[0]

                # ========= Get Rotatation ============
                # Find difference in theta
                # delta_theta = prev_GPS.theta - current_GPS.theta 
                delta_theta = current_GPS.theta - prev_GPS.theta 

                # ======== Get Translation ============
                hypotenuse = self.distanceGPS2d(current_GPS, prev_GPS)

                

                # Getting Row
                angle_row = self.gps_To_Ang(prev_GPS, current_GPS) # start -> goal

                # # Getting Phi 
                # angle_phi =  math.pi/2 - prev_GPS.theta - angle_row

                # # Finding X-Y in local of prev
                # delta_x = math.sin(angle_phi) * hypotenuse
                # delta_y = math.cos(angle_phi) * hypotenuse


                # Finding X-Y in local of prev
                delta_x = math.cos(angle_row) * hypotenuse
                delta_y = math.sin(angle_row) * hypotenuse

                # #Delta x will always be positive initially. To get an accurate angle I need to determine whether the goal is north (delta x is positive) or south (negative) of the current location
                # if prev_GPS.x > current_GPS.x:
                #     delta_x = -delta_x

                # #As positive y would be west, however, longitude is the opposite of that, hence the current < goal for negative
                # if prev_GPS.y < current_GPS.y: 
                #     delta_y = -delta_y

                # self.get_logger().info(f'Delta X:  {delta_x}')
                # self.get_logger().info(f'Delta Y:  {delta_y}')
                # self.get_logger().info(f'Delta Theta:  {delta_theta}')  

                # ========== Get Transform Info ===============
                object_locale = copy.deepcopy(seen_obj[1])

                updated_locale = self.tf_estimator(Point(x=object_locale.point.point.x, y = object_locale.point.point.y) , Pose2D(x=delta_x,y=delta_y, theta=delta_theta))
                
                object_locale.point.point.x = updated_locale.x
                object_locale.point.point.y = updated_locale.y

                # ========== Publsh Object Perminance ===========
                
                self.point_pub.publish(object_locale)
                # self.get_logger().info(f'MARKER LOCALE {object_locale}')

    def clear_seen_callback(self, msg):
        if(msg.data == Status.NEXT_POINT.value or msg.data == Status.ARRIVED.value):
            # self.get_logger().info(f'\n**********ERASING SEEN OBJECTS ********\n')
            self.seen_objects = [ () ] * 6 



    # New Transform
    def tf_estimator(self, last_saw_point : Point, old_to_new : Pose2D): ## saved, 
        
        # Set up the tf stuff
        estimation_tree = Buffer()

        # Link 1
        detection_tf = TransformStamped()
        detection_tf.header.frame_id = "parent"
        detection_tf.child_frame_id = "child1"
        detection_tf.transform.translation.x = last_saw_point.x
        detection_tf.transform.translation.y = last_saw_point.y

        # Link 2
        movement_tf = TransformStamped()
        movement_tf.header.frame_id = "parent"
        movement_tf.child_frame_id = "child2"
        movement_tf.transform.translation.x = old_to_new.x
        movement_tf.transform.translation.y = old_to_new.y

        quat = Quaternion()
        e_quat = euler_to_quaternion(old_to_new.theta, 0, 0)
        quat.x = e_quat[0]
        quat.y = e_quat[1]
        quat.z = e_quat[2]
        quat.w = e_quat[3]

        movement_tf.transform.rotation = quat

        # Calculate the transform between them
        estimation_tree.set_transform(detection_tf, "")
        estimation_tree.set_transform(movement_tf, "")

        estimation_tf = estimation_tree.lookup_transform("child2", "child1", Time())

        return Point(x=estimation_tf.transform.translation.x, y=estimation_tf.transform.translation.y)

def euler_to_quaternion(yaw, pitch, roll):
        """
        Converts Euler angles (yaw, pitch, roll) in radians to a quaternion.

        Args:
            yaw (float): The yaw angle in radians.
            pitch (float): The pitch angle in radians.
            roll (float): The roll angle in radians.

        Returns:
            list: A list representing the quaternion [qx, qy, qz, qw].
        """
        
        # Calculate half angles
        ry = roll / 2
        py = pitch / 2
        wy = yaw / 2
        
        # Calculate quaternion components
        qx = math.sin(ry) * math.cos(py) * math.cos(wy) - math.cos(ry) * math.sin(py) * math.sin(wy)
        qy = math.cos(ry) * math.sin(py) * math.cos(wy) + math.sin(ry) * math.cos(py) * math.sin(wy)
        qz = math.cos(ry) * math.cos(py) * math.sin(wy) - math.sin(ry) * math.sin(py) * math.cos(wy)
        qw = math.cos(ry) * math.cos(py) * math.cos(wy) + math.sin(ry) * math.sin(py) * math.sin(wy)
    
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)

    localization_node = ObjectLocalizationNode()

    rclpy.spin(localization_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    localization_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
