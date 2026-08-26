# autonomy_2025
Most of the Autonomous Navigation Logic and Processing for Daedalus

## Nodes
- [autonomy_targets](#autonomY-targets)
- [brian_node](#brian-node)
- [decision_status](#decision-status)
- [led_interface](#led-interface)
- [local_path_planner](#local-path-planner)
- [move_controller_mux](#move-controller-mux)
- [new_pid_planner](#new-pid-planner)
- [position_localizer](#position-localizer)
- [static_p2p_vel](#static-p2p-vel)
- [velocity_mux](#velocity-mux)
- [waypoint_manager](#waypoint-manager)

## Autonomy Targets

## Brian Node

⚠️⚠️⚠️ Work In Progress ⚠️⚠️⚠️

This node is the Autonomy manager node, most control code should go here

### Parameters
| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `danger_timeout` | Double | 2.0 | Used in Reactive Protection, if the time since last pitch / roll message exceeds this value, protection will be disabled |
| `roll_threshold` | Integer | 30 | Used to determine if the rover has passed the safe threshold in the roll direction |
| `pitch_threshold` | Integer | 60 | Used to determine if the rover has passed the safe threshold in the pitch direction |
| `stopping_threshold` | Double | 1.0 | Used to determine how close the rover should get to a goal point |

### Subscriptions
| Topic | Type | Description |
| --- | --- | --- |
| `next_waypoint` | Pose2D | The point the rover is trying to get to |
| `object_locale` | ObjectPoint | The last object the rover has seen |
| `/health_monitor/chassis_orientation` | Vector3 | The rover orientation, rpy |
| `/mavros/global_position/global` | NavSatFix | The gps position of the rover |

### Publishers
| Topic | Type | Description |
| --- | --- | --- |
| `decision_status` | Int32 | The current behavior of the rover |
| `next_local` | Pose2D | The next point the rover will try to go to, converted to the local frame |
| `decision_led_topic` | String | The state the LEDs should be in |

## Decision Status

## Led Interface

## Local Path Planner

## Move Controller Mux
This node selects which movement controller to use (Static P2P or PID). Commands are simply forwarded to the appropriate controller.

### Parameters
| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `velocity_input_topic` | String | 'vel_pose' | Desired navigation (Pose) topic string |
| `p2p_control_topic`| String | 'p2p_pose' | P2P controller publisher target topic |
| `pid_control_topic` | String | 'pid_pose' | PID controller publisher target topic |
| `gui_mode_topic` | String | 'control_mux' | Subscription topic for changing controller target |

### Subscriptions
Please note topics may have been changed by parameters in launch files.
| Topic | Type | Description |
| --- | --- | --- |
| `vel_pose` | Pose2D | Echo subscription for Pose2D |
| `control_mux` | Bool | True for P2P controller, False for PID controller |

### Publishers
Please note topics may have been changed by parameters in launch files.
| Topic | Type | Description |
| --- | --- | --- |
| `p2p_pose` | Pose2D | P2P controller echo publisher |
| `pid_pose` | Pose2D | PID controller echo publisher |

## New PID Planner
This node calculates the output veocity for the rover to travel to a 2D position. 

### Parameters
| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `dist_prop_gain` | Float | 0.1 | Static multplier for distance |
| `dist_kp` | Float | 0.04 | Weighted value of proportional response for distance  |
| `dist_kd` | Float | 0.0 | Weighted value of derivative response for distance |
| `dist_ki` | Float | 0.0 | Weighted value of integral response for distance |
| `theta_prop_gain` | Float | 0.1 | Static multiplier for direction |
| `head_kp` | Float | 0.1 |  Weighted value of proportional response for direction  |
| `head_kd` | Float | 0.0 |  Weighted value of derivative response for direction  |
| `head_ki` | Float | 0.0 |  Weighted value of integral response for direction  |
| `min_linear_speed` | Float | 0.0 | Value for the minimum linear speed |
| `max_linear_speed` | Float | 1.0 | Value for the maximum linear speed |
| `min_angular_speed` | Float | -1.0 | Value for minimum angular speed |
| `max_angular_speed` | Float | 1.0 | Value for maximum angular speed |
| `angle_threshold` | Float | 10.0 | Value where angles less than it will be disregarded |
| `target_pose_topic` | String | 'pid_pose' | Topic for the destination position |
| `cmd_vel_topic` | String | 'cmd_vel' | Topic for the output command velocity |


### Subscriptions
| Topic | Type | Description |
| --- | --- | --- |
| `pid_pose` | Pose2D | Position of intended desination on a 2D grid |


### Publishers
| Topic | Type | Description |
| --- | --- | --- |
| `/cmd_vel` | Twist | Publishes the velocity for traveling to the stination based off the calculated proportional, derivative, and integral parameters. |



## Position Localizer
⚠️⚠️⚠️ Work In Progress ⚠️⚠️⚠️

This node is the designed to turn all object positions relative to the cameras into positions relative to a central point on the rear chasis. It can handle positions from both ArUco Markers and the Hammer and Water Bottle YOLO model for 2025.

### Parameters
| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `subscription_topics` | String[] | 2.0 | Holds the four topic names to retrieve ArucoMarker locations from. |
| `global_origin_frame` | String | 'base_link' | The name of the node in the transform tree to transform to/from.  |
| `camera_info` | String | '/image_topic' | The topic name for the camera_info subscriber |

### Subscriptions
| Topic | Type | Description |
| --- | --- | --- |
| `image_topic` | Image | The images being sent by the cameras. Used for pairing camera infor for the ROS2 Aruco Node |
| `/logitech_27/aruco_markers` | ArucoMarkers | The location and type of the currently spotted ArucoMarker found by ROS2 Auro Node on the Logitech 27 Camera |
| `/logitech_28/aruco_markers` | ArucoMarkers | The location and type of the currently spotted ArucoMarker found by ROS2 Auro Node on the Logitech 28 Camera |
| `/logitech_29/aruco_markers` | ArucoMarkers | The location and type of the currently spotted ArucoMarker found by ROS2 Auro Node on the Logitech 29 Camera |
| `/logitech_30/aruco_markers` | ArucoMarkers | The location and type of the currently spotted ArucoMarker found by ROS2 Auro Node on the Logitech 30 Camera |

### Publishers
| Topic | Type | Description |
| --- | --- | --- |
| `object_locale` | ObjectPoint | Publishes custom data message with the position of the object of interest after conversion. |
| `camera_info` | CameraInfo | Publishes camera info for the ROS2 Aruco to use when calculating position of Aruco markers. |


## Static P2P Velocity Controller
This is one of the two movement controllers for the rover. This node has no 'memory' of rover Pose and instead performs the immediate apprpriate instruction to move to a given local point. This node remains silent when no instruction is given (e.g. When it is not selected by the move controller mux).

### Parameters
⚠️⚠️⚠️ Parameters are currently being added for this node ⚠️⚠️⚠️

### Subscriptions
| Topic | Type | Description |
| --- | --- | --- |
| `p2p_pose` | Pose2D | Input target point from the velocity mux |

### Publishers
| Topic | Type | Description |
| --- | --- | --- |
| `cmd_vel` | Twist | The movement instructions for the rover |


## Velocity Mux
This node is the designed to act as a switch between command velocities of the game controllers and the autonomy package.

### Parameters
| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `autonomy_vel_topic` | String | '/autonomy/cmd_vel' | Topic name for the autonomy's velocity commands |
| `manual_vel_topic` | String | '/base_station/cmd_vel' | Topic name for the game controllers velocity commands |
| `cmd_vel_topic` | String | '/drivetrain/cmd_vel' | Topic name for the output to drivetrain to actually control the rover |
| `cmd_vel_indicator` | String | 'indicator_cmd_vel' | Topic name for the indicator |
| `velocity_indicator` | Bool | 'True' | Value which determines if we are controlling velocity through game controllers (True) or autonomy (False) |


### Subscriptions
| Topic | Type | Description |
| --- | --- | --- |
| `/autonomy/cmd_vel` | Twist | The velocity commands being sent by the autonomy package. |
| `/base_station/cmd_vel` | Twist | The velocity commands being sent by the game controllers. |
| `indicator_cmd_vel` | Bool | The inicator which determines if we are running the game controllers or autonomy package velocities. |


### Publishers
| Topic | Type | Description |
| --- | --- | --- |
| `/drivetrain/cmd_vel` | Twist | Publishes the velocity to drivetrain for the rover to actually execute. |



## Waypoint Manager
⚠️⚠️⚠️ Work In Progress ⚠️⚠️⚠️

This node is the designed to work as a database of the paths the rover should follow

### Parameters
| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `publish_freq` | Double | 2.0 | Used to determine the frequency at which the node should pubilsh the next point for the rover to try to go to |

### Subscriptions
| Topic | Type | Description |
| --- | --- | --- |
| `decision_status` | Int32 | The current behavior of the rover |

### Publishers
| Topic | Type | Description |
| --- | --- | --- |
| `next_waypoint` | Pose2D | The point the rover is trying to get to |
