from enum import Enum
from geometry_msgs.msg import Pose2D, Vector3
import math


# Global Constants
EARTH_RADIUS_METERS = 6_378_137

class AutonomyTargets(Enum):
    STOP     = 0
    GPS      = 1
    WAYPOINT = 2
    ARUCO_0  = 3
    ARUCO_1  = 4
    ARUCO_2  = 5
    ARUCO_3  = 6
    BOTTLE   = 7
    HAMMER   = 8
    EMPTY    = 9


class AutonomyHelper():
    print("Hello")

    # Get the distance between two GPS points
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
