#! /usr/bin/env python3

import rospy
import interpolation

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class WaypointsGenerator:
    def __init__(self) -> None:
        self.waypoints: list[PoseStamped] = []
        self.path: Path = None

        ## Publisher
        self.path_publisher = rospy.Publisher('/path', Path, queue_size=1)

        ## Subscriber
        self.pose_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.add_waypoint, queue_size=1)

    def add_waypoint(self, pose: PoseStamped) -> None:
        """
        Add the waypoints from goal topic 
        @param pose: The waypoints position
        """
        self.waypoints.append(pose)
        self.path = interpolation.path_generator(self.waypoints, 0.05)
        self.path_publisher.publish(self.path)


if __name__ == '__main__':
    rospy.init_node('waypoint')
    generator = WaypointsGenerator()
    rospy.spin()
