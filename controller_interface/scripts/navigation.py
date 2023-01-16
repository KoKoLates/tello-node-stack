#! /usr/bin/env python3

import tf, copy, rospy, math, numpy as np
from controller_interface.srv import set_reference # services
import interpolation


from nav_msgs.msg import Path
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Empty, Float64, Header
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, Quaternion

class Navigation:
    def __init__(self) -> None:
        rospy.init_node("Navigation")
        self.rate = rospy.Rate(30) # 30 Hz
        self.frame_id: str = "map"
        self.waypoint_index: int = 0
        self.home_waypoint: PoseStamped = PoseStamped(
            header = Header(stamp = rospy.Time.now(), frame_id = self.frame_id),
            pose = Pose(position = Point(0, 0, 0.2), orientation = Quaternion(0, 0, 0, 1))
        )

        self.waypath: Path = None
        self.waypoints: list = []
        self.current_pose = copy.deepcopy(self.home_waypoint)

        self.control_command: Twist = Twist()

        # Initialize clinet side for reference point setup serve 
        rospy.loginfo('Waiting for reference pose serve from control interface node.')
        rospy.wait_for_service('/set_reference')
        self.update_reference = rospy.ServiceProxy('/set_reference', set_reference)
        

        # Publisher 
        self.take_off_publisher = rospy.Publisher('/tello/take_off', Empty, queue_size=1)
        self.land_publisher = rospy.Publisher('/tello/land', Empty, queue_size=1)
        self.control_publisher = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.waypath_publisher = rospy.Publisher('/way_path', Path, queue_size=1)


        # Subscriber
        # self.ekf_pose_subscriber = rospy.Subscriber('/tf', TFMessage, self.ekf_pose_callback)
        self.pose_subscriber = rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, self.pose_callback)
        self.pid_x_subscriber = rospy.Subscriber('/pid_pitch/control_effort', Float64, self.control_x_callback)
        self.pid_y_subscriber = rospy.Subscriber('/pid_roll/control_effort', Float64, self.control_y_callback)
        self.pid_z_subscriber = rospy.Subscriber('/pid_thrust/control_effort', Float64, self.control_z_callback)
        self.pid_yaw_subscriber = rospy.Subscriber('/pid_yaw/control_effort', Float64, self.control_yaw_callback)

        rospy.sleep(1)
    
    # Callback Function
    def pose_callback(self, pose: PoseStamped) -> None:
        """
        Update current pose with ORB SLAM 2
        :param pose (PoseStamped): the current pose from ORB SLAM 2
        """
        pose.header.frame_id = self.frame_id
        self.current_pose = pose
        ## There are some method to calibrate the scale

    def ekf_pose_callback(self, pose: TFMessage) -> None:
        self.current_pose.header = pose.transforms[0].header
        self.current_pose.pose.position.x = pose.transforms[0].transform.translation.x
        self.current_pose.pose.position.y = pose.transforms[0].transform.translation.y
        self.current_pose.pose.position.z = pose.transforms[0].transform.translation.z
        self.current_pose.pose.orientation = pose.transforms[0].transform.rotation


    def control_x_callback(self, msg: Float64) -> None:
        self.control_command.linear.x = msg.data

    def control_y_callback(self, msg: Float64) -> None:
        self.control_command.linear.y = msg.data

    def control_z_callback(self, msg: Float64) -> None:
        self.control_command.linear.z = msg.data

    def control_yaw_callback(self, msg: Float64) -> None:
        self.control_command.angular.z = msg.data


    def run(self) -> None:
        """ Execute the navigation mission """
        # I'm trying to use the take off finishede pose as first position
        self.add_waypoint(-0.1,-0.15, 0.2, 0.0)
        self.add_waypoint( 0.3,-0.15, 0.2, 0.0)
        self.add_waypoint( 0.3, 0.15, 0.2, 0.0)
        self.add_waypoint(-0.1, 0.15, 0.2, 0.0)
        self.waypath = interpolation.path_generator(self.waypoints, 0.05)
        
        # take off -> tracking -> back -> landing
        # self.take_off() # manually
        self.start()
        self.back()
        self.land()
        

    def start(self) -> None:
        """ Start to tracking the waypoint that setup """
        rospy.loginfo('Navigation start!')
        self.set_waypoint(self.waypoints[self.waypoint_index])

        while not rospy.is_shutdown():
            self.control_publisher.publish(self.control_command)
            self.waypath_publisher.publish(self.waypath)
            if self.distance_from(self.waypoints[self.waypoint_index]) < 0.1:
                self.control_publisher.publish(Twist()) # Stop
                if self.is_finish():
                    return
                else:
                    self.waypoint_index += 1
                    self.set_waypoint(self.waypoints[self.waypoint_index])
            self.rate.sleep()


    def is_finish(self) -> bool:
        return self.waypoint_index == len(self.waypoints) - 1

    ## Helper Function
    def add_waypoint(self, x: float, y: float, z: float, yaw: float) -> None:
        """
        Add the waypint to the waypoints list
        :param x (float): x position
        :param y (float): y position
        :param z (float): z position
        :param yaw (float): yaw angle
        """
        q = tf.transformations.quaternion_from_euler(0, 0, yaw).tolist()
        waypoint = PoseStamped(
            Header(stamp=rospy.Time.now(), frame_id=self.frame_id),
            Pose(position=Point(x, y, z), orientation=Quaternion(*q))
        )
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.position.z = z
        self.waypoints.append(waypoint)

    def set_waypoint(self, waypoint: PoseStamped) -> bool:
        """
        Set the waypoints to the controller by calling the set reference service
        :param waypoint(PoseStamped): the waypoint to be setup
        """
        assert(isinstance(waypoint, PoseStamped)), "waypoint should be PoseStamped"
        try:
            _, _, yaw = Navigation.getEuler(waypoint)
            res = self.update_reference(
                waypoint.pose.position.x, 
                waypoint.pose.position.y, 
                waypoint.pose.position.z, 
                yaw
            )
            return res
        except rospy.ServiceException as exception:
            rospy.logerr('Service call failed: %s' %exception)

    def back(self) -> None:
        """ Back to the origin position """
        rospy.loginfo("Navigation End. Back to origin")
        self.set_waypoint(self.home_waypoint)
        while not rospy.is_shutdown():
            self.control_publisher.publish(self.control_command)
            if self.distance_from(self.home_waypoint) < 0.1:
                self.control_publisher.publish(Twist()) # Stop
                return
            self.rate.sleep()


    def take_off(self) -> None:
        """ Take off the drone """
        rospy.loginfo('Take Off...')
        self.take_off_publisher.publish(Empty())
        rospy.sleep(3)
        rospy.loginfo('Take Off Done')

    def land(self) -> None:
        """ Landing the drone """
        rospy.loginfo('Landing ...')
        # self.control_publisher.publish(Twist())
        # rospy.sleep(1)
        self.land_publisher.publish(Empty())
        rospy.sleep(3)
        rospy.loginfo('Landing Done')
        # rospy.on_shutdown(Navigation.shutdown())
        
    @staticmethod
    def shutdown() -> None:
        """ Shutdown handler """
        rospy.loginfo('Shutdown')

    @staticmethod
    def yaw_norm(radius: float) -> float:
        """
        Normalize the radius of yaw angle
        :param radius (float): The yaw angle in raduis unit
        :return: The normalized yaw angle
        :rtype: float
        """
        while(radius > math.pi):
            radius -= math.pi * 2
        while(radius < -math.pi):
            radius += math.pi * 2
        return radius

    @staticmethod
    def getEuler(waypoint: PoseStamped) -> list:
        """
        Transform from quaternion to euler angle for waypoint
        :param waypoint (PoseStamped): Waypoint
        :return: euler anlge
        :rtype: list of float
        """
        assert(isinstance(waypoint, PoseStamped)), "Waypoint should be a PoseStamped"
        q = waypoint.pose.orientation
        return tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))

    @staticmethod
    def deg2rad(degree: float) -> float:
        """
        Convert angle in degree to radian unit 
        :param degree(float): The angle in degree unit
        :return: the angle in radian
        :rtype: float
        """
        return Navigation.yaw_norm(degree) * math.pi / 180.0

    def distance_from_2D(self, position: PoseStamped) -> float:
        """
        Calculate the L2 distance a position to this waypoint
        :param position (PoseStamped.pose.position): The position to be checked
        :return: the l2 distances
        :rtype: float
        """
        return math.sqrt(
            (self.current_pose.pose.position.x - position.pose.position.x) ** 2 + 
            (self.current_pose.pose.position.y - position.pose.position.y) ** 2 )

    def distance_from(self, position: PoseStamped) -> float:
        """
        Calculate the L2 distance a position to this waypoint
        :param position (PoseStamped.pose.position): The position to be checked
        :return: the l2 distances
        :rtype: float
        """
        return math.sqrt(
            (self.current_pose.pose.position.x - position.pose.position.x) ** 2 + 
            (self.current_pose.pose.position.y - position.pose.position.y) ** 2 +
            (self.current_pose.pose.position.z - position.pose.position.z) ** 2 )


if __name__ == '__main__':
    try:
        navigation = Navigation()
        navigation.run()
        rospy.spin()

    except rospy.ROSInterruptException:
        navigation.land()

