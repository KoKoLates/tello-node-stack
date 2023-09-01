#! /usr/bin/env python3

import rospy, math, copy, tf

from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Point, Quaternion

class Interpolation:
    def __init__(self, start: PoseStamped, end: PoseStamped, res: int | float) -> None:
        """
        @param start: Start point
        @param end: end point
        @param res: distance resolution
        """
        assert(isinstance(start, PoseStamped)), "The start point should be PoseStamped"
        assert(isinstance(end, PoseStamped)), "The end point should be PoseStamped"
        assert(isinstance(res, (int, float)) and res > 0), "The distance resolution should be a positive number"

        self.start, self.end, self.res = start, end, res
        self.frame_id = self.start.header.frame_id

        self.disp_vector = self.__getDisplacementVector()
        self.dirc_vector = self.__getDirectionVector()
        self.distance = self.__L2_Norm(self.disp_vector)
        self.start_euler = self.__getEulerAngle(self.start)
        self.end_euler = self.__getEulerAngle(self.end)
        self.path = None
        self.__interpolate()


    def get_path(self) -> Path:
        return self.path

    def __L2_Norm(self, vector: Vector3) -> float:
        """
        L2 normalization the given vector.
        @param vector: The given vector
        @return: The L2 norm of the vector
        @rtype: float
        """
        assert(isinstance(vector, Vector3)), "The given vector should be Vector3 type."
        return math.sqrt(vector.x ** 2 + vector.y ** 2 + vector.z ** 2)

    def __getDisplacementVector(self) -> Vector3:
        """
        @return: Vector between start and end point
        @rtype: geometry_msgs.msgs.Vector3
        """
        vector = Vector3()
        vector.x = self.end.pose.position.x - self.start.pose.position.x
        vector.y = self.end.pose.position.y - self.start.pose.position.y
        vector.z = self.end.pose.position.z - self.start.pose.position.z
        return vector

    def __getDirectionVector(self) -> Vector3:
        """
        @return: the direction vector between start and end point
        @rtype: geometry_msgs.msgs.Vector3
        """
        vector = copy.deepcopy(self.disp_vector)
        magnitude = self.__L2_Norm(vector)
        vector.x /= magnitude
        vector.y /= magnitude
        vector.z /= magnitude
        return vector

    def __getEulerAngle(self, waypoint: PoseStamped) -> list[float]:
        """
        Get the euler angle that transform from quaternion.
        @param waypoint: THe point to be transformed
        @return: the list of euler angle
        @rtype: list[float]
        """
        assert(isinstance(waypoint, PoseStamped)), "waypoints should be PoseStamped."
        q = waypoint.pose.orientation
        return tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))

    def __interpolate(self) -> None:
        """
        Processing the interpolating calculation.
        """
        self.path = Path()
        self.path.header = self.start.header
        step = (int)(math.ceil(self.distance / self.res))

        if step == 1: 
            # Don't need to interpolate
            self.path.poses.append(self.start)
            self.path.poses.append(self.end)

        else:
            roll_dif = self.end_euler[0] - self.start_euler[0]
            pitch_dif = self.end_euler[1] - self.start_euler[1]
            yaw_dif = Interpolation.yaw_norm(self.end_euler[2] - self.start_euler[2])

            roll_res, pitch_res, yaw_res = roll_dif / step, pitch_dif / step, yaw_dif / step
            position = [[None, None, None] for n in range(step)]
            quaternion = [None for n in range(step)]

            # Linear Interpolation
            for n in range(step):
                position[n][0] = self.start.pose.position.x + n * self.res * self.dirc_vector.x
                position[n][1] = self.start.pose.position.y + n * self.res * self.dirc_vector.y
                position[n][2] = self.start.pose.position.z + n * self.res * self.dirc_vector.z

                roll = self.start_euler[0] + n * roll_res
                pitch = self.start_euler[1] + n * pitch_res
                yaw = self.start_euler[2] + n * yaw_res
                quaternion[n] = tf.transformations.quaternion_from_euler(roll, pitch, yaw).tolist()

            position.append([self.end.pose.position.x, self.end.pose.position.y, self.end.pose.position.z])
            quaternion.append([self.end.pose.orientation.x, self.end.pose.orientation.y, 
                                self.end.pose.orientation.z, self.end.pose.orientation.w])

            for i in range(len(position)):
                self.path.poses.append(PoseStamped(
                    header = Header(stamp = rospy.Time.now(), frame_id = self.frame_id), 
                    pose = Pose(position = Point(*position[i]), orientation = Quaternion(*quaternion[i]))
                    ))
    

    @staticmethod
    def yaw_norm(radius: float) -> float:
        """
        Normalize the radius of yaw angle
        @param radius: The yaw angle in raduis unit
        @return: The normalized yaw angle
        @rtype: float
        """
        while(radius > math.pi):
            radius -= math.pi * 2
        while(radius < -math.pi):
            radius += math.pi * 2
        return radius


def path_generator(waypoints: list, res: int | float) -> Path:
    """
    Generate the path.
    @param waypoints: The list of waypoints
    @param res: distance resolution
    @return: path for waypoints
    @rtype: Path
    """
    assert(isinstance(waypoints, list) and isinstance(waypoints[0], PoseStamped)), "waypoints should be list of PoseStamped."
    assert(len(waypoints) > 1), "The length of waypoints should be more than one."
    waypoint_num = len(waypoints)
    path = Path()
    path.header = waypoints[0].header
    for n in range(waypoint_num - 1):
        interpolator = Interpolation(waypoints[n], waypoints[n + 1], res)
        temp_path = interpolator.get_path()
        poses = temp_path.poses

        if(n != waypoint_num - 2):
            poses.pop() # delete last element, otherwise redundent

        path.poses += poses
    
    return path
