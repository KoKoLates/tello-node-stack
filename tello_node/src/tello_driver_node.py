#! /usr/bin/env python3

import time
import rospy
import threading
import cv2
import av
import numpy as np

from tellopy._internal import tello, error, logger
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Image

class Tello_Node(tello.Tello):
    def __init__(self) -> None:
        super(Tello_Node, self).__init__(port=9000)
        
        self.cv_bridge = CvBridge()

        # Publisher for image signal
        self.image_publisher = rospy.Publisher('image_raw', Image, queue_size=1)

        # Subscriber for control signal
        rospy.Subscriber('take_off', Empty, self.take_off_callback, queue_size=1)
        rospy.Subscriber('land', Empty, self.land_callback, queue_size=1)
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback, queue_size=1)

        # Tello Connection
        rospy.loginfo('Connecting to drone {}'.format(self.tello_addr))
        self.connect()
        try:
            self.wait_for_connection(timeout=30.0)
        except error.TelloError as err:
            rospy.logerr(str(err))
            rospy.signal_shutdown(str(err))
            self.quit()
            return
        rospy.loginfo('Connected to drone')

        # Skip first 300 frame to avoid initial stream lag
        self.initial_frame = 300

        # Video Stream
        self.request_terminate = threading.Event()
        self.frame_thread = threading.Thread(target=self.frame_work_loop)
        self.frame_thread.start()

        rospy.on_shutdown(self.shutdown_routine)
        rospy.loginfo('The driver node is ready')

    # Subscriber callback function
    def cmd_vel_callback(self, msg:Twist) -> None:
        self.set_pitch(msg.linear.x)
        self.set_roll(-msg.linear.y)
        self.set_yaw(-msg.angular.z)
        self.set_throttle(msg.linear.z)

    def take_off_callback(self, msg:Empty) -> None:
        self.takeoff()
        

    def land_callback(self, msg:Empty) -> None:
        self.land()

    # Video threading
    def frame_work_loop(self) -> None:
        video_stream = self.get_video_stream()
        try:
            container = av.open(video_stream)
        except BaseException as err:
            rospy.logerr('frame grab: pyav stream failed - {}'.format(str(err)))
            return

        rospy.loginfo('Video stream Starting')
        try:
            for frame in container.decode(video=0):
                if self.initial_frame > 0:
                    self.initial_frame -= 1
                    continue

                # convert pyav image -> PIL image -> opencv image
                img = np.array(frame.to_image())
                # img = cv2.resize(img, (480, 360), interpolation=cv2.INTER_LINEAR)
                # video resolution adjust, original 960 x 720

                # opencv image -> ros image message
                try:
                    img_msg = self.cv_bridge.cv2_to_imgmsg(img, 'rgb8')
                    img_msg.header.frame_id = rospy.get_namespace()
                except CvBridgeError as err:
                    rospy.logerr('frame grab: cv bridge failed - {}'.format(str(err)))
                    continue

                self.image_publisher.publish(img_msg)

        except BaseException as err:
            rospy.logerr('frame grab: pyav decoder failed - {}'.format(str(err)))
            return
        
        # Checking for normal shutdown
        if self.request_terminate.isSet():
            return

    # Shutdown
    def shutdown_routine(self):
        self.land()

        # Stop video stream
        self.request_terminate.set()
        self.frame_thread.join()

        self.quit()

def main() -> None:
    rospy.init_node('tello_driver_node', anonymous=False)
    drone = Tello_Node()
    rospy.spin()

if __name__ == '__main__':
    main()