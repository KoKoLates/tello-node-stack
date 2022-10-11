#! /usr/bin/env python3
import av
import math
import time
import rospy
import tf2_ros
import threading
import numpy as np
import camera_info_manager as camera_info

from tello_node.msg import tello_status
from tellopy._internal import tello, error, logger, event
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Empty
from sensor_msgs.msg import Image, CompressedImage, Imu, CameraInfo
from nav_msgs.msg import Odometry


class Tello_Node(tello.Tello):
    def __init__(self) -> None:
        # Parameter Server
        self.tellop_IP = rospy.get_param('~tello_ip', '192.168.10.1')
        self.tello_cmd_server_port = int(rospy.get_param('~tello_cmd_server_port', 8889))
        self.local_cmd_client_port = int(rospy.get_param('~local_cmd_client_port', 8890))
        self.local_vid_server_port = int(rospy.get_param('~local_vid_server_port', 6038))

        self.initial_frame = int(rospy.get_param('~initial_frame', 100))
        self.connect_time_out =rospy.get_param('~connect_time_out', 10.0)
        self.calibration_path = rospy.get_param('~camera_calibraiton', '')
        self.h264_encoded_stream = bool(rospy.get_param('~h264_encoded_stream', False))

        # Height Information from EVENT_FLIGHT_DATA
        # That would be more accurate then Monocular Viausl SLAM
        self.height = 0

        self.cv_bridge = CvBridge()
        self.frame_thread = None
        self.EVENT_VIDEO_FRAME_H264 = event.Event('video frame h264')

        # Reconstruction H264 video frames
        self.prev_seq_id = None
        self.seq_block_count = 0

        self.log = RospyLogger('Tello')
        super(Tello_Node, self).__init__(port=9000)

        # Tello Connection
        rospy.loginfo('Connecting to drone {}'.format(self.tello_addr))
        self.connect()
        try:
            self.wait_for_connection(timeout=self.connect_time_out)
        except error.TelloError as err:
            rospy.logerr(str(err))
            rospy.signal_shutdown(str(err))
            self.quit()
            return
        rospy.loginfo('Connected to drone')
        rospy.on_shutdown(self.shutdown_callback)

        # Publihser for Tello Status
        self.odom_publisher = rospy.Publisher('odom', Odometry, queue_size=1, latch=True)
        self.status_publisher = rospy.Publisher('status', tello_status, queue_size=1, latch=True)
        # Publisher for image signal
        if self.h264_encoded_stream:
            self.image_publisher_h264 = rospy.Publisher('image_raw/h264', CompressedImage, queue_size=10)
        else:
            self.image_publisher_raw = rospy.Publisher('camera/image_raw', Image, queue_size=10)

        # Subscriber for control signal
        rospy.Subscriber('take_off', Empty, self.take_off_callback, queue_size=1)
        rospy.Subscriber('land', Empty, self.land_callback, queue_size=1)
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback, queue_size=1)

        # Video Stream
        if self.h264_encoded_stream:
            self.start_video()
            self.subscribe(self.EVENT_VIDEO_DATA, self.video_data_callback)
            self.subscribe(self.EVENT_VIDEO_FRAME_H264, self.h264_video_frame_callback)
        else:
            self.frame_thread = threading.Thread(target=self.frame_work_loop)
            self.frame_thread.start()

        self.subscribe(self.EVENT_LOG_DATA, self.log_data_callback)
        self.subscribe(self.EVENT_FLIGHT_DATA, self.flight_data_callback)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.camera_info = camera_info.loadCalibrationFile(self.calibration_path, 'camera_front')
        self.camera_info.header.frame_id = rospy.get_namespace() + 'camera_front'
        self.camera_info_publisher = rospy.Publisher('/tello/camera_info', CameraInfo, queue_size=1,  latch=True)
        self.camera_info_publisher.publish(self.camera_info)
        
        rospy.loginfo('The Tello driver node is ready')

    @staticmethod
    def cmd_success_notify(command:str, success:bool) -> None:
        """
        Inform the user whether the command is executed or failed
        Parameters
        ----------
            command(str): The command be executed expectly.
            success(bool): The flag that record the execution is success or not.
        """
        if success:
            rospy.loginfo('{} - executed.'.format(command))
        else:
            rospy.loginfo('{} - failed.'.format(command))

    # Callback function
    def cmd_vel_callback(self, msg:Twist) -> None:
        self.set_pitch(msg.linear.x)
        self.set_roll(-msg.linear.y)
        self.set_yaw(-msg.angular.z)
        self.set_throttle(msg.linear.z)

    def take_off_callback(self, msg:Empty) -> None:
        success = self.takeoff()
        self.cmd_success_notify('Take off', success)

    def land_callback(self, msg:Empty) -> None:
        success = self.land()
        self.cmd_success_notify('Land', success)

    def shutdown_callback(self) -> None:
        self.land()
        self.quit()
        if self.frame_thread is not None:
            self.frame_thread.join()

    def flight_data_callback(self, event, sender, data, **args):
        # THe fight data callback function is to publish the flying status of tello
        # The content is unfinished. 
        speed_horizontal_mps = math.sqrt(math.pow(data.north_speed, 2) + math.pow(data.east_speed, 2)) / 10.0
        self.height = data.height / 10.0
        msg = tello_status(
            height_m = data.height / 10.0,
            fight_time_s = data.fly_time / 10.0,
            speed_easting_mps = data.north_speed / 10.0,
            speed_northing_mps = -data.east_speed / 10.0,
            speed_vertival_mps = -data.ground_speed / 10.0,
            speed_horizontal_mps = speed_horizontal_mps,
        )
        self.status_publisher.publish(msg)

    def log_data_callback(self, event, sender, data, **args):
        now = rospy.Time.now()

        # if self.tf_broadcaster:
        #     tf = TransformStamped()
        #     tf.header.stamp = now
        #     tf.header.frame_id = 'base_link'
        #     tf.child_frame_id = 'tello_link'
        #     tf.transform.translation.x = 0.0
        #     tf.transform.translation.y = 0.0
        #     tf.transform.translation.z = self.height
        #     tf.transform.rotation.x = 0.0
        #     tf.transform.rotation.y = 0.0
        #     tf.transform.rotation.z = 0.0
        #     tf.transform.rotation.w = 1.0
        #     self.tf_broadcaster.sendTransform(tf)
                
    def video_data_callback(self, event, sender, data, **args):
        now = time.time()

        # Parsing the packet
        seq_id = data[0]
        sub_id = data[1]
        packet = data[2:]
        self.sub_last = False
        if sub_id >= 128:
            sub_id -= 128
            self.sub_last = True

        # Associate the packet to new frame
        if self.prev_seq_id is None or self.prev_seq_id != seq_id:
            # Dectect wrap-arounds
            if self.prev_seq_id is not None and self.prev_seq_id > seq_id:
                self.seq_block_count += 1
            
            self.frame_pkts = [None] * 128
            self.frame_time = now
            self.prev_seq_id = seq_id
        
        self.frame_pkts[sub_id] = packet

        # Publish frame if completed
        if self.sub_last and all(self.frame_pkts[:sub_id + 1]):
            if isinstance(self.frame_pkts[sub_id], str):
                frame = ''.join(self.frame_pkts[:sub_id + 1])
            else:
                frame = b''.join(self.frame_pkts[:sub_id + 1])

            self._Tello__publish(event=self.EVENT_VIDEO_FRAME_H264, data=(frame, self.seq_block_count * 256 + seq_id, self.frame_time))
    
    def h264_video_frame_callback(self, event, sender, data, **args):
        frame, seq_id, frame_secs = data
        pkt_msg = CompressedImage()
        pkt_msg.header.seq = seq_id
        pkt_msg.header.frame_id = 'tello/camera_front'
        pkt_msg.header.stamp = rospy.Time.from_sec(frame_secs)
        pkt_msg.data = frame
        self.image_publisher_h264.publish(pkt_msg)

    # Video threading
    def frame_work_loop(self) -> None:
        # Skip first 300 frame to avoid initial stream lag
        video_stream = self.get_video_stream()
        while self.state != self.STATE_QUIT:
            try:
                container = av.open(video_stream)
                break
            except BaseException as err:
                rospy.logerr('frame grab: pyav stream failed - {}'.format(str(err)))
                time.sleep(1.0)
        
        rospy.loginfo('Video stream Connected')
        while self.state != self.STATE_QUIT:
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

                    self.image_publisher_raw.publish(img_msg)
                break

            except BaseException as err:
                rospy.logerr('frame grab: pyav decoder failed - {}'.format(str(err)))
    
class RospyLogger(logger.Logger):
    def __init__(self, header=''):
        super(RospyLogger, self).__init__(header)

    def error(self, str):
        if self.log_level < logger.LOG_ERROR: return
        rospy.logerr(str)

    def warn(self, str):
        if self.log_level < logger.LOG_WARN: return
        rospy.logwarn(str)

    def info(self, str):
        if self.log_level < logger.LOG_INFO: return
        rospy.loginfo(str)

    def debug(self, str):
        if self.log_level < logger.LOG_DEBUG: return
        rospy.logdebug(str)

def main() -> None:
    rospy.init_node('tello_driver_node', anonymous=False)
    drone = Tello_Node()
    rospy.spin()

if __name__ == '__main__':
    main()