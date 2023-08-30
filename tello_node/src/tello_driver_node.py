#! /usr/bin/env python3
import numpy as np, yaml
import av, math, time, threading, rospy

from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

from tello_node.msg import tello_status
from tellopy._internal import tello, error, event



class Tello_Node(tello.Tello):
    def __init__(self) -> None:
        super(Tello_Node, self).__init__(port=9000)
        """ Parameter Servers """
        self.tellop_IP = rospy.get_param('~tello_ip', '192.168.10.1')
        self.tello_cmd_server_port = int(rospy.get_param('~tello_cmd_server_port', 8889))
        self.local_cmd_client_port = int(rospy.get_param('~local_cmd_client_port', 8890))
        self.local_vid_server_port = int(rospy.get_param('~local_vid_server_port', 6038))

        self.initial_frame = int(rospy.get_param('~initial_frame', 100))
        self.connect_time_out =rospy.get_param('~connect_time_out', 10.0)
        self.calibration_path = rospy.get_param('~camera_calibration', '')
        self.h264_encoded_stream = bool(rospy.get_param('~h264_encoded_stream', False))

        """ Connected to the drone """
        rospy.loginfo('Connecting to drone @ {}...'.format(self.tello_addr))
        self.connect()
        try:
            self.wait_for_connection(timeout=self.connect_time_out)
        except error.TelloError as err:
            rospy.logerr(str(err))
            rospy.signal_shutdown(str(err))
            self.quit()
            return
        rospy.loginfo('Tello drone connection done.')
        rospy.on_shutdown(self.shutdown_callback)


        """ Publisher Topics """
        self.odom_publisher = rospy.Publisher('odom', Odometry, queue_size=1, latch=True)
        self.status_publisher = rospy.Publisher('status', tello_status, queue_size=1, latch=True)
        # Image publisher
        if self.h264_encoded_stream:
            self.image_publisher_h264 = rospy.Publisher('image_raw/h264', CompressedImage, queue_size=10)
        else:
            self.image_publisher_raw = rospy.Publisher('camera/image_raw', Image, queue_size=10)
        
        # Camera Information
        self.camera_info = Tello_Node.loadCalibrationFile(self.calibration_path, 'camera_front')
        self.camera_info.header.frame_id = rospy.get_namespace() + 'camera_front'
        self.camera_info_publisher = rospy.Publisher('/tello/camera_info', CameraInfo, queue_size=1, latch=True)
        self.camera_info_publisher.publish(self.camera_info)

        """ Subscribe Topics"""
        rospy.Subscriber('take_off', Empty, self.take_off_callback, queue_size=1)
        rospy.Subscriber('land', Empty, self.land_callback, queue_size=1)
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback, queue_size=1)


        """ Events Handler """
        self.frame_thread, self.status_count = None, 0 
        self.EVENT_VIDEO_FRAME_H264 = event.Event('video frame h264')
        if self.h264_encoded_stream:
            self.start_video()
            self.subscribe(self.EVENT_VIDEO_DATA, self.video_data_callback)
            self.subscribe(self.EVENT_VIDEO_FRAME_H264, self.h264_video_frame_callback)
        else:
            self.frame_thread = threading.Thread(target=self.frame_work_loop)
            self.frame_thread.start()

        self.subscribe(self.EVENT_LOG_DATA, self.log_data_callback)
        self.subscribe(self.EVENT_FLIGHT_DATA, self.flight_data_callback)


        """ Member fields """
        # Height Information from EVENT_FLIGHT_DATA that obtain from IR sensor of Tello 
        # That would be more accurate then Monocular Viausl SLAM
        self.height = 0       

        # Reconstruction H264 video frames
        self.prev_seq_id = None
        self.seq_block_count = 0

        
        rospy.loginfo('The Tello driver node is ready')

    @staticmethod
    def cmd_success_notify(command:str, success:bool) -> None:
        """
        Inform of command execution success
        @param command: The command to be executed expectly
        @param success: The flag that record the execution is success or not
        """
        rospy.loginfo(f'{command} - executed.' if success else f'{command} - failed.')

    @staticmethod
    def loadCalibrationFile(filename: str, cname: str) -> CameraInfo:
        """ 
        Load the camera calibrate weights from a file.
        This function returns a `sensor_msgs/CameraInfo` message, based
        on the filename parameter.  An empty or non-existent file is `not`
        considered an error; a null CameraInfo being provided in that case.

        @param filename: location of CameraInfo to read
        @param cname: Camera name.
        @returns: `sensor_msgs/CameraInfo`_ message containing calibration, if file readable; null calibration message otherwise.
        @rtype: CameraInfo
        @raises: :exc:`IOError` if an existing calibration file is unreadable.
        """
        camera_info = CameraInfo()
        try:
            f = open(filename)
            calib = yaml.load(f, Loader=yaml.FullLoader)
            if calib is not None:
                if calib['camera_name'] != cname:
                    rospy.logwarn("[" + cname + "] does not match name " + calib['camera_name'] + " in file " + filename)

                # fill in CameraInfo fields
                camera_info.width = calib['image_width']
                camera_info.height = calib['image_height']
                camera_info.distortion_model = calib['distortion_model']
                camera_info.D = calib['distortion_coefficients']['data']
                camera_info.K = calib['camera_matrix']['data']
                camera_info.R = calib['rectification_matrix']['data']
                camera_info.P = calib['projection_matrix']['data']

        except IOError:
            pass

        return camera_info

    """ Callback functions """
    def cmd_vel_callback(self, msg:Twist) -> None:
        self.set_pitch(msg.linear.x)
        self.set_roll(msg.linear.y)
        self.set_yaw(msg.angular.z)
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

    def flight_data_callback(self, event, sender, data, **args) -> None:
        """
        Callback function for flight data 
        @param event: `EVENT_FLIGHT_DATA`
        @param data: flight data
        """
        horizontal_speed = math.sqrt(math.pow(data.north_speed, 2) + math.pow(data.east_speed, 2)) / 10.0
        self.height = data.height / 10.0
        msg = tello_status(
            height = self.height,
            # Notes that the x, y, z in Tello is different in ROS
            # Positive x is forward, Positive y is right
            x_speed = data.north_speed / 10.0, 
            y_speed = data.east_speed / 10.0,
            horizontal_speed = horizontal_speed, 
            vertical_speed = -data.ground_speed / 10.0,
            flight_time = data.fly_time / 10.0,
            battery_percentage = data.battery_percentage, 
            is_flying = data.em_sky, 
        )
        self.status_publisher.publish(msg)

        if self.status_count == 100:
            rospy.loginfo('Battery Percentage: {}%.' .format(data.battery_percentage))
            self.status_count = 0
        else:
            self.status_count += 1

    def log_data_callback(self, event, sender, data, **args) -> None:
        """
        Callback function for Odometry data that record in Tello
        This function is to transform the data into nav_msgs.msg of ROS
        @param event: `EVENT_LOG_DATA`
        @param data: Odometry data of position and velocity in three dimension
        """
        msg = Odometry()
        msg.header.frame_id = rospy.get_namespace() + 'local_origin'
        msg.header.stamp = rospy.Time.now()
        msg.child_frame_id = rospy.get_namespace() + 'base_link'
        
        # The odometry sensor are IMU and Monocular Visual Odometry in tello
        msg.pose.pose.position.z = data.mvo.pos_z
        msg.pose.pose.position.x = data.mvo.pos_x
        msg.pose.pose.position.y =-data.mvo.pos_y
        msg.pose.pose.orientation.w = data.imu.q0
        msg.pose.pose.orientation.x = data.imu.q1
        msg.pose.pose.orientation.y = data.imu.q2
        msg.pose.pose.orientation.z = data.imu.q3
        # Linear speeds from MVO received in dm/sec
        msg.twist.twist.linear.x = data.mvo.vel_y / 10
        msg.twist.twist.linear.y = data.mvo.vel_x / 10
        msg.twist.twist.linear.z =-data.mvo.vel_z / 10
        msg.twist.twist.angular.x = data.imu.gyro_x
        msg.twist.twist.angular.y = data.imu.gyro_y
        msg.twist.twist.angular.z = data.imu.gyro_z
        self.odom_publisher.publish(msg)

                
    def video_data_callback(self, event, sender, data, **args) -> None:
        """ 
        Callback function for video data to parse it into compression datatype
        @param event: `EVENT_VIDEO_DATA`
        @param data: video data in byte list type
        """
        # Parsing the packet
        seq_id, sub_id, packet = data[0], data[1], data[2:]
        self.sub_last = False
        if sub_id >= 128: # Most Significant Bit assert 
            sub_id -= 128
            self.sub_last = True

        # Associate the packet to new frame
        if self.prev_seq_id is None or self.prev_seq_id != seq_id:
            # Dectect wrap-arounds
            if self.prev_seq_id is not None and self.prev_seq_id > seq_id:
                self.seq_block_count += 1
            
            self.frame_pkts = [None] * 128
            self.frame_time = time.time()
            self.prev_seq_id = seq_id
        
        self.frame_pkts[sub_id] = packet

        # Publish frame if completed
        if self.sub_last and all(self.frame_pkts[:sub_id + 1]):
            if isinstance(self.frame_pkts[sub_id], str):
                frame = ''.join(self.frame_pkts[:sub_id + 1])
            else:
                frame = b''.join(self.frame_pkts[:sub_id + 1])

            self._Tello__publish(
                event = self.EVENT_VIDEO_FRAME_H264, 
                data = (frame, self.seq_block_count * 256 + seq_id, self.frame_time))
    
    def h264_video_frame_callback(self, event, sender, data, **args) -> None:
        """
        Callback function for h264 video to publish corresponding topic
        @param event: `EVENT_VIDEO_FRAME_H264`
        @data: pre-process video data
        """
        frame, seq_id, frame_secs = data
        pkt_msg = CompressedImage()
        pkt_msg.header.seq = seq_id
        pkt_msg.header.frame_id = self.camera_info.header.frame_id
        pkt_msg.header.stamp = rospy.Time.from_sec(frame_secs)
        pkt_msg.data = frame
        self.image_publisher_h264.publish(pkt_msg)

    """ Video Streaming function """
    def frame_work_loop(self) -> None:
        # Skip first 100 frame to avoid initial stream lag
        video_stream = self.get_video_stream()
        while self.state != self.STATE_QUIT:
            try:
                container = av.open(video_stream)
                break
            except BaseException as err:
                rospy.logerr('[ERROR] frame grab: pyav stream failed - {}'.format(str(err)))
                time.sleep(1.0)
        
        rospy.loginfo('[INFO] Video stream Connected')
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
                        img_msg = CvBridge().cv2_to_imgmsg(img, 'rgb8')
                        img_msg.header.frame_id = rospy.get_namespace()
                    except CvBridgeError as err:
                        rospy.logerr('[ERROR] frame grab: cv bridge failed - {}'.format(str(err)))
                        continue

                    self.image_publisher_raw.publish(img_msg)
                break

            except BaseException as err:
                rospy.logerr('[ERROR] frame grab: pyav decoder failed - {}'.format(str(err)))


if __name__ == '__main__':
    rospy.init_node('tello_driver_node', anonymous=False)
    drone = Tello_Node()
    if drone.state != drone.STATE_QUIT:
        rospy.spin()

