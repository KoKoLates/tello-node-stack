#! /usr/bin/env python3

import threading

import rospy
import sys, select, termios, tty

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

move_bindings = {
    'w':( 0, 0, 1, 0),  # Up
    's':( 0, 0,-1, 0),  # Down
    'a':( 0, 0, 0, 1),  # Yaw Left
    'd':( 0, 0, 0, -1),  # Yaw Right

    'i':( 1, 0, 0, 0),  # Front
    'k':(-1, 0, 0, 0),  # Back
    'j':( 0, 1, 0, 0),  # Left
    'l':( 0, -1, 0, 0),  # Right
}

trigger_bindings = {
    '+': -1,  # Take off
    '-': -2,  # Land
}

class Publish_Threading(threading.Thread):
    def __init__(self, rate:float) -> None:
        super(Publish_Threading, self).__init__()
        self.publisher = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)

        self.x, self.y, self.z, self.theta, self.speed = 0.0, 0.0, 0.0, 0.0, 0.0
        self.condition = threading.Condition()
        self.flag = False

        if rate != 0.0:
            self.time_out = 1.0 / rate
        else: 
            self.time_out = None
            # Set time out to be None as the rate is 0
            # Avoid unlimited waiting for new data to publish

        self.start()

    def wait_for_subscriber(self) -> None:
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                rospy.loginfo('Waiting for subscriber to connect to {}'.format(self.publisher.name))
                rospy.sleep(0.5)
                i += 1
                i %= 5

        if rospy.is_shutdown():
            rospy.logerr('Got shutdown request before subscribers connected')

    def update(self, x:float, y:float, z:float, theta:float, speed:float) -> None:
        self.condition.acquire()
        self.x, self.y, self.z, self.theta, self.speed = x, y, z, theta, speed
        self.condition.notify() # Notify publish thread that a new message obtained
        self.condition.release()

    def run(self) -> None:
        twist = Twist()
        while not self.flag:
            self.condition.acquire()
            self.condition.wait(self.time_out) # Wait for a new message or timeout

            # Twist message
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x, twist.angular.y = 0, 0
            twist.angular.z = self.theta

            self.condition.release()

            # Publish
            self.publisher.publish(twist)
        
        # Publish stop message as the thread out
        twist.linear.x, twist.linear.y, twist.linear.z = 0, 0, 0
        twist.angular.x, twist.angular.y, twist.angular.z = 0, 0, 0
        self.publisher.publish(twist)

    def stop(self) -> None:
        self.flag = True
        self.update(0.0, 0.0, 0.0, 0.0, 0.0)
        self.join()

    @staticmethod
    def getKey(settings:list, timeout) -> str:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key 
   


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('keyboard_teleop_node')
    land_publisher = rospy.Publisher('/tello/land', Empty, queue_size=1)
    take_off_publisher = rospy.Publisher('/tello/take_off', Empty, queue_size=1)

    # Get paramter sever
    speed_scale = float(rospy.get_param('~speed', 1.0))
    repeat = float(rospy.get_param('~repeat_rate', 0.0))
    timeout = float(rospy.get_param('~key_timeout', 0.0))
    if timeout == 0.0:
        timeout = None

    publisher_thread = Publish_Threading(repeat)
    x, y, z, theta = 0, 0, 0, 0

    try:
        
        publisher_thread.wait_for_subscriber()
        publisher_thread.update(x, y, z, theta, speed_scale)
        while True:
            key = publisher_thread.getKey(settings, timeout)
            if key in move_bindings.keys():
                x = move_bindings[key][0]
                y = move_bindings[key][1]
                z = move_bindings[key][2]
                theta = move_bindings[key][3]

            elif key in trigger_bindings.keys():
                empty_msgs = Empty()
                twist_msgs = Twist()
                twist_msgs.linear.x, twist_msgs.linear.y, twist_msgs.linear.z = 0, 0, 0
                twist_msgs.angular.x, twist_msgs.angular.y, twist_msgs.angular.z = 0, 0, 0

                if trigger_bindings[key] == -1:
                    publisher_thread.publisher.publish(twist_msgs)
                    take_off_publisher.publish(empty_msgs)

                elif trigger_bindings[key] == -2:
                    publisher_thread.publisher.publish(twist_msgs)
                    land_publisher.publish(empty_msgs)
                else:
                    rospy.logerr('Command error: No such trigger command')
            
            else:
                if key == '' and x == 0 and y == 0 and z == 0 and theta == 0:
                    continue

                x, y, z, theta = 0, 0, 0, 0
                if key == '\x03': # <ctrl-c> ascii for exit
                    break
            
            publisher_thread.update(x, y, z, theta, speed_scale)
            rospy.loginfo('x:{} y:{} z:{} theta:{}'.format(x, y, z, theta))
    
    except Exception as exception:
        rospy.logerr(exception)

    finally:
        publisher_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

