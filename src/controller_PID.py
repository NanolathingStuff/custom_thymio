#!/usr/bin/env python
import os
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Range
from math import pi
import math


NUM_LAPS = 20    # constant: number of laps
SPEED = 0.4 
TRESHOLD = 0.5    # constant: size lap beginning 
KP = 0.6        # proportional constant
KD = 0.404        # derivative constant 
KI = 0.1
MAX_TURN = 2.2
E_SENSITIVITY = 0.04

class ThymioController:

    def __init__(self):
        """Initialization."""

        # initialize the node
        rospy.init_node(
            'thymio_controller'  # name of the node
        )

        self.name = rospy.get_param('~robot_name')

        # log robot name to console
        rospy.loginfo('Controlling %s' % self.name)

        self.last_e = None
        self.sum_e = 0 # NEW

        # create velocity publisher
        self.velocity_publisher = rospy.Publisher(
            self.name + '/cmd_vel',  # name of the topic
            Twist,  # message type
            queue_size=10  # queue size
        )

        # create pose subscriber
        self.pose_subscriber = rospy.Subscriber(
            self.name + '/odom',  # name of the topic
            Odometry,  # message type
            self.log_odometry  # function that hanldes incoming messages
        )

        # tell ros to call stop when the program is terminated
        rospy.on_shutdown(self.stop)

        # initialize pose to (X=0, Y=0, theta=0)
        self.pose = Pose()

        # initialize linear and angular velocities to 0
        self.velocity = Twist()

        # set node update frequency in Hz
        self.rate = rospy.Rate(10)
        """
        /thymio10/proximity/center
        /thymio10/proximity/center_left
        /thymio10/proximity/center_right
        /thymio10/proximity/rear_left
        /thymio10/proximity/rear_right
        """
        self.directions = ['center_left', 'center', 'center_right']
        self.proximity = [1.0, 1.0, 1.0]

        self.distance_left = 1.0
        self.distance_right = 1.0
        #this give constant value (float) of the distance from the wheell l
        self.range_left = rospy.Subscriber(
            self.name + '/wheel_laser/left',  # name of the topic
            Range,  # message type
            self.common_callback_left
        )

        #this give constant value (float) of the distance from the wheell r
        self.range_right = rospy.Subscriber(
            self.name + '/wheel_laser/right',  # name of the topic
            Range,  # message type
            self.common_callback_right
        )
        #create proximity subscribers
        for direction in self.directions:
            rospy.Subscriber(
                self.name + '/proximity/' + direction,  # name of the topic
                Range,  # message type
                self.update_proximity,  # function that handles incoming messages
                callback_args=direction)

    def update_proximity(self, data, direction):
        index = self.directions.index(direction)
        self.proximity[index] = round(data.range, 3) / 0.12

    def common_callback_left(self,msg):

        input_message_type = str(msg._type)

        rospy.logdebug("msg._type ==>"+input_message_type)
        #rospy.loginfo("type(msg)"+str(type(msg)))

        if input_message_type == "sensor_msgs/Range":
            self.distance_left = msg.range

    def common_callback_right(self,msg):

        input_message_type = str(msg._type)

        rospy.logdebug("msg._type ==>"+input_message_type)
        #rospy.loginfo("type(msg)"+str(type(msg)))

        if input_message_type == "sensor_msgs/Range":
            #rospy.loginfo("sub_laserscan called me, with type msg==>"+input_message_type)
            self.distance_right = msg.range

    def human_readable_pose2d(self, pose):
        """Converts pose message to a human readable pose tuple."""

        # create a quaternion from the pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        # convert quaternion rotation to euler rotation
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        result = (
            pose.position.x,  # x position
            pose.position.y,  # y position
            yaw  # theta angle
        )

        return result

    def log_odometry(self, data):
        """Updates robot pose and velocities, and logs pose to console."""

        self.pose = data.pose.pose
        self.velocity = data.twist.twist

        printable_pose = self.human_readable_pose2d(self.pose)

        # log robot's pose
        rospy.loginfo_throttle(
            period=5,  # log every 10 seconds
            msg=self.name + ' (%.3f, %.3f, %.3f) ' % printable_pose  # message
        )

# from here is my code
    def calculate_distance(self, new_position, old_position):
        """Calculate the distance between two Points (positions)."""
        x2 = new_position.position.x
        x1 = old_position.position.x
        y2 = new_position.position.y
        y1 = old_position.position.y
        dist = math.hypot(x2 - x1, y2 - y1)
        return dist

    def write_report(self, lap, min_left_distance, min_right_distance, max_left_distance, max_right_distance, time):
        print("writing report")
        f = open("/home/usiusi/catkin_ws/src/custom_thymio/reports/PID_report.txt", 'a+') 
        f.write("lap{} concluded in time {} from the start \r\n".format(lap, time)) #modified
        f.write("Minimum distance left = {}, Maximum distance left = {} on lap{} \r\n".format(min_left_distance, max_left_distance, lap))
        f.write("Minimum distance right = {}, Maximum distance right = {} on lap{} \r\n".format(min_right_distance, max_right_distance, lap))
        f.close()
        rospy.loginfo("Lap "+str(lap) + ' / ' + str(NUM_LAPS))
    
    def stop(self):
        """Stops the robot."""
        self.velocity_publisher.publish(
            Twist()  # set velocities to 0
        )

        self.rate.sleep()
    
    def controller_commands(self):
        e = self.distance_left - self.distance_right
        if abs(e) <= E_SENSITIVITY:
            e = 0
        dt = 1/10 # 1 / rate

        if(self.last_e is not None):
            derivative = (e - self.last_e) / dt
        else:
            derivative = 0
        self.last_e = e
        self.sum_e += e * dt # NEW
        angle = KP * e + KD * derivative + KI * self.sum_e
        if angle > MAX_TURN:
            angle = MAX_TURN
        if angle < -MAX_TURN:
            angle = -MAX_TURN
        #rospy.loginfo("error: " + str(e) + " turning: " + str(angle) + "/" + str(KP * e + KD * derivative + KI * self.sum_e))
        velocity = Twist(linear=Vector3(
                SPEED,  
                .0,
                .0,
                ),angular=Vector3(
                .0,
                .0,
                angle))

        return velocity

    def run(self):
        f = open("/home/usiusi/catkin_ws/src/custom_thymio/reports/PID_report.txt", 'a+') # "w+")
        f.write("using speed = {}; KP = {}, KI = {}, KD = {}; sensitivity = {}; maximum turn = {} \r\n".format(
            SPEED, KP, KI, KD, E_SENSITIVITY, MAX_TURN)) 
        f.close()
        # defining starting point to detect lap
        start_pose = self.pose
        start_range = False
        """Controls the Thymio."""
        min_left_distance = min_right_distance = 50.0 # i don't know how to get max range of the sensor, so i harcode it here
        max_left_distance = max_right_distance = 0.0
        start = rospy.get_rostime().secs    # take jus seconds
        
        # this should stop the robot and prevent it to start running before spawning
        while start < 1:
            start = rospy.get_rostime().secs
        start = start -1
        # counter of how many times the loop has been executed
        slept = lap = 0 
        while not rospy.is_shutdown() and lap <= NUM_LAPS:
            #detect crashes
            for proximity in self.proximity:
                if proximity < 0.4: #sensor result during crashes around 0.28
                    f = open("/home/usiusi/catkin_ws/src/custom_thymio/reports/PID_report.txt", 'a+') 
                    f.write("CRASHED! \r\n")
                    f.close()
                    self.stop()
                    rospy.signal_shutdown("unexpected crash")
                    break;

            if self.calculate_distance(self.pose, start_pose) < TRESHOLD:
                """if near starting point: write report once and reset"""
                if not start_range:
                    start_range = True 
                    self.write_report(lap, min_left_distance, min_right_distance, max_left_distance, max_right_distance, rospy.get_rostime().secs - start)
                    lap += 1
                    min_left_distance = min_right_distance = 50.0 # i don't know how to get max range of the sensor, so i harcode it here
                    max_left_distance = max_right_distance = 0.0         
            else:
                """activate write_report"""
                start_range = False  

            velocity = self.controller_commands()

            # publish velocity message
            self.velocity_publisher.publish(velocity)
            
            # print rospy.loginfo("distance==>"+str(self.distance))
            #rospy.loginfo("left: " + str(self.distance_left) + " right: " + str(self.distance_right))
            #rospy.loginfo("left: " + str(min_left_distance) + ' ; ' + str(max_left_distance) + " right: " + str(min_right_distance) +' ; ' + str(max_right_distance))
            #rospy.loginfo("lap: " + str(lap) + " Pose: " + self.name + ' (%.3f, %.3f, %.3f) ' % self.human_readable_pose2d(self.pose))
            #rospy.loginfo("lap: " + str(lap) + " distances: " + str(self.proximity)[1:-1])
            #rospy.loginfo("lap: " + str(lap) + " time: " + str(rospy.get_rostime().secs - start) + ' of ' + str(rospy.get_time()) + " from: " + str(start))

            # calcumlate max and mins
            if self.distance_left > max_left_distance:
                max_left_distance = self.distance_left
            if self.distance_left < min_left_distance:
                min_left_distance = self.distance_left
            if self.distance_right > max_right_distance:
                max_right_distance = self.distance_right
            if self.distance_right < min_right_distance:
                min_right_distance = self.distance_right
            
            # sleep until next step
            self.rate.sleep()
        # out of the loop
        self.write_report(lap, min_left_distance, min_right_distance, max_left_distance, max_right_distance, rospy.get_rostime().secs - start)
        f = open("/home/usiusi/catkin_ws/src/custom_thymio/reports/PID_report.txt", 'a+') 
        f.write(" \r\n")
        f.close()
        print("number of setted laps ({}) reached, press CTRL + C to exit".format(NUM_LAPS))

if __name__ == '__main__':
    # create report folder
    if not os.path.exists('/home/usiusi/catkin_ws/src/custom_thymio/reports'):
        os.mkdir('/home/usiusi/catkin_ws/src/custom_thymio/reports', mode= 0o777)
    controller = ThymioController()

    try:
        controller.run()
    except rospy.ROSInterruptException as e:
        pass
