#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Range

"""
controller tougth to work on the circuit3 in launch/worlds folder
roslaunch custom_thymio racing_thymio_gazebo.launch name:=thymio10 world:=circuit3 type:=controller_open_loop.py
"""

NUM_LAPS = 4

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

        self.distance_left = 0.0
        self.distance_right = 0.0
        #this give constant value (float) of the distance from wheels
        self.range_left = rospy.Subscriber(
            self.name + '/wheel_laser/left',  # name of the topic
            Range,  # message type
            self.common_callback_left
        )


        self.range_right = rospy.Subscriber(
            self.name + '/wheel_laser/right',  # name of the topic
            Range,  # message type
            self.common_callback_right
        )

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

    def write_report(self, lap, min_left_distance, min_right_distance, max_left_distance, max_right_distance):
        print("writing report")
        f = open("/home/usiusi/catkin_ws/src/custom_thymio/reports/open_loop_report.txt", 'a+') # "w+")
        f.write("Minimum distance left = {}, Maximum distance left = {} on lap{} \r\n".format(min_left_distance, max_left_distance, lap))
        f.write("Minimum distance right = {}, Maximum distance right = {} on lap{} \r\n".format(min_right_distance, max_right_distance, lap))
        f.close()
    
    def stop(self):
        """Stops the robot."""
        self.velocity_publisher.publish(
            Twist()  # set velocities to 0
        )

        self.rate.sleep()

    def update(self, slept):
        """update velocity based on the counter inside the loop"""
        # if not in list, go straight
        velocity = Twist(linear=Vector3(
                .4,  
                .0,
                .0,
                ),angular=Vector3(
                .0,
                .0,
                .0))
        if slept >= 0 and slept < 55:
            velocity = Twist(linear=Vector3(0.4, 0.0, 0.0,),
                angular=Vector3(0.0, 0.0, 0.0))
        if slept >= 55 and slept < 80:
            velocity = Twist(linear=Vector3(0.4, 0.0, 0.0,),
                angular=Vector3(0.0, 0.0, -0.5))

        if slept >= 135 and slept < 147:
            velocity = Twist(linear=Vector3(0.4, 0.0, 0.0,),
                angular=Vector3(0.0, 0.0, -0.3))            
        if slept >= 147 and slept < 190:
            velocity = Twist(linear=Vector3(0.4, 0.0, 0.0,),
                angular=Vector3(0.0, 0.0, -0.0))
        if slept >= 190 and slept < 240:
            velocity = Twist(linear=Vector3(0.4, 0.0, 0.0,),
                angular=Vector3(0.0, 0.0, -0.3))   

        if slept >= 390 and slept < 420:
            velocity = Twist(linear=Vector3(0.4, 0.0, 0.0,),
                angular=Vector3(0.0, 0.0, -0.5)) 
        if slept >= 440 and slept < 470:
            velocity = Twist(linear=Vector3(0.4, 0.0, 0.0,),
                angular=Vector3(0.0, 0.0, +0.1))
        if slept >= 470 and slept < 515:
            velocity = Twist(linear=Vector3(0.4, 0.0, 0.0,),
                angular=Vector3(0.0, 0.0, -0.0))
        if slept >= 515 and slept < 560:
            velocity = Twist(linear=Vector3(0.4, 0.0, 0.0,),
                angular=Vector3(0.0, 0.0, -0.4))
        if slept >= 670 and slept < 675:
            velocity = Twist(linear=Vector3(0.4, 0.0, 0.0,),
                angular=Vector3(0.0, 0.0, -0.2))

        return velocity

    def run(self):
        """Controls the Thymio."""
        min_left_distance = min_right_distance = 50.0 # i don't know how to get max range of the sensor, so i harcode it here
        max_left_distance = max_right_distance = 0.0

        # counter of how many times the loop has been executed
        slept = 0 
        lap = 1
        while not rospy.is_shutdown() and lap < NUM_LAPS:
            # new lap
            if slept > 675:
                self.write_report(lap, min_left_distance, min_right_distance, max_left_distance, max_right_distance)
                slept = 0
                lap += 1
                print(NUM_LAPS)
                # reset min,max at every lap
                min_left_distance = min_right_distance = 50.0 # i don't know how to get max range of the sensor, so i harcode it here
                max_left_distance = max_right_distance = 0.0


            # decide control action
            velocity = self.update(slept)

            # publish velocity message
            self.velocity_publisher.publish(velocity)
            
            # print rospy.loginfo("distance==>"+str(self.distance))
            #rospy.loginfo("left: " + str(self.distance_left) + " right: " + str(self.distance_right))
            #rospy.loginfo("left: " + str(min_left_distance) + ' ; ' + str(max_left_distance) + " right: " + str(min_right_distance) +' ; ' + str(max_right_distance))
            #rospy.loginfo("loop number: " + str(slept) + " lap: " + str(lap))

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

            slept += 1

        self.write_report(lap, min_left_distance, min_right_distance, max_left_distance, max_right_distance)
        f = open("/home/usiusi/catkin_ws/src/custom_thymio/reports/open_loop_report.txt", 'a+') 
        f.write(" \r\n")
        f.close()
        print("number of setted laps ({}) reached, press CTRL + C to exit".format(NUM_LAPS))


if __name__ == '__main__':
    controller = ThymioController()

    try:
        controller.run()
    except rospy.ROSInterruptException as e:
        pass