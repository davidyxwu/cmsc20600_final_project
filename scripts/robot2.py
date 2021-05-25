#!/usr/bin/env python3

import rospy, cv2,  cv_bridge
import numpy as np
import keras_ocr
import moveit_commander
import math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# HSV color ranges for RGB camera
# [lower range, upper range]
# https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html
HSV_COLOR_RANGES = {
        "blue" : [np.array([110, 100, 100]), np.array([130, 255, 255])],
        "green" : [np.array([50, 100, 100]), np.array([70, 255, 255])],
        "red" : [np.array([0, 100, 100]), np.array([15, 255, 255])]
        }


# List of possible states
STOP = 'stop' # robot not doing anything, either waiting for action or done
# Go to dumbell color
DUMBBELL = 'dumbbell'
# Gripper actions
PICKUP = 'pickup'
DROP = 'drop'

class Robot2(object):

     def __init__(self):
        self.initialized = False
        # init rospy node
        rospy.init_node("robot2")

        # set up ROS and CV bridge
        self.bridge = cv_bridge.CvBridge()

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('robot2/camera/rgb/image_raw',
                        Image, self.image_callback)

        # subscribe to robot's laser scan
        rospy.Subscriber("robot2/scan", LaserScan, self.scan_callback)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Set image, hsv, scan data to be NONE for now
        self.image = None
        self.hsv = None
        self.laserscan = None
        self.laserscan_front = None
        self.color = "blue"

        # Set up publisher for movement
        self.cmd_vel_pub = rospy.Publisher('robot2/cmd_vel',
                        Twist, queue_size=1)

        self.twist = Twist()

        # Set gripper to initial starting point
        self.reset_gripper()

        self.initialized = True

        # tell dispatch_actions node to start publish actions
        rospy.sleep(1)

    """Callback for lidar scan"""
    def scan_callback(self, data):
        if not self.initialized:
            return
        self.laserscan = data.ranges
        front = []
        for i in range(350, 375):
            front.append(self.laserscan[i % 360])
        self.laserscan_front = front

    """Callback for images"""
    def image_callback(self, data):
        if not self.initialized:
            return
        # converts the incoming ROS message to cv2 format and HSV (hue, saturation, value)
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

    """Publish movement"""
    def pub_cmd_vel(self, lin, ang):
        self.twist.linear.x = lin
        self.twist.angular.z = ang
        self.cmd_vel_pub.publish(self.twist)

        """Starting gripper setup to pickup"""
    def reset_gripper(self):
        arm_joint_goal = [0.0, 0.7, -0.260, -0.450]
        gripper_joint_goal = [0.01, 0.01]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
    
    """Gripper with dumbell"""
    def pick_up_gripper(self):
        arm_joint_goal = [0.0, 0.10, -0.5, -0.2]
        gripper_joint_goal = [0.004, 0.004]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_gripper.go(gripper_joint_goal, wait=True) 
        self.move_group_arm.stop()
        self.move_group_gripper.stop()
        # Move back away from dumbells
        init_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - init_time < 1.0:
            self.pub_cmd_vel(-0.2, 0)
        self.pub_cmd_vel(0,0)
        

    """Drop dumbell from gripper"""
    def drop_gripper(self):
        arm_joint_goal = [0.0, 0.43, 0.48, -0.92]
        gripper_joint_goal = [0.01, 0.01]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
        
        init_time = rospy.Time.now().to_sec()
        # move back away from dumbbell
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - init_time < 3.0:
            self.pub_cmd_vel(-0.2, 0)
        self.pub_cmd_vel(0,0)
        self.reset_gripper() 

        """Move to dumbell of particular color, states = GREEN, BLUE, RED"""
    def move_to_dumbell(self):
        if not self.initialized or self.hsv is None or self.image is None or self.laserscan is None:
            return

        # Mask
        mask = cv2.inRange(self.hsv, HSV_COLOR_RANGES[self.color][0], HSV_COLOR_RANGES[self.color][1])

        # Dimensions
        h, w, d = self.image.shape

        # front distance
        dist = min(self.laserscan_front)

        # determine center of color pixels
        M = cv2.moments(mask)
        # if the target color is in sight
        if M['m00'] > 0:
            # determine the center of the color pixels in the image
            cx = int(M['m10']/M['m00'])
            err = w/2 - cx # error
            k_p = 1.0 / 1000.0 # angular proportional control
            lin_k = 0.3 # linear proportional control
            # Time to pickup and change state
            if dist <= 0.22:
                self.state = PICKUP
                self.pub_cmd_vel(0, 0)
                return
            # All proportional control
            if dist >= 0.5:
                lin = 0.2
            else:
                linerr = dist - 0.22
                lin = linerr * lin_k
            ang = k_p * err
            self.pub_cmd_vel(lin, ang)
        else:
            # spin until we see image
            self.pub_cmd_vel(0, 0.2)

