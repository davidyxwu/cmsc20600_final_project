#!/usr/bin/env python3

import rospy, cv2,  cv_bridge
import numpy as np
from game import PLAYER_RED, PLAYER_BLUE
import moveit_commander
import math
import actionlib
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point, PoseWithCovariance, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from cmsc20600_final_project.msg import Action, RobotInitialized
from actionlib_msgs.msg import *


# HSV color ranges for RGB camera
# [lower range, upper range]
# https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html
HSV_COLOR_RANGES = {
        "blue" : [np.array([110, 100, 100]), np.array([130, 255, 255])],
        "green" : [np.array([50, 100, 100]), np.array([70, 255, 255])],
        "red" : [np.array([0, 100, 100]), np.array([15, 255, 255])]
        }


# Grid locations numbered 0-8, 'home base' locations for red and blue player
GRID_LOCATIONS = {
        "0" : (2, 2),
        "1" : (2, 0),
        "2" : (2, -2),
        "3" : (0, 2),
        "4" : (0, 0),
        "5" : (0, -2),
        "6" : (-2, 2),
        "7" : (-2, 0),
        "8" : (-2, -2),
        "red" : (3, 0),
        "blue" : (-3, 0)
        }

# List of possible states
STOP = 'stop' # robot not doing anything, either waiting for action or done
# Go to dumbell color
DUMBBELL = 'dumbbell'
# Gripper actions
PICKUP = 'pickup'
DROP = 'drop'
# Map locations
ZERO = '0'
ONE = '1'
TWO = '2'
THREE = '3'
FOUR = '4'
FIVE = '5'
SIX = '6'
SEVEN = '7'
EIGHT = '8'
RED = 'red'
BLUE = 'blue'

"""This class implements the one robot implementation of dumbbell placing in tictactoe grid"""
class OneRobot(object):

    def __init__(self):
        self.initialized = False
        # init rospy node
        rospy.init_node("one_robot_action")

        # set up ROS and CV bridge
        self.bridge = cv_bridge.CvBridge()

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',
                        Image, self.image_callback)

        rospy.sleep(1)
        # subscribe to robot's laser scan
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        print("set up movegroup")
        # Set image, hsv, scan data to be NONE for now
        self.image = None
        self.hsv = None
        self.laserscan = None
        self.laserscan_front = None
        self.color = "blue" # keep track of last player (start with blue since red goes first)
        self.state = STOP

        # Store actions in lists as they are recieved in the callback
        self.red_actions = [] # red actions
        self.blue_actions = [] # blue actions

        # Current action
        self.action_in_progress = None

        # Set up publisher for movement
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                        Twist, queue_size=1)

        self.twist = Twist()

        # Set up publisher for initial pose
        self.init_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        rospy.sleep(1)
        self.init_pose()

        # Set up subscribers for robot actions
        rospy.Subscriber("/tictactoe/red_action", Action, self.red_action_callback)
        rospy.Subscriber("/tictactoe/blue_action", Action, self.blue_action_callback)

        # Set gripper to initial starting point
        self.reset_gripper()

        # initialize node status publisher
        self.node_status_pub = rospy.Publisher("/tictactoe/node_status", RobotInitialized, queue_size=10)
        rospy.sleep(1)
        self.initialized = True

        # tell dispatch_actions node to start publish actions
        print("Robot initialized!")
        self.node_status_pub.publish(RobotInitialized(player="both", node_initialized=True))

    """Publish initial pose for robot"""
    def init_pose(self):
        # Set up pose msg and send to /initialpose topic
        init_pose = PoseWithCovariance()
        init_pose.pose.position.x = GRID_LOCATIONS["red"][0]
        init_pose.pose.position.y = GRID_LOCATIONS["red"][1]
        init_pose.pose.position.z = 0.0

        init_pose.pose.orientation.x = 0.0
        init_pose.pose.orientation.y = 0.0
        init_pose.pose.orientation.z = 0
        init_pose.pose.orientation.w = 1.0
        # covariance of particle cloud
        covariance = [0.16575166048810708, 0.005812119956448508, 0.0, 0.0, 0.0, 0.0,
                0.005812119956448534, 0.163246490374612, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05704654800158645]
        init_pose.covariance = covariance
        init_pose_stamped = PoseWithCovarianceStamped()
        init_pose_stamped.pose = init_pose
        init_pose_stamped.header.frame_id = "map"
        self.init_pose_pub.publish(init_pose_stamped)

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

    """Callback for red actions"""
    def red_action_callback(self, data):
        if not self.initialized:
            return
        print("Recieved red action")
        #if data is not None:
        self.red_actions.append(data)

    """Callback for blue actions"""
    def blue_action_callback(self, data):
        if not self.initialized:
            return
        print("Received blue action")
        #if data is not None:
        self.blue_actions.append(data)

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
        #self.move_back() # move_base moves back automatically for us
        self.pub_cmd_vel(0,0)
        # Determine which grid position to go to next
        grid = self.action_in_progress.position
        if grid == 0:
            self.state = ZERO
        elif grid == 1:
            self.state = ONE
        elif grid == 2:
            self.state = TWO
        elif grid == 3:
            self.state = THREE
        elif grid == 4:
            self.state = FOUR
        elif grid == 5:
            self.state = FIVE
        elif grid == 6:
            self.state = SIX
        elif grid == 7:
            self.state = SEVEN
        elif grid == 8:
            self.state = EIGHT



    """Drop dumbell from gripper"""
    def drop_gripper(self):
        arm_joint_goal = [0.0, 0.43, 0.48, -0.92]
        gripper_joint_goal = [0.01, 0.01]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
        self.move_back()
        self.pub_cmd_vel(0,0)
        self.reset_gripper()
        # Change state to stop to continue action list
        self.state = STOP

    """Move back slowly"""
    def move_back(self):
        # move back away from dumbbell
        init_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - init_time < 2.0:
            self.pub_cmd_vel(-0.3, 0)
        self.pub_cmd_vel(0, 0)


    """Move to dumbell of particular color, states = GREEN, BLUE, RED"""
    def move_to_dumbell(self):
        if not self.initialized or self.hsv is None or self.image is None or self.laserscan is None:
            return

        # dumbbell_mask
        dumbbell_mask = cv2.inRange(self.hsv, HSV_COLOR_RANGES[self.color][0], HSV_COLOR_RANGES[self.color][1])
        # Grid mask
        grid_mask = cv2.inRange(self.hsv, HSV_COLOR_RANGES["green"][0], HSV_COLOR_RANGES["green"][1])

        # Dimensions
        h, w, d = self.image.shape

        # front distance
        dist = min(self.laserscan_front)

        # determine center of color pixels
        M = cv2.moments(dumbbell_mask)
        # if the target color is in sight and the dumbbell is not on the grid
        if M['m00'] > 0 and cv2.countNonZero(grid_mask) == 0:
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

    """Move to a coordinate on the grid
    Source: https://edu.gaitech.hk/turtlebot/map-navigation.html"""
    def move_grid(self, xy):

        # Define a client for to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")

        # Set up goal
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(xy[0], xy[1], 0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        # Send goal location
        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        # Wait
        ac.wait_for_result(rospy.Duration(60))

        # Check status
        if (ac.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo("Goal success")
            # Update next state of robot
            if self.state == RED or self.state == BLUE:
                self.state = DUMBBELL
            else:
                self.state = DROP
            return True
        else:
            rospy.loginfo("Goal Fail")
            return False

    """Set up the next action from the queues"""
    def get_action(self):
        # Make sure we have all info we need to
        if not self.initialized or self.hsv is None or self.image is None or self.laserscan is None:
            return

        print(self.red_actions, self.blue_actions)
        # Red's turn
        if self.color == "blue" and self.red_actions:
            # Pop the next action to be done
            self.action_in_progress = self.red_actions.pop(0)

            # Get color of player to go to and set state accordingly
            color = self.action_in_progress.player
            if color == PLAYER_BLUE:
                self.color = 'blue'
            else:
                self.color = 'red'
            if self.color == 'red':
                self.state = RED
            elif self.color == 'blue':
                self.state = BLUE
            return
        elif self.color == "red" and self.blue_actions:
            # Blue's turn
            # Pop the next action to be done
            self.action_in_progress = self.blue_actions.pop(0)

            # Get color of player to go to and set state accordingly
            color = self.action_in_progress.player
            if color == PLAYER_BLUE:
                self.color = 'blue'
            else:
                self.color = 'red'
            if self.color == 'red':
                self.state = RED
            elif self.color == 'blue':
                self.state = BLUE
            return


    """The driver of our node, calls functions dpending on state"""
    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.initialized:
                print(self.state, self.action_in_progress)
                if self.state == DUMBBELL:
                    # Go to dumbell color
                    self.move_to_dumbell()
                elif self.state == PICKUP:
                    # Pick up dumbell
                    self.pick_up_gripper()
                elif self.state == DROP:
                    # Drop dumbell
                    self.drop_gripper()
                elif self.state == ZERO:
                    self.move_grid(GRID_LOCATIONS[ZERO])
                elif self.state == ONE:
                    self.move_grid(GRID_LOCATIONS[ONE])
                elif self.state == TWO:
                    self.move_grid(GRID_LOCATIONS[TWO])
                elif self.state == THREE:
                    self.move_grid(GRID_LOCATIONS[THREE])
                elif self.state == FOUR:
                    self.move_grid(GRID_LOCATIONS[FOUR])
                elif self.state == FIVE:
                    self.move_grid(GRID_LOCATIONS[FIVE])
                elif self.state == SIX:
                    self.move_grid(GRID_LOCATIONS[SIX])
                elif self.state == SEVEN:
                    self.move_grid(GRID_LOCATIONS[SEVEN])
                elif self.state == EIGHT:
                    self.move_grid(GRID_LOCATIONS[EIGHT])
                elif self.state == RED:
                    self.move_grid(GRID_LOCATIONS[RED])
                elif self.state == BLUE:
                    self.move_grid(GRID_LOCATIONS[BLUE])
                elif self.state == STOP:
                    # Done with action, wait for next one
                    self.get_action()
            r.sleep()


if __name__ == '__main__':
    # Declare a node and run it.
    node = OneRobot()
    node.run()
