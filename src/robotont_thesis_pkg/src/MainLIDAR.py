
#import all necessary libraries
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerRequest
import tf
import numpy
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from math import atan2, sqrt
from std_msgs.msg import String


x=rospy.get_param("/xarm_1/x")
#defining state machine
state="stand_by"

#defining transitions

navigation_to_goal_finished = False
tags_to_goal_finished = False
sorting_finished = False
navigation_to_start_finished = False
tags_to_start_finished = False


#global variables
tag_goal_finished=False
finished=False
back_to_base=False
in_base=False


def movebase_client(target):
    global navigation_to_goal_finished, navigation_to_base_finished

    # Create a MoveBaseAction client
    rospy.loginfo("Waiting for move_base action server...")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    rospy.loginfo("Connected to move base server")

    # Create a goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map" 
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the position and orientation of the goal

    # navigation goal  
    #specify from loaded parameters 
    x_goal = 0.9831671714782715
    y_goal = 2.663472890853882
    z_goal = 0.9991141103266642
    w_goal = 0.042083186026706014

    #robot base/starting point


    if target == "goal":

        goal.target_pose.pose.position.x = x_goal
        goal.target_pose.pose.position.y = y_goal
        goal.target_pose.pose.orientation.z = z_goal
        goal.target_pose.pose.orientation.w = w_goal

    elif target == "base":
        goal.target_pose.pose.position.x = x_base
        goal.target_pose.pose.position.y = y_base
        goal.target_pose.pose.orientation.z = z_base
        goal.target_pose.pose.orientation.w = w_base

    # Send the goal and wait for completion
    client.send_goal(goal)
    rospy.loginfo("Goal sent")
    client.wait_for_result() # this is a blocking call, so the code will wait here until the robot reaches the goal
    rospy.loginfo("I have reached the goal")

# here i need to do a check whether the robot has reached the goal or not#
    if target == "goal":
        navigation_to_goal_finished = True
    elif target == "base":
        navigation_to_base_finished = True


if state == "stand_by":
    #we start by navigating to the goal
    state = "navigation_to_goal"

if state == "navigation_to_goal":
    print("state is now: ", state)
    target= "goal"
    movebase_client(target)
    if navigation_to_goal_finished == True:
        state = "finished_navigation_to_goal"

if state == "finished_navigation_to_goal":

    rospy.loginfo("Navigation to goal finished, will start tag detection")
    state = "tags_to_goal"
    print("state is now: ", state)

if state == "tags_to_goal":
    print("state is now: ", state)
    rospy.loginfo("Subscribing to ar_pose_marker")
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
    rospy.loginfo("Succesfully subscribed to ar pose markers ")

    if tag_goal_finished == True:
        state = "finished_tags_to_goal"
        print("state is now: ", state)

if state == "finished_tags_to_goal":
    rospy.loginfo("Finished tag detection, will start going back to base")
    state= "navigation_to_base"

if state == "navigation_to_base":
    print("state is now: ", state)
    target= "base"
    movebase_client(target)

    if navigation_to_base_finished == True:
        state = "finished_navigation_to_base"
        print("state is now: ", state)

if state == "finished_navigation_to_base":
        rospy.loginfo("Navigation to base finished")
        state = "stand_by"
        print("state is now: ", state)