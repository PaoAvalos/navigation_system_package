#!/usr/bin/python3

import rospy
import tf
import numpy
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from math import atan2, sqrt

import actionlib
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Bool
from robotont_thesis_pkg.srv import Robotont, RobotontRequest

import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient


rospy.set_param("AR_Recovery", False)
rospy.set_param("AR_Adjustment", False)
rospy.set_param("AR_Finished", False)


def set_parameter(success):
    if success:
        rospy.set_param("AR_Adjustment", True)
        print("AR_Adjustment: ", rospy.get_param("AR_Adjustment"))
    else:
        rospy.set_param("AR_Recovery", True)
        print("AR_Recovery: ", rospy.get_param("AR_Recovery"))

def move_to_goal():

    goal = MoveBaseGoal()
    print("started goal")
    goal.target_pose.header.frame_id = "map"

    x_goal=  1.4334101676940918
    y_goal= -8.417564392089844
    z_goal= -0.935357373645112
    w_goal= 0.35370409040286555

    goal.target_pose.pose.position.x = x_goal
    goal.target_pose.pose.position.y = y_goal
    goal.target_pose.pose.orientation.z = z_goal
    goal.target_pose.pose.orientation.w = w_goal
    print("defined goal")


    # Create a SimpleActionClient for MoveBase
    move_base_client = SimpleActionClient("move_base", MoveBaseAction)
    move_base_client.wait_for_server()
    rospy.loginfo("Connected to move_base server")

    # Send the goal to MoveBase and wait for completion
    move_base_client.send_goal(goal)
    move_base_client.wait_for_result()

    # Check if the navigation was successful
    success = move_base_client.get_state() == 3  # State 3 corresponds to SUCCEEDED

    print("success:",success)
    # Once the goal is reached, set the appropriate ROS parameter
    set_parameter(success)


if __name__ == '__main__':
    rospy.init_node('MAIN_GOOD')
    

    try:
        # goal_number = int(goal_number)
        move_to_goal()
        
    except ValueError:
        rospy.logerr("Invalid goal number. Please enter a valid goal number or 'q' to quit.")

    rospy.loginfo("Exiting goal navigation.")

