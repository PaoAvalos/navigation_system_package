#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import tf
import numpy
from ar_track_alvar_msgs.msg import AlvarMarkers
from math import atan2, sqrt
from std_msgs.msg import String

navigation_to_goal_finished = False

def movebase_client():
    global navigation_to_goal_finished

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
    # goal for xarm 
    x_goal= -1.9241305589675903
    y_goal= -0.04057121276855469
    z_goal=  -0.9999746531240553
    w_goal=0.007119909369179759


    goal.target_pose.pose.position.x = x_goal
    goal.target_pose.pose.position.y = y_goal
    goal.target_pose.pose.orientation.z = z_goal
    goal.target_pose.pose.orientation.w = w_goal

    # Send the goal and wait for completion
    client.send_goal(goal)
    rospy.loginfo("Goal sent")
    client.wait_for_result() # this is a blocking call, so the code will wait here until the robot reaches the goal
    rospy.loginfo("I have reached the goal")

    navigation_to_goal_finished = True

def main():
    global navigation_to_goal_finished

    rospy.init_node('robot_client')

    # Wait for the move_base action server to become available
    rospy.loginfo("Waiting for move_base action server...")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    rospy.loginfo("Connected to move base server")

    # Send a goal to the move_base action server
    movebase_client()

    # Wait for the sorting service to become available
    rospy.wait_for_service('sorting')

    # Create a proxy for the sorting service
    rospy.loginfo("Starting sorting service")
    sorting_service = rospy.ServiceProxy('sorting', Trigger)

    # Check if the robot is in position
    while not rospy.is_shutdown():
        if navigation_to_goal_finished:
            rospy.loginfo("Starting sorting process...")

            # Send a request to start the sorting process
            response = sorting_service(TriggerRequest())
            if response.success:
                rospy.loginfo("Sorting process done.")
                break

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
