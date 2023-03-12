#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():
    # Create a MoveBaseAction client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Create a goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the position and orientation of the goal
    goal.target_pose.pose.position.x = -1.382
    goal.target_pose.pose.position.y =1.235
    goal.target_pose.pose.orientation.z =0.651
    goal.target_pose.pose.orientation.w = 0.759


    # Send the goal and wait for completion
    client.send_goal(goal)
    wait = client.wait_for_result()

    # Return the result of the goal
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('movebase_client')
        # Call the movebase_client function to move the robot to the goal
        result = movebase_client()

        if result:
            rospy.loginfo("Goal execution done!")
        else:
            rospy.loginfo("Failed to execute the goal")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
