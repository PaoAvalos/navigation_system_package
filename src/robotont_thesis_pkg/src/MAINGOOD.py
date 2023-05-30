#!/usr/bin/python3

import rospy
import actionlib
from std_msgs.msg import Bool
from robotont_thesis_pkg.srv import Robotont, RobotontRequest
from robotont_thesis_pkg.srv import Navigation, NavigationRequest

import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient

rospy.set_param("AR_Recovery", False)
rospy.set_param("AR_Adjustment", False)
rospy.set_param("AR_Finished", False)

def move_to_goal(goal_id):

    global mode
    goal = MoveBaseGoal()
    print("started goal")
    goal.target_pose.header.frame_id = "map"

    if goal_id == "goal1":
        x_goal=  1.4334101676940918
        y_goal= -8.417564392089844
        z_goal= -0.935357373645112
        w_goal= 0.35370409040286555

    elif goal_id == "base":
        x_goal=  18
        y_goal= -8
        z_goal= -0.5
        w_goal= 0.65

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
    if move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        # Move base client was successful, send "AR_Adjust" mode 
        print("success")
        mode = "AR_Adjustment"

    else:
        # Move base client was not successful, send "AR_Recovery" mode
        print("not successful")
        mode = "AR_Adjustment"

    
def main():
    global success
    rospy.init_node('main_node')
        
    move_to_goal("goal1")

    rospy.loginfo("Starting AR request")

    navigation_service = rospy.ServiceProxy('navigation', Navigation)
    mode_request = NavigationRequest(mode = "AR_Adjust")
    navigation_response = navigation_service(mode_request)

    
    while not rospy.is_shutdown():
        # Wait for the sorting service to become available

        rospy.wait_for_service('robotont')
        rospy.loginfo("Starting service")
        sorting_service = rospy.ServiceProxy('robotont', Robotont)

        if navigation_response.success:
                rospy.loginfo("At goal will send request to kitting station")

                # Create a TriggerRequest object and set the string data
                request = RobotontRequest(req= [6,7])
                
                # Send the request to start the kitting process
                response = sorting_service(request)
                if response.success:
                        rospy.loginfo("Success! Free of payload! Back to base!")
                        move_to_goal("base")


    rospy.spin()

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

