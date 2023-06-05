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
        x_goal=  0.14046257734298706
        y_goal= -4.143133640289307
        z_goal= -0.2853129993437435
        w_goal= 0.9584343965058209
        
    elif goal_id == "base":
        x_goal=  2.7663493156433105
        y_goal= -0.7572194337844849
        z_goal= 0.9683679394633669
        w_goal= 0.249526619460676


    goal.target_pose.pose.position.x = x_goal
    goal.target_pose.pose.position.y = y_goal
    goal.target_pose.pose.orientation.z = z_goal
    goal.target_pose.pose.orientation.w = w_goal

    print("defined goal")


    # Create a SimpleActionClient for MoveBase
    move_base_client = SimpleActionClient("move_base", MoveBaseAction)
    # Send the goal to MoveBase and wait for completion
    move_base_client.wait_for_server()
    move_base_client.send_goal(goal)
    print("sent goal")
    move_base_client.wait_for_result()

    #here should wait until the movebase is finished

    # Check if the navigation was successful
    if move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        # Move base client was successful, send "AR_Adjust" mode 
        print("success")
        mode = "AR_Adjustment"

    else:
        # Move base client was not successful, send "AR_Recovery" mode
        print("not successful")
        mode = "AR_Recovery"

    
def main():
    global success
    rospy.init_node('main_node')
        
    move_to_goal("goal1")

    rospy.loginfo("Starting AR request")

    navigation_service = rospy.ServiceProxy('navigation', Navigation)
    mode_request = NavigationRequest(mode = mode)
    navigation_req = navigation_service(mode_request)

    
    while not rospy.is_shutdown():
        # Wait for the sorting service to become available

        # rospy.wait_for_service('robotont')
        # rospy.loginfo("Starting service")
        # sorting_service = rospy.ServiceProxy('robotont', Robotont)
        print("waiting for navigation result")

        if navigation_req.success:
                rospy.loginfo("At goal will send request to kitting station")
                rospy.sleep(4)
                # # Create a TriggerRequest object and set the string data
                # request = RobotontRequest(req= [6,7])
                
                # # Send the request to start the kitting process
                # response = sorting_service(request)
                # if response.success:
                #         rospy.loginfo("Success! Free of payload! Back to base!")
                move_to_goal("base")


    rospy.spin()

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

