#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerRequest
import nav_tags 

finished=False
back_to_base=False

def movebase_client():
    global finished, back_to_base
    # Create a MoveBaseAction client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Create a goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the position and orientation of the goal
    #goal for xarm 
    x_goal= 0.9831671714782715
    y_goal= 2.663472890853882
    z_goal=  0.9991141103266642
    w_goal=0.042083186026706014

    x_base=0.030399322509765625
    y_base= -0.03087368980050087
    z_base=-0.009441505537851355
    w_base= 0.9999554279932574

    if back_to_base: 
        goal.target_pose.pose.position.x = x_base
        goal.target_pose.pose.position.y = y_base
        goal.target_pose.pose.orientation.z = z_base
        goal.target_pose.pose.orientation.w = w_base
    else:
        goal.target_pose.pose.position.x = x_goal
        goal.target_pose.pose.position.y = y_goal
        goal.target_pose.pose.orientation.z = z_goal
        goal.target_pose.pose.orientation.w = w_goal

    # Send the goal and wait for completion
    client.send_goal(goal)
    client.wait_for_result()

    # how to assigned finished only when result is positive?
    if back_to_base== False:
        nav_tags.ar_demo()
        if nav_tags.finished==True:
            finished=True
            back_to_base=True
            movebase_client()

    #if going back to base, start nav_tags again
    else:
        nav_tags.ar_demo()

    return client.get_result()

def main():
    global finished, back_to_base
    rospy.init_node('movebase_client')

    should_i_start= input("Do you want to start the sorting process? (y/n)")
    if should_i_start=="y":
        move= movebase_client()
    else:
        rospy.loginfo("launched but I have not started moving")
        return
    
    # Wait for the sorting server to become available
    rospy.wait_for_service('sorting')

    # Create a proxy for the sorting service
    sorting_service = rospy.ServiceProxy('sorting', Trigger)

    # Check if the robot is in position
    while not rospy.is_shutdown():
        if finished==True:

            rospy.loginfo("Starting sorting process...")
            # Send a request to start the sorting process
            response = sorting_service(TriggerRequest())
            if response.success:
                    rospy.loginfo("Sorting process done.")
                    back_to_base= True
                    movebase_client()
                    break
    rospy.spin()

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
