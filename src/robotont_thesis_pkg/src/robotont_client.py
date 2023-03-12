#!/usr/bin/env python

import rospy
import nav_tags
import nav_if_walls

from std_srvs.srv import Trigger, TriggerRequest

global back_to_base
def sorting_client():
    # Initialize the node
    rospy.init_node('sorting_client')

    # Wait for the sorting server to become available
    rospy.wait_for_service('sorting')

    # Create a proxy for the sorting service
    sorting_service = rospy.ServiceProxy('sorting', Trigger)

    # Check if the robot is in position
    if nav_tags.finished:
        in_position= True
    else: 
        rospy.loginfo("Robot has not finished")


    if in_position:
        rospy.loginfo("Starting sorting process...")
        # Send a request to start the sorting process
        response = sorting_service(TriggerRequest())
        if response.success:
            rospy.loginfo("Sorting process done.")
            back_to_base = False
        else:
            rospy.loginfo("Sorting process failed.")
    else:
        rospy.loginfo("Robot not in position.")
        
    rospy.spin() #keeping the node going until process is finished 
if __name__ == '__main__':
    sorting_client()
