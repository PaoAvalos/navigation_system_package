#!/usr/bin/env python

import rospy
import actionlib
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from robotont_thesis_pkg.srv import Robotont, RobotontRequest
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

def main():
    rospy.init_node('simple_client')
    rospy.loginfo("waiting for service")

    # Wait for the sorting service to become available
    rospy.wait_for_service('robotont')
    rospy.loginfo("Starting service")
    sorting_service = rospy.ServiceProxy('robotont', Robotont)

    rospy.loginfo("sending request...")

    # Create a TriggerRequest object and set the string data
    request = RobotontRequest(req= [6,7])
    
    # Send the request to start the sorting process
    response = sorting_service(request)
    if response.success:
        rospy.loginfo("Sorting process done.You can move now ")
        quit()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("clientserver test finished.")
