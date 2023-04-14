#!/usr/bin/env python

import rospy
import actionlib
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from robotont_thesis_pkg.srv import Robotont, RobotontRequest

def main():
    rospy.init_node('simple_client')
    rospy.loginfo("waiting for service")

    # Wait for the sorting service to become available
    rospy.wait_for_service('robotont')
    rospy.loginfo("Starting service")
    sorting_service = rospy.ServiceProxy('robotont', Robotont)

    rospy.loginfo("sending request...")

    # Create a TriggerRequest object and set the string data
    request = RobotontRequest(req= [6,1])
    
    # Send the request to start the sorting process
    response = sorting_service(request)
    if response.success:
        rospy.loginfo("Sorting process done.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("clientserver test finished.")
