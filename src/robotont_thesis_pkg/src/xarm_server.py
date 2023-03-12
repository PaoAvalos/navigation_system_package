#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerResponse

# Define a callback function to handle the service request
def handle_sorting_request(req):
    rospy.loginfo("Sorting process started.")

    rospy.sleep(10) # Sleeps for 1 sec
    
    rospy.loginfo("Sorting process done.")
    return TriggerResponse(success=True, message="Sorting process done.")


def sorting_server():
    # Initialize the node
    rospy.init_node('sorting_server')

    # Create a service to handle incoming requests
    service = rospy.Service('sorting', Trigger, handle_sorting_request)

    rospy.loginfo("Sorting server ready.")

    # Spin to keep the node running
    rospy.spin()

if __name__ == '__main__':
    sorting_server()
