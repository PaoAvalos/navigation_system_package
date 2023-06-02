#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from robotont_thesis_pkg.srv import Navigation, NavigationResponse

# Define a callback function to handle the service request
def define_mode(req):
    rospy.loginfo("Sorting process started.")
    if req.mode == "AR_Adjustment":
        
        response = NavigationResponse()
        response.success = True
        print("Response sent")     

    return NavigationResponse(success=True)


def sorting_server():
    # Initialize the node
    rospy.init_node('response')

    # Create a service to handle incoming requests
    rospy.Service('navigation', Navigation, define_mode)
    #wait until a request is received to being the rest 
    rospy.wait_for_service('navigation')

    rospy.spin()

if __name__ == '__main__':
    sorting_server()
