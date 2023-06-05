#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerRequest
import tf
import numpy
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from math import atan2, sqrt
from std_msgs.msg import String
from robotont_thesis_pkg.srv import Robotont, RobotontRequest
import sys


#parameters for AR tag detection
MAX_LIN_VEL = 0.07
MAX_ANG_VEL = 0.5

GOAL_TOLERANCE = 0.05
ANGLE_TOLERANCE = 0.1
GOAL_DIST_FROM_MARKER = 0.4
DETECTING_SPEED_ROTATION = 0.25


last_heartbeat = 0
k=0
i=0
param = 0
twist_msg = Twist()
distance_achieved = False
side_rotation = [1, -1]
calc_time_dur = 0
moving = True
rotation = False
rospy.set_param('ar_present', True)

#global variables
tag_goal_finished=False
finished=False
back_to_base=False
in_base=False


#adding functions for ar tag detection
def callback(data):
        global last_heartbeat, marker_ids, base_id, k,i, param, twist_msg, in_base
        global cmd_vel_pub, distance_achieved, finished,back_to_base, tag_goal_finished
        
        if len(data.markers)==0 and k<len(marker_ids):
                rospy.loginfo_once("No marker detected")
                rotation()

        elif len(data.markers)!=0 and k<len(marker_ids):
                print("MARKER DETECTED")
                for marker in data.markers:
                        # Check the marker id
                        if marker.id != int(marker_ids[k]):

                                rotation()
                                continue
                
                        rospy.set_param('ar_present', True)
                        param = 0
                        rospy.loginfo(rospy.get_caller_id() + " I heard %s", marker)
                        marker_pos = (
                                marker.pose.pose.position.x,
                                marker.pose.pose.position.y,
                                marker.pose.pose.position.z,
                                0)
                        marker_ori = (
                                marker.pose.pose.orientation.x,
                                marker.pose.pose.orientation.y,
                                marker.pose.pose.orientation.z,
                                marker.pose.pose.orientation.w)

                        # Start with a unit vector pointing forward
                        goal_dir = numpy.array([0, 0, 1, 0])

                        # Rotation matrix that represents to marker orientation
                        rot_mat = tf.transformations.quaternion_matrix(marker_ori)

                        # Rotate the unit vec so it points to the direction of the marker
                        goal_dir = rot_mat.dot(goal_dir)
                        #
                        print("Goal direction: ", goal_dir)
                        goal_pos = marker_pos + goal_dir * GOAL_DIST_FROM_MARKER
                        # Find planar distance to the goal
                        dist_to_goal = numpy.linalg.norm(goal_pos[0:2])
                        dist_to_goal_x = numpy.linalg.norm(goal_pos[0])
                        dist_to_marker = numpy.linalg.norm(marker_pos[0:2])
                        #twist_msg = Twist()
                        angle = atan2(marker_pos[1],marker_pos[0])
                        rospy.logdebug("ANGLE: %s", angle)
                        rospy.logdebug("DISTTOMARKER: %s", dist_to_marker)
                        rospy.logdebug("GoalDir: %s", goal_dir)
                        rospy.logdebug("DISTTOGOAL: %s", dist_to_goal)
                        rospy.logdebug("DISTTOGOALX: %s", dist_to_goal_x)
                        rospy.logdebug("\nPOS: %s", marker_pos)
                        rospy.logdebug("\nGOAL: %s", goal_pos)

                        if dist_to_goal_x > GOAL_TOLERANCE and not distance_achieved:
                                print("Calibrating distance")
                                print(dist_to_goal_x)
                                twist_msg.linear.x = min(max(goal_pos[0],-MAX_LIN_VEL),MAX_LIN_VEL)
                                twist_msg.linear.y = 0
                                twist_msg.angular.z = 4*min(max(angle,-MAX_ANG_VEL),MAX_ANG_VEL)
                                print("Updating cmd vel %s", twist_msg)
                                cmd_vel_pub.publish(twist_msg)
                                if dist_to_goal_x <= GOAL_TOLERANCE:
                                        distance_achieved = True
                                
                        elif abs(angle) > ANGLE_TOLERANCE or dist_to_goal > GOAL_TOLERANCE:
                                print("Calibrating angle")
                                twist_msg.linear.x = min(max(goal_pos[0],-MAX_LIN_VEL),MAX_LIN_VEL)
                                twist_msg.linear.y = min(max(goal_pos[1],-MAX_LIN_VEL),MAX_LIN_VEL)
                                twist_msg.angular.z = 4*min(max(angle,-MAX_ANG_VEL),MAX_ANG_VEL)
                                print("Updating cmd vel %s", twist_msg)
                                cmd_vel_pub.publish(twist_msg)
                        else:
                                k=k+1
                                distance_achieved = False
        elif k>=len(marker_ids):
                rospy.set_param('ar_present', True)
                tag_goal_finished = True
                rospy.loginfo_once("Robot is in the goal tag, tag goal finished= true ")
        

#adding rotation function
def rotation():
        global last_heartbeat, param
        if param==0:
                param = 1
                last_heartbeat = rospy.get_time()
        # print("Next marker:", marker_ids[k])
        print(last_heartbeat)
        print(6.4//DETECTING_SPEED_ROTATION)
        print(rospy.get_time() - last_heartbeat)
        if (rospy.get_time() - last_heartbeat) >= 6.4//DETECTING_SPEED_ROTATION:
                rospy.set_param('ar_present', False)
        else:
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                twist_msg.angular.z = DETECTING_SPEED_ROTATION
                rospy.loginfo(twist_msg)
                last_heartbeat = rospy.get_time()
                cmd_vel_pub.publish(twist_msg)
                print("Rotating")

def rotation_base():
        global last_heartbeat, param
        if param==0:
                param = 1
                last_heartbeat = rospy.get_time()
        print("Next marker:", base_id[i])
        print(last_heartbeat)
        print(6.4//DETECTING_SPEED_ROTATION)
        print(rospy.get_time() - last_heartbeat)
        if (rospy.get_time() - last_heartbeat) >= 6.4//DETECTING_SPEED_ROTATION:
                rospy.set_param('ar_present', False)
        else:
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                twist_msg.angular.z = DETECTING_SPEED_ROTATION
                rospy.loginfo(twist_msg)
                last_heartbeat = rospy.get_time()
                cmd_vel_pub.publish(twist_msg)
                print("Rotating")

def movebase_client():
    global finished, back_to_base,navigation_to_goal_finished,tag_goal_finished,base_id ,in_base
    # Create a MoveBaseAction client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Create a goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"

    # Set the position and orientation of the goal
    #goal for xarm 
    x_goal=  1.4334101676940918
    y_goal= -8.417564392089844
    z_goal= -0.935357373645112
    w_goal= 0.35370409040286555

    x_base=-0.09474891424179077
    y_base=  0.10085391998291016
    z_base=-0.00929586053010463
    w_base= 0.9999567925550608

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
    client.wait_for_result() #this is a blocking call, so the code will wait here until the robot reaches the goal


    if back_to_base== False: #if we are going to goal 
        navigation_to_goal_finished=True

    #if going back to base, start nav_tags again
    else:
        rospy.loginfo("I am in base")
        in_base=True

    return client.get_result()

#main function, that merges movebase and nav_tags
def main():
    global marker_ids, back_to_base, finished, base_id, cmd_vel_pub, navigation_to_goal_finished, tag_goal_finished, in_base


    marker_ids = "17"

    base_id= "0"

    rospy.init_node('robot_controller')
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    movebase_client()

    if navigation_to_goal_finished==True:

        # Set up subscriber for /ar_pose_marker
        rospy.loginfo("Subscribing to ar_pose_marker")

        rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
        rospy.loginfo("Succesfully subscribed to ar pose markers ")       

    while not rospy.is_shutdown():
        if tag_goal_finished == True:

            

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
                rospy.loginfo("Free to go")

    rospy.spin()




if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

