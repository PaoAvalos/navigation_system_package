
#import all necessary libraries
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

#defining state machine
state="stand_by"

#defining transitions

navigation_to_goal_finished = False
tags_to_goal_finished = False
sorting_finished = False
navigation_to_start_finished = False
tags_to_start_finished = False

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


def callback(data):

    global last_heartbeat, marker_ids, base_id, k,i, param, twist_msg, in_base
    global cmd_vel_pub, distance_achieved, finished,back_to_base, tag_goal_finished


    if len(data.markers)==0 and i<len(base_id):
            rotation()

    elif len(data.markers)!=0 and i<len(base_id):
            print("MARKER DETECTED")
            for marker in data.markers:
                    # Check the marker id
                    if marker.id != int(base_id[i]):

                            rotation_base()
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
                                    break
                            
                    elif abs(angle) > ANGLE_TOLERANCE or dist_to_goal > GOAL_TOLERANCE:
                            print("Calibrating angle")
                            twist_msg.linear.x = min(max(goal_pos[0],-MAX_LIN_VEL),MAX_LIN_VEL)
                            twist_msg.linear.y = min(max(goal_pos[1],-MAX_LIN_VEL),MAX_LIN_VEL)
                            twist_msg.angular.z = 4*min(max(angle,-MAX_ANG_VEL),MAX_ANG_VEL)
                            print("Updating cmd vel %s", twist_msg)
                            cmd_vel_pub.publish(twist_msg)
                    else:
                            i=i+1
                            distance_achieved = False

                    rospy.loginfo_once("Robot is back to base")

    elif k>=len(base_id):
            rospy.set_param('ar_present', True)
            tag_goal_finished = True
            rospy.loginfo_once("Robot is in the goal tag, tag goal finished= true ")

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

def movebase_client(target):
    global navigation_to_goal_finished, navigation_to_base_finished

    # Create a MoveBaseAction client
    rospy.loginfo("Waiting for move_base action server...")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    rospy.loginfo("Connected to move base server")

    # Create a goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the position and orientation of the goal

    # navigation goal  
    x_goal = 0.9831671714782715
    y_goal = 2.663472890853882
    z_goal = 0.9991141103266642
    w_goal = 0.042083186026706014

    #robot base/starting point


    if target == "goal":

        goal.target_pose.pose.position.x = x_goal
        goal.target_pose.pose.position.y = y_goal
        goal.target_pose.pose.orientation.z = z_goal
        goal.target_pose.pose.orientation.w = w_goal

    elif target == "base":
        goal.target_pose.pose.position.x = x_base
        goal.target_pose.pose.position.y = y_base
        goal.target_pose.pose.orientation.z = z_base
        goal.target_pose.pose.orientation.w = w_base

    # Send the goal and wait for completion
    client.send_goal(goal)
    rospy.loginfo("Goal sent")
    client.wait_for_result() # this is a blocking call, so the code will wait here until the robot reaches the goal
    rospy.loginfo("I have reached the goal")

# here i need to do a check whether the robot has reached the goal or not#
    if target == "goal":
        navigation_to_goal_finished = True
    elif target == "base":
        navigation_to_base_finished = True


if state == "stand_by":
    #we start by navigating to the goal
    state = "navigation_to_goal"

if state == "navigation_to_goal":
    print("state is now: ", state)
    target= "goal"
    movebase_client(target)
    if navigation_to_goal_finished == True:
        state = "finished_navigation_to_goal"

if state == "finished_navigation_to_goal":

    rospy.loginfo("Navigation to goal finished, will start tag detection")
    state = "tags_to_goal"
    print("state is now: ", state)

if state == "tags_to_goal":
    print("state is now: ", state)
    rospy.loginfo("Subscribing to ar_pose_marker")
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
    rospy.loginfo("Succesfully subscribed to ar pose markers ")

    if tag_goal_finished == True:
        state = "finished_tags_to_goal"
        print("state is now: ", state)

if state == "finished_tags_to_goal":
    rospy.loginfo("Finished tag detection, will start going back to base")
    state= "navigation_to_base"

if state == "navigation_to_base":
    print("state is now: ", state)
    target= "base"
    movebase_client(target)

    if navigation_to_base_finished == True:
        state = "finished_navigation_to_base"
        print("state is now: ", state)

if state == "finished_navigation_to_base":
        rospy.loginfo("Navigation to base finished")
        state = "stand_by"
        print("state is now: ", state)