#!/usr/bin/python3
import rospy
import tf
from std_msgs.msg import Int16MultiArray
import numpy
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from math import atan2, sqrt
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest

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
finished= False
back_to_base=False
distance_achieved = False
side_rotation = [1, -1]
calc_time_dur = 0
moving = True
rotation = False
rospy.set_param('ar_present', True)

def callback(data):
        global last_heartbeat, marker_ids, base_id, k,i, param, twist_msg, cmd_vel_pub, distance_achieved, finished,back_to_base
        
        if back_to_base:
                rospy.loginfo_once("back to base triggered")

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
                                
                                break

        else:

                if len(data.markers)==0 and k<len(marker_ids):
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
                elif k>=len(base_id):
                        rospy.set_param('ar_present', True)
                        finished=True
                        rospy.loginfo_once("Finished marker ids ")
                        

               
               
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

def ar_demo():
        global marker_ids, back_to_base, finished, base_id
        # Initialize this ROS node
        rospy.init_node('nav_tags', anonymous=True)
        # get target marker id
        marker_ids = rospy.get_param('~marker_ids').split(",")

        base_id= rospy.get_param('~base_id')
        # Create publisher for command velocity
        global cmd_vel_pub
        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Set up subscriber for /ar_pose_marker
        rospy.loginfo("Subscribing to ar_pose_marker")

        rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
        rospy.loginfo("Succesfully subscribed to ar pose markers ")
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
                                break
                              
        rospy.spin()
  
if __name__ == '__main__':
    try:
        ar_demo()
    except rospy.ROSInterruptException:
        pass