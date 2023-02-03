#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from robotont_laserscan_to_distance.msg import LaserScanSplit

DETECTING_SPEED_ROTATION = 0.5
twist_msg = Twist()
side_rotation = [1, -1]
calc_time_dur = 0
moving = True
rotation = False

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

def callback(data):
    global calc_time_dur, moving, rotation, moving_start_time, rotation_start_time
    if not rospy.get_param('ar_present'):
        if data.center_min < 0.4 or data.left_min < 0.3 or data.right_min < 0.3:
            moving = True
            calc_time_dur = 0
            rotation = False
            if data.center_min < 0.4:
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                twist_msg.angular.z = side_rotation[0] * DETECTING_SPEED_ROTATION
                rospy.loginfo(twist_msg)
                cmd_vel_pub.publish(twist_msg)
                print("Rotation because of center obstacle")
            elif data.left_min < 0.3:
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                twist_msg.angular.z = side_rotation[1] * DETECTING_SPEED_ROTATION
                rospy.loginfo(twist_msg)
                cmd_vel_pub.publish(twist_msg)
                print("Rotation because of left obstacle")
            elif data.right_min < 0.3:
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                twist_msg.angular.z = side_rotation[0] * DETECTING_SPEED_ROTATION
                rospy.loginfo(twist_msg)
                cmd_vel_pub.publish(twist_msg)
                print("Rotation because of right obstacle")
        else:
            if moving:
                if calc_time_dur == 0: 
                    moving_start_time = rospy.get_time()
                if (rospy.get_time() - moving_start_time) < 3.0:
                    calc_time_dur = 1
                    twist_msg.linear.x = 1.0
                    twist_msg.linear.y = 0.0
                    twist_msg.angular.z = 0.0
                    rospy.loginfo(twist_msg)
                    cmd_vel_pub.publish(twist_msg)
                    print("Center: ", data.center_min)
                    print("Left: ", data.left_min)
                    print("Right: ", data.right_min)
                    print("Moving to find tag")
                elif (rospy.get_time() - moving_start_time) >= 3.0:
                    calc_time_dur = 0
                    moving = False
                    rotation = True
            elif rotation:
                if calc_time_dur == 0:
                    rotation_start_time = rospy.get_time()
                if (rospy.get_time() - rotation_start_time) < 6.4//DETECTING_SPEED_ROTATION:
                    calc_time_dur = 1
                    twist_msg.linear.x = 0.0
                    twist_msg.linear.y = 0.0
                    twist_msg.angular.z = DETECTING_SPEED_ROTATION
                    rospy.loginfo(twist_msg)
                    cmd_vel_pub.publish(twist_msg)
                    print("Rotation to find tag")
                elif (rospy.get_time() - rotation_start_time) >= 6.4//DETECTING_SPEED_ROTATION:
                    calc_time_dur = 0
                    moving = True
                    rotation = False

def ar_listener():

    rospy.init_node('nav_if_walls', anonymous=True)

    rospy.Subscriber("scan_to_distance", LaserScanSplit, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        ar_listener()
    except rospy.ROSInterruptException:
        pass