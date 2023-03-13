#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion

if __name__ == '__main__':
    rospy.init_node('send_goal_node')

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = Point(1.0, 1.0, 0.0)  #get from loading map in rviz and looking at echo /move base simple goal
    goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0) #same as before orientation given 

    goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
    goal_publisher.publish(goal)
 
    rospy.spin()
