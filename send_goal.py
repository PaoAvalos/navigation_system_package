#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def send_goal_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    
    goal = MoveBaseGoal()
    x = [0.2, 2.2]
    y = [1.0, 2.0]
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x[k]
    goal.target_pose.pose.position.y = y[k]
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    global k
    try:
        rospy.init_node('send_goal')
        k = 0
        result_1 = send_goal_client()
        if result_1:
            rospy.loginfo("Goal 1 reached!")
            k += 1
        result_2 = send_goal_client()
        if result_2:
            rospy.loginfo("Goal 2 reached!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")