#!/usr/bin/env python
from std_msgs.msg import Bool
import rospy
import rospy
from std_msgs.msg import String
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerRequest

state= 2



def movebase_client():
    global state
    # Create a MoveBaseAction client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Create a goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the position and orientation of parameters
    #goal for xarm 

    x_goal= 0.9831671714782715
    y_goal= 2.663472890853882
    z_goal=  0.9991141103266642
    w_goal=0.042083186026706014



    x1_goal=1.1974222660064697

    y1_goal= -0.11812585592269897

    z1_goal=  -0.3962580943366397

    w1_goal= 0.9181391630208869


    # x_goal= 1.4274
    # y_goal= -0.4333
    # z_goal= -0.37603
    # w_goal= 0.926


    #turning before aiming back to base
    x1_base=1.42362153
    y1_base= -0.410843
    z1_base=0.9004779
    w1_base= 0.44490166

    #going back to base 
    x_base=0.030399322509765625
    y_base= -0.03087368980050087
    z_base=-0.009441505537851355
    w_base= 0.9999554279932574

    if state==2: 
        rospy.loginfo("going to base")

        # goal.target_pose.pose.position.x = x1_base
        # goal.target_pose.pose.position.y = y1_base
        # goal.target_pose.pose.orientation.z = z1_base
        # goal.target_pose.pose.orientation.w = w1_base
            
        # client.send_goal(goal)
        # wait = client.wait_for_result()


        goal.target_pose.pose.position.x = x_base
        goal.target_pose.pose.position.y = y_base
        goal.target_pose.pose.orientation.z = z_base
        goal.target_pose.pose.orientation.w = w_base

        client.send_goal(goal)
        wait = client.wait_for_result()
        rospy.loginfo("at base")

     

    elif state==1:

        rospy.loginfo("going to goal ")

        # goal.target_pose.pose.position.x = x1_goal
        # goal.target_pose.pose.position.y = y1_goal
        # goal.target_pose.pose.orientation.z = z1_goal
        # goal.target_pose.pose.orientation.w = w1_goal

        # client.send_goal(goal)
        # wait = client.wait_for_result()


        goal.target_pose.pose.position.x = x_goal
        goal.target_pose.pose.position.y = y_goal
        goal.target_pose.pose.orientation.z = z_goal
        goal.target_pose.pose.orientation.w = w_goal

        client.send_goal(goal)
        wait = client.wait_for_result()
        rospy.loginfo("at goal")


    return client.get_result()


# def callback(data):
#     global xarm_data
#     xarm_data=data.data

def main():
    global state
    rospy.init_node('robotont_demoDay')

    movebase_client()
    # rospy.Subscriber("xarm_chatter", Bool, callback)
    # rate=rospy.Rate(10)

    # pub = rospy.Publisher('robotont_chatter', Bool, queue_size=10)
    # robotont= Bool()
    # Check if the robot is in position
        
        # if finished==True:

        #     rospy.loginfo_once("sending msg to xarm...")
        #     # Send a request to start the sorting process
        #     if xarm_data== False:
        #         robotont.data= True
        #         pub.publish(robotont)
        #         rospy.loginfo_once("sent true to xarm...")
        #         rate.sleep()


        #     elif xarm_data== True:
        #         rospy.loginfo_once("xarm has finished")
        #         robotont.data= False
        #         pub.publish(robotont)
        #         back_to_base= True
        #         movebase_client()
        #         rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")



    # """ # Check if the robot is in position
    # if nav_tags.finished:
    #     in_position= True
    # else: 
    #     rospy.loginfo("Robot has not finished")


    # if in_position:
    #     rospy.loginfo("Starting sorting process...")
    #     # Send a request to start the sorting process
    #     response = sorting_service(TriggerRequest())
    #     if response.success:
    #         rospy.loginfo("Sorting process done.")
    #         back_to_base = False
    #     else:
    #         rospy.loginfo("Sorting process failed.")
    # else:
    #     rospy.loginfo("Robot not in position.")
        
    # rospy.spin() #keeping the node going until process is finished  """


# if robot in place pub true 
# and receive false 

# if receive true 
# then move 
