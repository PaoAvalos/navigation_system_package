#!/usr/bin/python3
import rospy
import numpy as np
import std_msgs.msg
from sensor_msgs.msg import LaserScan
from robotont_laserscan_to_distance.msg import LaserScanSplit

pub = rospy.Publisher('scan_to_distance', LaserScanSplit, queue_size=1)

np.warnings.filterwarnings('ignore')

def scan_callback(data):
    laserscan_distance = LaserScanSplit()

    laserscan_distance.header.stamp = rospy.Time.now()      

    distances = data.ranges
    array_len = len(distances)

    right_array = distances[0:array_len//3]
    laserscan_distance.right_min = np.nanmin(right_array)

    center_array = distances[array_len//3:2*array_len//3]
    laserscan_distance.center_min = np.nanmin(center_array)

    left_array = distances[2*array_len//3:]
    laserscan_distance.left_min = np.nanmin(left_array)

    pub.publish(laserscan_distance)


def convert():
    # Starts a new node
    rospy.init_node('laserscan_to_distance', anonymous=True)
    rospy.Subscriber('camera_scan', LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        convert()
    except rospy.ROSInterruptException: pass
