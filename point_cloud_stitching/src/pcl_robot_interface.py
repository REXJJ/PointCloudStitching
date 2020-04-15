#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo('Data Received.... %s', data.data)

    #Robot Code ....

    rospy.wait_for_service('/pcl_filter_node/capture_point_cloud')
    try:
       capture_point_cloud = rospy.ServiceProxy('/pcl_filter_node/capture_point_cloud',std_srvs.Trigger)#Not sure about the params
       resp = capture_point_cloud()
    except rospy.ServiceException, e:
       print "Service call failed: %s"%e

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/data_collected', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
