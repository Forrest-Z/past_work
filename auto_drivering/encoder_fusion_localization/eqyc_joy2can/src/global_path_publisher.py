#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 17 20:35:20 2021

@author: xin
"""
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def global_path_publisher():
    rospy.init_node('global_path_publisher', anonymous=True)
    global_path_pub = rospy.Publisher('frenet_path',Path, queue_size=1)
    rate = rospy.Rate(200) 
    waypoints = np.load("waypoints.npy")
    path_msg = Path()
    
    
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = rospy.Time.now()
    for i in range(len(waypoints)):
            loc = PoseStamped() 
            loc.pose.position.x = waypoints[i][0]
            loc.pose.position.y = waypoints[i][1]
            loc.pose.orientation.x = waypoints[i][3]
            loc.pose.orientation.y = waypoints[i][4]
            loc.pose.orientation.z = waypoints[i][5]
            loc.pose.orientation.w = waypoints[i][6]  
            path_msg.poses.append(loc)
    while not rospy.is_shutdown():
        
        global_path_pub.publish(path_msg)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        global_path_publisher()
    except rospy.ROSInterruptException:
        pass
             
    
#    pose = PoseStamped()
#    pose.pose = trans.carla_transform_to_ros_pose(wp[0].transform)
#    msg.poses.append(pose)
    
#    rospy.loginfo("Published {} waypoints.".format(len(msg.poses)))    
