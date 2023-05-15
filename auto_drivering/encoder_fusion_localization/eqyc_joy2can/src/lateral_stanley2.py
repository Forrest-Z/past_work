'''
Geometric method for path tracking with Stanley steering control
Link to paper- https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
Authors - Manthan Patel, Sombit Dey
'''

#!/usr/bin/env python
import rospy
import math
import time
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import Int64



def callback_feedback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    print ("Odometry position is ",[x,y])

def start():
    rospy.init_node('path_tracking', anonymous=True)
    
    rospy.Subscriber("/localization/Imu_incremental", Odometry, callback_feedback)
    rospy.spin()


if __name__ == '__main__':
    start()
