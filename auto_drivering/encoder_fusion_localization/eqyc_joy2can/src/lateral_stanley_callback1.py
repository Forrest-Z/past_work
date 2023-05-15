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

# Node name      - path_tracking
# Publish topic  - cmd_delta (Twist)
# Subscribe topic- base_pose_ground_truth , astroid_path



k_v = 2 #gain parameter
#k_v = 1.5  #gain parameter
alpha = 0.1
wheelbase = 1.6  #in meters
global steer
global n
global ep_max
global ep_sum
global ep_avg
global x_p


n=0
ep_avg = 0
ep_sum = 0
ep_max = 0
x = 0.0
y = 0.0
O_w = 0.0
O_z = 0.0
O_x = 0.0
O_y = 0.0
T_x = 0.0
T_y = 0.0

def callback_feedback(data):
    global x
    global y
    global O_w
    global O_z
    global O_x
    global O_y
    global T_x
    global T_y
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    O_w = data.pose.pose.orientation.w
    O_z = data.pose.pose.orientation.z
    O_x = data.pose.pose.orientation.x
    O_y = data.pose.pose.orientation.y
    T_x = data.twist.twist.linear.x
    T_y = data.twist.twist.linear.y
    
def normalize_angle(angle):
        if angle > math.pi:
            angle -= 2.0 * np.pi
    
        if angle < -math.pi:
            angle += 2.0 * np.pi
        return angle

def dist(a, x, y):
    """
    calculates the euclidian distance between 2 points

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param a: () contains the coordinates of other point
    """
    return (((a.pose.position.x - x)**2) + ((a.pose.position.y - y)**2))**0.5

def path_length_distance(a,b):
    return (((a.pose.position.x - b.pose.position.x)**2) + ((a.pose.position.y - b.pose.position.y)**2))**0.5

def calc_path_length(data):
    global path_length
    path_length = []

    for i in range(len(data.poses)):
        if i == 0:
            path_length.append(0)

        else:
            path_length.append(path_length[i-1] + path_length_distance(data.poses[i], data.poses[i-1]))


def callback_path(data):
    global ep
    global cp
    global bot_theta1
    global ep_max
    global ep_sum
    global ep_avg
    global n
    global path_length
    global x_p

    x_p=data

def start():
    global pub1
    global pub2
    rospy.init_node('path_tracking', anonymous=True)
    pub2 = rospy.Publisher('cross_track_error', Twist, queue_size=100)
    pub1 = rospy.Publisher('cmd_delta', Twist, queue_size=100)
#    rospy.Subscriber("frenet_path", Path, callback_path)
#    rospy.Subscriber("/localization/Imu_incremental", Odometry, callback_feedback)
    
#    start_t = rospy.get_time()
#    start = time.clock()
    
    global bot_f_x
    global bot_f_y
    global bot_theta
    global bot_vel
    global x_p
    global ep
    global cp
    global bot_theta1
    global ep_max
    global ep_sum
    global ep_avg
    global n
    global path_length
    global x
    global y
    
    
    data = rospy.wait_for_message("/localization/Imu_incremental", Odometry, timeout=None)
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    print ("Odometry position is ",[x,y])

    siny = 2.0 * (O_w *
                  O_z +
                  O_x *
                  O_y)
    cosy = 1.0 - 2.0 * (O_y *
                        O_y +
                        O_z *
                        O_z)
    bot_theta = math.atan2(siny, cosy) 
    bot_vel = math.fabs(T_x * math.cos(bot_theta) + T_y * math.sin(bot_theta))
    
    bot_f_x = x + wheelbase * 0.5 * math.cos(normalize_angle(bot_theta))
    bot_f_y = y + wheelbase * 0.5 * math.sin(normalize_angle(bot_theta))  
    
    x_f = bot_f_x 
    y_f = bot_f_y
    #print ("Front wheel angle position is ",[x,y])
    cross_err = Twist()
    x_p = rospy.wait_for_message("frenet_path", Path, timeout=None)
    calc_path_length(x_p)
    data1=x_p
    
    distances = []
    for i in range(len(x_p.poses)):
        a = x_p.poses[i]
        distances += [dist(a, x_f, y_f)]
    ep = min(distances)
    if (ep > ep_max):
        ep_max = ep

    n = n + 1
    ep_sum = ep_sum + ep
    ep_avg = ep_sum / n

#   cp--index of min distance point 
    cp = distances.index(ep) + 1 
#    cp = distances.index(ep) 

    steer_path = math.atan2(x_p.poses[cp+1].pose.position.y - x_p.poses[cp].pose.position.y, x_p.poses[cp+1].pose.position.x - x_p.poses[cp].pose.position.x )
    path_yaw = normalize_angle(steer_path)
    #print ("path_yaw & bot_yaw is",[steer_path *180/math.pi,bot_theta *180/math.pi])
    steer_err = normalize_angle(steer_path - bot_theta)
    #print ("path_yaw - bot_yaw is",steer_err *180/math.pi)
    cmd = Twist()
    cross2 = [(x_f - data1.poses[cp].pose.position.x),
              (y_f - data1.poses[cp].pose.position.y)] ##################
#    cross = [math.cos(bot_theta), math.sin(bot_theta)]
#    cross_prod = cross[0] * cross2[1] - cross[1] * cross2[0]
#    if (cross_prod > 0):
#        ep = -ep
    cross = [math.sin(path_yaw), -math.cos(path_yaw)]
    cross_dot = np.dot(cross2,cross)
    ep = np.sign(cross_dot)*ep
    
    
    #print ("cp %f ep %+f" % (cp, ep))
    
    cross_err.linear.x = ep
    cross_err.angular.x = ep_max
    cross_err.angular.y = ep_avg

    siny = +2.0 * (x_p.poses[cp].pose.orientation.w *
                   x_p.poses[cp].pose.orientation.z +
                   x_p.poses[cp].pose.orientation.x *
                   x_p.poses[cp].pose.orientation.y)
    cosy = +1.0 - 2.0 * (x_p.poses[cp].pose.orientation.y *
                         x_p.poses[cp].pose.orientation.y +
                         x_p.poses[cp].pose.orientation.z *
                         x_p.poses[cp].pose.orientation.z)
    ###
    tar_x = data1.poses[cp].pose.position.x
    tar_y = data1.poses[cp].pose.position.y
    v_begin_x = bot_f_x
    v_begin_y = bot_f_y
    v_end_x = v_begin_x + math.cos(normalize_angle(bot_theta))
    v_end_y = v_begin_y + math.sin(normalize_angle(bot_theta))
    v_vec = np.array([v_end_x - v_begin_x, v_end_y - v_begin_y, 0.0])
    w_vec = np.array([tar_x - v_begin_x,  tar_y - v_begin_y, 0.0])   
    w_cos = np.dot(w_vec, v_vec) / (np.linalg.norm(w_vec) * np.linalg.norm(v_vec))
    w_sin = math.sqrt(1- w_cos ** 2)
    ep_lat = ep *  w_sin 
    
    #print ("min distance point idx is",cp)
    #print ("calculate min distance is",ep)
    #print ("lat_min distance is",ep_lat)
    
    # print "steer_error : ", steer_err, "\n"
    if bot_vel < 0.000001:
       tan = 0.0 
    else:
        tan = math.atan(k_v * ep_lat /(bot_vel+0.2))             

   # print ("bot_vel is",bot_vel)
    #print ("---path-bot yaw_error angle  is",steer_err*180/math.pi)
    #print ("---lateral e angle is",tan*180/math.pi)

    
    #delta = (2*steer_err + tan)
    delta = (1*steer_err + tan)


    delta = delta * 180 / math.pi  #converting delta into degrees from radian
    #print ("stanley delta is",delta)
#    delta = min(47.5,max(-47.5,delta))
    delta = min(47,max(-47,delta))
    #print ("actual front wheel angle is",delta)
    #print ("\n")
    
#    print delta
    cmd.angular.z = delta
    cross_err.linear.y = steer_err
    cross_err.linear.z = path_length[cp]

    pub1.publish(cmd)
    pub2.publish(cross_err)
    
#    end_t = rospy.get_time()
#    elapsed = (time.clock() - start)
    #print ("stanley run time is",start_t-end_t)
    #print("Stanley Time used:",elapsed)
    rospy.spin()


if __name__ == '__main__':
    start()
