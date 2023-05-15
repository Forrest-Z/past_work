#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Sep  4 17:23:25 2021

@author: xin
"""
#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat May  8 15:09:52 2021

@author: xin
"""
import rospy
import threading
#import tf
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import math
from collections import deque
from geometry_msgs.msg import PointStamped 
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt


class StanleyControl(object):
    
    def __init__(self):
        self._global_plan = None
        self._route_assigned = False
        self._current_pose = Pose()
        self._current_speed  = 0.0
        self.target_route_point = None
        self.waypoints_queue = deque(maxlen=20000)
        self.current_pose_x = 0.0
        self.current_pose_y = 0.0  
#        self.x = []
#        self.y = []
#        self.waypoints_x = []
#        self.waypoints_y = []
#        self.cur_heading = []
#        self.tar_heading = []
#        self.DeltaHeading = []
#        self.lateral_e = []
#        self.steer_fd = []
#        self.waypoints_x_poly = []
#        self.waypoints_y_poly = []
#        self.tar_point_x = []
#        self.tar_point_y  = []
        
        self._route_subscriber = rospy.Subscriber(
            "frenet_path", Path, self.path_updated)
        self._odometry_subscriber = rospy.Subscriber(
            	"/localization/Imu_incremental", Odometry, self.odometry_updated)
        self.vehicle_control_publisher = rospy.Publisher(
            "cmd_delta", Twist, queue_size=1)
    
    def path_updated(self, path):
        rospy.loginfo("New plan with {} waypoints received.".format(len(path.poses)))
        self._global_plan = path
#        self._route_assigned = False
        
        
    def odometry_updated(self, odo):
        self._current_speed = math.sqrt(odo.twist.twist.linear.x ** 2 +
                                            odo.twist.twist.linear.y ** 2 +
                                            odo.twist.twist.linear.z ** 2) 
        self._current_pose = odo.pose.pose
        rospy.loginfo("current speed is :{}.".format(self._current_speed)) 
        
    def set_global_plan(self, current_plan):   
        self.waypoints_queue.clear()
        for elem in current_plan:
            self.waypoints_queue.append(elem.pose)     
            self._route_assigned = True
        rospy.loginfo("waypoints_queue_assigned...")
        
    def normalize_angle(self,angle):
        if angle > np.pi:
            angle -= 2.0 * np.pi
    
        if angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    
    def lateral_Controller(self,current_pose,current_speed):
        L = 1.6 
        current_pose_x = current_pose.position.x
        current_pose_y = current_pose.position.y
        current_pose_quaternion = (
                  current_pose.orientation.x,
                  current_pose.orientation.y,
                  current_pose.orientation.z,
                  current_pose.orientation.w)
        rospy.loginfo("current_pose_x:{} & current_pose_y:{}".format(current_pose_x,current_pose_y))
#        rospy.loginfo("current_pose_x & current_pose_x  is %d,%d " %(current_pose_x,current_pose_y))
        tempHeading = euler_from_quaternion(current_pose_quaternion) ### (Roll Pitch Yaw)
        current_pose_heading = self.normalize_angle(tempHeading[-1])
        rospy.loginfo("current_pose_heading is {}".format(current_pose_heading *180 /math.pi))
#        self.cur_heading.append(current_pose_heading)
#        np.save('/home/xin/carla-ros-bridge/ros-bridge/carla_ad_agent/src/carla_ad_agent/current_pose_heading.npy',self.cur_heading)
#        current_pose_x  = current_pose_x + 0.5*L * math.cos(current_pose_heading)
#        current_pose_y = current_pose_y +  0.5*L * math.sin(current_pose_heading) 
        
        front_pose_x  = current_pose_x + 0.5*L * math.cos(current_pose_heading)
        front_pose_y = current_pose_y +  0.5*L * math.sin(current_pose_heading) 
        
#        if current_pose_x and current_pose_y !=0:
#            self.x.append(current_pose_x)
#            self.y.append(current_pose_y)
#            np.save('/home/xin/carla-ros-bridge/ros-bridge/carla_ad_agent/src/carla_ad_agent/cur_y.npy',self.y)
#            np.save('/home/xin/carla-ros-bridge/ros-bridge/carla_ad_agent/src/carla_ad_agent/cur_x.npy',self.x)
        minError = 10000000000
        minIdx = 0
        numPath = len(self.waypoints_queue) 
#        rospy.loginfo("numPath is {}.".format(numPath)) 
        for i in range(numPath):  
            tempErr = math.sqrt ( (front_pose_x - self.waypoints_queue[i].position.x ) 
            ** 2  + (front_pose_y - self.waypoints_queue[i].position.y) 
            ** 2 )
            if tempErr <= minError:
                minIdx = i
                minError = tempErr
#                    np.save('/home/xin/carla-ros-bridge/ros-bridge/carla_ad_agent/src/carla_ad_agent/target_Idx', minIdx)
#                    np.save('/home/xin/carla-ros-bridge/ros-bridge/carla_ad_agent/src/carla_ad_agent/minError', minError) 
        rospy.loginfo("minIdx is {}.".format(minIdx))
        target_route_point = self.waypoints_queue[minIdx]
        quaternion_path = (
              target_route_point.orientation.x,
              target_route_point.orientation.y,
              target_route_point.orientation.z,
              target_route_point.orientation.w
            )    
        tempHeading = euler_from_quaternion(quaternion_path)
        tar_point_heading = self.normalize_angle(tempHeading[-1])  ################################################### the tar_point_heading
        rospy.loginfo("tar_point_heading is {}".format(tar_point_heading *180 /math.pi))
        rospy.loginfo("tar idx is {}".format(minIdx))
#        self.cur_heading.append(current_pose_heading)
#        self.tar_heading.append(tar_point_heading)

        e =  math.sqrt ( (front_pose_x - target_route_point.position.x) 
                    ** 2  + (front_pose_y - target_route_point.position.y) 
                    ** 2 )
#        rospy.loginfo("the lateral erro is:{}".format(e))   ################################################### the lateral e
#        if e < 200.0:
#            self.lateral_e.append(e)
#            np.save('/home/xin/carla-ros-bridge/ros-bridge/carla_ad_agent/src/carla_ad_agent/lateral_e.npy',self.lateral_e)
        
        v_begin_x = front_pose_x
        v_begin_y = front_pose_y
        v_end_x = v_begin_x + math.cos(self.normalize_angle(current_pose_heading))
        v_end_y = v_begin_y + math.sin(self.normalize_angle(current_pose_heading))
        v_vec = np.array([v_end_x - v_begin_x, v_end_y - v_begin_y, 0.0])
        w_vec = np.array([target_route_point.position.x -
                      v_begin_x, target_route_point.position.y -
                      v_begin_y, 0.0])   
        w_cos = np.dot(w_vec, v_vec) / (np.linalg.norm(w_vec) * np.linalg.norm(v_vec))
        w_sin = math.sqrt(1- w_cos ** 2)
        lat_e_0 = e *  w_sin  
        
        ex1 = current_pose_x - target_route_point.position.x  
        ey1 = current_pose_y - target_route_point.position.y 
        ex2 = math.sin(tar_point_heading)
        ey2 = -math.cos(tar_point_heading)
        e = np.sign(ex1*ex2 + ey1*ey2)*e
        lat_e_0 = np.sign(ex1*ex2 + ey1*ey2)*lat_e_0
        
        rospy.loginfo("the e is {}".format(e))
        rospy.loginfo("the lat e is {}".format(lat_e_0))
#        rospy.loginfo("the lat e is {}".format(lat_e_0))
        
#            self.lateral_e.append(e)
#        self.lateral_e.append(lat_e_0)
#            np.save('/home/xin/carla-ros-bridge/ros-bridge/carla_ad_agent/src/carla_ad_agent/lateral_e.npy',self.lateral_e)
            #计算航向角度误差
        deltaHeading = self.normalize_angle(tar_point_heading - current_pose_heading)
        rospy.loginfo("the rad deltaHeading is {}".format(deltaHeading))
	rospy.loginfo("the deltaHeading is {}".format(deltaHeading*180 /math.pi))
        
        k_v = 2.5
        k_phi = 1
      
#        _steering = k_phi * deltaHeading + math.atan(k_v*e/(current_speed+ k_soft))
        if current_speed < 0.01:
            tan = 0.0
        else:
            tan = math.atan(k_v*lat_e_0/current_speed) 
	rospy.loginfo("the tan is {}".format(tan))
        _steering = k_phi * deltaHeading + tan
        _steering = _steering * 180.0 / math.pi
        rospy.loginfo("the Stanley angle is {}".format(_steering))
        _steering = min(40,max(-40,_steering))
        
#        _steering = steer_fd + k_phi * deltaHeading + math.atan(k_v*e/(current_speed + k_soft))  # add a feedforward term 
#        return _steering,current_pose_x,current_pose_y, target_route_point
        rospy.loginfo("the actual steering angle is {}".format(_steering))
        rospy.loginfo("===========================================") 
        return _steering

    #save variavbles
#    def text_save(self,filename, data):#filename为写入CSV文件的路径，data为要写入数据列表.
#		with open(filename, 'a') as file_object:
#	         for i in range(len(data)):
#        		 s = str(data[i]).replace('[','').replace(']','')#去除[],这两行按数据不同，可以选择
#        		 s = s.replace("'",'').replace(',','') +'\n'   #去除单引号，逗号，每行末尾追加换行符
#        		 file_object.write(s)
#		file_object.close()

    
    def run_step(self):
        control = Twist()
        if not self._route_assigned and self._global_plan:
            rospy.loginfo("Assigning plan...")
            self.set_global_plan(self._global_plan.poses)
            self._route_assigned = True
        else:
            delta = self.lateral_Controller(self._current_pose, self._current_speed)
            control.angular.z = delta
        return control

    def run(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self._global_plan:
                control = self.run_step()
                if control:
                    self.vehicle_control_publisher.publish(control)
            else: 
                try:
                    r.sleep()
                except rospy.ROSInterruptException:
                    pass 
#            rospy.spin()             
def main():
    """

    main function

    :return:
    """
    rospy.init_node('path_tracking', anonymous=True)
    controller = StanleyControl()
    try:
        controller.run()
    finally:
        del controller
    rospy.loginfo("Done")
#        rospy.spin()

if __name__ == "__main__":
    main()
        


