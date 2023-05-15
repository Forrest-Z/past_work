

#include <ros/ros.h>
#include <iostream>

#include <nav_msgs/Odometry.h>

using namespace std;
void joy_Callback(const nav_msgs::OdometryConstPtr &msg)
{
    std::cout <<" Odometry x is" << msg->pose.pose.position.x << "Odometry y is" << msg->pose.pose.position.y <<std::endl;
}

//主函数
int main(int argc, char **argv) {
  // ROS节点初始化
  ros::init(argc, argv, "Odometry_Test");

  // 创建节点句柄
  ros::NodeHandle n;

  ros::Subscriber joy_sub = n.subscribe("/localization/Imu_incremental", 10, joy_Callback);

  ros::spin();
  return 0;
 
}
