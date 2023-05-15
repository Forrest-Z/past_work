/*
 * @Author: your name
 * @Date: 2021-09-07 16:29:35
 * @LastEditTime: 2022-06-13 13:32:00
 * @LastEditors: luo
 * @Description: In User Settings Edit
 * @FilePath: /ws_catkin/src/odom_pkg/src/odom_node.cpp
 */

#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <yaml-cpp/yaml.h>
#include <deque>
#include <mutex>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>


using namespace std;


ros::Publisher odom_pub_;
ros::Publisher odom_cal_pub_;

float wheel_base_;
float wheel_radius_;
std::string config_path_;

float nr_speed = 0;
float nl_speed = 0;

float left_v = 0;
float right_v = 0;
float odom_v = 0;

float left_w = 0;
float right_w = 0;
float odom_w = 0;

float delta_left_position_ = 0;
float delta_right_position_ = 0;

float delta_s = 0;
float delta_th = 0;
float x = 0.0;
float y = 0.0;
float th = 0.0;

bool is_first_ = true;

nav_msgs::Odometry odom_data_;

ros::Time current_time, last_time;

// const double D = 0.375;

void WheelOdomCallBack(const nav_msgs::OdometryConstPtr& odom_msg)
{
    current_time = odom_msg->header.stamp;

    if(is_first_)
    {
        last_time = current_time;

        x = 0.0;
        y = 0.0;
        th = 0.0;

        delta_left_position_ = 0;
        delta_right_position_ = 0;

        is_first_ = false;

        return;
    }
    
    double v_x = odom_msg->twist.twist.linear.x;
    double v_z = odom_msg->twist.twist.angular.z;


    nl_speed = v_x - 0.5 * (v_z * wheel_radius_ * 2);
    nr_speed = v_x + 0.5 * (v_z * wheel_radius_ * 2);

    geometry_msgs::Vector3Stamped odom_cal;
    odom_cal.header.stamp = odom_msg->header.stamp;
    odom_cal.header.frame_id = "odom_link";
    odom_cal.vector.x = nl_speed;
    odom_cal.vector.y = nr_speed;
    odom_cal.vector.z = 0.0;

    odom_cal_pub_.publish(odom_cal);


    right_w = nr_speed * M_PI * 2;
    left_w = nl_speed * M_PI * 2;

    right_v = nr_speed * wheel_radius_ * M_PI * 2;
    left_v = nl_speed * wheel_radius_ * M_PI * 2;

    std::cout<< "velo_left = " << left_v << std::endl;
    std::cout<< "velo_right = " << right_v << std::endl;

    odom_v =  (right_v + left_v) / 2;
    odom_w = ( right_v - left_v) / wheel_base_ ;


    float dt = (current_time - last_time).toSec();
    delta_right_position_ = right_v * dt;
    delta_left_position_ = left_v * dt;

    std::cout<< "delta_left_p = " << delta_left_position_ << std::endl;
    std::cout<< "delta_right_p = " << delta_right_position_ << std::endl;

    delta_s = (delta_right_position_ + delta_left_position_) / 2.0;
    delta_th = odom_w * dt;

     
    last_time = current_time;
    
    x += delta_s * cos(delta_th + th);
    y += delta_s * sin(delta_th + th);
    th += delta_th;


    if(th > M_PI)
    {
        th -= (M_PI * 2);
    }else if(th < -M_PI) 
    {
        th += (M_PI * 2);
    }


    std::cout<< "time = " << current_time << std::endl;
    std::cout<< "x = " << x << std::endl;
    std::cout<< "y = " << y << std::endl;
    std::cout<< "th = " << th * 180 / M_PI << std::endl;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    odom_data_.header.stamp = current_time;
    odom_data_.header.frame_id = "odom";
    odom_data_.child_frame_id = "base_footprint";

    odom_data_.pose.pose.position.x = x;
    odom_data_.pose.pose.position.y = y;
    odom_data_.pose.pose.position.z = 0.0;
    odom_data_.pose.pose.orientation = odom_quat;

    odom_pub_.publish(odom_data_);

    static tf::TransformBroadcaster boadcaster;
    
    tf::Quaternion tf_qua(odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w);
    tf::Vector3 tf_vec(x, y, 0.0);

    boadcaster.sendTransform(
                            tf::StampedTransform(tf::Transform(tf_qua, tf_vec), 
                            ros::Time::now(),
                            "map",
                            "base_link"));

}



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "odometry_crawler_node");

    ros::NodeHandle nh;

    std::string config_path_ = "/home/luo/carto_ws/catkin_works/src/odom_pkg/config/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_path_);
    wheel_base_ = config_node["wheel_base"].as<float>();
    wheel_radius_ = config_node["wheel_radius"].as<float>();

    ros::Subscriber odom_sub = nh.subscribe("/motor_velocity", 2, WheelOdomCallBack);

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/crawler_encoder_odom", 2);

    odom_cal_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>("/odom_calibration", 2);

    ros::spin();

    return 0;
}