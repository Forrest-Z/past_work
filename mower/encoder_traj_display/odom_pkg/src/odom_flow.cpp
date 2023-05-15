/*
 * @Author: your name
 * @Date: 2021-09-15 14:22:27
 * @LastEditTime: 2022-05-20 11:47:57
 * @LastEditors: luo
 * @Description: In User Settings Edit
 * @FilePath: /ws_catkin/src/odom_pkg/src/odom_flow.cpp
 */

#include <sensor_msgs/JointState.h>
#include "odom_pkg/odom_flow.hpp"
#include <geometry_msgs/Pose2D.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <deque>


boost::array<double, 36UL> ODOM_POSE_COVARIANCE = {1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3};

boost::array<double, 36UL> ODOM_POSE_COVARIANCE2 = {1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9};
 
boost::array<double, 36UL> ODOM_TWIST_COVARIANCE = {1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3};
boost::array<double, 36UL> ODOM_TWIST_COVARIANCE2 = {1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9};


WheelOdomMsg::WheelOdomMsg(ros::NodeHandle& nh): nh_(nh), is_fisrst_(true)
{
    std::string config_path_ = "/home/luo/carto_ws/catkin_works/src/odom_pkg/config/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_path_);
    wheel_base_ = config_node["wheel_base"].as<float>();
    wheel_radius_ = config_node["wheel_radius"].as<float>();
    num_ = config_node["num"].as<int>();

    std::cout<< "wheel_base_ = " << wheel_base_ << std::endl;
    std::cout<< "wheel_radius_ = " << wheel_radius_ << std::endl;
    std::cout<< "num_ = " << num_ << std::endl;

    
    odom_sub_ = nh_.subscribe("/motor_velocity", 1000, &WheelOdomMsg::WheelOdomCallBack, this);

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/encoder_odom", 1000);

}

WheelOdomMsg::~WheelOdomMsg()
{

}

void WheelOdomMsg::WheelOdomCallBack(const sensor_msgs::JointState& odom_msg)
{
    buff_mutex_.lock();

    sensor_odom_ptr = odom_msg;

    current_time = odom_msg.header.stamp;
    
    if(is_fisrst_)
    {

        last_time = current_time;

        left_current_position_ = 0;
        right_current_position_ = 0;
        
        left_last_position_ = left_current_position_;
        right_last_position_ = right_current_position_;
        
        delta_left_position_ = 0;
        delta_right_position_ = 0;

        x = 0.0;
        y = 0.0;
        th = 0.0;

        is_fisrst_ = false;
    }

    float mid_FreL, mid_FreR;

    nl_speed = odom_msg.velocity.at(0) / 51;
    nr_speed = odom_msg.velocity.at(1) / 51;


    left_last_position_ = left_current_position_;
    right_last_position_ = right_current_position_;

  
    buff_mutex_.unlock();
}

int i_count = 0;
bool is_first_data(true);
void WheelOdomMsg::Process()
{
    std::cout<< "**************Process***************" << std::endl;


    current_time = sensor_odom_ptr.header.stamp;
    
    right_w = nr_speed * PI * 2;
    left_w = nl_speed * PI * 2;


    right_v = -nr_speed * wheel_radius_ * PI * 2;
    left_v = nl_speed * wheel_radius_ * PI * 2;
    
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

    // float th2 = 0;
    th2 += delta_th;
    std::cout<< "delta_th = " << delta_th << std::endl;
    std::cout<< "th2 = " << th2 << std::endl;
    
    last_time = current_time;
    
    x += delta_s * cos(delta_th + th);
    y += delta_s * sin(delta_th + th);
    th += delta_th;


    // th = yaw;

    if(th > PI)
    {
        th -= PI2;
    }else if(th < -PI) 
    {
        th += PI2;
    }

 
    std::cout<< "time = " << current_time << std::endl;
    std::cout<< "x = " << x << std::endl;
    std::cout<< "y = " << y << std::endl;
    std::cout<< "th = " << th << std::endl;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    odom_data_.header.stamp = sensor_odom_ptr.header.stamp;
    odom_data_.header.frame_id = "odom";
    odom_data_.child_frame_id = "base_footprint";

    odom_data_.pose.pose.position.x = x;
    odom_data_.pose.pose.position.y = y;
    odom_data_.pose.pose.position.z = 0.0;
    odom_data_.pose.pose.orientation = odom_quat;

    odom_pub_.publish(odom_data_);
}


bool WheelOdomMsg::PubOdom(float v, float w, float r)
{
    
    // double delta_left_position = 

    // odom_data_.header.stamp = ros::Time::now();
    // odom_data_.header.frame_id = "map";
    // odom_data_.child_frame_id = "odom_link";
    // odom_data_.pose.pose.position = 
    // odom_data_.pose.pose.orientation = 
    
}