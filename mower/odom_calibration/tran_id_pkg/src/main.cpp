/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2022-03-18 13:20:24
 * @LastEditors: luo
 * @LastEditTime: 2022-06-22 14:18:00
 */
#include <iostream>
#include <vector>
#include <thread>

#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CompressedImage.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/JointState.h>

#include <mutex>

using namespace std;

struct ImuData
{
    sensor_msgs::Imu imu;

    double roll;
    double pitch;
    double yaw;
};

const double D = 0.375;

ros::Publisher imu_pub;
ros::Publisher laser_pub;
ros::Publisher laser_d_pub;
ros::Publisher odom_pub;
ros::Publisher odom_vec3_pub;

image_transport::Publisher image_pub;

sensor_msgs::LaserScan current_scan;
sensor_msgs::Imu current_imu;
std::mutex imu_mutex, laser_mutex;

ImuData imu_data_str;

Eigen::Vector3d g(0, 0, -9.809432);

void ImuCallBack(const sensor_msgs::ImuConstPtr& imu_msg)
{
    std::unique_lock<std::mutex> lock(imu_mutex);

    sensor_msgs::Imu imu_data;
    sensor_msgs::Imu last_imu_data;

    imu_data = *imu_msg;
    imu_data.header.stamp = ros::Time::now();
    imu_data.header.frame_id = "imu_link";

    static bool sensor_inited = false;

    if(sensor_inited == false)
    {
        current_imu = imu_data;
        last_imu_data = imu_data;
    }

    current_imu = imu_data;


    double delta_time = current_imu.header.stamp.toSec() - last_imu_data.header.stamp.toSec();

    Eigen::Vector3d wb;
    wb[0] = 0.5 * (current_imu.angular_velocity.x + last_imu_data.angular_velocity.x);
    wb[1] = 0.5 * (current_imu.angular_velocity.y + last_imu_data.angular_velocity.y);
    wb[2] = 0.5 * (current_imu.angular_velocity.z + last_imu_data.angular_velocity.z);

    wb = wb - g;
    wb = wb * delta_time;

    double wb_mag = wb.norm();

    std::cout << "wb_mag = " << wb_mag << std::endl;

    Eigen::Vector3d wb_delta_dir = wb.normalized();

    double wb_cos = std::cos(wb_mag / 2.0);
    double wb_sin = std::sin(wb_mag / 2.0);

    Eigen::Quaterniond dq(wb_cos, 
                            wb_sin * wb_delta_dir.x(),
                            wb_sin * wb_delta_dir.y(),
                            wb_sin * wb_delta_dir.z()
                            );

    Eigen::Matrix3d matrix = dq.toRotationMatrix();

    Eigen::Vector3d euler_angle = matrix.eulerAngles(2, 1, 0);
    
    imu_data_str.imu = imu_data;

    imu_data_str.roll = euler_angle[0];
    imu_data_str.pitch = euler_angle[1];
    imu_data_str.yaw = euler_angle[2];


    std::cout << "roll = " << euler_angle[0] << std::endl;
    std::cout << "pitch = " << euler_angle[1] << std::endl;
    std::cout << "yaw = " << euler_angle[2] << std::endl;
        
    last_imu_data = current_imu;

    imu_pub.publish(imu_data);

}

void LaserCallBack(const sensor_msgs::LaserScanConstPtr& laser_msg)
{  
     
    std::unique_lock<std::mutex> lock(laser_mutex);

    sensor_msgs::LaserScan laser_data;
    // sensor_msgs::LaserScan laser_b_data;

    // sensor_msgs::Imu imu_data;

    laser_data = *laser_msg;
    laser_data.header.stamp = laser_msg->header.stamp;
    laser_data.header.frame_id = "lidar";

    // current_scan = laser_data;
    // laser_b_data = laser_data;

    laser_pub.publish(laser_data);


}


cv::Mat imgCallback;
void ImageCallBack(const sensor_msgs::CompressedImageConstPtr& image_msg)
{

    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    imgCallback = cv_ptr_compressed->image;
    
    // cv::imshow("imgCallback",imgCallback);

    image_pub.publish(cv_ptr_compressed->toImageMsg());
    cv::waitKey(3);

}



void OdomCallBack(const nav_msgs::OdometryConstPtr& odom_msg)
{
    geometry_msgs::Vector3Stamped odom_velo;
    odom_velo.header.stamp = odom_msg->header.stamp;
    odom_velo.header.frame_id = "velo_link";
    
    double v_x = odom_msg->twist.twist.linear.x;
    double v_z = odom_msg->twist.twist.angular.z;
    
    odom_velo.vector.x = v_x - 0.5 * (v_z * D * 2);
    odom_velo.vector.y = v_x + 0.5 * (v_z * D * 2);
    odom_velo.vector.z = 0.0;

    odom_pub.publish(odom_velo);
}

void MotorVelocityCallBack(const sensor_msgs::JointStateConstPtr& motor_velo_msg)
{
    geometry_msgs::Vector3Stamped motor_velo;

    motor_velo.header = motor_velo_msg->header;
    motor_velo.vector.x = motor_velo_msg->velocity[0] / 60.0;
    motor_velo.vector.y = motor_velo_msg->velocity[1] / 60.0;
    motor_velo.vector.z = 0.0;

    odom_vec3_pub.publish(motor_velo);
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tran_id_node");

    ros::NodeHandle nh;



    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 10000, ImuCallBack);
    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/lidar1/scan", 10000, LaserCallBack);

    image_transport::ImageTransport it(nh);
    ros::Subscriber image_sub = nh.subscribe("/image_raw/compressed", 10, ImageCallBack);
    
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/raw_odom", 100, OdomCallBack);

    ros::Subscriber odom_sub2 = nh.subscribe("/motor_velocity", 100, MotorVelocityCallBack);

    image_pub = it.advertise("/raw_image", 10000);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 10000);
    laser_pub = nh.advertise<sensor_msgs::LaserScan>("/laser_data", 10000);
    // laser_d_pub = nh.advertise<sensor_msgs::LaserScan>("/laser_d_data", 10000);

    odom_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/odom_velo", 100);

    odom_vec3_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/ff_motor_velocity", 100);

    ros::spin();

    std::cout << "tran_id_pkg" << std::endl;

    return 0;
}