
//  ==> COPYRIGHT (C) 2019 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE <==
//  WARNING: COPYRIGHT (C) 2019 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE. ALL RIGHTS RESERVED.
//  THIS FILE AND THE SOURCE CODE IT CONTAINS (AND/OR THE BINARY CODE FILES FOUND IN THE SAME
//  FOLDER THAT CONTAINS THIS FILE) AND ALL RELATED SOFTWARE (COLLECTIVELY, "CODE") ARE SUBJECT
//  TO AN END USER LICENSE AGREEMENT ("AGREEMENT") BETWEEN XSENS AS LICENSOR AND THE AUTHORIZED
//  LICENSEE UNDER THE AGREEMENT. THE CODE MUST BE USED SOLELY WITH XSENS PRODUCTS INCORPORATED
//  INTO LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY USE, MODIFICATION, COPYING OR
//  DISTRIBUTION OF THE CODE IS STRICTLY PROHIBITED UNLESS EXPRESSLY AUTHORIZED BY THE AGREEMENT.
//  IF YOU ARE NOT AN AUTHORIZED USER OF THE CODE IN ACCORDANCE WITH THE AGREEMENT, YOU MUST STOP
//  USING OR VIEWING THE CODE NOW, REMOVE ANY COPIES OF THE CODE FROM YOUR COMPUTER AND NOTIFY
//  XSENS IMMEDIATELY BY EMAIL TO INFO@XSENS.COM. ANY COPIES OR DERIVATIVES OF THE CODE (IN WHOLE
//  OR IN PART) IN SOURCE CODE FORM THAT ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE
//  COPYRIGHT NOTICE AND THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
//  
//  THIS SOFTWARE CAN CONTAIN OPEN SOURCE COMPONENTS WHICH CAN BE SUBJECT TO 
//  THE FOLLOWING GENERAL PUBLIC LICENSES:
//  ==> Qt GNU LGPL version 3: http://doc.qt.io/qt-5/lgpl.html <==
//  ==> LAPACK BSD License:  http://www.netlib.org/lapack/LICENSE.txt <==
//  ==> StackWalker 3-Clause BSD License: https://github.com/JochenKalmbach/StackWalker/blob/master/LICENSE <==
//  ==> Icon Creative Commons 3.0: https://creativecommons.org/licenses/by/3.0/legalcode <==
//  

#ifndef IMUPUBLISHER_H
#define IMUPUBLISHER_H

#include "packetcallback.h"
#include <sensor_msgs/Imu.h>
#include<Eigen/Core>

static void variance_from_stddev_param(std::string param, double *variance_out)
{
    std::vector<double> stddev;
    if (ros::param::get(param, stddev))
    {
        if (stddev.size() == 3)
        {
            auto squared = [](double x) { return x * x; };
            std::transform(stddev.begin(), stddev.end(), variance_out, squared);
        }
        else
        {
            ROS_WARN("Wrong size of param: %s, must be of size 3", param.c_str());
        }
    }
    else
    {
        memset(variance_out, 0, 3 * sizeof(double));
    }
}

struct ImuPublisher : public PacketCallback
{
    ros::Publisher pub;
    std::string frame_id = DEFAULT_FRAME_ID;


    double orientation_variance[3];
    double linear_acceleration_variance[3];
    double angular_velocity_variance[3];

    ImuPublisher(ros::NodeHandle &node)
    {
        int pub_queue_size = 5;
        ros::param::get("~publisher_queue_size", pub_queue_size);
        pub = node.advertise<sensor_msgs::Imu>("imu/data", pub_queue_size);
        ros::param::get("~frame_id", frame_id);

        // REP 145: Conventions for IMU Sensor Drivers (http://www.ros.org/reps/rep-0145.html)
        variance_from_stddev_param("~orientation_stddev", orientation_variance);
        variance_from_stddev_param("~angular_velocity_stddev", angular_velocity_variance);
        variance_from_stddev_param("~linear_acceleration_stddev", linear_acceleration_variance);
    }

    void operator()(const XsDataPacket &packet, ros::Time timestamp)
    {
        bool quaternion_available = packet.containsOrientation();
        bool gyro_available = packet.containsCalibratedGyroscopeData();
        bool accel_available = packet.containsCalibratedAcceleration();

        geometry_msgs::Quaternion quaternion;
        if (quaternion_available)
        {
            XsQuaternion q = packet.orientationQuaternion();

            quaternion.w = q.w();
            quaternion.x = q.x();
            quaternion.y = q.y();
            quaternion.z = q.z();
        }

        geometry_msgs::Vector3 gyro;
        if (gyro_available)
        {
            XsVector g = packet.calibratedGyroscopeData();
             Eigen::Matrix3d T_gy,K_gy;
            Eigen::Vector3d B_gy,ang_,ang;
            ang <<  g[0],g[1],g[2];
            T_gy <<           1 ,0.000863856  ,-0.0055031,
                       0.00403737 ,          1  ,0.00101516,
                        -0.00264566, -0.00470071       ,    1;
            K_gy << 0.996546    ,    0     ,   0,
                            0 , 0.99788    ,    0,
                                 0    ,    0, 0.997493;
            B_gy << -0.00524163,0.00120228,0.0018925;
            ang_  = T_gy*K_gy*(ang - B_gy);
            gyro.x = ang_[0];
            gyro.y = ang_[1];
            gyro.z = ang_[2];
        }

        geometry_msgs::Vector3 accel;
        if (accel_available)
        {
            XsVector a = packet.calibratedAcceleration();
              Eigen::Matrix3d T_ac,K_ac;
            Eigen::Vector3d B_ac,acc_,acc;
            acc << a[0],a[1],a[2];
            T_ac << 1, -0.000546782, -6.04363e-05,
           0      ,      1  ,0.000442544,
          -0  ,          0       ,     1;
            K_ac << 1.00151  ,     0   ,    0,
      0 ,1.00091     ,  0,
      0   ,    0 ,1.00126;
            B_ac << -0.0204286,0.016756,0.0239116;
            acc_  = T_ac*K_ac*(acc - B_ac);

            accel.x = acc_[0];
            accel.y = acc_[1];
            accel.z = acc_[2];
            double  G_test;
            G_test = sqrt(pow(accel.x,2) + pow(accel.y,2) + pow(accel.z,2));
            std::cout<<"grevity_xsens:"<<G_test<<std::endl;
            
        }

        // Imu message, publish if any of the fields is available
        if (quaternion_available || accel_available || gyro_available)
        {
            sensor_msgs::Imu msg;

            msg.header.stamp = timestamp;
            msg.header.frame_id = frame_id;

            msg.orientation = quaternion;
            if (quaternion_available)
            {
                msg.orientation_covariance[0] = orientation_variance[0];
                msg.orientation_covariance[4] = orientation_variance[1];
                msg.orientation_covariance[8] = orientation_variance[2];
            }
            else
            {
                msg.orientation_covariance[0] = -1; // mark as not available
            }

            msg.angular_velocity = gyro;
            if (gyro_available)
            {
                msg.angular_velocity_covariance[0] = angular_velocity_variance[0];
                msg.angular_velocity_covariance[4] = angular_velocity_variance[1];
                msg.angular_velocity_covariance[8] = angular_velocity_variance[2];
            }
            else
            {
                msg.angular_velocity_covariance[0] = -1; // mark as not available
            }

            msg.linear_acceleration = accel;
            if (accel_available)
            {
                msg.linear_acceleration_covariance[0] = linear_acceleration_variance[0];
                msg.linear_acceleration_covariance[4] = linear_acceleration_variance[1];
                msg.linear_acceleration_covariance[8] = linear_acceleration_variance[2];
            }
            else
            {
                msg.linear_acceleration_covariance[0] = -1; // mark as not available
            }

            pub.publish(msg);
        }
    }
};

#endif
