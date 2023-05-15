//
// Created by xc on 2020/12/4.
//

#ifndef GP_LIO_LOCALIZATION_FLOW_H
#define GP_LIO_LOCALIZATION_FLOW_H
// cpp
#include <iostream>
#include <mutex>
#include <cstdio>
#include <unistd.h>
#include <thread>
//pcl
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
// ros
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <queue>

// local
#include "gp_lio/localization/odom_publisher.h"
#include "gp_lio/localization/tf_broadcaster.h"
#include "gp_lio/localization/lgi_based_localization.h"
#include "gp_lio/utility/file_manager.h"

namespace gp_lio{


    struct LidarData{
        LidarData():lidar_data(new pcl::PointCloud<pcl::PointXYZI>){
        }
        double time = 0.0;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_data;
    };


    class LocalizationNode{
    public:
        LocalizationNode(ros::NodeHandle& nh,std::string config_path);
        bool Run();

    private:
        void GnssCallBack(const sensor_msgs::NavSatFixPtr gnss_msg);
        void LidarCallBack(const sensor_msgs::PointCloud2Ptr lidar_msg);
        bool HasData();
        bool ValidData();
        bool UpdateMatching();
        bool PublishData();
        void Command();

    private:
        std::string config_path_;
        FileManager file_manager_;
        std::queue<sensor_msgs::NavSatFixPtr> gnss_buffer_;
        std::queue<sensor_msgs::PointCloud2Ptr> lidar_buffer_;

        ros::NodeHandle nh_;
        ros::Subscriber sub_lidar_;
        ros::Subscriber sub_gnss_;

        ros::Publisher pub_global_map_;
        ros::Publisher pub_local_map_;
        ros::Publisher pub_current_scan_;
        std::shared_ptr<OdometryPublisher> pub_odom_ptr_;
        std::shared_ptr<TFBroadCaster> pub_tf_ptr_;
        Localization localization_;

        LidarData current_lidar_data_;
        Eigen::Vector3d current_gnss_data_;
        Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
        std::vector<std::pair<double,std::array<float ,7>>> odom_v_;
    };
}
#endif //GP_LIO_LOCALIZATION_FLOW_H
