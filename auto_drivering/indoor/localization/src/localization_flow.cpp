//
// Created by xc on 2020/12/4.
//

#include "localization_flow.h"
#include <glog/logging.h>
#include <time.h>

namespace gp_lio{
    LocalizationNode::LocalizationNode(ros::NodeHandle& nh):
    nh_(nh){
        sub_lidar_ = nh_.subscribe("/velodyne_points",5000,&LocalizationNode::LidarCallBack,this);
        sub_aruco_ = nh_.subscribe("/aruco_detect_pose",5000,&LocalizationNode::ArucoCallBack,this);  //aruco_detect_pose
        pub_global_map_ = nh_.advertise<sensor_msgs::PointCloud2>("/localization/global_map",1);
        pub_local_map_ = nh_.advertise<sensor_msgs::PointCloud2>("/localization/local_map",1);
        pub_current_scan_ = nh_.advertise<sensor_msgs::PointCloud2>("/localization/current_scan",1);
        pub_odom_ptr_ = std::make_shared<OdometryPublisher>(nh_,"/localization/odom","/map","lidar",100);
        pub_tf_ptr_ = std::make_shared<TFBroadCaster>("/map","vehicle_link");

//        std::thread*command(new std::thread(&LocalizationNode::Command,this));

    }

    bool LocalizationNode::Run() {
        if(localization_.HasNewGlobalMap() && pub_global_map_.getNumSubscribers()){
            pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            localization_.GetGlobalMap(global_map_ptr);
            sensor_msgs::PointCloud2 global_map_msgs;
            pcl::toROSMsg(*global_map_ptr,global_map_msgs);
            global_map_msgs.header.frame_id = "/map";
            pub_global_map_.publish(global_map_msgs);
        }
        if(localization_.HasNewLocalMap() && pub_local_map_.getNumSubscribers()){
            pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            localization_.GetLocalMap(local_map_ptr);
            sensor_msgs::PointCloud2 local_map_msgs;
            pcl::toROSMsg(*local_map_ptr,local_map_msgs);
            local_map_msgs.header.frame_id ="/map";
            pub_local_map_.publish(local_map_msgs);
        }
        while (HasData()){  //data buffer is not empty
            if(!ValidData())
                continue;
            if(UpdateMatching()){
                PublishData();
            }

        }

    }

    void LocalizationNode::LidarCallBack(const sensor_msgs::PointCloud2Ptr lidar_msg) {
        lidar_buffer_.push(lidar_msg);
//        std::cout <<"callbackLidar: "<< lidar_buffer_.size()<<std::endl;
    }

    void LocalizationNode::ArucoCallBack(const zed_interfaces::PoseStampedArrayConstPtr marker_msg) {
        aruco_buffer_.push(marker_msg);
//        std::cout <<"callbackAruco: "<< aruco_buffer_.size()<<std::endl;
    }

    bool LocalizationNode::HasData() {
        return !(lidar_buffer_.empty() || aruco_buffer_.empty());

    }

    bool LocalizationNode::ValidData() {

        sensor_msgs::PointCloud2Ptr tmp_lidar;
        zed_interfaces::PoseStampedArrayConstPtr tmp_aruco;
        tmp_lidar = lidar_buffer_.front();
        pcl::PCLPointCloud2 lidar_pc2;
        pcl_conversions::toPCL(*tmp_lidar,lidar_pc2);
        pcl::fromPCLPointCloud2(lidar_pc2,*current_lidar_data_.lidar_data);
        current_lidar_data_.time = tmp_lidar->header.stamp.toSec();
        if(localization_.HasInited()){
            lidar_buffer_.pop();
            return  true;
        }
        if(aruco_buffer_.empty())
            return false;
        tmp_aruco = aruco_buffer_.front();

        if(tmp_aruco->poses.empty())
        {
            std::cout << "not detect aruco pose" << std::endl;
            aruco_buffer_.pop();
            return false;
        }
        std:: cout << tmp_aruco->poses.front().pose.orientation.x << "  " << tmp_aruco->poses.front().pose.position.z << std::endl;
        if(tmp_aruco->poses.front().pose.position.z >3)
        {
            std::cout << "not detect valid aruco data" << std::endl;
            aruco_buffer_.pop();
            return false;
        }
        double time_diff_0 = tmp_lidar->header.stamp.toSec() - tmp_aruco->header.stamp.toSec();
        if(time_diff_0 < -0.02)
        {
            lidar_buffer_.pop();
            return false;
        }
        if(time_diff_0 > 0.02)
        {
            aruco_buffer_.pop();
            return false;
        }

        current_aruco_data_.id = tmp_aruco->poses.front().header.seq;
        current_aruco_data_.translation.x() = tmp_aruco->poses.front().pose.position.x;
        current_aruco_data_.translation.y() = tmp_aruco->poses.front().pose.position.y;
        current_aruco_data_.translation.z() = tmp_aruco->poses.front().pose.position.z;
        current_aruco_data_.orientation.x() = tmp_aruco->poses.front().pose.orientation.x;
        current_aruco_data_.orientation.y() = tmp_aruco->poses.front().pose.orientation.y;
        current_aruco_data_.orientation.z() = tmp_aruco->poses.front().pose.orientation.z;
        current_aruco_data_.orientation.w() = tmp_aruco->poses.front().pose.orientation.w;
        lidar_buffer_.pop();
        aruco_buffer_.pop();
        return true;

    }

    bool LocalizationNode::UpdateMatching() {
        if(!localization_.HasInited()){
            std::cout << std::fixed << std::setprecision(5);
// TODO Gauss init pose by aruco marker
            std::cout << "UpdateMatching ---> current_aruco_data_; " << current_aruco_data_.getPose()<<std::endl;
            Eigen::Matrix4f current_gauss_pose;
            if(!localization_.GaussPoseByAruco(current_aruco_data_,current_gauss_pose))
                return false;
//            pub_odom_ptr_->Publish(current_gauss_pose,current_lidar_data_.time);
//            sleep(1000);
        }
        bool flag = localization_.Update(current_lidar_data_.lidar_data,laser_odometry_);
        Eigen::Quaternionf tmp_q(laser_odometry_.block<3,3>(0,0));
        Eigen::Vector3f tmp_p(laser_odometry_.topRightCorner(3,1));
        double tmp_t = current_lidar_data_.time;
        std::array<float,7> tmp_odom{tmp_p.x(),tmp_p.y(),tmp_p.z(),
                                      tmp_q.x(),tmp_q.y(),tmp_q.z(),tmp_q.w()};
        odom_v_.push_back(std::make_pair(tmp_t,tmp_odom));
        return flag;

    }

    bool LocalizationNode::PublishData() {

        pub_tf_ptr_->SendTransform(laser_odometry_,current_lidar_data_.time);
        pub_odom_ptr_->Publish(laser_odometry_,current_lidar_data_.time);
        pcl::PointCloud<pcl::PointXYZI>::Ptr current_scan(new pcl::PointCloud<pcl::PointXYZI> );
        localization_.GetCurrentScan(current_scan);
        sensor_msgs::PointCloud2 current_scan_msgs;
        pcl::toROSMsg(*current_scan,current_scan_msgs);
        current_scan_msgs.header.frame_id = "/map";
        pub_current_scan_.publish(current_scan_msgs);
        return true;
    }

    void LocalizationNode::Command() {
        while (1){
            char c = std::getchar();
            if('l'==c){
                std::cout <<"get command s ..."<<std::endl;
                std::string dir_path = MAP_PATH + "/odom";
                std::string file_path = dir_path + "/odom_l.txt";
                if(file_manager_.CreatDirectory(dir_path)){
                    std::FILE* ofile_odom = std::fopen(file_path.c_str(),"w");
                    ROS_ERROR("ready to save odom datas, please waite ...");
                    for(auto & odom:odom_v_){
                        std::fprintf(ofile_odom,"%lf %f %f %f %f %f %f %f\n"
                                ,odom.first,odom.second[0],odom.second[1],odom.second[2],odom.second[3],odom.second[4],odom.second[5],odom.second[6]);
                    }
                    std::fclose(ofile_odom);
                    ROS_ERROR("save odom datas finshed ...");
                    ros::shutdown();
                }
            }
            std::chrono::microseconds dura(5);
            std::this_thread::sleep_for(dura);
        }
    }



}
