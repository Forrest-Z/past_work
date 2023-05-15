//
// Created by xc on 2020/12/4.
//

#include "gp_lio/localization/localization_flow.h"

namespace gp_lio{
    LocalizationNode::LocalizationNode(ros::NodeHandle& nh,std::string config_path):
    nh_(nh),config_path_(config_path){
        sub_lidar_ = nh_.subscribe("/velodyne_points",5000,&LocalizationNode::LidarCallBack,this);
        sub_gnss_ = nh_.subscribe("/fix",5000,&LocalizationNode::GnssCallBack,this);
        pub_global_map_ = nh_.advertise<sensor_msgs::PointCloud2>("/localization/global_map",1);
        pub_local_map_ = nh_.advertise<sensor_msgs::PointCloud2>("/localization/local_map",1);
        pub_current_scan_ = nh_.advertise<sensor_msgs::PointCloud2>("/localization/current_scan",1);
        pub_odom_ptr_ = std::make_shared<OdometryPublisher>(nh_,"/localization/odom","/map","lidar",100);
        pub_tf_ptr_ = std::make_shared<TFBroadCaster>("/map","vehicle_link");

        std::thread*command(new std::thread(&LocalizationNode::Command,this));

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
        while (HasData()){
            if(!ValidData())
                continue;
            if(UpdateMatching()){
                PublishData();
            }

        }

    }

    void LocalizationNode::LidarCallBack(const sensor_msgs::PointCloud2Ptr lidar_msg) {
        lidar_buffer_.push(lidar_msg);
    }

    void LocalizationNode::GnssCallBack(const sensor_msgs::NavSatFixPtr gnss_msg) {
        gnss_buffer_.push(gnss_msg);
    }

    bool LocalizationNode::HasData() {
        return !(lidar_buffer_.empty() || gnss_buffer_.empty());

    }

    bool LocalizationNode::ValidData() {
        sensor_msgs::PointCloud2Ptr tmp_lidar;
        sensor_msgs::NavSatFixPtr tmp_gnss;
        tmp_lidar = lidar_buffer_.front();
        tmp_gnss = gnss_buffer_.front();
        pcl::PCLPointCloud2 lidar_pc2;
        pcl_conversions::toPCL(*tmp_lidar,lidar_pc2);
        pcl::fromPCLPointCloud2(lidar_pc2,*current_lidar_data_.lidar_data);
        current_lidar_data_.time = tmp_lidar->header.stamp.toSec();
        if(localization_.HasInited()){
            lidar_buffer_.pop();
            gnss_buffer_.pop();
            return  true;
        }
        current_gnss_data_.x() = floor(tmp_gnss->latitude*1e6);
        current_gnss_data_.y() = floor(tmp_gnss->longitude*1e6);
        current_gnss_data_.z() = 0.0;
        double time_diff = (tmp_lidar->header.stamp.toSec() - tmp_gnss->header.stamp.toSec());
//        std::cout << "time diff: " << time_diff <<std::endl;
        if(time_diff < -0.02){
            lidar_buffer_.pop();
            return false;
        }
        if(time_diff>0.02){
            gnss_buffer_.pop();
            return false;
        }
        lidar_buffer_.pop();
        gnss_buffer_.pop();
        return true;

    }

    bool LocalizationNode::UpdateMatching() {
        if(!localization_.HasInited()){
            std::cout << std::fixed << std::setprecision(1);
            localization_.GaussPoseByGnss(current_gnss_data_);
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
            if('s'==c){
                std::cout <<"get command s ..."<<std::endl;
                std::string dir_path = MAP_PATH + "/odom";
                std::string file_path = dir_path + "/odom_localization.txt";
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
