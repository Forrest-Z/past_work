//
// Created by xc on 2021/3/8.
//

#include "luvi-mapping/globalOptimization.h"

namespace LIRO{
    GlobalOptimization::GlobalOptimization():nh_("~") {
        nh_.getParam("/coef_loop_closure", coef_loop_closure_);
        nh_.getParam("/loop_closure_frequence", loop_closure_frequence_);
        nh_.getParam("/isam_relinearize_threshold", isam_relinearize_threshold_);
        nh_.getParam("/odom_r_noise", odom_r_noise_);
        nh_.getParam("/odom_t_noise", odom_t_noise_);
        nh_.getParam("/loop_closure_search_radius", loop_closure_search_radius_);
        nh_.getParam("/history_keyframe_num", history_keyframe_num_);
        nh_.getParam("/data_dir_path", data_dir_path_);

        lio_opt_initializd_ = false;
        loop_count = 0;
        loop_is_closed_ = false;
        matching_min_id_distance = 50;
        keyframe_time_diff_ = 0.3;
        keyframe_rotation_ = 0.2; // 15
        keyframe_distance_ = 1.0;
        Eigen::Matrix3d rota = Eigen::Matrix3d::Zero();
        // rota << 0, -1, 0, 1, 0, 0,0, 0, 1;
        rota << 1, 0, 0, 
                        0, 1, 0,
                        0, 0, 1;
        extrinsic_maritx4_ = Eigen::Matrix4d::Identity();
        extrinsic_maritx4_(0, 3) = 0;
        extrinsic_maritx4_(1, 3) = 0;
        extrinsic_maritx4_(2, 3) = 0.3330773;
        extrinsic_maritx4_.block<3,3>(0,0) = rota;
 
        Eigen::Quaterniond exq(extrinsic_maritx4_.block<3,3>(0,0));

        Eigen::Vector3d ext = extrinsic_maritx4_.block<3,1>(0,3);
        lidar2imu_ = gtsam::Pose3(gtsam::Rot3(exq),gtsam::Point3(ext.x(),ext.y(),ext.z()));
        // cam2lidar_ = gtsam::Pose3({0,0,1,-1,0,0,0,-1,0},{0.055,0.035,-0.060});
        cam2lidar_ = gtsam::Pose3({0, 0, 1 , -1, 0, 0, 0, -1, 0},{0.4634345, 0.0000001, -0.300378104});
        imu2lidar_ = lidar2imu_.inverse();

        // imu
        last_imu_opt_t_ = -1;
        imu_acc_noise_ = 0.1 ;
        imu_gyr_noise_ = 0.03;
        imu_acc_bias_noise_ = 0.02;
        imu_gyr_bias_noise_ = 4.0e-6;
        imu_gravity_ = 9.81007;
        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imu_gravity_);
        p->accelerometerCovariance = gtsam::Matrix33::Identity(3,3) * std::pow(imu_acc_noise_,2);
        p->gyroscopeCovariance = gtsam::Matrix33::Identity(3,3) * std::pow(imu_gyr_noise_,2);
        p->integrationCovariance = gtsam::Matrix33::Identity(3,3) * std::pow(1e-4,2); // v ---->  p
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0,0,0,0,0,0).finished());

        prior_pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<< 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); //1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2
        prior_vel_noise_ =  gtsam::noiseModel::Isotropic::Sigma(3,0.1);
        prior_bias_noise_ =  gtsam::noiseModel::Isotropic::Sigma(6,1e-3);
        correction_noise1_ = gtsam::noiseModel::Isotropic::Sigma(6,1e-3);   // rad,rad,rad,m, m, m //权重调大
        noise_model_between_bias_ = (gtsam::Vector(6) << imu_acc_bias_noise_,imu_acc_bias_noise_,imu_acc_bias_noise_,
                imu_gyr_bias_noise_,imu_gyr_bias_noise_,imu_gyr_bias_noise_).finished();
        imu_integrator_ = new gtsam::PreintegratedImuMeasurements(p,prior_imu_bias);

        // isam
        ISAM2Params parameters;
        parameters.relinearizeThreshold = isam_relinearize_threshold_;
        parameters.relinearizeSkip = 1;
        isam_odom_ = new gtsam::ISAM2(parameters);
        isam_li_ = new gtsam::ISAM2(parameters);

        surf_filter_for_visualization_.setLeafSize(0.6,0.6,0.6);
        corner_filter_for_visualization_.setLeafSize(0.4,0.4,0.4);
        surf_filter_for_icp_.setLeafSize(0.4,0.4,0.4);
        corner_filter_for_icp_.setLeafSize(0.2,0.2,0.2);
        icp_fitness_score_ = 0.3;
        icp_.setMaximumIterations(150);
        icp_.setMaxCorrespondenceDistance(100);
        icp_.setTransformationEpsilon(1e-6);
        icp_.setEuclideanFitnessEpsilon(1e-6);
        icp_.setRANSACIterations(0);

        sub_aruco_ = nh_.subscribe<zed_interfaces::PoseStampedArray>("/aruco_detect_pose",1000,&GlobalOptimization::ArucoCallback,this);
        sub_uwb_ = nh_.subscribe<nlink_parser::LinktrackNodeframe2>("/nlink_linktrack_nodeframe2",100,&GlobalOptimization::UwbCallback,this);
        sub_imu_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data_xsens",100,&GlobalOptimization::ImuCallback,this);
        sub_lidar_odom_ = nh_.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init",100,&GlobalOptimization::LidarOdomCallback,this);
        sub_corner_pointcloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last",100,&GlobalOptimization::CornerPointcloudCallback,this);
        sub_surf_pointcloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last",100,&GlobalOptimization::SurfPointcloudCallback,this);
        pub_global_map_ = nh_.advertise<sensor_msgs::PointCloud2>("/liro/globalmap",2);
        pub_surf_map_ = nh_.advertise<sensor_msgs::PointCloud2>("/liro/surf_map",2);
        pub_corner_map_ = nh_.advertise<sensor_msgs::PointCloud2>("/liro/corner_map",2);
        pub_global_path_lo_ = nh_.advertise<sensor_msgs::PointCloud2>("/liro/lo_path",2);
        pub_global_path_lio_ = nh_.advertise<sensor_msgs::PointCloud2>("/liro/lio_path",2);
        pub_global_path_liro_ = nh_.advertise<sensor_msgs::PointCloud2>("/liro/liro_path",2);
        pub_uwb_data_flag_ = nh_.advertise<std_msgs::Bool>("/liro/data_flag",1);
        pub_aruco_ = nh_.advertise<visualization_msgs::MarkerArray>("/liro/aruco",2);
        pub_aruco_opt_ = nh_.advertise<visualization_msgs::MarkerArray>("/liro/aruco_opt",2);
        AllocateMemory();
//        LoadMap();
        std::thread* command(new std::thread(&GlobalOptimization::Command,this));

    }
    GlobalOptimization::~GlobalOptimization() {}

    void GlobalOptimization::LoadMap() {
        std::string map_dir_path = data_dir_path_ + "/map";
        std::string pg_path = data_dir_path_ + "/pose_graph/pose_graph1.txt";
        std::FILE* pg_ifile = std::fopen(pg_path.c_str(),"r");
        if (pg_ifile == NULL) {
            std::cout << "Error ! Open [ " << pg_path << " ] failed ..." << std::endl;
            return;
        }
        int i;
        KeyFrame tmp_kf;
        pcl::PointCloud<pcl::PointXYZI> surf_map;
        double t, px, py, pz, qx, qy, qz, qw;
        while(std::fscanf(pg_ifile,"%d %lf %lf %lf %lf %lf %lf %lf %lf",&i, &t,&px,&py,&pz,&qx,&qy,&qz,&qw)!=EOF){
            std::cout << "i " << i <<std::endl;
            tmp_kf.state_.position2_.x() = px;
            tmp_kf.state_.position2_.y() = py;
            tmp_kf.state_.position2_.z() = pz;
            tmp_kf.state_.quaternion2_.x() = qx;
            tmp_kf.state_.quaternion2_.y() = qy;
            tmp_kf.state_.quaternion2_.z() = qz;
            tmp_kf.state_.quaternion2_.w() = qw;
            pcl::io::loadPCDFile(map_dir_path + "/" + std::to_string(i) + "_surf_cloud.pcd",tmp_kf.surf_cloud_ );
            pcl::io::loadPCDFile(map_dir_path + "/" + std::to_string(i) + "_corner_cloud.pcd",tmp_kf.corner_cloud_ );
            global_map_ += tmp_kf.getTransformedCloud();
            surf_map += tmp_kf.surf_cloud_;
        }
        std::fclose(pg_ifile);
        ROS_ERROR_STREAM("load map done ...  with size : " << global_map_.points.size() << " surf : " << surf_map.points.size()<<"\n");
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(global_map_,map_msg);
        map_msg.header.frame_id = "camera_init";
        map_msg.header.stamp = ros::Time::now();
        pub_global_map_.publish(map_msg);
    }

    /*按s键保存地图，Aruco位姿*/
    void GlobalOptimization::Command() {
        while(1){
            char c = std::getchar();
            if('s'==c){
                std::cout << "get command s ..."<<std::endl;
                SaveMap();
                SaveArucoData();
            }
            std::chrono::microseconds dura(5);
            std::this_thread::sleep_for(dura);
        }
    }

    bool GlobalOptimization::SaveMap() {
        std::string map_dir_path = data_dir_path_ + "/map";
        std::string pg_dir_path = data_dir_path_ + "/pose_graph";
        std::string pg_file = pg_dir_path + "/pose_graph.txt";
        std::string surf_cloud = "_surf_cloud.pcd";
        std::string corner_cloud = "_corner_cloud.pcd";
        if(!file_manager_.CreatDirectory(map_dir_path)){
            ROS_ERROR_STREAM("error ! can't create directory : " << map_dir_path << "\n");
            return false;
        }
        if(!file_manager_.CreatDirectory(pg_dir_path)){
            ROS_ERROR_STREAM("error ! can't create directory : " << pg_dir_path << "\n");
            return false;
        }
        std::FILE* ofile_pg;
        ofile_pg = std::fopen(pg_file.c_str(),"w");
        if (ofile_pg == NULL) {
            std::cout << "! Open [ " << pg_file << " ] failed ..." << std::endl;
            return false ;
        }
        ROS_ERROR("ready to save pose graph and map data, please waite ... \n");
        int j = 0;
        for(auto kf : global_kf_v_){
            // pose graph
            std::fprintf(ofile_pg,"%d %lf %lf %lf %lf %lf %lf %lf %lf\n",j, kf.state_.timestamp_,
                         kf.state_.position2_.x(), kf.state_.position2_.y(), kf.state_.position2_.z(),
                         kf.state_.quaternion2_.x(), kf.state_.quaternion2_.y(), kf.state_.quaternion2_.z(), kf.state_.quaternion2_.w());
            // map data
            pcl::io::savePCDFileBinary(map_dir_path + "/" + std::to_string(j) + surf_cloud, kf.surf_cloud_);
            pcl::io::savePCDFileBinary(map_dir_path + "/" + std::to_string(j) + corner_cloud, kf.corner_cloud_);
            j++;
        }
        std::fclose(ofile_pg);
        ROS_ERROR("save pose graph and map data done ... \n");
        return true;

    }

    void GlobalOptimization::SaveUwbData() {
        std::cout << "SaveUwbData\n" ;
        if(uwb_anchor_map_.empty())
            return;
        std::FILE * uwb_data_file;
        std::string uwb_data_path = data_dir_path_ + "/uwb_data.txt";
        uwb_data_file = std::fopen(uwb_data_path.c_str(),"w");
        if (uwb_data_file == NULL) {
            std::cout << "! Open [ " << uwb_data_path << " ] failed ..." << std::endl;
            return  ;
        }
        for(auto anchor : uwb_anchor_map_){
            for(auto it : anchor.second){
                Eigen::Vector3d kf_p = global_kf_v_.at(it.first).state_.position2_;
                std::fprintf(uwb_data_file,"%d %d %lf %lf %lf %lf\n",
                        anchor.first,it.first,kf_p.x(),kf_p.y(),kf_p.z(),it.second);
            }
        }
        std::fclose(uwb_data_file);
        std_msgs::Bool flag_msgs;
        flag_msgs.data = 1;
        pub_uwb_data_flag_.publish(flag_msgs);
    }

    void GlobalOptimization::SaveArucoData() {
        std::cout << "SaveArucoData\n" ;
        if(aruco_initial_.empty())
            return;
        std::FILE* aruco_file;
        std::string aruco_data_path = data_dir_path_ + "/aruco_data.txt";
        aruco_file = std::fopen(aruco_data_path.c_str(),"w");
        if (aruco_file == NULL) {
            std::cout << "! Open [ " << aruco_data_path << " ] failed ..." << std::endl;
            return  ;
        }
        for(auto pose : aruco_initial_){
            std::fprintf(aruco_file,"%d %lf %lf %lf %lf %lf %lf %lf\n",pose.first,pose.second.at(0),pose.second.at(1),
                         pose.second.at(2),pose.second.at(3),pose.second.at(4),pose.second.at(5),pose.second.at(6));
        }
        std::fclose(aruco_file);

        std::FILE* aruco_optfile;
        std::string aruco_opt_data_path = data_dir_path_ + "/aruco_optimized.txt";
        aruco_optfile = std::fopen(aruco_opt_data_path.c_str(),"w");
        if (aruco_optfile == NULL) {
            std::cout << "! Open [ " << aruco_opt_data_path << " ] failed ..." << std::endl;
            return  ;
        }
        for(auto pose : aruco_optimized_){
            std::fprintf(aruco_optfile,"%d %lf %lf %lf %lf %lf %lf %lf\n",pose.first,pose.second.at(0),pose.second.at(1),
                         pose.second.at(2),pose.second.at(3),pose.second.at(4),pose.second.at(5),pose.second.at(6));
        }
        std::fclose(aruco_optfile);
    }


    void GlobalOptimization::ArucoCallback(const zed_interfaces::PoseStampedArrayConstPtr &aruco_msg) {
        mutx_.lock();
        aruco_queue_.push(aruco_msg);
        mutx_.unlock();
    }


    void GlobalOptimization::UwbCallback(const nlink_parser::LinktrackNodeframe2ConstPtr& uwb) {
        mutx_.lock();
        uwb_queue_.push(uwb);
        mutx_.unlock();
    }

    void GlobalOptimization::LidarOdomCallback(const nav_msgs::OdometryConstPtr &odom) {
        mutx_.lock();
        lidar_odom_queue_.push(odom);
        mutx_.unlock();
    }

    void GlobalOptimization::ImuCallback(const sensor_msgs::ImuConstPtr &imu) {
        mutx_.lock();
        imu_queue_.push(imu);
        mutx_.unlock();
    }

    void GlobalOptimization::SurfPointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &surf) {
        mutx_.lock();
        surf_cloud_queue_.push(surf);
        mutx_.unlock();
    }

    void GlobalOptimization::CornerPointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &corner) {
        mutx_.lock();
        corner_cloud_queue_.push(corner);
        mutx_.unlock();
    }


    void GlobalOptimization::UwbDataPrepare() {
//        ROS_ERROR("UwbDataPrepare() \n");
        mutx_.lock();
        if(uwb_queue_.empty()){
            mutx_.unlock();
            return;
        }
        while (!uwb_queue_.empty() && uwb_queue_.front()->header.stamp.toSec() < global_kf_v_.back().state_.timestamp_){
            uwb_queue_.pop();
        }
        if(uwb_queue_.empty()){
            mutx_.unlock();
            return;
        }
        int kf_idx = global_kf_v_.size()-1;
        // std::map<int,std::map<int,double>
        //           |           |      |
        //     anchor idx     kf_idx distance
        for(auto data : uwb_queue_.front()->nodes){
            if(data.rx_rssi-data.fp_rssi > 6.0) //rx_rssi-fp_rss <6 : bad data (NLOP)
                continue;
            int anchor_id = data.id;
            if(!uwb_anchor_map_.count(anchor_id)){ // first insert
                std::map<int,double > kf_measurements;
                kf_measurements.insert(std::pair<int,double>(kf_idx,data.dis));
                uwb_anchor_map_[anchor_id] = kf_measurements;
            }
            else
                uwb_anchor_map_[anchor_id].insert(std::pair<int,double>(kf_idx,data.dis));
        }
        mutx_.unlock();
        return;
    }

    void GlobalOptimization::ArucoDataPrepare() {
        mutx_.lock();
        ROS_ERROR_STREAM("aruco queue size : " <<aruco_queue_.size() <<"\n");
        if(aruco_queue_.empty()){
            mutx_.unlock();
            return;
        }
        while (!aruco_queue_.empty() && ((global_kf_v_.back().state_.timestamp_ - aruco_queue_.front()->header.stamp.toSec()) > 0.1)  ){
            aruco_queue_.pop();
        }
        if(aruco_queue_.empty()){
            mutx_.unlock();
            return;
        }
        if(aruco_queue_.front()->poses.empty()){
            mutx_.unlock();
            return;
        }
        ROS_ERROR_STREAM("timestamp differ: " << global_kf_v_.back().state_.timestamp_ - aruco_queue_.front()->header.stamp.toSec() <<"\n");
        for(auto aruco : aruco_queue_.front()->poses){
            auto kf_p = global_kf_v_.back().state_.position_;
            auto kf_q = global_kf_v_.back().state_.quaternion_;
            auto aruco_p = aruco.pose.position;
            auto aruco_q = aruco.pose.orientation;
            std::array<double,14> pose_pair{kf_p.x(), kf_p.y(), kf_p.z(), kf_q.x(), kf_q.y(), kf_q.z(), kf_q.w(),
                                            aruco_p.x, aruco_p.y, aruco_p.z, aruco_q.x, aruco_q.y, aruco_q.z, aruco_q.w};
            int aruco_id = aruco.header.seq;
            aruco_id_vec.push_back(aruco_id);
            ROS_ERROR_STREAM("aruco id : kf_id " << aruco_id <<"  " <<global_kf_v_.size()-1 <<"\n");
            if(!aruco_map_.count(aruco_id)){
                std::vector<std::array<double,14>> aruco_vec;
                aruco_vec.push_back(pose_pair);
                aruco_map_[aruco_id] = aruco_vec;
            }
            else{
                aruco_map_[aruco_id].push_back(pose_pair);
            }

        }
        mutx_.unlock();
        ROS_ERROR_STREAM("aruco map size : " << aruco_map_.size() << "\n");
        return;

    }

    void GlobalOptimization::GauessArucoInitial() {
        std::cout << "GauessArucoInitial\n" ;
        if(aruco_map_.empty())
            return;
        for(auto aruco : aruco_map_){
            ROS_ERROR_STREAM( "i "<< aruco.first <<"  size:" <<aruco.second.size() <<"\n");
            double x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=0;
            int count = 0;
            for(auto pose : aruco.second){
                // if(pose.at(10) < 0.99){//0.99 (接近1.0表示与Aruco平行)结果较可靠．数据较少时可适当减小该阈值，放宽条件
                    // continue;
                // }
                gtsam::Rot3 lidar_r3 = gtsam::Rot3::Quaternion(pose.at(6),pose.at(3),pose.at(4),pose.at(5));
                gtsam::Point3 lidar_t{pose.at(0),pose.at(1),pose.at(2)};
                gtsam::Pose3 lidar_pose(lidar_r3,lidar_t);
                gtsam::Rot3 aruco_local_r3 = gtsam::Rot3::Quaternion(pose.at(13),pose.at(10),pose.at(11),pose.at(12));
                gtsam::Point3 aruco_local_t{pose.at(7),pose.at(8),pose.at(9)};
                gtsam::Pose3 aruco_local_pose(aruco_local_r3,aruco_local_t);
                gtsam::Pose3 aruco_pose = lidar_pose * cam2lidar_ * aruco_local_pose;
                gtsam::Quaternion aruco_q = aruco_pose.rotation().toQuaternion();
                x += aruco_pose.x();
                y += aruco_pose.y();
                z += aruco_pose.z();
                qx += aruco_q.x();
                qy += aruco_q.y();
                qz += aruco_q.z();
                qw += aruco_q.w();
                count++;
            }
            if(count){
                double size = double(count);
                aruco_initial_[aruco.first] = {x/size,y/size,z/size,qx/size,qy/size,qz/size,qw/size};
            }
        }
    }

    void GlobalOptimization::ArucoVisualization() {
        visualization_msgs::MarkerArray aruco_array;
        for(auto pose : aruco_initial_){
            visualization_msgs::Marker aruco;
            aruco.header.stamp = ros::Time::now();
            aruco.header.frame_id = "camera_init";
            aruco.id = pose.first;
            aruco.type = visualization_msgs::Marker::CUBE;
            aruco.action = visualization_msgs::Marker::ADD;
            aruco.scale.x = 0.18 * 3;
            aruco.scale.y = 0.18 * 3;
            aruco.scale.z = 0.01;
            geometry_msgs::Pose aruco_pose;
            aruco_pose.position.x = pose.second.at(0);
            aruco_pose.position.y = pose.second.at(1);
            aruco_pose.position.z = pose.second.at(2);
            aruco_pose.orientation.x = pose.second.at(3);
            aruco_pose.orientation.y = pose.second.at(4);
            aruco_pose.orientation.z = pose.second.at(5);
            aruco_pose.orientation.w = pose.second.at(6);
            aruco.pose = aruco_pose;
            aruco.color.a = 1.0;
            aruco.color.r = 1.0;
            aruco.color.g = 1.0;
            aruco.color.b = 0;
            aruco_array.markers.push_back(aruco);
        }
        pub_aruco_.publish(aruco_array);

        visualization_msgs::MarkerArray aruco_array_opt;
        for(auto pose : aruco_optimized_){
            visualization_msgs::Marker aruco_opt;
            aruco_opt.header.stamp = ros::Time::now();
            aruco_opt.header.frame_id = "camera_init";
            aruco_opt.id = pose.first;
            aruco_opt.type = visualization_msgs::Marker::CUBE;
            aruco_opt.action = visualization_msgs::Marker::ADD;
            aruco_opt.scale.x = 0.18 * 3;
            aruco_opt.scale.y = 0.18 * 3;
            aruco_opt.scale.z = 0.01;
            geometry_msgs::Pose aruco_pose_pot;
            aruco_pose_pot.position.x = pose.second.at(0);
            aruco_pose_pot.position.y = pose.second.at(1);
            aruco_pose_pot.position.z = pose.second.at(2);
            aruco_pose_pot.orientation.x = pose.second.at(3);
            aruco_pose_pot.orientation.y = pose.second.at(4);
            aruco_pose_pot.orientation.z = pose.second.at(5);
            aruco_pose_pot.orientation.w = pose.second.at(6);
            aruco_opt.pose = aruco_pose_pot;
            aruco_opt.color.a = 1.0;
            aruco_opt.color.r = 1.0;
            aruco_opt.color.g = 1.0;
            aruco_opt.color.b = 1.0;
            aruco_array_opt.markers.push_back(aruco_opt);
        }
        pub_aruco_opt_.publish(aruco_array_opt);

    }


    bool GlobalOptimization::AddKeyFrame(LIRO::KeyFrame &keyframe) {
        if(loop_is_closed_)
            return true;
        int kf_size = global_kf_v_.size();
        if(kf_size){
            keyframe.state_.absolut_pose_ = keyframe.state_.getTransformation();
            keyframe.state_.toXYZRPY();
            global_kf_v_.back().state_.toXYZRPY();
            std::array<double,6> pose1 = keyframe.state_.xyzrpy_;
            std::array<double,6> pose0 = global_kf_v_.back().state_.xyzrpy_;
            double t1 = keyframe.state_.timestamp_;
            double t0 = global_kf_v_.back().state_.timestamp_;
            bool flag1 = (std::sqrt(std::pow(pose1[0]-pose0[0],2) +
                    std::pow(pose1[1]-pose0[1],2)+std::pow(pose1[2]-pose0[2],2)) >= keyframe_distance_);
            bool flag2 = std::abs(pose1[3]-pose0[3]) >= keyframe_rotation_ ||
                    std::abs(pose1[4]-pose0[4]) >= keyframe_rotation_ || std::abs(pose1[5]-pose0[5]) >= keyframe_rotation_;
            bool flag3 = t1 - t0 >= keyframe_time_diff_;
//            ROS_ERROR_STREAM("flag 1 2 3 : "<< flag1 <<" " <<flag2 <<" " <<flag3 <<"\n");
            if(!flag1 && !flag2 && !flag3)
                return false;
        }
        global_kf_v_.emplace_back(keyframe);
        ROS_ERROR_STREAM("Add keyframe ...   [" << keyframe.state_.position_.x() <<" "<< keyframe.state_.position_.y() <<" "
        << keyframe.state_.position_.z() <<"]      "<< global_kf_v_.size() <<"    loop count: " <<loop_count <<"\n");
        return true;
    }

    void GlobalOptimization::GlobalMapPublish() {
        if(!global_kf_v_.empty()){
            map_cloud_.clear();
            pcl::PointCloud<pcl::PointXYZI> surf;
            pcl::PointCloud<pcl::PointXYZI> corner;
//            pcl::PointCloud<pcl::PointXYZI> surf_dw;
//            pcl::PointCloud<pcl::PointXYZI> corner_dw;
            for(uint i=0;i<global_kf_v_.size();i++){
                surf += global_kf_v_.at(i).getTransformedSurfCloud();
                corner += global_kf_v_.at(i).getTransformedCornerCloud();
            }
            // down sample for visualization
//            surf_filter_for_visualization_.setInputCloud(surf.makeShared());
//            surf_filter_for_visualization_.filter(surf_dw);
            map_cloud_ += surf;
//            corner_filter_for_visualization_.setInputCloud(corner.makeShared());
//            corner_filter_for_visualization_.filter(corner_dw);
            map_cloud_ += corner;
            sensor_msgs::PointCloud2 map_msg,surf_msg, corner_msg;
            pcl::toROSMsg(map_cloud_,map_msg);
            map_msg.header.stamp = ros::Time().fromSec(odom_timestamp_);
            map_msg.header.frame_id = "camera_init";
            pub_global_map_.publish(map_msg);

//            pcl::toROSMsg(surf_dw,surf_msg);
//            surf_msg.header.stamp = ros::Time().fromSec(odom_timestamp_);
//            surf_msg.header.frame_id = "camera_init";
//            pub_surf_map_.publish(surf_msg);
//
//            pcl::toROSMsg(corner_dw,corner_msg);
//            corner_msg.header.stamp = ros::Time().fromSec(odom_timestamp_);
//            corner_msg.header.frame_id = "camera_init";
//            pub_corner_map_.publish(corner_msg);
            surf.clear();
            corner.clear();
//            surf_dw.clear();
//            corner_dw.clear();


        }
    }

    void GlobalOptimization::GlobalPathPublish() {
        if(!global_kf_v_.empty()){
            global_path_lo_.clear();
            global_path_lio_.clear();
            global_path_liro_.clear();
            pcl::PointXYZI kf_pose_lo,kf_pose_lio,kf_pose_liro;
            for(uint i=0;i<global_kf_v_.size();i++){
                Eigen::Vector3d tmp = global_kf_v_.at(i).state_.absolut_pose_.block<3,1>(0,3);
                kf_pose_lo.x = tmp.x();
                kf_pose_lo.y = tmp.y();
                kf_pose_lo.z = tmp.z();
                global_path_lo_.points.push_back(kf_pose_lo);
                kf_pose_lio.x = global_kf_v_.at(i).state_.position_.x();
                kf_pose_lio.y = global_kf_v_.at(i).state_.position_.y();
                kf_pose_lio.z = global_kf_v_.at(i).state_.position_.z();
                global_path_lio_.points.push_back(kf_pose_lio);
                kf_pose_liro.x = global_kf_v_.at(i).state_.position2_.x();
                kf_pose_liro.y = global_kf_v_.at(i).state_.position2_.y();
                kf_pose_liro.z = global_kf_v_.at(i).state_.position2_.z();
                global_path_liro_.points.push_back(kf_pose_liro);
            }
// lo
            sensor_msgs::PointCloud2 global_path_msg_lo;
            pcl::toROSMsg(global_path_lo_,global_path_msg_lo);
            global_path_msg_lo.header.stamp = ros::Time().fromSec(odom_timestamp_);
            global_path_msg_lo.header.frame_id = "camera_init";
            pub_global_path_lo_.publish(global_path_msg_lo);
// lio
            sensor_msgs::PointCloud2 global_path_lio_msg;
            pcl::toROSMsg(global_path_lio_,global_path_lio_msg);
            global_path_lio_msg.header.stamp = ros::Time().fromSec(odom_timestamp_);
            global_path_lio_msg.header.frame_id = "camera_init";
            pub_global_path_lio_.publish(global_path_lio_msg);

// liro with loop closure
            sensor_msgs::PointCloud2 global_path_liro_msg;
            pcl::toROSMsg(global_path_liro_,global_path_liro_msg);
            global_path_liro_msg.header.stamp = ros::Time().fromSec(odom_timestamp_);
            global_path_liro_msg.header.frame_id = "camera_init";
            pub_global_path_liro_.publish(global_path_liro_msg);
        }
    }


    void GlobalOptimization::Run() {
        while(!lidar_odom_queue_.empty() && !surf_cloud_queue_.empty()
        && !corner_cloud_queue_.empty() &&!imu_queue_.empty() &&!uwb_queue_.empty() && !aruco_queue_.empty()){
            odom_timestamp_ = lidar_odom_queue_.front()->header.stamp.toSec();
            mutx_.lock();
            while(!surf_cloud_queue_.empty() && surf_cloud_queue_.front()->header.stamp.toSec() < odom_timestamp_){
                surf_cloud_queue_.pop();
            }
            if(surf_cloud_queue_.empty()){
                mutx_.unlock();
                break;
            }
            while(!corner_cloud_queue_.empty() && corner_cloud_queue_.front()->header.stamp.toSec() < odom_timestamp_){
                corner_cloud_queue_.pop();
            }
            if(corner_cloud_queue_.empty()){
                mutx_.unlock();
                break;
            }
            KeyFrame keyframe;
            pcl::fromROSMsg(*surf_cloud_queue_.front(),keyframe.surf_cloud_);
            pcl::fromROSMsg(*corner_cloud_queue_.front(),keyframe.corner_cloud_);
            surf_cloud_queue_.pop();
            corner_cloud_queue_.pop();
            keyframe.state_.timestamp_ = odom_timestamp_;
            keyframe.state_.position_.x() = lidar_odom_queue_.front()->pose.pose.position.x;
            keyframe.state_.position_.y() = lidar_odom_queue_.front()->pose.pose.position.y;
            keyframe.state_.position_.z() = lidar_odom_queue_.front()->pose.pose.position.z;
            keyframe.state_.quaternion_.x() = lidar_odom_queue_.front()->pose.pose.orientation.x;
            keyframe.state_.quaternion_.y() = lidar_odom_queue_.front()->pose.pose.orientation.y;
            keyframe.state_.quaternion_.z() = lidar_odom_queue_.front()->pose.pose.orientation.z;
            keyframe.state_.quaternion_.w() = lidar_odom_queue_.front()->pose.pose.orientation.w;
            lidar_odom_queue_.pop();
            mutx_.unlock();
            std::lock_guard<std::mutex> lock(kf_mutx_);
            if(!AddKeyFrame(keyframe))
                continue;
            UwbDataPrepare();
            ArucoDataPrepare();
            AddLIFactor();
            CorrectLIOPath();
            AddLoopAndRangFactor();
            CorrectGlobalKeyFramePose();
            GlobalPathPublish();
            size_t kf_size = global_kf_v_.size();
            if(loop_is_closed_ || (kf_size % 10 ==0)){
                SaveUwbData();
                GauessArucoInitial();
                ArucoOptimization();
                SaveArucoData();
                loop_is_closed_ = false;
            }
            ArucoVisualization();
            if(kf_size % 10 == 0){
                GlobalMapPublish();
            }
        }

    }

    bool GlobalOptimization::DetectLoopClosure(int& latest_id,int& closed_id) {
        std::lock_guard<std::mutex> lock(kf_mutx_);
        int lastest_closure_id,closest_history_id;
        latest_keyframe_cloud_->clear();
        near_history_keyframe_cloud_->clear();
        lastest_closure_id = global_kf_v_.size()-1;
        // to build kdtree for loop closure
        for(size_t i=0;i<global_kf_v_.size();++i){
            pcl::PointXYZI kfpose;
            kfpose.x = global_kf_v_.at(i).state_.position2_.x();
            kfpose.y = global_kf_v_.at(i).state_.position2_.y();
            kfpose.z = global_kf_v_.at(i).state_.position2_.z();
            cloud_key_pose3d_->points.push_back(kfpose);
        }
        std::vector<int> loop_search_ids;
        std::vector<float > loop_search_distances;
        kdtree_history_key_poses_->setInputCloud(cloud_key_pose3d_);
        kdtree_history_key_poses_->radiusSearch(cloud_key_pose3d_->back(),loop_closure_search_radius_,loop_search_ids,loop_search_distances,0);
        closest_history_id = -1;
        // TODO::该参数需要调试
        for(unsigned int i=0;i< loop_search_ids.size();++i){
            int id = loop_search_ids[i];
            if( 1.2*matching_min_id_distance <(lastest_closure_id - id )){ //&& (id - latest_closed_kf_index_) > 0.2*matching_min_id_distance
                closest_history_id = id;
                break;
            }
        }
        if(closest_history_id == -1){
            return false;
        }
        std::cout << "closed idx : " << closest_history_id << std::endl;
        // save keyframe pose of the lateset and closest
        latest_keyframe_pose_ = global_kf_v_.at(lastest_closure_id).state_.getTransformation();
        closest_keyframe_pose_ = global_kf_v_.back().state_.getTransformation();
        // save cloud near closed frame
        pcl::PointCloud<pcl::PointXYZI> surf;
        pcl::PointCloud<pcl::PointXYZI> corner;
        for(int j= -history_keyframe_num_;j<=history_keyframe_num_;++j){
            if(closest_history_id + j < 0 || closest_history_id + j > lastest_closure_id)
                continue;
            surf += global_kf_v_.at(closest_history_id + j).getTransformedSurfCloud();
            corner += global_kf_v_.at(closest_history_id + j).getTransformedCornerCloud();

        }
        *latest_keyframe_cloud_ = global_kf_v_.back().getTransformedCloud();
//         down sample for icp
//        pcl::PointCloud<pcl::PointXYZI> surf_dw;
        surf_filter_for_icp_.setInputCloud(surf.makeShared());
        surf_filter_for_icp_.filter(surf);
        *near_history_keyframe_cloud_ += surf;
//        pcl::PointCloud<pcl::PointXYZI> corner_dw;
        corner_filter_for_icp_.setInputCloud(corner.makeShared());
        corner_filter_for_icp_.filter(corner);
        *near_history_keyframe_cloud_ += corner;
//         clouds near closed history key farme are unavaialable;
        if(latest_keyframe_cloud_->points.size() < 300 || near_history_keyframe_cloud_->points.size()<1000){
            return false;
        }
        latest_id = lastest_closure_id;
        closed_id = closest_history_id;
        return true;
    }

    void GlobalOptimization::PerformLoopClosure() {
        if(global_kf_v_.size() < matching_min_id_distance)
            return;
        int latest_frame_id_closure,closest_history_frame_id;
        if(!DetectLoopClosure(latest_frame_id_closure,closest_history_frame_id)){
            return;
        }
        ROS_ERROR("  detect loop *** ");
        icp_.setInputSource(latest_keyframe_cloud_);
        icp_.setInputTarget(near_history_keyframe_cloud_);
        pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>);
        icp_.align(*unused_result);
        if(!icp_.hasConverged() || icp_.getFitnessScore() > icp_fitness_score_){
            ROS_ERROR(" loop closure failed ********");
            return;
        }
        loop_count++;
        gtsam::Pose3 relative_t = MaritxToGtsamPose(icp_.getFinalTransformation().cast<double>());
        ROS_ERROR_STREAM("loop count :  " << loop_count <<" realtive trans : " <<
        "\nt  :" << relative_t.translation().x() <<" " << relative_t.translation().y() <<" " << relative_t.translation().z()
        <<"\nr  : " << relative_t.rotation().roll() <<" " << relative_t.rotation().pitch()<<" " << relative_t.rotation().yaw() <<" \n");
        ROS_ERROR_STREAM(" **************** loop closure succesed  ********  matched pair : " << latest_frame_id_closure<<" - " <<closest_history_frame_id <<"\n");

        double x, y, z, roll, pitch, yaw;
        Eigen::Affine3d relative_trans;
        relative_trans = icp_.getFinalTransformation().cast<double>();
        Eigen::Affine3d lastest_kf_pose_old(latest_keyframe_pose_);
        Eigen::Affine3d lastest_kf_pose_new = relative_trans * lastest_kf_pose_old;
        pcl::getTranslationAndEulerAngles(lastest_kf_pose_new,x,y,z,roll,pitch,yaw);
        gtsam::Pose3 pose_from = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll,pitch,yaw),gtsam::Point3(x,y,z));
        gtsam::Pose3 pose_to = MaritxToGtsamPose(closest_keyframe_pose_);
        gtsam::Vector vector_6(6);
        float noise_score = coef_loop_closure_*icp_.getFitnessScore();
        vector_6 << noise_score, noise_score,  noise_score, noise_score, noise_score, noise_score;
        gtsam::noiseModel::Diagonal::shared_ptr constrain_noise = gtsam::noiseModel::Diagonal::Variances(vector_6);
        // add pose constrain
        std::lock_guard<std::mutex> lock(kf_mutx_);
        gtSAMgraph_odom_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(latest_frame_id_closure),X(closest_history_frame_id),pose_from.between(pose_to),constrain_noise));
        isam_odom_->update(gtSAMgraph_odom_);
        isam_odom_->update();
        gtSAMgraph_odom_.resize(0);
        loop_is_closed_ = true;
        ROS_ERROR_STREAM(" **************** loop closure succesed  ******** ");
    }

    void GlobalOptimization::CorrectGlobalKeyFramePose() {
//        ROS_ERROR("CorrectGlobalKeyFramePose\n");
        if(loop_is_closed_){
            unsigned int tmp_size = isamCurrentEstimate_odom_.size();
            for(unsigned i=0;i<tmp_size;++i){
                global_kf_v_.at(i).state_.position2_.x() = isamCurrentEstimate_odom_.at<gtsam::Pose3>(X(i)).translation().x();
                global_kf_v_.at(i).state_.position2_.y() = isamCurrentEstimate_odom_.at<gtsam::Pose3>(X(i)).translation().y();
                global_kf_v_.at(i).state_.position2_.z() = isamCurrentEstimate_odom_.at<gtsam::Pose3>(X(i)).translation().z();
                global_kf_v_.at(i).state_.quaternion2_.x() = isamCurrentEstimate_odom_.at<gtsam::Pose3>(X(i)).rotation().toQuaternion().x();
                global_kf_v_.at(i).state_.quaternion2_.y() = isamCurrentEstimate_odom_.at<gtsam::Pose3>(X(i)).rotation().toQuaternion().y();
                global_kf_v_.at(i).state_.quaternion2_.z() = isamCurrentEstimate_odom_.at<gtsam::Pose3>(X(i)).rotation().toQuaternion().z();
                global_kf_v_.at(i).state_.quaternion2_.w() = isamCurrentEstimate_odom_.at<gtsam::Pose3>(X(i)).rotation().toQuaternion().w();
            }
        }
    }

    void GlobalOptimization::CorrectLIOPath() {
        if(loop_is_closed_)
            return ;
        unsigned int tmp_size = isamCurrentEstimate_li_.size()/3;
      //  ROS_ERROR_STREAM("isam_lio size :" << tmp_size);
        if(!tmp_size)
            return;
        // transfer to lidar coordinate
        gtsam::Pose3 lidar_pose = isamCurrentEstimate_li_.at<gtsam::Pose3>(X(tmp_size-1)).compose(imu2lidar_);
        global_kf_v_.back().state_.position_.x() = lidar_pose.translation().x();
        global_kf_v_.back().state_.position_.y() = lidar_pose.translation().y();
        global_kf_v_.back().state_.position_.z() = lidar_pose.translation().z();
        gtsam::Quaternion tmp_q = lidar_pose.rotation().toQuaternion();
        global_kf_v_.back().state_.quaternion_.w() = tmp_q.w();
        global_kf_v_.back().state_.quaternion_.x() = tmp_q.x();
        global_kf_v_.back().state_.quaternion_.y() = tmp_q.y();
        global_kf_v_.back().state_.quaternion_.z() = tmp_q.z() ;
        global_kf_v_.back().state_.position2_ = global_kf_v_.back().state_.position_;
        global_kf_v_.back().state_.quaternion2_.w() = global_kf_v_.back().state_.quaternion_.w();
        global_kf_v_.back().state_.quaternion2_.x() = global_kf_v_.back().state_.quaternion_.x();
        global_kf_v_.back().state_.quaternion2_.y() = global_kf_v_.back().state_.quaternion_.y();
        global_kf_v_.back().state_.quaternion2_.z() = global_kf_v_.back().state_.quaternion_.z() ;
        // update lio pose for loop closure
        if(tmp_size>1){
            Eigen::Matrix4d relative_trans = global_kf_v_.at(tmp_size-2).state_.getTransformation2().inverse()*
                    global_kf_v_.back().state_.getTransformation2();
            Eigen::Matrix4d absolute_pose = global_kf_v_.at(tmp_size-2).state_.getTransformation2() * relative_trans;
            global_kf_v_.back().state_.position2_ = absolute_pose.block<3,1>(0,3);
            global_kf_v_.back().state_.quaternion2_ = absolute_pose.block<3,3>(0,0);
        }
    }

    void GlobalOptimization::AddLoopAndRangFactor() {
        if(loop_is_closed_)
            return ;
        int kf_size = global_kf_v_.size();
        if(!kf_size){
            return;
        }
        if(kf_size == 1){
            gtsam::Vector vector_6(6);
            vector_6 << odom_r_noise_, odom_r_noise_, odom_r_noise_, odom_t_noise_, odom_t_noise_, odom_t_noise_;
            gtsam::noiseModel::Diagonal::shared_ptr prior_noise  = gtsam::noiseModel::Diagonal::Variances(vector_6);
            gtSAMgraph_odom_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0),MaritxToGtsamPose(global_kf_v_.back().state_.getTransformation()),prior_noise));
            initialEstimate_odom_.insert(X(0),MaritxToGtsamPose(global_kf_v_.back().state_.getTransformation()));
        }
        else{
            gtsam::Vector vector_6(6);
            vector_6 << odom_r_noise_, odom_r_noise_, odom_r_noise_, odom_t_noise_, odom_t_noise_, odom_t_noise_;
            gtsam::noiseModel::Diagonal::shared_ptr odom_noise  = gtsam::noiseModel::Diagonal::Variances(vector_6);
            gtsam::Pose3 pose_from = MaritxToGtsamPose(global_kf_v_.at(kf_size-2).state_.getTransformation());
            gtsam::Pose3 pose_to = MaritxToGtsamPose(global_kf_v_.back().state_.getTransformation());
            gtSAMgraph_odom_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(kf_size-2),X(kf_size-1),pose_from.between(pose_to),odom_noise));
            initialEstimate_odom_.insert(X(kf_size-1),pose_to);
        }
        isam_odom_->update(gtSAMgraph_odom_,initialEstimate_odom_);
        isam_odom_->update();
        gtSAMgraph_odom_.resize(0);
        initialEstimate_odom_.clear();
        isamCurrentEstimate_odom_ = isam_odom_->calculateEstimate();
    }

    void GlobalOptimization::AddLIFactor() {
        if(loop_is_closed_)
            return ;
        gtsam::Pose3 lidar_pose = MaritxToGtsamPose(global_kf_v_.back().state_.getTransformation());
        mutx_.lock();
        if(imu_queue_.empty()){
            mutx_.unlock();
            return;
        }
        int kf_size = global_kf_v_.size();
        if(!kf_size){
            mutx_.unlock();
            return;
        }
        if(!lio_opt_initializd_){
            while (!imu_queue_.empty()){
                if (imu_queue_.front()->header.stamp.toSec() < odom_timestamp_){
                    last_imu_opt_t_ = imu_queue_.front()->header.stamp.toSec();
                    imu_queue_.pop();
                }
                else
                    break;
            }
            prev_pose_ = lidar_pose.compose(lidar2imu_);
            // prior p
            gtsam::PriorFactor<gtsam::Pose3> prior_pose(X(0),prev_pose_,prior_pose_noise_);
            gtSAMgraph_li_.add(prior_pose);
            // prior v
            prev_vel_ = gtsam::Vector3(0,0,0);
            gtsam::PriorFactor<gtsam::Vector3> prior_vel(V(0),prev_vel_,prior_vel_noise_);
            gtSAMgraph_li_.add(prior_vel);
            // prior b
            prev_bias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> prior_bais(B(0),prev_bias_,prior_bias_noise_);
            gtSAMgraph_li_.add(prior_bais);
            // add values
            initialEstimate_li_.insert(X(0),prev_pose_);
            initialEstimate_li_.insert(V(0),prev_vel_);
            initialEstimate_li_.insert(B(0),prev_bias_);
            isam_li_->update(gtSAMgraph_li_,initialEstimate_li_);
            gtSAMgraph_li_.resize(0);
            initialEstimate_li_.clear();
            isamCurrentEstimate_li_ = isam_li_->calculateEstimate();
            imu_integrator_->resetIntegrationAndSetBias(prev_bias_);
            mutx_.unlock();
            lio_opt_initializd_ = true;
            return;
        }
        while(!imu_queue_.empty()){
            sensor_msgs::ImuConstPtr this_imu = imu_queue_.front();
            double imu_timestamp = this_imu->header.stamp.toSec();
            if(imu_timestamp < odom_timestamp_){
                double dt = (last_imu_opt_t_ < 0) ? (1.0/200.0) : (imu_timestamp - last_imu_opt_t_);
                imu_integrator_->integrateMeasurement(gtsam::Vector3(this_imu->linear_acceleration.x,this_imu->linear_acceleration.y,this_imu->linear_acceleration.z),
                        gtsam::Vector3(this_imu->angular_velocity.x,this_imu->angular_velocity.y,this_imu->angular_velocity.z),dt);
                last_imu_opt_t_ = imu_timestamp;
                imu_queue_.pop();
            }
            else{
                break;
            }
        }
        mutx_.unlock();
//        if(kf_size % 99 == 0){
//            // get updated noise before reset
//            gtsam::noiseModel::Gaussian::shared_ptr updated_p_noise = gtsam::noiseModel::Gaussian::Covariance(isam_li_->marginalCovariance(X(kf_size-2)));
//            gtsam::noiseModel::Gaussian::shared_ptr updated_v_noise  = gtsam::noiseModel::Gaussian::Covariance(isam_li_->marginalCovariance(V(kf_size-2)));
//            gtsam::noiseModel::Gaussian::shared_ptr updated_b_noise = gtsam::noiseModel::Gaussian::Covariance(isam_li_->marginalCovariance(B(kf_size-2)));
//            RestLIOOptimization();
//            ROS_ERROR("reset optimization done !");
//            gtsam::PriorFactor<gtsam::Pose3> prior_pose(X(kf_size-1),prev_pose_,updated_p_noise);
//            gtSAMgraph_li_.add(prior_pose);
//            ROS_ERROR(" p !");
//            // prior v
//            prev_vel_ = gtsam::Vector3(0,0,0);
//            gtsam::PriorFactor<gtsam::Vector3> prior_vel(V(kf_size-1),prev_vel_,updated_v_noise);
//            gtSAMgraph_li_.add(prior_vel);
//            ROS_ERROR(" v !");
//            // prior b
//            prev_bias_ = gtsam::imuBias::ConstantBias();
//            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> prior_bais(B(kf_size-1),prev_bias_,updated_b_noise);
//            gtSAMgraph_li_.add(prior_bais);
//            ROS_ERROR(" b !");
//            // add values
//            initialEstimate_li_.insert(X(kf_size-1),prev_pose_);
//            initialEstimate_li_.insert(V(kf_size-1),prev_vel_);
//            initialEstimate_li_.insert(B(kf_size-1),prev_bias_);
//            ROS_ERROR(" insert value done !");
//            isam_li_->update(gtSAMgraph_li_,initialEstimate_li_);
//            gtSAMgraph_li_.resize(0);
//            initialEstimate_li_.clear();
//            return;
//        }

        // add factors
        const gtsam::PreintegratedImuMeasurements &preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imu_integrator_);
        gtsam::ImuFactor imu_factor(X(kf_size-2),V(kf_size-2),X(kf_size-1),V(kf_size-1),B(kf_size-2),preint_imu);
        gtSAMgraph_li_.add(imu_factor);
        gtSAMgraph_li_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(kf_size-2),B(kf_size-1),gtsam::imuBias::ConstantBias(),
                gtsam::noiseModel::Diagonal::Sigmas(std::sqrt(imu_integrator_->deltaTij()) * noise_model_between_bias_)));
        gtsam::Pose3 cur_pose = lidar_pose.compose(lidar2imu_);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(kf_size-1),cur_pose,correction_noise1_);
        gtSAMgraph_li_.add(pose_factor);
        // add values
        gtsam::NavState prop_state = imu_integrator_->predict(prev_state_,prev_bias_);
        initialEstimate_li_.insert(X(kf_size-1),prop_state.pose());
        initialEstimate_li_.insert(V(kf_size-1),prop_state.v());
        initialEstimate_li_.insert(B(kf_size-1),prev_bias_);
       // ROS_ERROR_STREAM("imu integrator predict :  " << prop_state.pose().x() <<" " << prop_state.pose().y() <<" " << prop_state.pose().z() <<" ;"
      //  << prop_state.pose().rotation().roll() <<" " << prop_state.pose().rotation().pitch() <<" " << prop_state.pose().rotation().yaw() <<" \n" );

        isam_li_->update(gtSAMgraph_li_,initialEstimate_li_);
        isam_li_->update();
        gtSAMgraph_li_.resize(0);
        initialEstimate_li_.clear();
        gtsam::Values result = isam_li_->calculateEstimate();
        isamCurrentEstimate_li_ = isam_li_->calculateEstimate();
        prev_pose_ = result.at<gtsam::Pose3>(X(kf_size-1));
        prev_vel_ = result.at<gtsam::Vector3>(V(kf_size-1));
        prev_state_ = gtsam::NavState(prev_pose_,prev_vel_);
        prev_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(kf_size-1));
        imu_integrator_->resetIntegrationAndSetBias(prev_bias_);

    }

    void GlobalOptimization::ArucoOptimization() {
        std::cout << "ArucoOptimization\n" ;
        if(aruco_initial_.empty())
            return;
        gtsam::Vector vectorp_6(6);
        vectorp_6 <<  odom_t_noise_, odom_t_noise_, odom_t_noise_,odom_r_noise_,odom_r_noise_,odom_r_noise_;
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise  = gtsam::noiseModel::Diagonal::Variances(vectorp_6);
        gtsam::Vector vector_6(6);
        vector_6 << 0.1,0.1,0.1,0.1,0.1,0.1;
        gtsam::noiseModel::Diagonal::shared_ptr br_noise = gtsam::noiseModel::Diagonal::Sigmas(vector_6);
        for(auto aruco : aruco_map_){
            if(aruco_initial_.count(aruco.first)==0)
                continue;
            gtsam::NonlinearFactorGraph graph;
            gtsam::Values initial_estimate;
            int j =0;
            for(auto pose_pair : aruco.second){
                // if(pose_pair.at(10) < 0.92)
                    // continue;
                gtsam::Rot3 bearing_rot3 = gtsam::Rot3::Quaternion(pose_pair.at(13), pose_pair.at(10),pose_pair.at(11),pose_pair.at(12));
                gtsam::Point3 range_point3 {pose_pair.at(7),pose_pair.at(8),pose_pair.at(9)};
                gtsam::Pose3 cam_to_aruco (bearing_rot3,range_point3);
                // aruco pose in w
                gtsam::Pose3 aruco_pose (gtsam::Rot3::Quaternion(aruco_initial_.at(aruco.first).at(6),aruco_initial_.at(aruco.first).at(3),
                        aruco_initial_.at(aruco.first).at(4),aruco_initial_.at(aruco.first).at(5)),
                        {aruco_initial_.at(aruco.first).at(0),aruco_initial_.at(aruco.first).at(1),aruco_initial_.at(aruco.first).at(2)});
                // camear pose in w
                gtsam::Rot3 lidar_r3 = gtsam::Rot3::Quaternion(pose_pair.at(6),pose_pair.at(3),pose_pair.at(4),pose_pair.at(5));
                gtsam::Point3 lidar_t{pose_pair.at(0),pose_pair.at(1),pose_pair.at(2)};
                gtsam::Pose3 lidar_pose(lidar_r3,lidar_t);
                gtsam::Pose3 cam_pose  = lidar_pose * cam2lidar_;
                // factors
                graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(j),cam_pose,prior_noise));
                graph.add(gtsam::BetweenFactor<gtsam::Pose3>(X(j),A(aruco.first),cam_to_aruco,br_noise));
                if(j==0) // insert once
                    initial_estimate.insert(A(aruco.first),aruco_pose);
                initial_estimate.insert(X(j),cam_pose);
                j++;
            }
            gtsam::LevenbergMarquardtOptimizer lm_optimizer(graph,initial_estimate);
            gtsam::Values result = lm_optimizer.optimize();
            if(!(result.exists(A(aruco.first))))
                continue;
            gtsam::Pose3 aruco_result = result.at<gtsam::Pose3>(A(aruco.first));
            aruco_optimized_[aruco.first] = {aruco_result.x(),aruco_result.y(),aruco_result.z(),aruco_result.rotation().toQuaternion().x(),
            aruco_result.rotation().toQuaternion().y(),aruco_result.rotation().toQuaternion().z(),aruco_result.rotation().toQuaternion().w()} ;
        }
    }

    void GlobalOptimization::RestLIOOptimization() {
        gtsam::ISAM2Params opt_parametrs;
        opt_parametrs.relinearizeThreshold = 0.1;
        opt_parametrs.relinearizeSkip = 1;
        isam_li_ = new gtsam::ISAM2(opt_parametrs);
        gtsam::NonlinearFactorGraph new_graph;
        gtSAMgraph_li_ = new_graph;
        gtsam::Values new_estimate;
        isamCurrentEstimate_li_ = new_estimate;
    }

    gtsam::Pose3 GlobalOptimization::MaritxToGtsamPose(const Eigen::Matrix4d &trans) {
        Eigen::Quaterniond q(trans.block<3,3>(0,0).matrix());
        Eigen::Vector3d t = trans.block<3,1>(0,3);
        return gtsam::Pose3(gtsam::Rot3(q),gtsam::Point3(t.x(),t.y(),t.z()));
    }

    void GlobalOptimization::LoopClosureThread() {
        ros::Rate rate(loop_closure_frequence_);
        while(ros::ok()){
            rate.sleep();
            PerformLoopClosure();
        }
    }

    void GlobalOptimization::AllocateMemory() {
        cloud_key_pose3d_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        kdtree_history_key_poses_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
        latest_keyframe_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        near_history_keyframe_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

}


int main(int argc, char** argv){
    ros::init(argc,argv,"LIRO");
    LIRO::GlobalOptimization GMO;
    std::thread loopthread(&LIRO::GlobalOptimization::LoopClosureThread,&GMO);
    ros::Rate rate(200);
    while(ros::ok()){
        ros::spinOnce();
        GMO.Run();
        rate.sleep();
    }
    loopthread.join();
    return 0;
}
