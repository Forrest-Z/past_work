//
// Created by wchen on 2019/12/3.
//

#include "gp_lio/mapper/GlobalMapper.h"


namespace gp_lio{

    GlobalMapper::GlobalMapper(): initialized_(false),loop_closed_(false),optimize_pose_graph_(false), add_new_kf_(false),to_detect_(false), loop_count_(0), need_update_(false), add_latest_map_(
            false), last_optimized_idx_(10),ndt_ptr_(new pcl::NormalDistributionsTransform<pcl::PointXYZI,pcl::PointXYZI>) {

        distance_Mat_ = cv::Mat(5000, 5000, CV_32F, cv::Scalar::all(FLT_MAX));
        smallest_ref_idx_.resize(5000, INT_MAX);

        downsize_filter_.setLeafSize(0.2, 0.2, 0.2);
        map_downsize_filter_.setLeafSize(0.8, 0.8, 0.8);

        all_feature_cloud_map_.reset(new PointCloudT());
        all_ground_cloud_map_.reset(new PointCloudT());
        all_plane_cloud_map_.reset(new PointCloudT());
        all_cloud_map_.reset(new PointCloudT());

        global_index_ = 0;
        to_merge_map = false;
        map_merged = false;
        build_tree_ = true;
        last_closed_index = -1;
        kf_timestamp = 0.0;


        ndt_ptr_->setResolution(0.1);
        ndt_ptr_->setStepSize(0.01);
        ndt_ptr_->setTransformationEpsilon(0.1);
        ndt_ptr_->setMaximumIterations(35);

    }

    GlobalMapper::~GlobalMapper(){

    }

    void GlobalMapper::LoadConfig(ros::NodeHandle &nh) {

        nh_ = nh;
        sub_kf_descriptor_ = nh_.subscribe("/lpd_net/feature_vector", 1, &GlobalMapper::DescriptorCallback, this);
        sub_kf_gnss_ = nh.subscribe("/fix",500,&GlobalMapper::GnssMsgsCallback,this);

        all_feature_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_mapper/all_feature_cloud", 1);
        all_ground_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_mapper/all_ground_cloud", 1);
        all_plane_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_mapper/all_plane_cloud", 1);
        all_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_mapper/all_cloud", 1);
        loop_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/global_mapper/loop_pair", 2);
        pub_kf_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/lpd_net/feature_pointclouds", 1);
        pub_pose_graph_old_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_mapper/pose_graph_old", 1);
        pub_pose_graph_new_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_mapper/pose_graph_new", 1);
        pub_global_path_ = nh.advertise<sensor_msgs::PointCloud2>("/global_mapper/global_path",2);
        SearchTrajecoryOffsets();
        initialized_=true;
    }

    void GlobalMapper::Run() {

        ros::Rate rate(100);

        while (ros::ok()){

            if(initialized_) {
                if(DETECTION_METHOD=="lpdnet"){
                    PubCloudForDescriptor();
                }
                if (REUSE_MAP || global_KFs_v_.size() >= 1.5* MATCHING_MIN_IDX){
                    if(DETECTION_METHOD=="lpdnet"){
                        if(add_new_kf_){ //两次闭环之间保证一定间隔
                            LoopClosureDetection();
                            add_new_kf_ = false;
                        }
                    }
                    else if(DETECTION_METHOD=="loam"){
                        if(to_detect_ ){//&& global_KFs_v_.size()-last_kf_idx_>=0.25*MATCHING_MIN_IDX
                            LoopClosureDetection();
                            to_detect_ = false;
                        }
                    }
                    else{
                        ROS_ERROR("ERROR !!! : no such detection method !!!");
                    }

                }
                if(optimize_pose_graph_){
                    Optimize4DOF();
                    UpdateMap();
                    loop_closed_ = false;
                    optimize_pose_graph_ = false;
                    loop_count_++;
                }
                if(to_merge_map){
                    Optimize4DOF();
                    MapMerge();
                    UpdateMap();
                    ROS_ERROR("-------------------------------------------------------------------map merged !!!");
                    std::cout << " ============================================================       map merged ";
                }
                Publish();
                if(GLOBAL_OPT){
                    PubGlobalPath();
                }

            }
            ros::spinOnce();
            rate.sleep();

        }
    }

    bool GlobalMapper::AddNewKF(const gp_lio::KeyFrame& newKeyFrame) {
        if(need_update_)
            return false;
        global_KFs_v_.push_back(newKeyFrame);
        global_KFs_v_.back().index_ = global_index_;

        if(GLOBAL_OPT){
            Eigen::Quaterniond localQ;
            Eigen::Vector3d localP;
            localQ = global_KFs_v_.back().state_.quaternion_;
            localP = global_KFs_v_.back().state_.position_;
            double odom_t = global_KFs_v_.back().state_.timestamp_;
            global_estimator_.inputOdom(odom_t,localP,localQ);
            // to add gnss msgs
            gnss_mutex.lock();
            double  gnss_t = gnss_buf.front()->header.stamp.toSec();
            while(!gnss_buf.empty()){
                if( gnss_t >= odom_t - 0.01 && gnss_t <= odom_t + 0.01){
                    double  t_diff = std::abs(gnss_t - odom_t);
                    ROS_ERROR_STREAM( "       gnss vs odom  timestamp diff: "<< t_diff);
                    double pos_accuracy = gnss_buf.front()->position_covariance[0];
                    if (pos_accuracy <=0){
                        pos_accuracy = 1;
                    }
                    global_estimator_.inputGPS(odom_t,gnss_buf.front()->latitude,
                            gnss_buf.front()->longitude,gnss_buf.front()->altitude,pos_accuracy);
                    gnss_buf.pop();
                    break;
                }
                else if(gnss_t < odom_t -0.01)
                    gnss_buf.pop();
                else if (gnss_t > odom_t + 0.01)
                    break;
            }
            gnss_mutex.unlock();

        }
        if(loop_closed_){
            global_KFs_v_.back().loop_index_ = loop_pairs_v_.back().traj_pair_.first.back();
            global_KFs_v_.back().has_loop = true;
        }
        global_index_++;
        std::cout << std::endl << "************************************************ add new KF in global mapper: " << global_KFs_v_.size() - 1 << std::endl;
        add_latest_map_ = true;
        if(global_index_ % 10 == 0)
            to_detect_ = true;
        if(global_index_ % 10 == 0){
            optimize_pose_graph_ = true;
        }
        if(global_index_ % 10 == 0){
            build_tree_ = true;
        }
        return true;

    }

    void GlobalMapper::SearchTrajecoryOffsets() {

        std::vector<int> qs;
        for (int i = -(SEARCH_D_S - 1); i <= 0; ++i) {
            qs.push_back(i);
        }

        std::vector<int> traj_min, traj_max;
        traj_min.resize(qs.size(), 0);
        traj_max.resize(qs.size(), 0);
        for (int j = 0; j < qs.size(); ++j) {
            traj_min.at(j) = int(std::round(double(qs.at(j)) * SEARCH_V_MIN));
            traj_max.at(j) = int(std::round(double(qs.at(j)) * SEARCH_V_MAX));
        }

        ref_base_trajs_.reserve(3);
        ref_base_trajs_.emplace_back(traj_min);
        ref_base_trajs_.emplace_back(qs);
        ref_base_trajs_.emplace_back(traj_max);

        query_base_trajs_ = qs;

    }

    void GlobalMapper::PubCloudForDescriptor() {

        for (int i = 0; i < global_KFs_v_.size(); ++i) {

            if (!global_KFs_v_.at(i).feature_cloud_.empty())
                continue;

            // publish cloud for extracting global descriptor
            global_KFs_v_.at(i).ConstructFeatureCloud();

            assert(global_KFs_v_.at(i).feature_cloud_.size() != 4096);

            pcl::PointCloud<PointxyzWithHandcraftFeature>::Ptr pointxyz_features(new pcl::PointCloud<PointxyzWithHandcraftFeature>);
            int kstart = 20;
            int kend = 100;
            int kstep = 10;

            std::string ground_save_file = OUTPUT_PATH + "feature_cloud_for_descriptor.pcd";
            pcl::io::savePCDFileBinary(ground_save_file, global_KFs_v_.at(i).feature_cloud_);

            TicToc time_feature;
            LidarHandcraftFeature lidar_handcraft_feature(global_KFs_v_.at(i).feature_cloud_);
            lidar_handcraft_feature.generate_feature(pointxyz_features, kstart, kend, kstep);
            ROS_DEBUG_STREAM("lidar_handcraft_feature: " <<  time_feature.toc() << "ms");

            sensor_msgs::PointCloud2 feature_cloud_msg;
            pcl::toROSMsg(*pointxyz_features, feature_cloud_msg);
            feature_cloud_msg.header.stamp = ros::Time(global_KFs_v_.back().state_.timestamp_);

            pub_kf_cloud_.publish(feature_cloud_msg);
        }

    }

    void GlobalMapper::DescriptorCallback(const gp_lio::FeatureVector& descriptor_msg) {
        if(global_index_-last_closed_index > 30)
            add_new_kf_ = true;
        idx_ = -1;
        for (int i = 0; i < global_KFs_v_.size(); ++i) {
            if (std::abs(global_KFs_v_.at(i).state_.timestamp_ - descriptor_msg.header.stamp.toSec()) <= 1e-8)
                idx_ = i;
        }
        //   std::cout << "Receive new KF descriptor " << idx_ << "/" << global_KFs_v_.size() - 1 << std::endl;

        if (global_KFs_v_.at(idx_).descriptor_.calculated_)
            return;
        global_KFs_v_.at(idx_).descriptor_.calculated_ = true;
        std::vector<float> des(descriptor_msg.data);
        global_KFs_v_.at(idx_).descriptor_.descriptor_ = des;

        UpdateDistanceMat();

    }

    void GlobalMapper::UpdateDistanceMat() {

        for (int i = 0; i <= idx_; ++i) {

            if (distance_Mat_.at<float>(idx_, i) == FLT_MAX){
                if(global_KFs_v_.at(i).descriptor_.calculated_){
                    float d = float(ComputeDistance(global_KFs_v_.at(idx_).descriptor_.descriptor_, global_KFs_v_.at(i).descriptor_.descriptor_));
                    distance_Mat_.at<float>(idx_, i) = d;
                    distance_Mat_.at<float>(i, idx_) = d;
                }
            }
        }

        if(idx_ > MATCHING_MIN_IDX){

            std::vector<float> distance_v;
            for (int i = 0; i < idx_ - MATCHING_MIN_IDX; ++i) {
                distance_v.emplace_back(distance_Mat_.at<float>(i, idx_));
            }
            auto smallest = std::min_element(distance_v.begin(), distance_v.end());
            smallest_ref_idx_.at(idx_) = std::distance(distance_v.begin(), smallest);
            //     std::cout << idx_ << " smallest_ref_idx_: " << smallest_ref_idx_.at(idx_) << ", " << *smallest << std::endl;
        }

        if(SAVE_FEATURE_TMP_RESULT){

            std::string range_img_save_path = OUTPUT_PATH + "LoopClosureMat.txt";
            std::ofstream fout(range_img_save_path.c_str());
            for (int i = 0; i <= idx_; ++i) {
                for (int j = 0; j <= idx_; ++j) {
//                    if (distance_Mat_.at<float>(i, j) != FLT_MAX)
                    fout << distance_Mat_.at<float>(i, j) << "\t";
                }
                fout << std::endl;
            }
            fout.close();
        }

    }



    void GlobalMapper::LoopClosureDetection() {
        if (DETECTION_METHOD=="loam"){
            int seq_size = SEQUENCE_N;
            pcl::PointXYZ currentRobotPosPoint;
            currentRobotPosPoint.x = global_KFs_v_.back().state_.position_[0];
            currentRobotPosPoint.y = global_KFs_v_.back().state_.position_[1];
            currentRobotPosPoint.z = global_KFs_v_.back().state_.position_[2];
            pcl::PointCloud<pcl::PointXYZ>::Ptr keyFramesPoses3D (new pcl::PointCloud<pcl::PointXYZ>) ;
            int keyfram_szie = global_KFs_v_.size();
            if(build_tree_){
                ROS_ERROR("        >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>.  build tree");
                keyFramesPoses3D->points.resize(keyfram_szie);
                for(int i=0; i<keyfram_szie; ++i){
                    keyFramesPoses3D->points[i].x = global_KFs_v_.at(i).state_.position_[0];
                    keyFramesPoses3D->points[i].y = global_KFs_v_.at(i).state_.position_[1];
                    keyFramesPoses3D->points[i].z = global_KFs_v_.at(i).state_.position_[2];
                }
                kdtreeHistoryKeyPoses.setInputCloud(keyFramesPoses3D);
                build_tree_ = false;
            }

            std::vector<int> pointSearchIndLoop;
            std::vector<float> pointSearchSqDisLoop;
            double  historyKeyframeSearchRadius = 5; // 参考lego-loam 　参数需调试
            kdtreeHistoryKeyPoses.radiusSearch(currentRobotPosPoint, historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
            std::cout << pointSearchIndLoop.size() <<"  "<< pointSearchSqDisLoop.size() <<std::endl;
            int closestHistoryFrameID = -1;
            double closestHistoryFrameDS = -1;
            for(int i=0; i<pointSearchIndLoop.size();++i){
                int idx = pointSearchIndLoop[i];
                if(idx < (keyfram_szie-1.2*MATCHING_MIN_IDX) && idx > seq_size ){ // 保证querry帧（当前帧）与ref帧有一定间隔
//                    if(closestHistoryFrameID && idx >= closestHistoryFrameID){//尽可能选择indx更小（更old）的帧，大闭环效果更好
//                        continue;
//                    }
                    closestHistoryFrameID = idx;
                    closestHistoryFrameDS = pointSearchSqDisLoop[i];
                }

            }
            std::cout << "keyfram_szie          : "<<keyfram_szie<<std::endl;
            std::cout << "closestHistoryFrameID : "<<closestHistoryFrameID<<std::endl;
            std::cout << "closestHistoryFrameDS : "<<closestHistoryFrameDS<<std::endl;
            bool invald_seq = true;
            std::vector<int> query_traj, found_ref_traj;
            if(closestHistoryFrameID-seq_size >=0) {
                for (int seq_idx = seq_size; seq_idx > 0; --seq_idx) {
                    Eigen::Vector3d query_frame(global_KFs_v_.at(keyfram_szie - 1 - seq_idx).state_.position_);
                    Eigen::Vector3d ref_frame(global_KFs_v_.at(closestHistoryFrameID - seq_idx).state_.position_);
                    if (std::sqrt(std::pow(query_frame[0] - ref_frame[0], 2) + std::pow(query_frame[1] - ref_frame[1], 2) +
                                  std::pow(query_frame[2] - ref_frame[2], 2)) >= 1.5 * closestHistoryFrameDS) {
                        invald_seq = false;
                        break;
                    }
                    query_traj.push_back(keyfram_szie - seq_idx);
                    found_ref_traj.push_back(closestHistoryFrameID - seq_idx + 1);
                }
            }
            if(invald_seq && query_traj.size() == seq_size){
                last_kf_idx_ = keyfram_szie;
                std::cout << found_ref_traj.back() << "-" << query_traj.back() << std::endl;
                LoopClosurePair new_pair;
                new_pair.traj_pair_.first = found_ref_traj;
                new_pair.traj_pair_.second = query_traj;
                if (RelativeTransformation(new_pair)) {
                    loop_pairs_v_.push_back(new_pair);
                    loop_closed_ = true;
                    last_closed_index = query_traj.back();
                    if(!map_merged && global_KFs_v_.at(found_ref_traj.back()).sequence_ == 0){  //发现新旧序列的第一个有效闭环，则进行MapMerge
                        to_merge_map = true;
                        merge_point.first = query_traj.back();
                        merge_point.second = global_KFs_v_.at(query_traj.back()).state_;
                    }
                    ROS_ERROR("------------------------------- invalid loop ----------------------!!");
                }
            }

        }else{
            if (idx_ > MATCHING_MIN_IDX){
                // get query trajs
                std::vector<int> query_traj;
                std::vector<int> smallest_ref_idx_seg;
                std::vector<std::vector<int>> dynamic_ref_base_trajs; dynamic_ref_base_trajs.resize(3);
//            std::cout << "query_traj: ";
                for (int j = 0; j < query_base_trajs_.size(); ++j) {
                    if(idx_ + query_base_trajs_.at(j) > global_KFs_v_.size() )
                        continue;
                    if(!global_KFs_v_.at(idx_ + query_base_trajs_.at(j)).descriptor_.calculated_ ||
                       smallest_ref_idx_.at(idx_ + query_base_trajs_.at(j)) > global_KFs_v_.size())
                        continue;

                    query_traj.emplace_back(idx_ + query_base_trajs_.at(j));
                    smallest_ref_idx_seg.emplace_back(smallest_ref_idx_.at(query_traj.back()));
//                std::cout << query_traj.back() << "-" << smallest_ref_idx_seg.back() << " ";
                    dynamic_ref_base_trajs.at(0).emplace_back(ref_base_trajs_.at(0).at(j));
                    dynamic_ref_base_trajs.at(1).emplace_back(ref_base_trajs_.at(1).at(j));
                    dynamic_ref_base_trajs.at(2).emplace_back(ref_base_trajs_.at(2).at(j));

                }
//            std::cout << " " << "size: " << query_traj.size() << std::endl;

                // find candidate trajectories and scores
                std::vector<float> score_v;
                std::vector<std::vector<int>> candidate_ref_trajs;
                for (int i = SEARCH_D_S - 1; i < idx_ - MATCHING_MIN_IDX; ++i) {
                    // get reference trajs
                    std::vector<std::vector<int>> ref_trajs;
                    for (int j = 0; j < dynamic_ref_base_trajs.size(); ++j) {
//                    std::cout << "ref_traj " << j << ": ";
                        std::vector<int> ref_traj(dynamic_ref_base_trajs.at(j));
                        for (int k = 0; k < dynamic_ref_base_trajs.at(j).size(); ++k) {
                            ref_traj.at(k) = (i + dynamic_ref_base_trajs.at(j).at(k));
//                        std::cout << ref_traj.at(k) << " ";
                        }
//                    std::cout << std::endl;
                        ref_trajs.emplace_back(ref_traj);
                    }

                    float score;
                    std::vector<int> chosed_ref_traj;
                    SearchScore(query_traj, smallest_ref_idx_seg, ref_trajs, score, chosed_ref_traj);

                    if (score != -1){
                        score_v.emplace_back(score);
                        candidate_ref_trajs.emplace_back(chosed_ref_traj);
                    }
                }

                //      std::cout << "scoring finished" << std::endl;

                if (!score_v.empty()){

//                std::cout << "window thresholding" << std::endl;
                    float uniqueness; int ref_traj_index;
                    WindowThresholding(score_v, uniqueness, ref_traj_index);
                    //    std::cout << "window thresholding finished. ref_traj_index: " << ref_traj_index << std::endl;

                    if(ref_traj_index != -1){

                        for (auto it=candidate_ref_trajs.at(ref_traj_index).begin(); it!=candidate_ref_trajs.at(ref_traj_index).end();){
                            if(*it > global_KFs_v_.size())
                                it = candidate_ref_trajs.at(ref_traj_index).erase(it);
                            else
                                it++;
                        }

                        for (auto it=query_traj.begin(); it!=query_traj.end();){
                            if(*it > global_KFs_v_.size())
                                it = query_traj.erase(it);
                            else
                                it++;
                        }

                        std::cout << std::endl << "uniqueness " << uniqueness << "  : ref ";
                        for (int i : candidate_ref_trajs.at(ref_traj_index)) {
                            std::cout <<  i << " ";
                        }
                        std::cout << "; query ";
                        for (int j : query_traj) {
                            std::cout << j << " ";
                        }
                        std::cout << std::endl << std::endl;


                        if(uniqueness > MATCHING_U_RATE){
                            auto found_ref_traj = candidate_ref_trajs.at(ref_traj_index);

                            //  std::cout << found_ref_traj.back() << "-" << query_traj.back() << std::endl;
                            LoopClosurePair new_pair;
                            new_pair.traj_pair_.first = found_ref_traj;
                            new_pair.traj_pair_.second = query_traj;
                            std::cout << std::endl << "found          : "<<found_ref_traj.back()<<std::endl;
                            std::cout <<std::endl  << "reference      : "<<query_traj.back()<<std::endl;
                            if(RelativeTransformation(new_pair)){
                                loop_pairs_v_.push_back(new_pair);
                                loop_closed_ = true;
                                last_closed_index = query_traj.back();
                                if(!map_merged && global_KFs_v_.at(found_ref_traj.back()).sequence_ == 0){  //发现新旧序列的第一个有效闭环，则进行MapMerge
                                    to_merge_map = true;
                                    merge_point.first = query_traj.back();
                                    merge_point.second = global_KFs_v_.at(query_traj.back()).state_;
                                }
                                ROS_ERROR("------------------------------- invalid loop ----------------------!!");
                            }

                        }
                    }

                }
            }

        }


    }

    void GlobalMapper::SearchScore(std::vector<int> query_traj,
                                   std::vector<int> smallest_ref_idx_seg,
                                   std::vector<std::vector<int>> ref_trajs,
                                   float& score, std::vector<int>& chosed_ref_traj) {

//        std::cout << "eliminate the ref traj which don't have any smallest ref idx" << std::endl;
        std::vector<int> candidate_ref_traj;
        for (int i = 0; i < ref_trajs.size(); ++i) {
            std::vector<int> inter_v;
            std::set_intersection(smallest_ref_idx_seg.begin(), smallest_ref_idx_seg.end(), ref_trajs.at(i).begin(), ref_trajs.at(i).end(), std::back_inserter(inter_v));
            if (!inter_v.empty())
                candidate_ref_traj.emplace_back(i);
        }

        if (candidate_ref_traj.empty()){
            score = -1;
            chosed_ref_traj.clear();
            return;
        }


        std::vector<float> traj_scores; traj_scores.resize(candidate_ref_traj.size(), 0.0);
        for (int i_ref_traj = 0; i_ref_traj < candidate_ref_traj.size(); i_ref_traj++) {
//            std::cout << "size: "<< query_traj.size()  << " " <<  ref_trajs.at(candidate_ref_traj.at(i_ref_traj)).size() << std::endl;
            int count = 0;
            for (int i_pair = 0; i_pair < query_traj.size(); ++i_pair) {
                auto i_ref = ref_trajs.at(candidate_ref_traj.at(i_ref_traj)).at(i_pair);
                if(i_ref > global_KFs_v_.size()) continue;
                auto i_que = query_traj.at(i_pair);
                if(i_ref < 0 || !global_KFs_v_.at(i_ref).descriptor_.calculated_) continue;
                traj_scores.at(i_ref_traj) += distance_Mat_.at<float>(i_ref, i_que);
                count++;
            }
            traj_scores.at(i_ref_traj) /= float(count);
//            std::cout << "candidate ref_taj score: " << traj_scores.at(i_ref_traj) << std::endl;
        }

        auto smallest = std::min_element(traj_scores.begin(), traj_scores.end());
        auto smallest_idx = std::distance(traj_scores.begin(), smallest);

        score = *smallest;
        chosed_ref_traj = ref_trajs.at(candidate_ref_traj.at(smallest_idx));

    }

    void GlobalMapper::WindowThresholding(std::vector<float> score_v, float& uniqueness, int& traj_idx) {

//        std::cout << "score: ";
//        for(auto & i_score : score_v){
//            std::cout << i_score << " ";
//        }

        auto smallest = std::min_element(score_v.begin(), score_v.end());
        auto smallest_idx = std::distance(score_v.begin(), smallest);

        float smallest_score = *smallest;

        for (int i = std::max(smallest_idx - std::round(float(MATCHING_WINDOW)/2.0), 0.0); i < std::min(smallest_idx + std::round(float(MATCHING_WINDOW)/2.0), double (score_v.size())); ++i) {
            score_v.at(i) = FLT_MAX;
        }
        auto second_smallest = std::min_element(score_v.begin(), score_v.end());
        if (*second_smallest != FLT_MAX){
            uniqueness =  *second_smallest / smallest_score;
            traj_idx = smallest_idx;
        }else{
            uniqueness = 0.0;
            traj_idx = -1;
        }
    }

    double GlobalMapper::ComputeDistance(const std::vector<float>& v_1, const std::vector<float>& v_2) {

        assert((v_1.size()!=v_2.size()));
        double sum = 0;
        for (int i = 0; i < v_1.size(); ++i) {
            sum += pow((v_1.at(i)-v_2.at(i)), 2);
        }
        return sqrt(sum);
    }

    bool GlobalMapper::RelativeTransformation(LoopClosurePair& new_pair) {
        // data preparation
        pcl::PointCloud<pcl::PointXYZI> ref_point_cloud, query_point_cloud,
                ref_point_cloud_DS, query_point_cloud_DS;
        pcl::PointCloud<pcl::PointXYZI> ref_ground_cloud, query_ground_cloud;
        std::vector<PlaneFeature> ref_planes, query_planes;

        Eigen::Matrix4d ref_traj_end_pose = global_KFs_v_.at(new_pair.traj_pair_.first.back()).state_.getTransformation();

        for (int i = 0; i < new_pair.traj_pair_.first.size(); ++i) {

            ref_point_cloud += global_KFs_v_.at(new_pair.traj_pair_.first.at(i)).GetTransformedCloud(extrinsic_parameters_);
            ref_ground_cloud += global_KFs_v_.at(new_pair.traj_pair_.first.at(i)).GetTransformedGroundCloud(extrinsic_parameters_);

            Eigen::Matrix4d i_traj_pose = global_KFs_v_.at(new_pair.traj_pair_.first.at(i)).state_.getTransformation();
            Eigen::Matrix4d rel_T = ref_traj_end_pose.inverse() * i_traj_pose;
            if(!global_KFs_v_.at(new_pair.traj_pair_.first.at(i)).cloud_feature_.plane_feature_.empty()){
                for (int i_plane = 0; i_plane < global_KFs_v_.at(new_pair.traj_pair_.first.at(i)).cloud_feature_.plane_feature_.size(); i_plane++) {
                    auto plane = global_KFs_v_.at(new_pair.traj_pair_.first.at(i)).cloud_feature_.plane_feature_.at(i_plane);
                    plane.Transform(rel_T);
                    ref_planes.push_back(plane);
                }
            }
        }

        Eigen::Matrix4d query_traj_end_pose = global_KFs_v_.at(new_pair.traj_pair_.second.back()).state_.getTransformation();

        for (int i = 0; i < new_pair.traj_pair_.second.size(); ++i) {

            query_point_cloud += global_KFs_v_.at(new_pair.traj_pair_.second.at(i)).GetTransformedCloud(extrinsic_parameters_);
            query_ground_cloud += global_KFs_v_.at(new_pair.traj_pair_.second.at(i)).GetTransformedGroundCloud(extrinsic_parameters_);

            Eigen::Matrix4d i_traj_pose = global_KFs_v_.at(new_pair.traj_pair_.second.at(i)).state_.getTransformation();
            Eigen::Matrix4d rel_T = query_traj_end_pose.inverse() * i_traj_pose;
            if(!global_KFs_v_.at(new_pair.traj_pair_.second.at(i)).cloud_feature_.plane_feature_.empty()){
                for (int i_plane = 0; i_plane < global_KFs_v_.at(new_pair.traj_pair_.second.at(i)).cloud_feature_.plane_feature_.size(); i_plane++) {
                    auto plane = global_KFs_v_.at(new_pair.traj_pair_.second.at(i)).cloud_feature_.plane_feature_.at(i_plane);
                    plane.Transform(rel_T);
                    query_planes.push_back(plane);
                }
            }
        }

        downsize_filter_.setInputCloud(ref_point_cloud.makeShared());
        downsize_filter_.filter(ref_point_cloud_DS);

        downsize_filter_.setInputCloud(query_point_cloud.makeShared());
        downsize_filter_.filter(query_point_cloud_DS);

        // transform ref point cloud {presented in {G}} into {ref_traj_end} frame
        pcl::PointCloud<pcl::PointXYZI> transformed_ref_cloud;
        Eigen::Matrix4d trans = global_KFs_v_.at(new_pair.traj_pair_.first.back()).state_.getInverseTransformation();
        pcl::transformPointCloud(ref_point_cloud_DS, transformed_ref_cloud, trans);
        // transform query point cloud {presented in {G}} into {query_traj_end} frame
        pcl::PointCloud<pcl::PointXYZI> transformed_query_cloud;
        trans = global_KFs_v_.at(new_pair.traj_pair_.second.back()).state_.getInverseTransformation();
        pcl::transformPointCloud(query_point_cloud_DS, transformed_query_cloud, trans);

        // transform ref point cloud {presented in {G}} into {ref_traj_end} frame
        PointCloudT transformed_ref_ground_cloud;
        trans = global_KFs_v_.at(new_pair.traj_pair_.first.back()).state_.getInverseTransformation();
        pcl::transformPointCloud(ref_ground_cloud, transformed_ref_ground_cloud, trans);
        // transform query point cloud {presented in {G}} into {query_traj_end} frame
        PointCloudT transformed_query_ground_cloud;
        trans = global_KFs_v_.at(new_pair.traj_pair_.second.back()).state_.getInverseTransformation();
        pcl::transformPointCloud(query_ground_cloud, transformed_query_ground_cloud, trans);

        PointCloudT ref_plane_cloud, query_plane_cloud;
        if(!ref_planes.empty() && !query_planes.empty()){
            for (int j = 0; j < ref_planes.size(); ++j) {
                ref_plane_cloud += ref_planes.at(j).cloud_;
            }
            for (int j = 0; j < query_planes.size(); ++j) {
                query_plane_cloud += query_planes.at(j).cloud_;
            }
        }

        if(SAVE_FEATURE_TMP_RESULT){

            std::string ref_save_file = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) +"_target_"+ std::to_string(new_pair.traj_pair_.first.back()) + ".pcd";
            pcl::io::savePCDFileBinary(ref_save_file,  transformed_ref_cloud);

            std::string query_save_file = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) + "_source_"+ std::to_string(new_pair.traj_pair_.second.back()) + ".pcd";
            pcl::io::savePCDFileBinary(query_save_file, transformed_query_cloud);

            std::string ref_ground_save_file = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) +"_target_ground_" + std::to_string(new_pair.traj_pair_.first.back()) + ".pcd";
            pcl::io::savePCDFileBinary(ref_ground_save_file,  transformed_ref_ground_cloud);

            std::string query_ground_save_file = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) + "_source_ground_"+ std::to_string(new_pair.traj_pair_.second.back()) + ".pcd";
            pcl::io::savePCDFileBinary(query_ground_save_file, transformed_query_ground_cloud);

            if(!ref_planes.empty() && !query_planes.empty()){

                std::string plane_2d_line_path = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) + "_plane_2d_line_correspondence.txt";
                std::ofstream fout(plane_2d_line_path.c_str());

                for (int j = 0; j < ref_planes.size(); ++j) {
                    auto line = ref_planes.at(j).Get2DLine();
                    auto cp = ref_planes.at(j).cp_;
                    fout << "0 " << line.at(0).transpose() << " " << line.at(1).transpose() << " " << cp.transpose() << std::endl;
                }

                for (int j = 0; j < query_planes.size(); ++j) {
                    auto line = query_planes.at(j).Get2DLine();
                    auto cp = query_planes.at(j).cp_;
                    fout << "1 " << line.at(0).transpose() << " " << line.at(1).transpose() << " " << cp.transpose() << std::endl;
                }

                fout.close();

                if (!ref_plane_cloud.empty()){
                    std::string cloud_save_file = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) + "_ref_plane_cloud_" + std::to_string(new_pair.traj_pair_.first.back()) + ".pcd";
                    pcl::io::savePCDFileBinary(cloud_save_file, ref_plane_cloud);
                }
                if (!query_plane_cloud.empty()){
                    std::string cloud_save_file = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) + "_query_plane_cloud_" + std::to_string(new_pair.traj_pair_.second.back()) + ".pcd";
                    pcl::io::savePCDFileBinary(cloud_save_file, query_plane_cloud);
                }
            }
        }

        float query_size = transformed_query_cloud.size();
        float ref_size = transformed_ref_cloud.size();
        float diff_size = std::abs(query_size - ref_size);
        float size_rate = std::max(diff_size/query_size, diff_size/ref_size);
        // std::cout << "size diff rate: " <<  size_rate << std::endl;
        if(size_rate > 0.3) return false;

        if (query_planes.empty() || ref_planes.empty())
            return false;


        std::vector<pcl::ModelCoefficients> planes_target;
        std::vector<pcl::ModelCoefficients> planes_source;
        for (int k = 0; k < query_planes.size(); ++k) {
            pcl::ModelCoefficients i_plane;
            i_plane.values.emplace_back(query_planes.at(k).normal_.x());
            i_plane.values.emplace_back(query_planes.at(k).normal_.y());
            i_plane.values.emplace_back(query_planes.at(k).normal_.z());
            i_plane.values.emplace_back(query_planes.at(k).d_);
            planes_source.emplace_back(i_plane);
        }
        for (int l = 0; l < ref_planes.size(); ++l) {
            pcl::ModelCoefficients i_plane;
            i_plane.values.emplace_back(ref_planes.at(l).normal_.x());
            i_plane.values.emplace_back(ref_planes.at(l).normal_.y());
            i_plane.values.emplace_back(ref_planes.at(l).normal_.z());
            i_plane.values.emplace_back(ref_planes.at(l).d_);
            planes_target.emplace_back(i_plane);
        }

        std::cout << "[[ " << "ref_cloud: " << transformed_ref_cloud.size() <<"query_cloud: " << transformed_query_cloud.size()<<std::endl;
        std::cout << "   " << "ref_ground: " << transformed_ref_ground_cloud.size() <<"query_ground: " << transformed_query_ground_cloud.size()<<std::endl;
        std::cout << "   " << "ref_plane: " << ref_plane_cloud.size() <<"query_plane: " << query_plane_cloud.size() <<"  ]]"<<std::endl;

        RegistrationResult cloud_result = local_map_matcher_.Match(transformed_query_cloud, transformed_ref_cloud,
                                                                   transformed_query_ground_cloud, transformed_ref_ground_cloud,
                                                                   query_plane_cloud, ref_plane_cloud,
                                                                   planes_source, planes_target);

        if(cloud_result.convergence_ &&  cloud_result.fitness_score_< ICP_FITNESS_SCORE){
            auto ypr = cloud_result.GetYPR();
            //       std::cout << "ypr : " << ypr.transpose() * 180.0 / M_PI << std::endl;
            //       std::cout << "position : " << cloud_result.rel_trans_.transpose() << std::endl;
            PointCloudT transformed_cloud;
            pcl::transformPointCloud(transformed_query_cloud, transformed_cloud, cloud_result.GetTransformation());
//            ShowClouds(ref_cloud, transformed_cloud, "cloud_refined");
            if(SAVE_FEATURE_TMP_RESULT){
                PointCloudT saved_cloud = transformed_cloud + transformed_ref_cloud;
                std::string save_file = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) +"_result_" + ".pcd";
                pcl::io::savePCDFileBinary(save_file,  saved_cloud );
            }
            new_pair.rel_rotation_ = cloud_result.rel_rotation_;
            new_pair.rel_trans_ = cloud_result.rel_trans_;
            new_pair.fitness_score_ = cloud_result.fitness_score_;
            new_pair.convergence_ = cloud_result.convergence_;

        }else{

            new_pair.fitness_score_ = cloud_result.fitness_score_;
            new_pair.convergence_ = cloud_result.convergence_;
            //std::cout << "All cloud registration failed!!!" << std::endl;
        }

//        // 1 - use ground plane points to determine the (roll, pitch, z)
//        GroundRegistration(new_pair, transformed_ref_ground_cloud, transformed_query_ground_cloud);
//        // 2 - compute the heading angle;
//        if(!ref_plane_cloud.empty() && !query_plane_cloud.empty() && new_pair.convergence_){
//            PlaneRegistration(new_pair, ref_plane_cloud, query_plane_cloud);
//        }
//        // 3 - compute the remaining parameters;
//        if (new_pair.convergence_)
//            AllCloudRegistration(new_pair, transformed_ref_cloud, transformed_query_cloud);


        return new_pair.convergence_ && new_pair.fitness_score_ < ICP_FITNESS_SCORE;

    }

    // solve the relative trans between loop closure pair
    bool GlobalMapper::RelativeTransformationNDT(gp_lio::LoopClosurePair &new_pair) {
        PointCloudT ref_point_cloud, query_point_cloud,
                ref_point_cloud_DS, query_point_cloud_DS,
                to_transform_ref_cloud, to_transform_query_cloud;
        Eigen::Matrix4d ref_traj_end_pose = global_KFs_v_.at(new_pair.traj_pair_.first.back()).state_.getTransformation();
        for(int i=0;i<new_pair.traj_pair_.first.size();++i){
            ref_point_cloud += global_KFs_v_.at(new_pair.traj_pair_.first.at(i)).GetTransformedAllCloud(extrinsic_parameters_);
        }
        Eigen::Matrix4d query_traj_end_pose = global_KFs_v_.at(new_pair.traj_pair_.second.back()).state_.getTransformation();
        for(int i=0;i<new_pair.traj_pair_.second.size();++i){
            query_point_cloud += global_KFs_v_.at(new_pair.traj_pair_.second.at(i)).GetTransformedAllCloud(extrinsic_parameters_);
        }

        downsize_filter_.setInputCloud(ref_point_cloud.makeShared());
        downsize_filter_.filter(ref_point_cloud_DS);
        downsize_filter_.setInputCloud(query_point_cloud.makeShared());
        downsize_filter_.filter(query_point_cloud_DS);

        Eigen::Matrix4d ref_trans = global_KFs_v_.at(new_pair.traj_pair_.first.back()).state_.getInverseTransformation();
        pcl::transformPointCloud(ref_point_cloud_DS,to_transform_ref_cloud,ref_trans);
        Eigen::Matrix4d query_trans = global_KFs_v_.at(new_pair.traj_pair_.second.back()).state_.getInverseTransformation();
        pcl::transformPointCloud(query_point_cloud_DS,to_transform_query_cloud,query_trans);

        pcl::PointCloud<pcl::PointXYZI> result_pointcloud;
        Eigen::Matrix4f guss_trans = Eigen::Matrix4f::Identity(),rel_trans;
        ndt_ptr_->setInputTarget(to_transform_ref_cloud.makeShared());
        ndt_ptr_->setInputSource(to_transform_query_cloud.makeShared());
        ndt_ptr_->align(result_pointcloud,guss_trans);
        rel_trans = ndt_ptr_->getFinalTransformation();

        if(ndt_ptr_->hasConverged()){
            PointCloudT transformed_cloud;
            pcl::transformPointCloud(to_transform_query_cloud,transformed_cloud,rel_trans);
            new_pair.rel_rotation_ = rel_trans.block<3,3>(0,0);
            new_pair.rel_trans_ = rel_trans.block<3,1>(0,3);
            new_pair.fitness_score_ = ndt_ptr_->getFitnessScore();
            new_pair.convergence_ = true;
        }else{
            new_pair.fitness_score_ = ndt_ptr_->getFitnessScore();
            new_pair.convergence_ = ndt_ptr_->hasConverged();
        }
        return new_pair.convergence_ ;

    }



    void GlobalMapper::GroundRegistration(LoopClosurePair& new_pair, PointCloudT ref_cloud, PointCloudT query_cloud) {

        // std::cout << std::endl << std::endl << "ground registration" << std::endl;
        // Roll Pitch Z using ground points
        auto ground_result = ICPRegistration(ref_cloud, query_cloud);

        Eigen::Matrix4f init_guess; init_guess.setIdentity();
        if(ground_result.convergence_ && ground_result.fitness_score_ < ICP_FITNESS_SCORE){

            auto ypr = ground_result.GetYPR();
            // std::cout << "ypr : " << ypr.transpose() * 180.0 / M_PI << std::endl;
            // std::cout << "position : " << ground_result.rel_trans_.transpose() << std::endl;
            PointCloudT transformed_cloud;

            Eigen::Vector3f xyz = ground_result.rel_trans_;
            Eigen::Matrix3f rot = Utility::ypr2R(Eigen::Vector3f{0, ypr[1], ypr[2]});
            Eigen::Vector3f position(0,0,xyz[2]);

            new_pair.rel_rotation_ = rot;
            new_pair.rel_trans_ = position;
            new_pair.convergence_ = true;

        }else{
            new_pair.convergence_ = false;
            // std::cout << "ground registration failed!!!" << std::endl;
        }

    }

    void GlobalMapper::PlaneRegistration(LoopClosurePair& new_pair, PointCloudT ref_cloud, PointCloudT query_cloud) {


        //std::cout <<  std::endl <<  std::endl << "Plane registration " << std::endl;

        Eigen::Matrix4f init_guess = new_pair.GetTransform();
        // Yaw using plane points
        RegistrationResult plane_result = ICPRegistration(ref_cloud, query_cloud, init_guess);
        if(plane_result.convergence_){

            auto plane_ypr = plane_result.GetYPR();
            // std::cout << "plane_ypr : " << plane_ypr.transpose()  * 180.0 / M_PI << std::endl;
            //std::cout << "position : " << plane_result.rel_trans_.transpose() << std::endl;

            Eigen::Matrix3f ground_rot = init_guess .block(0, 0, 3, 3);
            Eigen::Vector3f ground_ypr = Utility::R2ypr(ground_rot) / 180.0 * M_PI;
            Eigen::Vector3f ground_position = init_guess.block(0, 3, 3, 1);

            Eigen::Matrix3f rot = Utility::ypr2R(Eigen::Vector3f{plane_ypr[0], ground_ypr[1], ground_ypr[2]});
            Eigen::Vector3f position(0, 0, ground_position[2]);

            new_pair.rel_rotation_ = rot;
            new_pair.rel_trans_ = position;
            new_pair.convergence_ = true;

        }else{
            new_pair.convergence_ = false;
            //std::cout << "plane registration failed!!!" << std::endl;
        }

    }

    void GlobalMapper::AllCloudRegistration(LoopClosurePair& new_pair, PointCloudT ref_cloud, PointCloudT query_cloud) {


        //std::cout <<  std::endl <<  std::endl << "cloud registration "  << std::endl;

        Eigen::Matrix4f init_guess = new_pair.GetTransform();

        RegistrationResult cloud_result = ICPRegistration(ref_cloud, query_cloud, init_guess);
        if(cloud_result.convergence_ &&  cloud_result.fitness_score_< ICP_FITNESS_SCORE){
            auto ypr = cloud_result.GetYPR();
            // std::cout << "ypr : " << ypr.transpose() * 180.0 / M_PI << std::endl;
            //  std::cout << "position : " << cloud_result.rel_trans_.transpose() << std::endl;
            PointCloudT transformed_cloud;
            pcl::transformPointCloud(query_cloud, transformed_cloud, cloud_result.GetTransformation());
//            ShowClouds(ref_cloud, transformed_cloud, "cloud_refined");
            if(SAVE_FEATURE_TMP_RESULT){
                PointCloudT saved_cloud = transformed_cloud + ref_cloud;
                std::string save_file = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) +"_result_" + ".pcd";
                pcl::io::savePCDFileBinary(save_file,  saved_cloud );
            }
            new_pair.rel_rotation_ = cloud_result.rel_rotation_;
            new_pair.rel_trans_ = cloud_result.rel_trans_;
            new_pair.fitness_score_ = cloud_result.fitness_score_;
            new_pair.convergence_ = cloud_result.convergence_;

        }else{

            new_pair.fitness_score_ = cloud_result.fitness_score_;
            new_pair.convergence_ = cloud_result.convergence_;
            //  std::cout << "All cloud registration failed!!!" << std::endl;
        }

    }

    RegistrationResult GlobalMapper::ICPRegistration(gp_lio::PointCloudT ref_cloud, gp_lio::PointCloudT query_cloud, Eigen::Matrix4f init_guess) {


        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setMaxCorrespondenceDistance(100);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);
        // Align clouds
        icp.setInputSource(query_cloud.makeShared());
        icp.setInputTarget(ref_cloud.makeShared());
        pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());
        icp.align(*unused_result, init_guess);

        // std::cout << "icp Converged: " << icp.hasConverged() << " fitnessScore: " << icp.getFitnessScore() << std::endl;

        RegistrationResult new_result;
        new_result.fitness_score_ = icp.getFitnessScore();
        new_result.convergence_ = icp.hasConverged();
        Eigen::Matrix4f rel_T  = icp.getFinalTransformation();
        new_result.rel_rotation_  = rel_T.block(0, 0, 3, 3);
        new_result.rel_trans_ = rel_T.block(0, 3, 3, 1);

        return new_result;
    }

    PointCloudT GlobalMapper::ChangeCloud(pcl::PointCloud<pcl::PointXYZ> cloud) {

        PointCloudT new_cloud;
        for (int i = 0; i < cloud.size(); ++i) {

            PointT point;
            point.x = cloud.at(i).x;
            point.y = cloud.at(i).y;
            point.z = cloud.at(i).z;

            new_cloud.points.emplace_back(point);
        }
        return new_cloud;

    }

    void GlobalMapper::ShowClouds(gp_lio::PointCloudT ref_cloud, gp_lio::PointCloudT query_cloud, std::string type) {

        pcl::visualization::PCLVisualizer::Ptr cloud_viewer(new pcl::visualization::PCLVisualizer("show_" + type + "_cloud"));
        cloud_viewer->setBackgroundColor(255, 255, 255);
        cloud_viewer->addCoordinateSystem(1.0);
        cloud_viewer->initCameraParameters();

        pcl::visualization::PointCloudColorHandlerCustom<PointT> ref_color(255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> query_color(0, 0, 255);

        cloud_viewer->addPointCloud<PointT>(ref_cloud.makeShared(), ref_color, "ref_" + type);
        cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "ref_" + type);

        cloud_viewer->addPointCloud<PointT>(query_cloud.makeShared(), query_color, "query_" + type);
        cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "query_" + type);

        while (!cloud_viewer->wasStopped()){

            cloud_viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100));
        }

    }

    void GlobalMapper::Optimization() {

        // if (global_KFs_v_.size()-last_optimized_idx_ <= 0.8 * MATCHING_MIN_IDX)
        //     return;

        ROS_ERROR("start pose graph optimization! ");
        if(SAVE_FEATURE_TMP_RESULT){

            std::string plane_2d_line_path = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) + "_pose_graph.txt";
            std::ofstream fout(plane_2d_line_path.c_str());

            for (size_t i = 0;  i < global_KFs_v_.size(); i++) {
                fout << "0 " << i << " "
                     << global_KFs_v_.at(i).state_.position_.x() << " "
                     << global_KFs_v_.at(i).state_.position_.y() << " "
                     << global_KFs_v_.at(i).state_.position_.z() << " "
                     << global_KFs_v_.at(i).state_.quaternion_.x() << " "
                     << global_KFs_v_.at(i).state_.quaternion_.y() << " "
                     << global_KFs_v_.at(i).state_.quaternion_.z() << " "
                     << global_KFs_v_.at(i).state_.quaternion_.w() << std::endl;
            }

            if(!loop_pairs_v_.empty()){

                for (auto & k : loop_pairs_v_) {

                    int i_a = k.traj_pair_.first.back();
                    int i_b = k.traj_pair_.second.back();
                    Eigen::Quaterniond rel_q = Eigen::Quaterniond(k.rel_rotation_.cast<double>());
                    Eigen::Vector3d rel_p = k.rel_trans_.cast<double>();
                    fout << "1 " << i_a << " " << i_b << " "
                         << rel_p.x() << " "
                         << rel_p.y() << " "
                         << rel_p.z() << " "
                         << rel_q.x() << " "
                         << rel_q.y() << " "
                         << rel_q.z() << " "
                         << rel_q.w() << std::endl;
                }
            }
            fout.close();
        }

        // std::cout << "last_optimized_idx: " << last_optimized_idx_ << "; current size: " << global_KFs_v_.size() << std::endl;

        // 1 --- build ceres problem
        ceres::Problem problem;
        ceres::LossFunction *lossFunction;
        lossFunction = new ceres::HuberLoss(0.1);
        size_t kf_size = global_KFs_v_.size();
        double para_pose[kf_size][SIZE_POSE];

        for (int i = 0; i < kf_size; i++)
        {
            ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
            problem.AddParameterBlock(para_pose[i], SIZE_POSE, local_parameterization);
        }

        for(int i = 0; i < 10; i++){
            problem.SetParameterBlockConstant(para_pose[i]);
        }

        // States to Para
        for (size_t i = 0;  i < kf_size; i++) {
            para_pose[i][0] = global_KFs_v_.at(i).state_.position_.x();
            para_pose[i][1] = global_KFs_v_.at(i).state_.position_.y();
            para_pose[i][2] = global_KFs_v_.at(i).state_.position_.z();
            para_pose[i][3] = global_KFs_v_.at(i).state_.quaternion_.x();
            para_pose[i][4] = global_KFs_v_.at(i).state_.quaternion_.y();
            para_pose[i][5] = global_KFs_v_.at(i).state_.quaternion_.z();
            para_pose[i][6] = global_KFs_v_.at(i).state_.quaternion_.w();
        }

        // add pose graph
        Eigen::Matrix<double, 6, 6> covariance;
        covariance.setZero();
//        Eigen::DiagonalMatrix<double, 6> covariance_diag;
        covariance.diagonal() << STD_X * STD_X, STD_Y * STD_Y, STD_Z * STD_Z, STD_R * STD_R, STD_P * STD_P, STD_YAW * STD_YAW;
//        covariance = covariance_diag.toDenseMatrix();

        Eigen::Matrix<double, 6, 6> sqrt_information = covariance.llt().matrixL();

        for (int i = 1; i < kf_size; ++i) {

            for (int j = 1; j < 5; j++)
            {
                if (i - j >= 0)
                {
                    Eigen::Vector3d pose_A_p = global_KFs_v_.at(i - j).state_.position_;
                    Eigen::Quaterniond pose_A_q = global_KFs_v_.at(i - j).state_.quaternion_;
                    Eigen::Vector3d pose_B_p = global_KFs_v_.at(i).state_.position_;
                    Eigen::Quaterniond pose_B_q = global_KFs_v_.at(i).state_.quaternion_;

                    Eigen::Quaterniond rel_q = pose_A_q.conjugate() * pose_B_q;
                    Eigen::Vector3d rel_p = pose_A_q.conjugate() * (pose_B_p - pose_A_p);
                    ceres::CostFunction *cost_function = PoseGraph3dErrorTerm::Create(rel_p, rel_q, sqrt_information);
                    problem.AddResidualBlock(cost_function, NULL, para_pose[i-j], para_pose[i]);

                }
            }

        }

        // add loop closure factor
        if(!loop_pairs_v_.empty()){

            for (auto & k : loop_pairs_v_) {

                int i_a = k.traj_pair_.first.back();
                int i_b = k.traj_pair_.second.back();

//                problem.SetParameterBlockConstant(para_pose[i_a]);
                //  std::cout << "Add loop pair: " << i_a << " - " << i_b << std::endl;

                Eigen::Quaterniond rel_q = Eigen::Quaterniond(k.rel_rotation_.cast<double>());
                Eigen::Vector3d rel_p = k.rel_trans_.cast<double>();

                covariance.setIdentity();
                covariance = covariance * pow(k.fitness_score_, 2);
                sqrt_information = covariance.llt().matrixL();

                ceres::CostFunction* cost_function = PoseGraph3dErrorTerm::Create( rel_p, rel_q, sqrt_information);
                problem.AddResidualBlock(cost_function, NULL, para_pose[i_a], para_pose[i_b]);
            }
        }

        // 2 --- run the solver
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.trust_region_strategy_type = ceres::DOGLEG;
//        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 100;

        // 3 --- get the optimized results
        TicToc t_solver;
        //  std::cout << "Pose graph optimization: \n begin solver" << std::endl;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        //std::cout << summary.FullReport() << std::endl;
        // std::cout << "Iterations : " <<summary.iterations.size() << " ";
        //std::cout << "ceres solver costs: " << t_solver.toc() << std::endl;

        // std::cout << t_solver.toc() << " " << summary.jacobian_evaluation_time_in_seconds*1000 << " ";

        State old_kf_size_state = global_KFs_v_.at(kf_size-1).state_;
        // Para to States
        for (size_t i = 0;  i < kf_size; i++) {

            global_KFs_v_.at(i).state_.position_.x() = para_pose[i][0];
            global_KFs_v_.at(i).state_.position_.y() = para_pose[i][1];
            global_KFs_v_.at(i).state_.position_.z() =  para_pose[i][2];
            global_KFs_v_.at(i).state_.quaternion_.x() = para_pose[i][3];
            global_KFs_v_.at(i).state_.quaternion_.y() =  para_pose[i][4];
            global_KFs_v_.at(i).state_.quaternion_.z() =  para_pose[i][5];
            global_KFs_v_.at(i).state_.quaternion_.w() = para_pose[i][6];

        }
        State optimized_kf_size_state = global_KFs_v_.at(kf_size-1).state_;

        need_update_ = true;

        last_optimized_idx_ = kf_size;

        // std::cout << "last_optimized_idx: " << last_optimized_idx_ << "; current size: " << global_KFs_v_.size() << std::endl;

        if (last_optimized_idx_ < global_KFs_v_.size()){

            Eigen::Matrix3d r_drift = optimized_kf_size_state.quaternion_.toRotationMatrix() * old_kf_size_state.quaternion_.toRotationMatrix().transpose();
            Eigen::Vector3d t_drift = optimized_kf_size_state.position_ - r_drift * old_kf_size_state.position_;

            for (int i = kf_size; i < global_KFs_v_.size(); i++)
            {
                Eigen::Vector3d P = global_KFs_v_.at(i).state_.position_;
                Eigen::Matrix3d R = global_KFs_v_.at(i).state_.quaternion_.toRotationMatrix();
                P = r_drift * P + t_drift;
                R = r_drift * R;
                global_KFs_v_.at(i).state_.quaternion_ = Eigen::Quaterniond(R);
                global_KFs_v_.at(i).state_.position_ = P;
            }
        }

        UpdateMap();

    }

    void GlobalMapper::Optimize4DOF() {

        // if (global_KFs_v_.size()-last_optimized_idx_ <= 0.8 * MATCHING_MIN_IDX)
        //     return;
        std::cout << "start pose graph optimization! " << std::endl;
        ROS_ERROR("start pose graph optimization! " );
        // std::cout << "last_optimized_idx: " << last_optimized_idx_ << "; current size: " << global_KFs_v_.size() << std::endl;
        if(SAVE_FEATURE_TMP_RESULT){

            std::string plane_2d_line_path = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) + "_pose_graph.txt";
            std::ofstream fout(plane_2d_line_path.c_str());

            for (size_t i = 0;  i < global_KFs_v_.size(); i++) {
                fout << "0 " << i << " "
                     << global_KFs_v_.at(i).state_.position_.x() << " "
                     << global_KFs_v_.at(i).state_.position_.y() << " "
                     << global_KFs_v_.at(i).state_.position_.z() << " "
                     << global_KFs_v_.at(i).state_.quaternion_.x() << " "
                     << global_KFs_v_.at(i).state_.quaternion_.y() << " "
                     << global_KFs_v_.at(i).state_.quaternion_.z() << " "
                     << global_KFs_v_.at(i).state_.quaternion_.w() << std::endl;
            }

            if(!loop_pairs_v_.empty()){

                for (auto & k : loop_pairs_v_) {

                    int i_a = k.traj_pair_.first.back();
                    int i_b = k.traj_pair_.second.back();
                    Eigen::Quaterniond rel_q = Eigen::Quaterniond(k.rel_rotation_.cast<double>());
                    Eigen::Vector3d rel_p = k.rel_trans_.cast<double>();
                    fout << "1 " << i_a << " " << i_b << " "
                         << rel_p.x() << " "
                         << rel_p.y() << " "
                         << rel_p.z() << " "
                         << rel_q.x() << " "
                         << rel_q.y() << " "
                         << rel_q.z() << " "
                         << rel_q.w() << std::endl;
                }
            }
            fout.close();
        }

        // 1 --- build ceres problem
        ceres::Problem problem;
        ceres::LossFunction *lossFunction, *lossFunctionGps;
        lossFunction = new ceres::HuberLoss(0.1);
        size_t kf_size = global_KFs_v_.size();
//            double para_pose[kf_size][SIZE_POSE];

        double t_array[kf_size][3];
        Eigen::Quaterniond q_array[kf_size];
        double euler_array[kf_size][3];
//            double sequence_array[kf_size];

        ceres::LocalParameterization* angle_local_parameterization =
                AngleLocalParameterization::Create();
//
        for (int i = 0; i < kf_size; i++) {

            // lio odom
            Eigen::Quaterniond tmp_q = global_KFs_v_.at(i).state_.quaternion_;
            Eigen::Matrix3d tmp_r = tmp_q.matrix();
            Eigen::Vector3d tmp_t = global_KFs_v_.at(i).state_.position_;

            tmp_q = tmp_r;
            t_array[i][0] = tmp_t(0);
            t_array[i][1] = tmp_t(1);
            t_array[i][2] = tmp_t(2);
            q_array[i] = tmp_q;

            Eigen::Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
            euler_array[i][0] = euler_angle.x();
            euler_array[i][1] = euler_angle.y();
            euler_array[i][2] = euler_angle.z();

            problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
            problem.AddParameterBlock(t_array[i], 3);
            // TODO::
            if(global_KFs_v_.at(i).sequence_ ==0){
                problem.SetParameterBlockConstant(euler_array[i]);
                problem.SetParameterBlockConstant(t_array[i]);
            }

            int j = 1;
            if (i - j >= 0)
            {
//                        std::cout << sequence_array[i] << " " << sequence_array[i-j] << std::endl;
                Eigen::Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                Eigen::Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                relative_t = q_array[i-j].inverse() * relative_t;
                double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
                ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                           relative_yaw, euler_conncected.y(), euler_conncected.z());
                problem.AddResidualBlock(cost_function, NULL, euler_array[i-j],
                                         t_array[i-j],
                                         euler_array[i],
                                         t_array[i]);
            }
//                }
        }

        for (int l = 0; l < 10; ++l) {
            problem.SetParameterBlockConstant(euler_array[l]);
            problem.SetParameterBlockConstant(t_array[l]);
        }


        // add loop closure factor
        if(!loop_pairs_v_.empty()){

            for (auto & k : loop_pairs_v_) {

                int i_a = k.traj_pair_.first.back();
                int i_b = k.traj_pair_.second.back();

                // std::cout << "Add loop pair: " << i_a << " - " << i_b << std::endl;

                Eigen::Quaterniond rel_q = Eigen::Quaterniond(k.rel_rotation_.cast<double>());
                Eigen::Vector3d rel_t = k.rel_trans_.cast<double>();

                Eigen::Vector3d euler_connected = Utility::R2ypr(q_array[i_a].toRotationMatrix());
                Eigen::Vector3d relative_t = rel_t;
                double relative_yaw =  Utility::R2ypr(rel_q.toRotationMatrix()).x();
                ceres::CostFunction* cost_function = FourDOFWeightError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                                                relative_yaw, euler_connected.y(), euler_connected.z());
                problem.AddResidualBlock(cost_function, lossFunction, euler_array[i_a],
                                         t_array[i_a],
                                         euler_array[i_b],
                                         t_array[i_b]);

            }
        }


        // 2 --- run the solver
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//            options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::DOGLEG;
        // options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 50;

        // 3 --- get the optimized results
        TicToc t_solver;
        // std::cout << "Pose graph optimization: \n begin solver" << std::endl;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        //std::cout << summary.FullReport() << std::endl;
        //std::cout << "Iterations : " << summary.iterations.size() << " ";
        // std::cout << "ceres solver costs: " << t_solver.toc() << "ms"<< std::endl;

        // std::cout << t_solver.toc() << " " << summary.jacobian_evaluation_time_in_seconds * 1000 << " ";

        State old_kf_size_state = global_KFs_v_.at(kf_size-1).state_;

        // Para to States
        for (size_t i = 0; i < kf_size; i++) {
            Eigen::Quaterniond tmp_q;
            tmp_q = Utility::ypr2R(Eigen::Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
            Eigen::Vector3d tmp_t = Eigen::Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
            Eigen::Matrix3d tmp_r = tmp_q.toRotationMatrix();
            global_KFs_v_.at(i).state_.quaternion_ = tmp_q;
            global_KFs_v_.at(i).state_.position_ = tmp_t;
        }
        State optimized_kf_size_state = global_KFs_v_.at(kf_size-1).state_;

        need_update_ = true;

        last_optimized_idx_ = kf_size;

        //  std::cout << "last_optimized_idx: " << last_optimized_idx_ << "; current size: " << global_KFs_v_.size() << std::endl;

        if (last_optimized_idx_ < global_KFs_v_.size()){

            double yaw_drift = Utility::R2ypr(optimized_kf_size_state.quaternion_.toRotationMatrix()).x() - Utility::R2ypr(old_kf_size_state.quaternion_.toRotationMatrix()).x();
            Eigen::Matrix3d r_drift = Utility::ypr2R(Eigen::Vector3d(yaw_drift, 0, 0));
            Eigen::Vector3d t_drift = optimized_kf_size_state.position_ - r_drift * old_kf_size_state.position_;

            for (int i = kf_size; i < global_KFs_v_.size(); i++)
            {
                Eigen::Vector3d P = global_KFs_v_.at(i).state_.position_;
                Eigen::Matrix3d R = global_KFs_v_.at(i).state_.quaternion_.toRotationMatrix();
                P = r_drift * P + t_drift;
                R = r_drift * R;
                global_KFs_v_.at(i).state_.quaternion_ = Eigen::Quaterniond(R);
                global_KFs_v_.at(i).state_.position_ = P;
            }
        }
    }

//    void GlobalMapper::Optimize4DOF() {
//
//        // if (global_KFs_v_.size()-last_optimized_idx_ <= 0.8 * MATCHING_MIN_IDX)
//        //     return;
//
//        ROS_ERROR("start pose graph optimization! " );
//        std::cout << "last_optimized_idx: " << last_optimized_idx_ << "; current size: " << global_KFs_v_.size() << std::endl;
//        if(SAVE_FEATURE_TMP_RESULT){
//
//            std::string plane_2d_line_path = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) + "_pose_graph.txt";
//            std::ofstream fout(plane_2d_line_path.c_str());
//
//            for (size_t i = 0;  i < global_KFs_v_.size(); i++) {
//                fout << "0 " << i << " "
//                     << global_KFs_v_.at(i).state_.position_.x() << " "
//                     << global_KFs_v_.at(i).state_.position_.y() << " "
//                     << global_KFs_v_.at(i).state_.position_.z() << " "
//                     << global_KFs_v_.at(i).state_.quaternion_.x() << " "
//                     << global_KFs_v_.at(i).state_.quaternion_.y() << " "
//                     << global_KFs_v_.at(i).state_.quaternion_.z() << " "
//                     << global_KFs_v_.at(i).state_.quaternion_.w() << std::endl;
//            }
//
//            if(!loop_pairs_v_.empty()){
//
//                for (auto & k : loop_pairs_v_) {
//
//                    int i_a = k.traj_pair_.first.back();
//                    int i_b = k.traj_pair_.second.back();
//                    Eigen::Quaterniond rel_q = Eigen::Quaterniond(k.rel_rotation_.cast<double>());
//                    Eigen::Vector3d rel_p = k.rel_trans_.cast<double>();
//                    fout << "1 " << i_a << " " << i_b << " "
//                         << rel_p.x() << " "
//                         << rel_p.y() << " "
//                         << rel_p.z() << " "
//                         << rel_q.x() << " "
//                         << rel_q.y() << " "
//                         << rel_q.z() << " "
//                         << rel_q.w() << std::endl;
//                }
//            }
//            fout.close();
//        }
//
//        // 1 --- build ceres problem
//        ceres::Problem problem;
//        ceres::LossFunction *lossFunction;
//        lossFunction = new ceres::HuberLoss(0.1);
//        size_t kf_size = global_KFs_v_.size();
////            double para_pose[kf_size][SIZE_POSE];
//
//        double t_array[kf_size][3];
//        Eigen::Quaterniond q_array[kf_size];
//        double euler_array[kf_size][3];
////            double sequence_array[kf_size];
//
//        ceres::LocalParameterization* angle_local_parameterization =
//                AngleLocalParameterization::Create();
////
//        for (int i = 0; i < kf_size; i++) {
//
//            Eigen::Quaterniond tmp_q = global_KFs_v_.at(i).state_.quaternion_;
//            Eigen::Matrix3d tmp_r = tmp_q.matrix();
//            Eigen::Vector3d tmp_t = global_KFs_v_.at(i).state_.position_;
//
//            tmp_q = tmp_r;
//            t_array[i][0] = tmp_t(0);
//            t_array[i][1] = tmp_t(1);
//            t_array[i][2] = tmp_t(2);
//            q_array[i] = tmp_q;
//
//            Eigen::Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
//            euler_array[i][0] = euler_angle.x();
//            euler_array[i][1] = euler_angle.y();
//            euler_array[i][2] = euler_angle.z();
//
//            problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
//            problem.AddParameterBlock(t_array[i], 3);
//
//            //add edge
////                for (int j = 1; j < 2; j++)
////                {
//            int j = 1;
//            if (i - j >= 0)
//            {
////                        std::cout << sequence_array[i] << " " << sequence_array[i-j] << std::endl;
//                Eigen::Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
//                Eigen::Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
//                relative_t = q_array[i-j].inverse() * relative_t;
//                double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
//                ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
//                                                                           relative_yaw, euler_conncected.y(), euler_conncected.z());
//                problem.AddResidualBlock(cost_function, NULL, euler_array[i-j],
//                                         t_array[i-j],
//                                         euler_array[i],
//                                         t_array[i]);
//            }
////                }
//        }
//
//        for (int l = 0; l < 10; ++l) {
//            problem.SetParameterBlockConstant(euler_array[l]);
//            problem.SetParameterBlockConstant(t_array[l]);
//        }
//
//
//        // add loop closure factor
//        if(!loop_pairs_v_.empty()){
//
//            for (auto & k : loop_pairs_v_) {
//
//                int i_a = k.traj_pair_.first.back();
//                int i_b = k.traj_pair_.second.back();
//
//                std::cout << "Add loop pair: " << i_a << " - " << i_b << std::endl;
//
//                Eigen::Quaterniond rel_q = Eigen::Quaterniond(k.rel_rotation_.cast<double>());
//                Eigen::Vector3d rel_t = k.rel_trans_.cast<double>();
//
//                Eigen::Vector3d euler_connected = Utility::R2ypr(q_array[i_a].toRotationMatrix());
//                Eigen::Vector3d relative_t = rel_t;
//                double relative_yaw =  Utility::R2ypr(rel_q.toRotationMatrix()).x();
//                ceres::CostFunction* cost_function = FourDOFWeightError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
//                                                                                relative_yaw, euler_connected.y(), euler_connected.z());
//                problem.AddResidualBlock(cost_function, lossFunction, euler_array[i_a],
//                                         t_array[i_a],
//                                         euler_array[i_b],
//                                         t_array[i_b]);
//
//                for (int i = -WINDOW_SIZE; i < WINDOW_SIZE; ++i) {
//                    if(std::max(i_a+i,i_b+i)>=global_KFs_v_.size() || std::min(i_a+i, i_b+i) < 0) continue;
//
//                    Eigen::Matrix4d  w_ia_T  = global_KFs_v_.at(i_a).state_.getTransformation();
//                    Eigen::Matrix4d  w_ib_T  = global_KFs_v_.at(i_b).state_.getTransformation();
//                    Eigen::Matrix4d  w_ia_i_T  = global_KFs_v_.at(i_a+i).state_.getTransformation();
//                    Eigen::Matrix4d  w_ib_i_T  = global_KFs_v_.at(i_b+i).state_.getTransformation();
//                    Eigen::Matrix4d ia_i_ia_T = w_ia_i_T.inverse() * w_ia_T;
//                    Eigen::Matrix4d ib_i_ib_T = w_ib_i_T.inverse() * w_ib_T;
//                    Eigen::Matrix4d  ia_ib_T =  k.GetTransform().cast<double>();
//                    Eigen::Matrix4d ia_i_ib_i_T = ia_i_ia_T * ia_ib_T * ib_i_ib_T.inverse();
//
//                    std::cout << "Add loop pair: " << i_a+i << " - " << i_b+i << std::endl;
//
//                    Eigen::Matrix3d ia_i_ib_i_R = ia_i_ib_i_T.block(0, 0, 3, 3);
//                    Eigen::Quaterniond q(ia_i_ib_i_R);
//                    Eigen::Vector3d ia_i_ib_i_t = ia_i_ib_i_T.block(0, 3, 3, 1);
//
//                    Eigen::Quaterniond rel_q = q;
//                    Eigen::Vector3d rel_p = ia_i_ib_i_t;
//
//                    Eigen::Quaterniond rel_q_i = q;
//                    Eigen::Vector3d rel_t_i = ia_i_ib_i_t;
//                    Eigen::Vector3d euler_connected_i = Utility::R2ypr(q_array[i_a+i].toRotationMatrix());
//                    Eigen::Vector3d relative_t_i = rel_t;
//                    double relative_yaw_i =  Utility::R2ypr(rel_q.toRotationMatrix()).x();
//                    ceres::CostFunction* cost_function_i = FourDOFWeightError::Create(relative_t_i.x(), relative_t_i.y(), relative_t_i.z(),
//                                                                                      relative_yaw_i, euler_connected_i.y(), euler_connected_i.z());
//                    problem.AddResidualBlock(cost_function_i, lossFunction, euler_array[i_a+i],
//                                             t_array[i_a+i],
//                                             euler_array[i_b+i],
//                                             t_array[i_b+i]);
//
//                }
//
//            }
//        }
//
//        // 2 --- run the solver
//        ceres::Solver::Options options;
//        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
////            options.linear_solver_type = ceres::DENSE_SCHUR;
//        options.trust_region_strategy_type = ceres::DOGLEG;
//        // options.minimizer_progress_to_stdout = true;
//        options.max_num_iterations = 50;
//
//        // 3 --- get the optimized results
//        TicToc t_solver;
//        std::cout << "Pose graph optimization: \n begin solver" << std::endl;
//        ceres::Solver::Summary summary;
//        ceres::Solve(options, &problem, &summary);
//
//        std::cout << summary.FullReport() << std::endl;
//        std::cout << "Iterations : " << summary.iterations.size() << " ";
//        std::cout << "ceres solver costs: " << t_solver.toc() << "ms"<< std::endl;
//
//        // std::cout << t_solver.toc() << " " << summary.jacobian_evaluation_time_in_seconds * 1000 << " ";
//
//        State old_kf_size_state = global_KFs_v_.at(kf_size-1).state_;
//
//        // Para to States
//        for (size_t i = 0; i < kf_size; i++) {
//            Eigen::Quaterniond tmp_q;
//            tmp_q = Utility::ypr2R(Eigen::Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
//            Eigen::Vector3d tmp_t = Eigen::Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
//            Eigen::Matrix3d tmp_r = tmp_q.toRotationMatrix();
//            global_KFs_v_.at(i).state_.quaternion_ = tmp_q.normalized();
//            global_KFs_v_.at(i).state_.position_ = tmp_t;
//        }
//        State optimized_kf_size_state = global_KFs_v_.at(kf_size-1).state_;
//
//        last_optimized_idx_ = kf_size;
//
//        std::cout << "last_optimized_idx: " << last_optimized_idx_ << "; current size: " << global_KFs_v_.size() << std::endl;
//
//        if (last_optimized_idx_ < global_KFs_v_.size()){
//
//            double yaw_drift = Utility::R2ypr(optimized_kf_size_state.quaternion_.toRotationMatrix()).x() - Utility::R2ypr(old_kf_size_state.quaternion_.toRotationMatrix()).x();
//            Eigen::Matrix3d r_drift = Utility::ypr2R(Eigen::Vector3d(yaw_drift, 0, 0));
//            Eigen::Vector3d t_drift = optimized_kf_size_state.position_ - r_drift * old_kf_size_state.position_;
//
//            for (int i = kf_size; i < global_KFs_v_.size(); i++)
//            {
//                Eigen::Vector3d P = global_KFs_v_.at(i).state_.position_;
//                Eigen::Matrix3d R = global_KFs_v_.at(i).state_.quaternion_.toRotationMatrix();
//                P = r_drift * P + t_drift;
//                R = r_drift * R;
//                global_KFs_v_.at(i).state_.quaternion_ = Eigen::Quaterniond(R);
//                global_KFs_v_.at(i).state_.position_ = P;
//            }
//        }
//
//        UpdateMap();
//        add_latest_map_ = false;
//        need_update_ = true;
//
//    }

    void GlobalMapper::UpdateMap() {

        all_feature_cloud_map_->clear();
        all_ground_cloud_map_->clear();
        all_plane_cloud_map_ ->clear();
        all_cloud_map_->clear();

        for (int j = 0; j < global_KFs_v_.size(); ++j) {

            map_downsize_filter_.setInputCloud(global_KFs_v_.at(j).GetTransformedEdgeCloud(extrinsic_parameters_).makeShared());
            PointCloudT::Ptr feature_cloud_DS(new PointCloudT());
            map_downsize_filter_.filter(*feature_cloud_DS);
            *all_feature_cloud_map_ += *feature_cloud_DS;

            map_downsize_filter_.setInputCloud(global_KFs_v_.at(j).GetTransformedGroundCloud(extrinsic_parameters_).makeShared());
            PointCloudT::Ptr ground_cloud_DS(new PointCloudT());
            map_downsize_filter_.filter(*ground_cloud_DS);
            *all_ground_cloud_map_ += *ground_cloud_DS;

            map_downsize_filter_.setInputCloud(global_KFs_v_.at(j).GetTransformedPlaneCloud(extrinsic_parameters_).makeShared());
            PointCloudT::Ptr plane_cloud_DS(new PointCloudT());
            map_downsize_filter_.filter(*plane_cloud_DS);
            *all_plane_cloud_map_ += *plane_cloud_DS;

            map_downsize_filter_.setInputCloud(global_KFs_v_.at(j).GetTransformedAllCloud(extrinsic_parameters_).makeShared());
            PointCloudT::Ptr all_cloud_DS(new PointCloudT());
            map_downsize_filter_.filter(*all_cloud_DS);
            *all_cloud_map_ += *all_cloud_DS;

        }

        need_update_ = false;

    }

    void GlobalMapper::GetClosedMap(PointCloudT &closed_edge_map, PointCloudT &closed_surf_map,
                                    PointCloudT &closed_ground_map, std::vector<PlaneFeature> &closed_planes, State state) {

        if (loop_pairs_v_.empty())
            return;
        TicToc get_close_map_t;
        auto pose = state.getInverseTransformation();
        for (int i = 0; i < global_KFs_v_.size(); ++i) {
            auto rel_pose = pose * global_KFs_v_.at(i).state_.getTransformation();
            auto t = rel_pose.block(0, 3, 3, 1);
            if(t.norm() < CLOSED_DISTANCE){
                closed_edge_map += global_KFs_v_.at(i).GetTransformedEdgeCloud(extrinsic_parameters_);
                closed_surf_map += global_KFs_v_.at(i).GetTransformedSurfCloud(extrinsic_parameters_);
                closed_ground_map += global_KFs_v_.at(i).GetTransformedGroundCloud(extrinsic_parameters_);
                auto planes = global_KFs_v_.at(i).GetTransformedPlane(extrinsic_parameters_);
                closed_planes.insert(closed_planes.end(), planes.begin(), planes.end());
            }
        }
        std:: cout << "  gcmap :" <<get_close_map_t.toc() ;

        ROS_ERROR_STREAM("Get closed map: " << closed_edge_map.points.size() << " " << closed_surf_map.points.size()<< " "  <<closed_ground_map.size() );

    }

    State GlobalMapper::GetUpdatePose() {

        if (global_KFs_v_.size()< WINDOW_SIZE || !need_update_)  // <=20
            return State();

        State latest_optimized_kf = global_KFs_v_.back().state_;
        need_update_ = false;
        return latest_optimized_kf;
    }

    void GlobalMapper::Publish() {
        PointCloudT pose_graph_new, pose_graph_old;
        for (int j = 0; j < global_KFs_v_.size(); ++j) {
            PointT position;
            position.x = global_KFs_v_.at(j).state_.position_.x();
            position.y = global_KFs_v_.at(j).state_.position_.y();
            position.z = global_KFs_v_.at(j).state_.position_.z();
            if(global_KFs_v_.at(j).sequence_ ==1)
                pose_graph_new.points.push_back(position);
            else
                pose_graph_old.points.push_back(position);

        }
        sensor_msgs::PointCloud2 pose_graph_new_msgs,pose_graph_old_msgs;
        pcl::toROSMsg(pose_graph_new,pose_graph_new_msgs);
        pcl::toROSMsg(pose_graph_old,pose_graph_old_msgs);
        pose_graph_new_msgs.header.frame_id ="map";
        pose_graph_old_msgs.header.frame_id = "map";
        pub_pose_graph_new_.publish(pose_graph_new_msgs);
        pub_pose_graph_old_.publish(pose_graph_old_msgs);

        if(add_latest_map_){

            map_downsize_filter_.setInputCloud(global_KFs_v_.back().GetTransformedEdgeCloud(extrinsic_parameters_).makeShared());
            PointCloudT::Ptr feature_cloud_DS(new PointCloudT());
            map_downsize_filter_.filter(*feature_cloud_DS);
            *all_feature_cloud_map_ += *feature_cloud_DS;

            map_downsize_filter_.setInputCloud(global_KFs_v_.back().GetTransformedGroundCloud(extrinsic_parameters_).makeShared());
            PointCloudT::Ptr ground_cloud_DS(new PointCloudT());
            map_downsize_filter_.filter(*ground_cloud_DS);
            *all_ground_cloud_map_ += *ground_cloud_DS;

            map_downsize_filter_.setInputCloud(global_KFs_v_.back().GetTransformedPlaneCloud(extrinsic_parameters_).makeShared());
            PointCloudT::Ptr plane_cloud_DS(new PointCloudT());
            map_downsize_filter_.filter(*plane_cloud_DS);
            *all_plane_cloud_map_ += *plane_cloud_DS;

            map_downsize_filter_.setInputCloud(global_KFs_v_.back().GetTransformedAllCloud(extrinsic_parameters_).makeShared());
            PointCloudT::Ptr all_cloud_DS(new PointCloudT());
            map_downsize_filter_.filter(*all_cloud_DS);
            *all_cloud_map_ += *all_cloud_DS;

            add_latest_map_ = false;
        }

        sensor_msgs::PointCloud2 all_feature_cloud_msg, all_ground_cloud_msg, all_plane_cloud_msg, all_cloud_msg;
        pcl::toROSMsg(*all_feature_cloud_map_, all_feature_cloud_msg);
        pcl::toROSMsg(*all_ground_cloud_map_, all_ground_cloud_msg);
        pcl::toROSMsg(*all_plane_cloud_map_, all_plane_cloud_msg);
        pcl::toROSMsg(*all_cloud_map_, all_cloud_msg);

        all_feature_cloud_msg.header.frame_id = "map";
        all_ground_cloud_msg.header.frame_id = "map";
        all_plane_cloud_msg.header.frame_id = "map";
        all_cloud_msg.header.frame_id = "map";

        all_feature_cloud_pub_.publish(all_feature_cloud_msg);
        all_ground_cloud_pub_.publish(all_ground_cloud_msg);
        all_plane_cloud_pub_.publish(all_plane_cloud_msg);
        all_cloud_pub_.publish(all_cloud_msg);

        visualization_msgs::MarkerArray loop_msg;
        for (int i = 0; i < loop_pairs_v_.size(); ++i) {

            visualization_msgs::Marker line_model;
            line_model.type = visualization_msgs::Marker::LINE_LIST;
            line_model.action = visualization_msgs::Marker::ADD;
            line_model.header.frame_id = "map";
            line_model.header.stamp = ros::Time::now();

            line_model.scale.x = 0.01;

            line_model.color.a = 1.0;
            line_model.color.r = 1.0;

            line_model.id = i;

            geometry_msgs::Point point0;
            geometry_msgs::Point point1;

            point0.x = global_KFs_v_.at(loop_pairs_v_.at(i).traj_pair_.first.back()).state_.position_.x();
            point0.y = global_KFs_v_.at(loop_pairs_v_.at(i).traj_pair_.first.back()).state_.position_.y();
            point0.z = global_KFs_v_.at(loop_pairs_v_.at(i).traj_pair_.first.back()).state_.position_.z();

            point1.x = global_KFs_v_.at(loop_pairs_v_.at(i).traj_pair_.second.back()).state_.position_.x();
            point1.y = global_KFs_v_.at(loop_pairs_v_.at(i).traj_pair_.second.back()).state_.position_.y();
            point1.z = global_KFs_v_.at(loop_pairs_v_.at(i).traj_pair_.second.back()).state_.position_.z();

            line_model.points.push_back(point0);
            line_model.points.push_back(point1);
            loop_msg.markers.push_back(line_model);
        }

        loop_pub_.publish(loop_msg);
    }

    void GlobalMapper::PublishReusedMap() {
        PointCloudT pose_graph;
        all_feature_cloud_map_->clear();
        all_ground_cloud_map_->clear();
        all_plane_cloud_map_->clear();
        all_cloud_map_->clear();
        PointCloudT::Ptr feature_cloud_DS(new PointCloudT());
        PointCloudT::Ptr ground_cloud_DS(new PointCloudT());
        PointCloudT::Ptr plane_cloud_DS(new PointCloudT());
        PointCloudT::Ptr all_cloud_DS(new PointCloudT());

        for(int j = 0;j<global_KFs_v_.size();++j){
            PointT position;
            position.x = global_KFs_v_.at(j).state_.position_.x();
            position.y = global_KFs_v_.at(j).state_.position_.y();
            position.z = global_KFs_v_.at(j).state_.position_.z();
            pose_graph.points.push_back(position);


            map_downsize_filter_.setInputCloud(global_KFs_v_.at(j).GetTransformedEdgeCloud(extrinsic_parameters_).makeShared());
            map_downsize_filter_.filter(*feature_cloud_DS);
            *all_feature_cloud_map_ += *feature_cloud_DS;

            map_downsize_filter_.setInputCloud(global_KFs_v_.at(j).GetTransformedGroundCloud(extrinsic_parameters_).makeShared());
            map_downsize_filter_.filter(*ground_cloud_DS);
            *all_ground_cloud_map_ += *ground_cloud_DS;

            map_downsize_filter_.setInputCloud(global_KFs_v_.at(j).GetTransformedPlaneCloud(extrinsic_parameters_).makeShared());
            map_downsize_filter_.filter(*plane_cloud_DS);
            *all_plane_cloud_map_ += *plane_cloud_DS;

            map_downsize_filter_.setInputCloud(global_KFs_v_.at(j).GetTransformedAllCloud(extrinsic_parameters_).makeShared());
            map_downsize_filter_.filter(*all_cloud_DS);
            *all_cloud_map_ += *all_cloud_DS;
        }

        sensor_msgs::PointCloud2 pose_graph_msg;
        pcl::toROSMsg(pose_graph, pose_graph_msg);
        pose_graph_msg.header.frame_id = "map";
        pub_pose_graph_old_.publish(pose_graph_msg);

        sensor_msgs::PointCloud2 all_feature_cloud_msg, all_ground_cloud_msg, all_plane_cloud_msg, all_cloud_msg;
        pcl::toROSMsg(*all_feature_cloud_map_, all_feature_cloud_msg);
        pcl::toROSMsg(*all_ground_cloud_map_, all_ground_cloud_msg);
        pcl::toROSMsg(*all_plane_cloud_map_, all_plane_cloud_msg);
        pcl::toROSMsg(*all_cloud_map_, all_cloud_msg);

        all_feature_cloud_msg.header.frame_id = "map";
        all_ground_cloud_msg.header.frame_id = "map";
        all_plane_cloud_msg.header.frame_id = "map";
        all_cloud_msg.header.frame_id = "map";

        all_feature_cloud_pub_.publish(all_feature_cloud_msg);
        all_ground_cloud_pub_.publish(all_ground_cloud_msg);
        all_plane_cloud_pub_.publish(all_plane_cloud_msg);
        all_cloud_pub_.publish(all_cloud_msg);

    }

    void GlobalMapper::LoadPosegraph() {
        std::string dir_path = OUTPUT_PATH + "/pose_graph";
        std::string file_path = dir_path + "/pose_graph.txt";
        std::string feature_v_fpath = dir_path + "/feature_vector.txt";

        std::string plane_cloud = "_plane.pcd";
        std::string ground_cloud = "_ground.pcd";
        std::string edge_cloud = "_edge.pcd";
        std::string less_edge_cloud = "_less_edge.pcd";
        std::string surf_cloud = "_surf.pcd";
        std::string less_surf_cloud = "_less_surf.pcd";
        std::string outlier_cloud = "_outlier.pcd";

        std::cout << "load previous pose graph" << std::endl;
        std::FILE *ifile_pose_graph, *ifile_feature_v;
        ifile_pose_graph = std::fopen(file_path.c_str(), "r");
        if (ifile_pose_graph == NULL) {
            cout << "Error ! Open [ " << file_path << " ] failed ..." << std::endl;
            return;
        }
        ifile_feature_v = std::fopen(feature_v_fpath.c_str(), "r");
        if (ifile_feature_v == NULL) {
            cout << "Error ! Open [ " << feature_v_fpath << " ] failed ..." << std::endl;
            return;
        }
        global_KFs_v_.clear();
        KeyFrame tmp_kf;
        int i;
        double px, py, pz, qx, qy, qz, qw, vx, vy, vz, ax, ay, az, ba_1, ba_2, ba_3, bg_1, bg_2, bg_3, time_stamp,gps_la,gps_lo,gps_al;
        while (std::fscanf(ifile_pose_graph,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                           &i, &px, &py, &pz, &qx, &qy, &qz, &qw, &vx, &vy, &vz, &ax, &ay, &az,
                           &ba_1, &ba_2, &ba_3,&bg_1, &bg_2, &bg_3, &time_stamp, &gps_la, &gps_lo, &gps_al) != EOF) {
            tmp_kf.state_.timestamp_ = time_stamp;
            tmp_kf.state_.position_.x() = px;
            tmp_kf.state_.position_.y() = py;
            tmp_kf.state_.position_.z() = pz;
            tmp_kf.state_.quaternion_.x() = qx;
            tmp_kf.state_.quaternion_.y() = qy;
            tmp_kf.state_.quaternion_.z() = qz;
            tmp_kf.state_.quaternion_.w() = qw;
            tmp_kf.state_.linear_velocity_ = {vx, vy, vz};
            tmp_kf.state_.angular_velocity_ = {ax, ay, az};
            tmp_kf.state_.bias_a_ = {ba_1, ba_2, ba_3};
            tmp_kf.state_.bias_g_ = {bg_1, bg_2, bg_3};
            tmp_kf.gnss_pose_ = {gps_la,gps_lo,gps_al};

            pcl::io::loadPCDFile(dir_path+"/"+std::to_string(i)+surf_cloud,tmp_kf.cloud_feature_.surf_feature_);
            pcl::io::loadPCDFile(dir_path+"/"+std::to_string(i)+edge_cloud,tmp_kf.cloud_feature_.edge_feature_);
            pcl::io::loadPCDFile(dir_path+"/"+std::to_string(i)+less_surf_cloud,tmp_kf.cloud_feature_.less_surf_feature_);
            pcl::io::loadPCDFile(dir_path+"/"+std::to_string(i)+less_edge_cloud,tmp_kf.cloud_feature_.less_edge_feature_);
            pcl::io::loadPCDFile(dir_path+"/"+std::to_string(i)+ground_cloud,tmp_kf.cloud_feature_.ground_feature_);
            pcl::io::loadPCDFile(dir_path+"/"+std::to_string(i)+outlier_cloud,tmp_kf.cloud_feature_.outlier_feature_);
            tmp_kf.ConstructFeatureCloud(); // feature_cloud

            std::string plane_dir = dir_path+"/"+std::to_string(i)+"_plane_feature";
            if(access(plane_dir.c_str(),0)== 0) {  // plane 特征不一定存在，先判断
                pcl::io::loadPCDFile(dir_path+"/"+std::to_string(i)+plane_cloud,tmp_kf.cloud_feature_.plane_cloud_);
                std::string plane_file_path = plane_dir + "/plane_feature.txt";
                std::FILE *planefile;
                planefile = std::fopen(plane_file_path.c_str(), "r");

                if (planefile == NULL) {
                    cout << "Error ! Open [ " << plane_file_path << " ] failed ..." << std::endl;
                    return;
                }

                int j;
                double c1, c2, c3, n1, n2, n3, d, r;
                PlaneFeature tmp_plane_feature;
                while (std::fscanf(planefile, "%d %lf %lf %lf %lf %lf %lf %lf %lf\n", &j, &c1, &c2, &c3, &n1, &n2,
                                   &n3, &d, &r) != EOF) {
                    tmp_plane_feature.center_ = {c1, c2, c3};
                    tmp_plane_feature.normal_ = {n1, n2, n3};
                    tmp_plane_feature.d_ = d;
                    tmp_plane_feature.cp_ = tmp_plane_feature.normal_ * d;
                    tmp_plane_feature.radius_ = r;
                    pcl::io::loadPCDFile(
                            dir_path + "/" + std::to_string(i) + "_plane_feature" + "/" + std::to_string(j) +
                            "_plane.pcd", tmp_plane_feature.cloud_);
                    tmp_kf.cloud_feature_.plane_feature_.emplace_back(tmp_plane_feature);
                }
                std::fclose(planefile);
            }

            tmp_kf.sequence_ = 0;
            tmp_kf.index_ = global_index_;
            global_index_++;
            global_KFs_v_.push_back(tmp_kf);
            tmp_kf.clear();
        }
        int calculated =0,j=0;
        std::vector<float > f_v(256,0.0);
        while(std::fscanf(ifile_feature_v,"%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
                          &calculated,&f_v[0],&f_v[1],&f_v[2],&f_v[3],&f_v[4],&f_v[5],&f_v[6],&f_v[7],&f_v[8],&f_v[9],&f_v[10],&f_v[11],&f_v[12],&f_v[13],&f_v[14],&f_v[15],&f_v[16],&f_v[17],&f_v[18],&f_v[19],
                          &f_v[20],&f_v[21],&f_v[22],&f_v[23],&f_v[24],&f_v[25],&f_v[26],&f_v[27],&f_v[28],&f_v[29],&f_v[30],&f_v[31],&f_v[32],&f_v[33],&f_v[34],&f_v[35],&f_v[36],&f_v[37],&f_v[38],&f_v[39]
                ,&f_v[40],&f_v[41],&f_v[42],&f_v[43],&f_v[44],&f_v[45],&f_v[46],&f_v[47],&f_v[48],&f_v[49],&f_v[50],&f_v[51],&f_v[52],&f_v[53],&f_v[54],&f_v[55],&f_v[56],&f_v[57],&f_v[58],&f_v[59]
                ,&f_v[60],&f_v[61],&f_v[62],&f_v[63],&f_v[64],&f_v[65],&f_v[66],&f_v[67],&f_v[68],&f_v[69],&f_v[70],&f_v[71],&f_v[72],&f_v[73],&f_v[74],&f_v[75],&f_v[76],&f_v[77],&f_v[78],&f_v[79]
                ,&f_v[80],&f_v[81],&f_v[82],&f_v[83],&f_v[84],&f_v[85],&f_v[86],&f_v[87],&f_v[88],&f_v[89],&f_v[90],&f_v[91],&f_v[92],&f_v[93],&f_v[94],&f_v[95],&f_v[96],&f_v[97],&f_v[98],&f_v[99]
                ,&f_v[100],&f_v[101],&f_v[102],&f_v[103],&f_v[104],&f_v[105],&f_v[106],&f_v[107],&f_v[108],&f_v[109],&f_v[110],&f_v[111],&f_v[112],&f_v[113],&f_v[114],&f_v[115],&f_v[116],&f_v[117],&f_v[118],&f_v[119]
                ,&f_v[120],&f_v[121],&f_v[122],&f_v[123],&f_v[124],&f_v[125],&f_v[126],&f_v[127],&f_v[128],&f_v[129],&f_v[130],&f_v[131],&f_v[132],&f_v[133],&f_v[134],&f_v[135],&f_v[136],&f_v[137],&f_v[138],&f_v[139]
                ,&f_v[140],&f_v[141],&f_v[142],&f_v[143],&f_v[144],&f_v[145],&f_v[146],&f_v[147],&f_v[148],&f_v[149],&f_v[150],&f_v[151],&f_v[152],&f_v[153],&f_v[154],&f_v[155],&f_v[156],&f_v[157],&f_v[158],&f_v[159]
                ,&f_v[160],&f_v[161],&f_v[162],&f_v[163],&f_v[164],&f_v[165],&f_v[166],&f_v[167],&f_v[168],&f_v[169],&f_v[170],&f_v[171],&f_v[172],&f_v[173],&f_v[174],&f_v[175],&f_v[176],&f_v[177],&f_v[178],&f_v[179]
                ,&f_v[180],&f_v[181],&f_v[182],&f_v[183],&f_v[184],&f_v[185],&f_v[186],&f_v[187],&f_v[188],&f_v[189],&f_v[190],&f_v[191],&f_v[192],&f_v[193],&f_v[194],&f_v[195],&f_v[196],&f_v[197],&f_v[198],&f_v[199]
                ,&f_v[200],&f_v[201],&f_v[202],&f_v[203],&f_v[204],&f_v[205],&f_v[206],&f_v[207],&f_v[208],&f_v[209],&f_v[210],&f_v[211],&f_v[212],&f_v[213],&f_v[214],&f_v[215],&f_v[216],&f_v[217],&f_v[218],&f_v[219]
                ,&f_v[220],&f_v[221],&f_v[222],&f_v[223],&f_v[224],&f_v[225],&f_v[226],&f_v[227],&f_v[228],&f_v[229],&f_v[230],&f_v[231],&f_v[232],&f_v[233],&f_v[234],&f_v[235],&f_v[236],&f_v[237],&f_v[238],&f_v[239]
                ,&f_v[240],&f_v[241],&f_v[242],&f_v[243],&f_v[244],&f_v[245],&f_v[246],&f_v[247],&f_v[248],&f_v[249],&f_v[250],&f_v[251],&f_v[252],&f_v[253],&f_v[254],&f_v[255])!=EOF){
            global_KFs_v_.at(j).descriptor_.descriptor_.assign(f_v.begin(),f_v.end());
            global_KFs_v_.at(j).descriptor_.calculated_ = calculated;
            j++;
        }
        std::fclose(ifile_feature_v);
        std::fclose(ifile_pose_graph);
        std::cout << "load finished ... ";
        PublishReusedMap();
        RecoveryDistanceMat();
        std::cout << "recovered distance matrix ..." <<std::endl;
    }


    void GlobalMapper::SavePosegraph() {
        std::string dir_path = OUTPUT_PATH + "/pose_graph";
        std::string odom_path = OUTPUT_PATH + "/odom";
        std::string pg_fpath = dir_path + "/pose_graph.txt";
        std::string feature_v_fpath = dir_path + "/feature_vector.txt";
        if(file_manager_.CreatDirectory(dir_path)){
            std::FILE * ofile_pose_graph;
            std::FILE * ofile_feature_vector;
            ofile_pose_graph =std::fopen(pg_fpath.c_str(),"w");
            ofile_feature_vector = std::fopen(feature_v_fpath.c_str(),"w");
            std::string plane_cloud = "_plane.pcd";
            std::string ground_cloud = "_ground.pcd";
            std::string edge_cloud = "_edge.pcd";
            std::string less_edge_cloud = "_less_edge.pcd";
            std::string surf_cloud = "_surf.pcd";
            std::string less_surf_cloud = "_less_surf.pcd";
            std::string outlier_cloud = "_outlier.pcd";
            int i=0 ;
            ROS_ERROR("ready to save posegraph please waite ...");
            for(auto & kf : global_KFs_v_){
                // pose_graph.txt　<--- position + quateration + linear_velocity + angular_velocity + bias_a + bias_g
                std::fprintf(ofile_pose_graph,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                             i ,kf.state_.position_.x(),kf.state_.position_.y(),kf.state_.position_.z(),
                             kf.state_.quaternion_.x(),kf.state_.quaternion_.y(),kf.state_.quaternion_.z(),kf.state_.quaternion_.w(),
                             kf.state_.linear_velocity_[0],kf.state_.linear_velocity_[1],kf.state_.linear_velocity_[2],
                             kf.state_.angular_velocity_[0],kf.state_.angular_velocity_[1],kf.state_.angular_velocity_[2],
                             kf.state_.bias_a_[0],kf.state_.bias_a_[1],kf.state_.bias_a_[2],
                             kf.state_.bias_g_[0],kf.state_.bias_g_[1],kf.state_.bias_g_[2],
                             kf.cloud_feature_.cloudTimeStamp_,
                             kf.gnss_pose_[0],kf.gnss_pose_[1],kf.gnss_pose_[2]);
                // feature vector
                std::fprintf(ofile_feature_vector,"%d ",kf.descriptor_.calculated_);
                for ( int j = 0; j<256; j++){
                    std::fprintf(ofile_feature_vector,"%f ",kf.descriptor_.descriptor_[j]);
                }
                std::fprintf(ofile_feature_vector,"\n");
                // feature clouds of key frames
                pcl::io::savePCDFileBinary(dir_path + "/" + std::to_string(i) + ground_cloud,kf.cloud_feature_.ground_feature_);
                pcl::io::savePCDFileBinary(dir_path + "/" + std::to_string(i) + edge_cloud,kf.cloud_feature_.edge_feature_);
                pcl::io::savePCDFileBinary(dir_path + "/" + std::to_string(i) + less_edge_cloud,kf.cloud_feature_.less_edge_feature_);
                pcl::io::savePCDFileBinary(dir_path + "/" + std::to_string(i) + surf_cloud,kf.cloud_feature_.surf_feature_);
                pcl::io::savePCDFileBinary(dir_path + "/" + std::to_string(i) + less_surf_cloud,kf.cloud_feature_.less_surf_feature_);
                pcl::io::savePCDFileBinary(dir_path + "/" + std::to_string(i) + outlier_cloud,kf.cloud_feature_.outlier_feature_);
                if(kf.cloud_feature_.plane_cloud_.size()) {
                    pcl::io::savePCDFileBinary(dir_path + "/" + std::to_string(i) + plane_cloud,
                                               kf.cloud_feature_.plane_cloud_);
                    // plane feature
                    std::string pf_dir_path = dir_path + "/" + std::to_string(i) + "_plane_feature";
                    std::string pf_fpath = pf_dir_path + "/plane_feature.txt";
                    if (file_manager_.CreatDirectory(pf_dir_path)) {
                        int pf_v_size = kf.cloud_feature_.plane_feature_.size();
                        if (pf_v_size) {
                            std::FILE *ofile_plane_feature;
                            ofile_plane_feature = std::fopen(pf_fpath.c_str(), "w");
                            for (int j = 0; j < pf_v_size; ++j) {
                                std::fprintf(ofile_plane_feature, "%d %lf %lf %lf %lf %lf %lf %lf %lf\n", j,
                                             kf.cloud_feature_.plane_feature_[j].center_[0],
                                             kf.cloud_feature_.plane_feature_[j].center_[1],
                                             kf.cloud_feature_.plane_feature_[j].center_[2],
                                             kf.cloud_feature_.plane_feature_[j].normal_[0],
                                             kf.cloud_feature_.plane_feature_[j].normal_[1],
                                             kf.cloud_feature_.plane_feature_[j].normal_[2],
                                             kf.cloud_feature_.plane_feature_[j].d_,
                                             kf.cloud_feature_.plane_feature_[j].radius_);
                                pcl::io::savePCDFileBinary(pf_dir_path + "/" + std::to_string(j) + plane_cloud,
                                                           kf.cloud_feature_.plane_feature_[j].cloud_);
                            }
                            std::fclose(ofile_plane_feature);
                        }
                    }
                }
                i++;
            }
            std::fclose(ofile_pose_graph);
            std::fclose(ofile_feature_vector);
        }
        ROS_ERROR("Save pose graph finshed !");
        std::string odom_file_path = odom_path + "/odom_maping.txt";
        if(file_manager_.CreatDirectory(odom_path)){
            std::FILE* ofile_odom = std::fopen(odom_file_path.c_str(),"w");
            ROS_ERROR("ready to save odom datas, please waite ...");
            int j=0;
            for(auto & kf:global_KFs_v_){
                double odom_t = global_KFs_v_.at(j).state_.timestamp_;
                Eigen::Vector3f odom_p(global_KFs_v_.at(j).state_.position_.cast<float>());
                Eigen::Quaternionf odom_q(global_KFs_v_.at(j).state_.quaternion_.cast<float>());
                std::fprintf(ofile_odom,"%lf %f %f %f %f %f %f %f\n"
                        ,odom_t,odom_p[0],odom_p[1],odom_p[2],odom_q.x(),odom_q.y(),odom_q.z(),odom_q.w());
                j++;
            }
            std::fclose(ofile_odom);
            ROS_ERROR("save odom datas finshed ...");
        }


    }

    bool GlobalMapper::IsLoopClosed(){
        return loop_closed_;

    }

    bool GlobalMapper::GetRelocalizationInfo(std::vector<std::pair<bool,Frame>>& loopframe_v){
        if(!loop_pairs_v_.size()){
            return false;
        }
        loopframe_v.clear();
        int kf_size = global_KFs_v_.size();
        Frame tmp_keyframe;
        FeatureCorPairs tmp_feature_cp;
        PlaneCorPair tmp_plane_cp = PlaneCorPair({0,0,0},{0,0,0},0,0);
        int loop_count = 0;
        for(int i=0;i<WINDOW_SIZE;++i){
            if(global_KFs_v_.at(kf_size-1-i).has_loop){
                int loop_index = global_KFs_v_.at(kf_size-1-i).loop_index_;
                for(int j=0;j<global_KFs_v_.at(loop_index).edgePairsGroup_.size();++j){
                    // edge
                    tmp_feature_cp = global_KFs_v_.at(loop_index).edgePairsGroup_.at(j);
                    tmp_keyframe.edgePairsGroup_.push_back(tmp_feature_cp);
                }
                for(int j=0;j<global_KFs_v_.at(loop_index).surfPairsGroup_.size();++j){
                    // surf
                    tmp_feature_cp = global_KFs_v_.at(loop_index).surfPairsGroup_.at(j);
                    tmp_keyframe.surfPairsGroup_.push_back(tmp_feature_cp);
                }
                for(int j=0;j<global_KFs_v_.at(loop_index).groundPairsGroup_.size();++j){
                    // ground
                    tmp_feature_cp = global_KFs_v_.at(loop_index).groundPairsGroup_.at(j);
                    tmp_keyframe.groundPairsGroup_.push_back(tmp_feature_cp);
                }
                for(int j=0;j<global_KFs_v_.at(loop_index).planeCorPairsGroup_.size();++j){
                    // plane
                    tmp_plane_cp = global_KFs_v_.at(loop_index).planeCorPairsGroup_.at(j);
                    tmp_keyframe.planeCorPairsGroup_.push_back(tmp_plane_cp);
                }

                loopframe_v.push_back(std::make_pair(true,tmp_keyframe));
                tmp_keyframe.clear();
                loop_count++;
            } else{
                loopframe_v.push_back(std::make_pair(false,tmp_keyframe));

            }
        };
        if(loop_count)
            return true;
        return false;

    }

    bool GlobalMapper::FastRelocalization(LoopClosurePair & loopcp) {
        if(loop_pairs_v_.empty())
            return false;
        loopcp = loop_pairs_v_.back();
        loop_closed_ = false;
        return true;

    }

    void GlobalMapper::FastRelocalization(State & updated_state){
        std::cout << "pose graph optimization　--------------------------------- for relocalization ! " << std::endl;
        Optimize4DOF();
        loop_closed_ = false;
        updated_state = GetUpdatePose();

    }


    void GlobalMapper::MapMerge() {
        int size = merge_point.first + 1;
        State old_state = merge_point.second;
        State updated_state = global_KFs_v_.at(merge_point.first).state_;
        for(int i=0;i<size;++i){
            if(global_KFs_v_.at(i).sequence_ == 1){
                auto rel_pose = old_state.getInverseTransformation() * global_KFs_v_.at(i).state_.getTransformation();
                auto new_state = updated_state.getTransformation() * rel_pose;
                global_KFs_v_.at(i).state_.resetPose(new_state);
            }
        }
        to_merge_map = false;
        map_merged = true;
    }

    void GlobalMapper::RecoveryDistanceMat() {
        int kf_size = global_KFs_v_.size();
        for(int i = 0; i<kf_size; ++i){
            for(int j = 0; j<=i; ++j){
                if(global_KFs_v_.at(i).descriptor_.calculated_){
                    float d = float(ComputeDistance(global_KFs_v_.at(i).descriptor_.descriptor_,global_KFs_v_.at(j).descriptor_.descriptor_));
                    distance_Mat_.at<float>(i,j) = d;
                    distance_Mat_.at<float>(j,i) = d;
                }

            }
        }

    }

    void GlobalMapper::GnssMsgsCallback(const sensor_msgs::NavSatFixPtr &gnss_msg) {
        gnss_mutex.lock();
        gnss_buf.push(gnss_msg);
        gnss_mutex.unlock();
    }



    void GlobalMapper::PubGlobalPath() {
        pcl::PointCloud<pcl::PointXYZ> global_path;
        global_estimator_.getGlobalPath(global_path);
        sensor_msgs::PointCloud2 global_path_msgs;
        pcl::toROSMsg(global_path,global_path_msgs);
        global_path_msgs.header.frame_id = "map";
        global_path_msgs.header.stamp = ros::Time::now();
        pub_global_path_.publish(global_path_msgs);
    }

}
