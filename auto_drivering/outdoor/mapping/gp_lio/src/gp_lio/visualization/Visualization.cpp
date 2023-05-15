//
// Created by wchen on 2019/12/3.
//

#include "gp_lio/visualization/Visualization.h"


namespace gp_lio{

    Visualization::Visualization() {
    }

    Visualization::~Visualization() {
    }

    void Visualization::LoadConfig(ros::NodeHandle nh) {

        nh_ = nh;
        local_feature_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/local_feature_cloud", 2);
        local_ground_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/local_ground_cloud", 2);
        local_plane_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/local_plane_cloud", 2);
        local_traj_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/local_traj", 2);
        local_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/local_map", 2);
        cors_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/cors", 2);

        velocity_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/velocity", 2);

        local_feature_cloud_map_DS_.reset(new PointCloudT());
        local_ground_cloud_map_DS_.reset(new PointCloudT());
        local_plane_cloud_map_DS_.reset(new PointCloudT());
        local_trajctory_.reset(new PointCloudT());
        local_map_DS_.reset(new PointCloudT());


        downsize_filter_.setLeafSize(0.4, 0.4, 0.4);

        i_ = 0;
    }

    void Visualization::UpdateLocalWindow(std::vector<State> local_window_state,
                                          std::vector<PointCloudT> local_window_feature,
                                          std::vector<PointCloudT> local_window_ground,
                                          std::vector<PointCloudT> local_window_plane) {

        PointCloudT::Ptr local_feature_map(new PointCloudT());
        PointCloudT::Ptr local_ground_map(new PointCloudT());
        PointCloudT::Ptr local_plane_map(new PointCloudT());
        local_trajctory_->clear();
        velocity_msg_.markers.resize(0);

        for (size_t i = 0; i < local_window_state.size(); ++i){

            *local_feature_map += local_window_feature.at(i);
            *local_ground_map += local_window_ground.at(i);
            *local_plane_map += local_window_plane.at(i);

            PointT position;
            position.x = local_window_state.at(i).position_.x();
            position.y = local_window_state.at(i).position_.y();
            position.z = local_window_state.at(i).position_.z();
            position.intensity = double(i)/double(local_window_state.size());

            local_trajctory_->push_back(position);

//            std::cout << local_window_state.at(i).linear_velocity_.transpose() << "/ ";
            visualization_msgs::Marker arrow_model;
            arrow_model.type = visualization_msgs::Marker::ARROW;
            arrow_model.action = visualization_msgs::Marker::ADD;
            arrow_model.header.frame_id = "map";
            arrow_model.header.stamp = ros::Time::now();

            arrow_model.color.a = 1.0;
            arrow_model.color.r = 1.0;

            arrow_model.scale.x = 0.5;
            arrow_model.scale.y = 0.8;

            arrow_model.id = i;

            geometry_msgs::Point point_msg_1;
            point_msg_1.x = position.x;
            point_msg_1.y = position.y;
            point_msg_1.z = position.z;

            Eigen::Vector3d velocity_dir = local_window_state.at(i).linear_velocity_.normalized();
            geometry_msgs::Point point_msg_2;
            point_msg_2.x = (KEYFRAME_DISTANCE)*velocity_dir.x()+position.x;
            point_msg_2.y = (KEYFRAME_DISTANCE)*velocity_dir.y()+position.y;
            point_msg_2.z = (KEYFRAME_DISTANCE)*velocity_dir.z()+position.z;

            arrow_model.points.push_back(point_msg_1);
            arrow_model.points.push_back(point_msg_2);

            velocity_msg_.markers.push_back(arrow_model);
        }
//        std::cout << std::endl;

        local_feature_cloud_map_DS_->clear();
        downsize_filter_.setInputCloud(local_feature_map);
        downsize_filter_.filter(*local_feature_cloud_map_DS_);

        local_ground_cloud_map_DS_->clear();
        downsize_filter_.setInputCloud(local_ground_map);
        downsize_filter_.filter(*local_ground_cloud_map_DS_);

        local_plane_cloud_map_DS_->clear();
        downsize_filter_.setInputCloud(local_plane_map);
        downsize_filter_.filter(*local_plane_cloud_map_DS_);

    }

    void Visualization::UpdateLocalMap(gp_lio::PointCloudT local_map) {

        local_map_DS_->clear();
        downsize_filter_.setInputCloud(local_map.makeShared());
        downsize_filter_.filter(*local_map_DS_);

    }

    void Visualization::AddMarginOldCloud(State& state, PointCloudT& feature_cloud, PointCloudT& ground_cloud, PointCloudT& plane_cloud) {
//
//        all_feature_cloud_vec_.push_back(feature_cloud);
//        all_ground_cloud_vec_.push_back(ground_cloud);
//        all_plane_cloud_vec_.push_back(plane_cloud);
//        all_state_vec.push_back(state);
//
//        downsize_filter_.setInputCloud(feature_cloud.makeShared());
//        PointCloudT::Ptr feature_cloud_DS(new PointCloudT());
//        downsize_filter_.filter(*feature_cloud_DS);
//        *all_feature_cloud_map_ += *feature_cloud_DS;
//
//        downsize_filter_.setInputCloud(ground_cloud.makeShared());
//        PointCloudT::Ptr ground_cloud_DS(new PointCloudT());
//        downsize_filter_.filter(*ground_cloud_DS);
//        *all_ground_cloud_map_ += *ground_cloud_DS;
//
//        downsize_filter_.setInputCloud(plane_cloud.makeShared());
//        PointCloudT::Ptr plane_cloud_DS(new PointCloudT());
//        downsize_filter_.filter(*plane_cloud_DS);
//        *all_plane_cloud_map_ += *plane_cloud_DS;
//
//        PointT position;
//        position.x = state.position_.x();
//        position.y = state.position_.y();
//        position.z = state.position_.z();
//
//        all_trajectory_->push_back(position);
    }

    void Visualization::Publish() {

        sensor_msgs::PointCloud2 local_feature_cloud_msg, local_ground_cloud_msg, local_plane_cloud_msg, local_traj_msg, local_map_cloud_msg;
//        sensor_msgs::PointCloud2 all_feature_cloud_msg, all_ground_cloud_msg, all_plane_cloud_msg, all_traj_msg;

        pcl::toROSMsg(*local_feature_cloud_map_DS_, local_feature_cloud_msg);
        pcl::toROSMsg(*local_ground_cloud_map_DS_, local_ground_cloud_msg);
        pcl::toROSMsg(*local_plane_cloud_map_DS_, local_plane_cloud_msg);
        pcl::toROSMsg(*local_trajctory_, local_traj_msg);
        pcl::toROSMsg(*local_map_DS_, local_map_cloud_msg);
//        pcl::toROSMsg(*all_feature_cloud_map_, all_feature_cloud_msg);
//        pcl::toROSMsg(*all_ground_cloud_map_, all_ground_cloud_msg);
//        pcl::toROSMsg(*all_plane_cloud_map_, all_plane_cloud_msg);
//        pcl::toROSMsg(*all_trajectory_, all_traj_msg);

        local_feature_cloud_msg.header.frame_id = "map";
        local_ground_cloud_msg.header.frame_id = "map";
        local_plane_cloud_msg.header.frame_id = "map";
        local_traj_msg.header.frame_id = "map";
        local_map_cloud_msg.header.frame_id = "map";
//        all_feature_cloud_msg.header.frame_id = "map";
//        all_ground_cloud_msg.header.frame_id = "map";
//        all_plane_cloud_msg.header.frame_id = "map";
//        all_traj_msg.header.frame_id = "map";

        local_feature_cloud_pub_.publish(local_feature_cloud_msg);
        local_ground_cloud_pub_.publish(local_ground_cloud_msg);
        local_plane_cloud_pub_.publish(local_plane_cloud_msg);
        local_traj_pub_.publish(local_traj_msg);
        local_map_pub_.publish(local_map_cloud_msg);
//        all_feature_cloud_pub_.publish(all_feature_cloud_msg);
//        all_ground_cloud_pub_.publish(all_ground_cloud_msg);
//        all_plane_cloud_pub_.publish(all_plane_cloud_msg);
//        all_traj_pub_.publish(all_traj_msg);
        cors_pub_.publish(cors_msg_);
        velocity_pub_.publish(velocity_msg_);

    }

    void Visualization::ShowCorrespondence(std::vector<std::pair<FeatureCorPairsGroup, State>> cors, ExternalParameters extrinsic_parameters) {

        cors_msg_.markers.resize(0);

        int count=0;

        std::ofstream fout;
        if (SAVE_FEATURE_TMP_RESULT){
            std::string correspendence_save_path = OUTPUT_PATH + "/cor_refine/correspondence_pair.txt";
            fout.open(correspendence_save_path, std::ios::out);
        }
        i_++;

        Eigen::Matrix3d rot_ex = extrinsic_parameters.quaternion_.toRotationMatrix();
        Eigen::Vector3d t_ex = extrinsic_parameters.position_;

        for (int i = 1; i < cors.size(); ++i) {

            visualization_msgs::Marker line_model;
            line_model.type = visualization_msgs::Marker::LINE_LIST;
            line_model.action = visualization_msgs::Marker::ADD;
            line_model.header.frame_id = "map";
            line_model.header.stamp = ros::Time::now();

            line_model.scale.x = 0.01;

            line_model.color.a = 1.0;
            line_model.color.g = double(i)/double(cors.size());

            line_model.id = i;

            for (int j = 0; j < cors.at(i).first.size(); ++j) {

                State state = cors.at(i).second;
                Eigen::Matrix3d rot = state.quaternion_.toRotationMatrix();
                Eigen::Vector3d t = state.position_;

                FeatureCorPairs fea = cors.at(i).first.at(j);

                Eigen::Vector3d point({fea.feature_.x, fea.feature_.y, fea.feature_.z});
                Eigen::Vector3d point_trans = rot * rot_ex * point + rot * t_ex + t;

                geometry_msgs::Point point_msg_1;
                point_msg_1.x = point_trans.x();
                point_msg_1.y = point_trans.y();
                point_msg_1.z = point_trans.z();

                geometry_msgs::Point point_msg_2;

//                point_msg_2.x = fea.corresponding_center_.x;
//                point_msg_2.y = fea.corresponding_center_.y;
//                point_msg_2.z = fea.corresponding_center_.z;
                double sum_x=0, sum_y=0, sum_z=0;
                for (int k = 0; k < fea.corresponding_points_.size(); ++k) {
                    sum_x += fea.corresponding_points_.at(k).x;
                    sum_y += fea.corresponding_points_.at(k).y;
                    sum_z += fea.corresponding_points_.at(k).z;
                }
                Eigen::Vector3d point_2;
                point_2.x() = sum_x/double(fea.corresponding_points_.size());
                point_2.y() = sum_y/double(fea.corresponding_points_.size());
                point_2.z() = sum_z/double(fea.corresponding_points_.size());

                if(fea.feature_type_ == FeatureType::Ground){
                    State state_2 = cors.at(i-1).second;
                    Eigen::Matrix3d rot_2 = state_2.quaternion_.toRotationMatrix();
                    Eigen::Vector3d t_2 = state_2.position_;

                    Eigen::Vector3d point2_trans = rot_2 * rot_ex * point_2 + rot_2 * t_ex + t_2;;

                    point_msg_2.x = point2_trans.x();
                    point_msg_2.y = point2_trans.y();
                    point_msg_2.z = point2_trans.z();

                } else{

                    point_msg_2.x = point_2.x();
                    point_msg_2.y = point_2.y();
                    point_msg_2.z = point_2.z();

                }

                line_model.points.push_back(point_msg_1);
                line_model.points.push_back(point_msg_2);
                count ++;

                if(SAVE_FEATURE_TMP_RESULT){
                    if (fea.feature_type_ == FeatureType::Surf){
                        fout << "0 "
                             <<  point_msg_1.x << " "
                             <<  point_msg_1.y << " "
                             <<  point_msg_1.z << " "
                             <<  point_msg_2.x << " "
                             <<  point_msg_2.y << " "
                             <<  point_msg_2.z << " "
                             << fea.dir_vec_.x() << " "
                             << fea.dir_vec_.y() << " "
                             << fea.dir_vec_.z() << " "<< std::endl;
                    }else{
                        fout << "1 "
                             <<  point_msg_1.x << " "
                             <<  point_msg_1.y << " "
                             <<  point_msg_1.z << " "
                             <<  point_msg_2.x << " "
                             <<  point_msg_2.y << " "
                             <<  point_msg_2.z << " "
                             << fea.dir_vec_.x() << " "
                             << fea.dir_vec_.y() << " "
                             << fea.dir_vec_.z() << " "<< std::endl;
                    }
                }

            }
            cors_msg_.markers.push_back(line_model);
        }
        fout.close();
//        ROS_DEBUG("correspondence %d pairs, show %d group", count, cors_msg_.markers.size());
    }


    void Visualization::PublishRelocalization(std::vector<State> &local_window_states) {
        local_trajctory_->clear();
        for (size_t i = 0; i < local_window_states.size(); ++i) {
            PointT position;
            position.x = local_window_states.at(i).position_.x();
            position.y = local_window_states.at(i).position_.y();
            position.z = local_window_states.at(i).position_.z();
            position.intensity = double(i) / double(local_window_states.size());
            local_trajctory_->push_back(position);
            sensor_msgs::PointCloud2 local_traj_msg;
            pcl::toROSMsg(*local_trajctory_, local_traj_msg);
            local_traj_pub_.publish(local_traj_msg);
        }

    }

}