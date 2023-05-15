//
// Created by wchen on 2019/12/5.
//

#include <pcl/point_cloud.h>
#include "gp_lio/estimator/Estimator.h"

namespace gp_lio
{

    Estimator::Estimator()
    {

        first_imu_ = true;
        frame_count_ = 0;
        first_pc_ = true;
        is_turning_keep_on = 0;

        imu_propagation_times_ = 0;

        solver_flag_ = SolverFlag::INITIAL;

        is_moving_ = false;
        moving_kf_count_ = 0;
        gravity_.setZero();

        ClearVariables();

        global_mapper_thread_ = std::thread(&GlobalMapper::Run, &this->globalMapper_);
       // global_mapper_pub_thread_ = std::thread(&GlobalMapper::Pub, &this->globalMapper_);

//        std::cout << "time consumption for feature extraction | data association |corresponding pairs | ceres solver | ceres jacobian evaluation | odometry + sliding window | processing a group data" << std::endl;

    }

    Estimator::~Estimator()
    {
    }

    void Estimator::LoadConfig(ros::NodeHandle &nh)
    {
        parameters_.ReadParameters(nh);
        localMapper_.extrinsic_parameters_.quaternion_ = QIL;
        localMapper_.extrinsic_parameters_.position_ = TIL;
        visualizer_.LoadConfig(nh);
        globalMapper_.LoadConfig(nh);

    }

    void Estimator::ClearVariables()
    {

        for (int i = 0; i < WINDOW_SIZE + 1; ++i)
        {

//        if (imu_preintegration_array_[i] != nullptr)
//            delete imu_preintegration_array_[i];
            imu_preintegration_array_[i] = nullptr;
            dt_buf_[i].clear();
            linear_acceleration_buf_[i].clear();
            angular_velocity_buf_[i].clear();
        }

        localMapper_.Clear();
        localMapper_.extrinsic_parameters_.clear();

//        if (last_marginalization_info_ != nullptr)
//            delete last_marginalization_info_;

        last_marginalization_info_ = nullptr;
        last_marginalization_parameter_blocks_.clear();
    }

    void Estimator::ProcessMeasurements(MeasurementsGroupType &measurements_group)
    {

        ROS_DEBUG("processing measurements");

        temp_states_high_freq_.clear();
        temp_states_high_freq_.emplace_back(last_state_);

        for (auto &measurement : measurements_group)
        {

            auto cloud_msg = measurement.second;

            double current_time = -1.0;

            // process imu firstly
            ImuMeasurement imuMeasurement;
            ROS_DEBUG("imu measurement size: %d", measurement.first.size());

            if (solver_flag_ == SolverFlag::NON_LINEAR)
            {

                bool is_moving_1 = static_initializer_.MovingDetector(measurement.first);
                bool is_moving_2 = Eigen::Vector2d(last_state_.linear_velocity_.x(), last_state_.linear_velocity_.y()).norm()> MOVING_CHECK_THRE; // 0.5 --xc
                is_moving_ = is_moving_1 || is_moving_2;
                if (is_moving_)
                    ROS_DEBUG("Robot is moving!!!");
                else
                    ROS_DEBUG("Robot is not moving!!!");
            }

            for (auto &imu_msg : measurement.first)
            {

                double t = imu_msg->header.stamp.toSec();
                double cloud_t = cloud_msg->header.stamp.toSec();
                if (t <= cloud_t)
                {
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;

                    Eigen::Vector3d acc(imu_msg->linear_acceleration.x,
                                        imu_msg->linear_acceleration.y,
                                        imu_msg->linear_acceleration.z);
                    Eigen::Vector3d gyr(imu_msg->angular_velocity.x,
                                        imu_msg->angular_velocity.y,
                                        imu_msg->angular_velocity.z);
                    imuMeasurement.setIMU(t, dt, acc, gyr);
                    ProcessImuMeasurement(imuMeasurement);
                }
                else
                {
                    //imu interpolation
                    double dt_1 = cloud_t - current_time;
                    double dt_2 = t - cloud_t;
                    current_time = cloud_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);

                    imuMeasurement.at_ = cloud_t;
                    imuMeasurement.dt_ = dt_1;
                    imuMeasurement.ddx_ = w1 * imuMeasurement.ddx_ + w2 * imu_msg->linear_acceleration.x;
                    imuMeasurement.ddy_ = w1 * imuMeasurement.ddy_ + w2 * imu_msg->linear_acceleration.y;
                    imuMeasurement.ddz_ = w1 * imuMeasurement.ddz_ + w2 * imu_msg->linear_acceleration.z;
                    imuMeasurement.drx_ = w1 * imuMeasurement.drx_ + w2 * imu_msg->angular_velocity.x;
                    imuMeasurement.dry_ = w1 * imuMeasurement.dry_ + w2 * imu_msg->angular_velocity.y;
                    imuMeasurement.drz_ = w1 * imuMeasurement.drz_ + w2 * imu_msg->angular_velocity.z;
                    ProcessImuMeasurement(imuMeasurement);
                }
            }

            UpdateFromPoseGraph();
            // then process cloud data
            CloudMeasurement cloud_measurement(cloud_msg);
            TicToc cloud_process_time;
            if (first_pc_)
            {
                first_pc_ = false;
                cloud_last = cloud_measurement.cloud_;
                pose_t_last = temp_states_high_freq_.back().getTransformation();
                // pose_t_last = getPose(cloud_measurement);
                return;
            }


            // Eigen::Matrix4d pose_t_now = getPose(cloud_measurement);
            Eigen::Matrix4d pose_t_now = temp_states_high_freq_.back().getTransformation();
            Eigen::Matrix4d transform_matrix = pose_t_last.inverse() * pose_t_now; //last time frame relative to the new frame (B_t_end)(B_t_i)T

            // ROS_ERROR_STREAM("transform_matrix: " << transform_matrix);

            // transform_matrix = relative_pose * 
            // pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_last(new pcl::PointCloud<pcl::PointXYZ>());
            ProcessCloudMeasurement(cloud_measurement, cloud_last, transform_matrix);
            cloud_last = lidar_feature_extractor_.last_cloud_;
            lidar_feature_extractor_.last_cloud_.clear();
            // cloud_last = cloud_measurement.cloud_;
            pose_t_last = pose_t_now;

//            ROS_ERROR("process cloud data with timestamp %f , costs %f ms", cloud_msg->header.stamp.toSec(), cloud_process_time.toc());
//            std::cout << cloud_process_time.toc() << " ";

//            std::cout << std::fixed << std::setprecision(6) << localMapper_.frame_in_window_[frame_count_].state_.timestamp_ << " ";
//            std::cout << localMapper_.frame_in_window_[frame_count_].state_.position_.transpose() << " "
//                      << localMapper_.frame_in_window_[frame_count_].state_.quaternion_.x() << " "
//                      << localMapper_.frame_in_window_[frame_count_].state_.quaternion_.y() << " "
//                      << localMapper_.frame_in_window_[frame_count_].state_.quaternion_.z() << " "
//                      << localMapper_.frame_in_window_[frame_count_].state_.quaternion_.w() << std::endl;


            Publish();
            std::cout  << cloud_process_time.toc()<<std::endl;
        }


    }

    State Estimator::GetUpdatedState()
    {
        //TODO
        return last_state_;
    }


    void Estimator::LoadPosegraph() {
//        kfs_mutex_.lock();
        globalMapper_.LoadPosegraph();
//        kfs_mutex_.unlock();

    }

    void Estimator::SavePosegraph() {
        kfs_mutex_.lock();
        globalMapper_.SavePosegraph();
        kfs_mutex_.unlock();

    }


    void Estimator::ProcessImuMeasurement(ImuMeasurement &imu_measurement)
    {

        if (first_imu_)
        {

            first_imu_ = false;
            last_acc_ = imu_measurement.getAcc();
            last_gyr_ = imu_measurement.getGyr();
            imu_propagation_times_ = 0;
        }

        if (!imu_preintegration_array_[frame_count_])
        {
            imu_preintegration_array_[frame_count_] = new ImuPreintegration(last_acc_, last_gyr_,
                                                                            localMapper_.frame_in_window_[frame_count_].state_.bias_a_,
                                                                            localMapper_.frame_in_window_[frame_count_].state_.bias_g_);
        }
        if (frame_count_ != 0)
        {

            imu_preintegration_array_[frame_count_]->PushBack(imu_measurement);

            dt_buf_[frame_count_].push_back(imu_measurement.dt_);
            linear_acceleration_buf_[frame_count_].push_back(imu_measurement.getAcc());
            angular_velocity_buf_[frame_count_].push_back(imu_measurement.getGyr());

            if (!(solver_flag_ == SolverFlag::INITIAL)) // imu propgation has big error if robot is not moving.
            {

                if (is_moving_)
                {
                    //mid-point integration in local frame
                    double dt = imu_measurement.dt_;
                    Eigen::Vector3d acc = imu_measurement.getAcc();
                    Eigen::Vector3d gyr = imu_measurement.getGyr();

                    Eigen::Vector3d mid_angular_velocity = 0.5 * (last_gyr_ + gyr) - localMapper_.frame_in_window_[frame_count_].state_.bias_g_;
                    Eigen::Quaterniond updated_q = localMapper_.frame_in_window_[frame_count_].state_.quaternion_ * Utility::deltaQ(mid_angular_velocity * dt);

                    Eigen::Vector3d last_acc_in_world = localMapper_.frame_in_window_[frame_count_].state_.quaternion_ * (last_acc_ - localMapper_.frame_in_window_[frame_count_].state_.bias_a_) - gravity_;
                    Eigen::Vector3d linear_acceleration_in_world = updated_q * (acc - localMapper_.frame_in_window_[frame_count_].state_.bias_a_) - gravity_;
                    Eigen::Vector3d mid_linear_acceleration = 0.5 * (last_acc_in_world + linear_acceleration_in_world);

                    // After applying mid-point integration on all imu measurements,
                    // the imu integration between two cloud measurements is done.
                    // Thus, state_in_window.at(frame_count_) stores the transformation
                    // relative to the last cloud measurements, which can be use to
                    // compensate the motion distortion of the cloud.
                    localMapper_.frame_in_window_[frame_count_].state_.timestamp_ = imu_measurement.at_;
                    localMapper_.frame_in_window_[frame_count_].state_.quaternion_ = updated_q;
                    localMapper_.frame_in_window_[frame_count_].state_.position_ += dt * localMapper_.frame_in_window_[frame_count_].state_.linear_velocity_ + 0.5 * dt * dt * mid_linear_acceleration;
                    localMapper_.frame_in_window_[frame_count_].state_.linear_velocity_ += dt * mid_linear_acceleration;
                    imu_propagation_times_++;
//                localMapper_.frame_in_window_[frame_count_].state_.angular_velocity_ = (1.0 / double(imu_propagation_times_)) * (localMapper_.frame_in_window_[frame_count_].state_.angular_velocity_ * (imu_propagation_times_ - 1) + (gyr - localMapper_.frame_in_window_[frame_count_].state_.bias_g_));
                    localMapper_.frame_in_window_[frame_count_].state_.angular_velocity_ = mid_angular_velocity;
                    // for DistortionPreCompensation
                    temp_states_high_freq_.emplace_back(localMapper_.frame_in_window_[frame_count_].state_);
                }
                else
                {
                    // if is static, only keep timestamp propgating
                    localMapper_.frame_in_window_[frame_count_].state_.timestamp_ = imu_measurement.at_;
                }
            }
        }
        last_acc_ = imu_measurement.getAcc();
        last_gyr_ = imu_measurement.getGyr();
        last_timestamp_ = imu_measurement.at_;
    }

    void Estimator::ProcessCloudMeasurement(CloudMeasurement &cloud_measurement, PointCloudT &cloud_last, Eigen::Matrix4d &transform_matrix)
    {

        ROS_DEBUG("************ new cloud data ***********");
        ROS_DEBUG("frame_count: %d", frame_count_);

        localMapper_.frame_in_window_[frame_count_].state_.timestamp_ = cloud_measurement.receive_time_;

        // Motion Distortion pre-compensation (need IMU data group and cloud_measurement) and Extract Features
        TicToc cloud_process;

        CloudFeature lidar_feature;

        // ROS_ERROR_STREAM("transform_matrix: " << transform_matrix);

        lidar_feature = lidar_feature_extractor_.PutinCloud(cloud_measurement, cloud_last, transform_matrix);
        ROS_DEBUG("Feature extraction costs: %fms", cloud_process.toc());
//        std::cout << cloud_process.toc() << " ";
        ROS_DEBUG_STREAM("size: " << lidar_feature.less_edge_feature_.size() << " "
                                  << lidar_feature.less_surf_feature_.size() << " "
                                  << lidar_feature.ground_feature_.size());
//    if (is_moving_)
//    {
//        lidar_feature.save("/home/wchen/Projects/Code/catkin_ws/src/gp_lio/result/distort.pcd");
//        DistortionPreCompensation(lidar_feature); // if static, we don't need it
//        lidar_feature.save("/home/wchen/Projects/Code/catkin_ws/src/gp_lio/result/undistort.pcd");
//    }
        localMapper_.AddNewCloudFeature(lidar_feature, frame_count_);

        if (solver_flag_ == SolverFlag::INITIAL)
        {
            if (frame_count_ == WINDOW_SIZE)
            {
                if (Init())
                {

                    solver_flag_ = SolverFlag::NON_LINEAR;
                    ROS_ERROR("Static initialization succeeds!");

                    // Set all the states in the window to be initial state
                    for (size_t i_count = 0; i_count <= frame_count_; i_count++)
                    {
                        localMapper_.frame_in_window_[i_count].state_.position_ = initial_state_.position_;
                        localMapper_.frame_in_window_[i_count].state_.quaternion_ = initial_state_.quaternion_;
                        localMapper_.frame_in_window_[i_count].state_.linear_velocity_ = initial_state_.linear_velocity_;
                        localMapper_.frame_in_window_[i_count].state_.bias_a_ = initial_state_.bias_a_;
                        localMapper_.frame_in_window_[i_count].state_.bias_g_ = initial_state_.bias_g_;
                    }

                    last_state_ = localMapper_.frame_in_window_[frame_count_].state_;

                    marginalization_flag_ = MarginalizationFlag::MARGIN_OLD;

                    SlideWindow();
                }
                else
                {
                    ROS_ERROR("Static initialization failed! Please stay still!!!!!!");
                }
            }
            else
            {

                localMapper_.frame_in_window_[frame_count_].state_.timestamp_ = cloud_measurement.receive_time_;

                if (frame_count_ == 0)
                {
                    frame_count_++;
                }
                else
                {
                    double time_diff = localMapper_.frame_in_window_[frame_count_].state_.timestamp_ - localMapper_.frame_in_window_[frame_count_ - 1].state_.timestamp_;
                    if (time_diff >= KEYFRAME_TIME_DIFF)
                        frame_count_++;
                }
            }
        }
        else
        {
            ROS_DEBUG("NON_LINEAR mode");
            // check marginalization type
            marginalization_flag_ = localMapper_.CheckMarginalizationType(frame_count_, is_moving_);
            ROS_DEBUG("this frame is--------------------%s", marginalization_flag_ ? "reject" : "accept");
            ROS_DEBUG("%s", marginalization_flag_ ? "Non-keyframe" : "Keyframe");
            ROS_DEBUG("Solving %d", frame_count_);

            if (is_moving_ && marginalization_flag_ == MarginalizationFlag::MARGIN_OLD)
            {
                moving_kf_count_++;
            }

            // solve Odometry and marginalization OLD or SECOND NEW
            ROS_DEBUG("moving kf count: %d", moving_kf_count_);
            TicToc t_solve;
            if (is_moving_ && moving_kf_count_ >= 1)
            {
                TicToc data_ass_time;
//                localMapper_.DataAssociationAdjacent();
                localMapper_.PlaneAssociationAdjacent();
                localMapper_.DataAssociationAdjacentGround();
                localMapper_.DataAssociation();
                ROS_DEBUG_STREAM("data association cost" << data_ass_time.toc() << "ms");
//                std::cout << " " << data_ass_time.toc() << " ";
                SolveOdometry();
            }
            SlideWindow();
//            std::cout << t_solve.toc() << " ";
            ROS_DEBUG("(odometry + sliding window) costs: %fms", t_solve.toc());
        }
    }

    Eigen::Matrix4d Estimator::getPose(CloudMeasurement &cloud_measurement)
    {

        Eigen::Matrix4d inter_pose;
        inter_pose.setIdentity(); //(B_t_end)(B_t_point_time)T

        // Eigen::Matrix4d pose_t_end = temp_states_high_freq_.back().getTransformation();

        // for (int i = 0; i < temp_states_high_freq_.size(); ++i)
        // {
        //     // current time frame relative to the end-time frame (B_t_end)(B_t_i)T
        //     auto pose_t_i = temp_states_high_freq_[i].getTransformation();
        //     auto relative_pose = pose_t_end.inverse() * pose_t_i;
        //     temp_states_high_freq_[i].resetPose(relative_pose);
        // }

        if (cloud_measurement.cloud_.points.empty())
            return inter_pose;

        // compensation
        // (L_t_end)p = (L_t_end)(L_t_i)T * (L_t_i)p
        // where (L_t_end)(L_t_i)T = (B)(L)T^(-1) * (B_t_end)(B_t_i)T * (B)(L)T;
        Eigen::Matrix4d B_L_T = localMapper_.extrinsic_parameters_.getTransform();
        Eigen::Matrix4d B_L_T_inv = B_L_T.inverse();

        // ROS_DEBUG_STREAM(" " << cloud_time - temp_states_high_freq_.front().timestamp_ << ", "
        //                      << cloud_time - temp_states_high_freq_.back().timestamp_);
        if (cloud_measurement.receive_time_ <= temp_states_high_freq_.front().timestamp_ || cloud_measurement.receive_time_ >= temp_states_high_freq_.back().timestamp_)
        {
//            ROS_ERROR("timestamp (cloud point not in the imu sequence) in compensation!");
            // continue;
            ROS_ERROR_STREAM("cloud_measurement.receive_time_: " << cloud_measurement.receive_time_);
            ROS_ERROR_STREAM("temp_states_high_freq_.front().timestamp_: " << temp_states_high_freq_.front().timestamp_);
            ROS_ERROR_STREAM("temp_states_high_freq_.back().timestamp_: " << temp_states_high_freq_.back().timestamp_);
            ROS_ERROR("timestamp (cloud point not in the imu sequence) in compensation!");
            double time1 = cloud_measurement.receive_time_ - temp_states_high_freq_.front().timestamp_;
            double time2 = cloud_measurement.receive_time_ - temp_states_high_freq_.back().timestamp_;
            ROS_ERROR_STREAM("time1: " << time1);
            ROS_ERROR_STREAM("time2: " << time2);
            return inter_pose;
        }
        size_t i_imu = 1;
        for (; i_imu < temp_states_high_freq_.size(); i_imu++)
        {
            if (temp_states_high_freq_[i_imu - 1].timestamp_ <= cloud_measurement.receive_time_ && temp_states_high_freq_[i_imu].timestamp_ >= cloud_measurement.receive_time_)
                break;
        }
        auto pose_t_i_imu_1 = temp_states_high_freq_[i_imu - 1].getTransformation();
        Eigen::Matrix4d pose_t_i_imu = temp_states_high_freq_[i_imu].getTransformation();
        auto rel_pos_state =  pose_t_i_imu_1.inverse() * pose_t_i_imu;
        Eigen::Matrix3d rot = rel_pos_state.block(0, 0, 3, 3);
        Eigen::Quaterniond rel_pos_qua(rot);

        State pre_state = temp_states_high_freq_.at(i_imu - 1);
        State pos_state = temp_states_high_freq_.at(i_imu);

        // interporlation: wiki "slerp"
        float inter_ratio = (cloud_measurement.receive_time_ - pre_state.timestamp_) / (pos_state.timestamp_ - pre_state.timestamp_);
        Eigen::Vector3d inter_position = (1 - inter_ratio) * pre_state.position_ + inter_ratio * pos_state.position_;

        // State rel_pos_state = temp_states_high_freq_.at(i_imu);
        // auto pose_t_i = temp_states_high_freq_[i].getTransformation();
        // auto relative_pose = pose_t_end.inverse() * pose_t_i;
        // rel_pos_state.resetPose(relative_pose);
        // quaternion_ = qua;


        Eigen::Quaterniond inter_qua = pre_state.quaternion_.slerp(inter_ratio, rel_pos_qua);

        inter_pose.block<3, 3>(0, 0) = inter_qua.toRotationMatrix();
        inter_pose.block<3, 1>(0, 3) = inter_position;
        Eigen::Matrix4d inter_pose_L = B_L_T_inv * inter_pose * B_L_T;

        return inter_pose_L;


    }

    void Estimator::DistortionPreCompensation(gp_lio::CloudFeature &lidar_feature)
    {

        Eigen::Matrix4d pose_t_end = temp_states_high_freq_.back().getTransformation();

        for (int i = 0; i < temp_states_high_freq_.size(); ++i)
        {
            // current time frame relative to the end-time frame (B_t_end)(B_t_i)T
            auto pose_t_i = temp_states_high_freq_[i].getTransformation();
            auto relative_pose = pose_t_end.inverse() * pose_t_i;
            temp_states_high_freq_[i].resetPose(relative_pose);
        }

        FeaturesTransform(lidar_feature.edge_feature_, lidar_feature.cloudTimeStamp_);
        FeaturesTransform(lidar_feature.surf_feature_, lidar_feature.cloudTimeStamp_);
        FeaturesTransform(lidar_feature.less_edge_feature_, lidar_feature.cloudTimeStamp_);
        FeaturesTransform(lidar_feature.less_surf_feature_, lidar_feature.cloudTimeStamp_);
//    FeaturesTransform(lidar_feature.ground_feature_, lidar_feature.cloudTimeStamp_);
    }

    void Estimator::FeaturesTransform(gp_lio::PointCloudT &feature, double &cloud_time)
    {

        if (feature.points.empty())
            return;
        // compensation
        // (L_t_end)p = (L_t_end)(L_t_i)T * (L_t_i)p
        // where (L_t_end)(L_t_i)T = (B)(L)T^(-1) * (B_t_end)(B_t_i)T * (B)(L)T;
        Eigen::Matrix4d B_L_T = localMapper_.extrinsic_parameters_.getTransform();
        Eigen::Matrix4d B_L_T_inv = B_L_T.inverse();

        // ROS_DEBUG_STREAM(" " << cloud_time - temp_states_high_freq_.front().timestamp_ << ", "
        //                      << cloud_time - temp_states_high_freq_.back().timestamp_);

        for (int i_point = 0; i_point < feature.points.size(); ++i_point)
        {
            PointT point = feature.points[i_point];
            float relTime = point.intensity - int(point.intensity);
            double point_time = (cloud_time - (SCAN_DURATION - relTime));

            // ROS_DEBUG_STREAM("point intensity: " << point.intensity << ", time " << cloud_time - point_time);

            if (point_time <= temp_states_high_freq_.front().timestamp_ || point_time >= temp_states_high_freq_.back().timestamp_)
            {
//            ROS_ERROR("timestamp (cloud point not in the imu sequence) in compensation!");
                continue;
            }

            //find pre_state...point_time...pos_state

            size_t i_imu = 1;
            for (; i_imu < temp_states_high_freq_.size(); i_imu++)
            {
                if (temp_states_high_freq_[i_imu - 1].timestamp_ <= point_time && temp_states_high_freq_[i_imu].timestamp_ >= point_time)
                    break;
            }
            State pre_state = temp_states_high_freq_.at(i_imu - 1);

            State pos_state = temp_states_high_freq_.at(i_imu);

            // interporlation: wiki "slerp"
            float inter_ratio = (point_time - pre_state.timestamp_) / (pos_state.timestamp_ - pre_state.timestamp_);
            Eigen::Vector3d inter_position = (1 - inter_ratio) * pre_state.position_ + inter_ratio * pos_state.position_;
            Eigen::Quaterniond inter_qua = pre_state.quaternion_.slerp(inter_ratio, pos_state.quaternion_);
            Eigen::Matrix4d inter_pose;
            inter_pose.setIdentity(); //(B_t_end)(B_t_point_time)T
            inter_pose.block<3, 3>(0, 0) = inter_qua.toRotationMatrix();
            inter_pose.block<3, 1>(0, 3) = inter_position;
            Eigen::Matrix4d inter_pose_L = B_L_T_inv * inter_pose * B_L_T;

            Eigen::Matrix3d inter_pose_L_rot = inter_pose_L.block(0, 0, 3, 3);
            Eigen::Vector3d inter_pose_L_p = inter_pose_L.block(0, 3, 3, 1);
            Eigen::Vector3d point_position(point.x, point.y, point.z);
            Eigen::Vector3d point_transformed = inter_pose_L_rot * point_position + inter_pose_L_p;
            feature.points[i_point].x = point_transformed.x();
            feature.points[i_point].y = point_transformed.y();
            feature.points[i_point].z = point_transformed.z();
        }
    }

    bool Estimator::Init()
    {
        if (frame_count_ != WINDOW_SIZE)
        {
            return false;
        }

        ROS_DEBUG("************ Initialization ***********");

        TicToc t_ini;
        // prepare data
        std::vector<ImuMeasurement> imu_mea_v;
        // size_t imu_buffer_size = imu_preintegration_array_[frame_count_]->GetIMUBuffer().size();
        // imu_mea_v.reserve(imu_buffer_size * (WINDOW_SIZE + 1));

        for (size_t i_imu_array = 0; i_imu_array <= WINDOW_SIZE; i_imu_array++)
        {
            std::vector<ImuMeasurement> frame_count_imus = imu_preintegration_array_[i_imu_array]->GetIMUBuffer();

            imu_mea_v.insert(imu_mea_v.end(), frame_count_imus.begin(), frame_count_imus.end());
        }

        // moving threshold
        // std::vector<ImuMeasurement>::iterator end = imu_mea_v.begin() + std::floor(imu_mea_v.size() / 2);
        // std::vector<ImuMeasurement> imu_mea_sgm_1(imu_mea_v.begin(), imu_mea_v.end());
        ROS_DEBUG_STREAM("imu_mea_v size: " << imu_mea_v.size());
        static_initializer_.MovingDetector(imu_mea_v);
        ROS_ERROR_STREAM( "moving threshold " << static_initializer_.moving_threshold_);

        if (static_initializer_.moving_threshold_ >= threshold_filter_still)
            return false;

        // // check moving
        // std::vector<ImuMeasurement>::iterator begin = imu_mea_v.end() - std::floor(imu_mea_v.size() / 2);
        // std::vector<ImuMeasurement> imu_mea_sgm_2(begin, imu_mea_v.end());
        // if (static_initializer_.MovingDetector(imu_mea_sgm_2, true))
        // {
        //     ROS_ASSERT("Please stay static for initialization!!!!!");
        //     return false;
        // }

        // initialize
        static_initializer_.GetIMUData(imu_mea_v);

        initial_state_.timestamp_ = imu_mea_v.back().at_;
        initial_state_.bias_a_ = static_initializer_.bias_a_;
        initial_state_.bias_g_ = static_initializer_.bias_g_;
        ACC_NOISE_STD = static_initializer_.noise_std_a_;
        GYR_NOISE_STD = static_initializer_.noise_std_w_;

        Eigen::Matrix3d rot_0 = initial_state_.quaternion_.toRotationMatrix();
        // double yaw = Utility::R2ypr(static_initializer_.gravity_aligned_rot_ * rot_0).x();
        // Eigen::Matrix3d rot_diff = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * rot_0; // yaw is unobservable
        // gravity_ = rot_diff * static_initializer_.gravity_;
        // rot_0 = rot_diff * rot_0;
        // last_state_.quaternion_ = Eigen::Quaterniond(rot_0);
        // last_state_.position_  = rot_diff*last_state_.position_;
        // last_state_.linear_velocity_ = rot_diff*last_state_.linear_velocity_;
        // ROS_DEBUG_STREAM("rot_diff     " << Utility::R2ypr(rot_diff).transpose());

        initial_state_.quaternion_ = Eigen::Quaterniond(static_initializer_.gravity_aligned_rot_ * rot_0);
        initial_state_.position_ = static_initializer_.gravity_aligned_rot_ * initial_state_.position_;
        initial_state_.linear_velocity_ = static_initializer_.gravity_aligned_rot_ * initial_state_.linear_velocity_;
        gravity_ = static_initializer_.gravity_aligned_rot_ * static_initializer_.local_gravity_;

        ROS_DEBUG_STREAM("bias (a, g):     " << initial_state_.bias_a_.transpose() << " " << initial_state_.bias_g_.transpose());
        ROS_DEBUG_STREAM("noise std (a, g):     " << ACC_NOISE_STD << "" << GYR_NOISE_STD);
        ROS_DEBUG_STREAM("g0     " << gravity_.transpose());
        ROS_DEBUG_STREAM("p0     " << initial_state_.position_.transpose());
        ROS_DEBUG_STREAM("v0     " << initial_state_.linear_velocity_.transpose());
        ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(initial_state_.quaternion_.toRotationMatrix()).transpose());

        last_acc_ = imu_mea_v.back().getAcc();
        last_gyr_ = imu_mea_v.back().getGyr();
        last_timestamp_ = imu_mea_v.back().at_;

        ROS_DEBUG("****************initialization spends %f ms", t_ini.toc());

        return true;
    }

    void Estimator::SolveOdometry()
    {

        if (frame_count_ < WINDOW_SIZE)
            return;
        if (solver_flag_ == SolverFlag::NON_LINEAR)
        {
            Optimization();
        }
    }

    void Estimator::Optimization()
    {
//   ROS_ERROR("Before ceres");
//   for (int i = 0; i < WINDOW_SIZE + 1; i++)
//   {
//       State _state = localMapper_.frame_in_window_[i].state_;
//       ROS_DEBUG_STREAM("State: " << _state.timestamp_ - localMapper_.frame_in_window_[0].state_.timestamp_);
//       ROS_DEBUG_STREAM("  --  position: " << _state.position_.transpose());
//       ROS_DEBUG_STREAM("  --  rotation: " << Utility::R2ypr(_state.quaternion_.toRotationMatrix()).transpose());
//       ROS_DEBUG_STREAM("  --  linear velocity: " << _state.linear_velocity_.transpose());
//       ROS_DEBUG_STREAM("  --  angular velocity: " << _state.angular_velocity_.transpose());
//       ROS_DEBUG_STREAM("  --  bias gyr: " << _state.bias_g_.transpose());
//       ROS_DEBUG_STREAM("  --  bias acc: " << _state.bias_a_.transpose());
//   }
//   ROS_DEBUG_STREAM("extrinsic_parameters_: " << localMapper_.extrinsic_parameters_.getTransform());

        // 1 --- build ceres problem
        ceres::Problem problem;
        ceres::LossFunction *lossFunction;
        lossFunction = new ceres::HuberLoss(0.1);
//        lossFunction = new ceres::CauchyLoss(1.0);

        // 1.0 --- add states parameters: position+quaternion, velocity, and imu bias.
//    ROS_DEBUG_STREAM("add states parameters");
        for (int i = 0; i < WINDOW_SIZE + 1; i++)
        {
            ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
            problem.AddParameterBlock(para_pose_[i], SIZE_POSE, local_parameterization);
            problem.AddParameterBlock(para_velocity_bias_[i], SIZE_VELOCITY_BIAS);

            if(i<FIXED_SIZE){
                problem.SetParameterBlockConstant(para_pose_[i]);
                problem.SetParameterBlockConstant(para_velocity_bias_[i]);
            }
        }
        // add extrinsic parameters: position+quaternion
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_ex_pose_[0], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC)
        {
            problem.SetParameterBlockConstant(para_ex_pose_[0]);
        }

        TicToc t_whole, t_prepare;
        States2Para();

        // 1.1 --- add Marginlization Factor as prior factor
        int count = 0;
        int count_plane = 0;
        if(USE_IMU_FACTOR){

            if (last_marginalization_info_)
            {
//         ROS_DEBUG_STREAM("add Marginlization Factor as prior factor");
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info_);
                problem.AddResidualBlock(marginalization_factor, NULL, last_marginalization_parameter_blocks_);
            }

            // 1.2 --- add IMU Factor
            for (int i = 0; i < WINDOW_SIZE; ++i)
            {

                int j = i + 1;
//                if (imu_preintegration_array_[j]->sum_dt_ > 10.0)
//                    continue;
                ImuFactor *imu_factor = new ImuFactor(imu_preintegration_array_[j]);
                problem.AddResidualBlock(imu_factor, NULL,
                                         para_pose_[i], para_velocity_bias_[i],
                                         para_pose_[j], para_velocity_bias_[j]);
                count++;
            }
            ROS_DEBUG("add %d IMU Factor", count);

        }


        // 1.3 --- add lidar Feature Factor
        count = 0;
        std::vector<std::pair<FeatureCorPairsGroup, State>> cors;
        for (int j = FIXED_SIZE; j < WINDOW_SIZE + 1; j++)
        {
            // visualization
            std::pair<FeatureCorPairsGroup, State> pair_1, pair_2;
            // 1.3.1 --- add edge factor (point-to-line redisual), and use autodiff
            if (USE_EDGE_FACTOR){

                pair_1.first = localMapper_.frame_in_window_[j].edgePairsGroup_;
                pair_1.second = localMapper_.frame_in_window_[j].state_;
                cors.push_back(pair_1);

                int edgeFeaturesSize = localMapper_.frame_in_window_[j].edgePairsGroup_.size();
                ROS_DEBUG("frame %d edgeFeaturesSize: %d", j, edgeFeaturesSize);
                for (int i = 0; i < edgeFeaturesSize; ++i)
                {
                    FeatureCorPairs featureCorPairs = localMapper_.frame_in_window_[j].edgePairsGroup_.at(i);
                    if (featureCorPairs.feature_type_ != FeatureType::Edge)
                        continue;
                    double weight = EDGE_FACTOR_WEIGHT;
                    ceres::CostFunction *costFunction =
                            new ceres::AutoDiffCostFunction<FeatureCostFunctorMap, 3, 7, 7>(new FeatureCostFunctorMap(featureCorPairs, weight));
                    problem.AddResidualBlock(costFunction, lossFunction,para_pose_[j], para_ex_pose_[0]);
                    count++;
                }
            }


            // 1.3.2 --- add surf factor (point-to-plane redisual), and use autodiff
            if(USE_SURF_FACTOR){

                if(USE_AUTO_DIFF){
                    int surfFeaturesSize = localMapper_.frame_in_window_[j].surfPairsGroup_.size();
                    ROS_DEBUG("frame %d surfFeaturesSize: %d", j, surfFeaturesSize);
                    for (int i = 0; i < surfFeaturesSize; ++i)
                    {
                        FeatureCorPairs featureCorPairs = localMapper_.frame_in_window_[j].surfPairsGroup_.at(i);
                        if (featureCorPairs.feature_type_ != FeatureType::Surf)
                            continue;
                        double weight = SURF_FACTOR_WEIGHT;
                        ceres::CostFunction *costFunction =
                                new ceres::AutoDiffCostFunction<FeatureCostFunctorMap, 1, 7, 7>(
                                        new FeatureCostFunctorMap(featureCorPairs, weight));
                        problem.AddResidualBlock(costFunction, lossFunction,   para_pose_[j], para_ex_pose_[0]);
                        count++;
                    }
                }else{
                    int surfFeaturesSize = localMapper_.frame_in_window_[j].surfPairsGroup_.size();
                    ROS_DEBUG("%d surfFeaturesSize: %d", j, surfFeaturesSize);
                    for (int k = 0; k < surfFeaturesSize; ++k)
                    {
                        FeatureCorPairs featureCorPairs = localMapper_.frame_in_window_[j].surfPairsGroup_.at(k);
                        if (featureCorPairs.feature_type_ != FeatureType::Surf)
                            continue;
                        double weight = SURF_FACTOR_WEIGHT;
                        SurfFactor *surfFactor = new SurfFactor(featureCorPairs, weight);
                        problem.AddResidualBlock(surfFactor, lossFunction, para_pose_[j-1], para_pose_[j], para_ex_pose_[0]);
                        count++;
                    }
                }
            }

            // 1.3.3 --- add plane factor (plane-to-plane redisual), and use autodiff
            if(USE_PLANE_FACTOR){

                auto planeFactors = localMapper_.frame_in_window_[j].planeCorPairsGroup_;
                ROS_DEBUG("frame %d PlaneSize: %d", j, planeFactors.size());
                for (int i = 0; i < planeFactors.size(); ++i) {

                    auto plane_factor = planeFactors.at(i);

                    ceres::CostFunction *costFunction =
                            new ceres::AutoDiffCostFunction<PlaneCostFunctor, 3, 7, 7, 7>(
                                    new PlaneCostFunctor(plane_factor.plane_cp_0_, plane_factor.plane_cp_1_, PLANE_FACTOR_WEIGHT));
                    problem.AddResidualBlock(costFunction, lossFunction, para_pose_[j-1], para_pose_[j], para_ex_pose_[0]);
                    count_plane++;

//                    ROS_DEBUG_STREAM("frame " << j << " cp:  " << plane_factor.plane_cp_0_.transpose() << " - " << plane_factor.plane_cp_1_.transpose());
                }
            }

            // 1.3.4 --- add ground factor (point-to-plane redisual), and use autodiff
            if(USE_GROUND_FACTOR){

                int groundFeaturesSize = localMapper_.frame_in_window_[j].groundPairsGroup_.size();
                ROS_DEBUG("frame %d groundFeaturesSize: %d", j, groundFeaturesSize);
                for (int i = 0; i < groundFeaturesSize-1; ++i)
                {
                    FeatureCorPairs featureCorPairs = localMapper_.frame_in_window_[j].groundPairsGroup_.at(i);
                    if (featureCorPairs.feature_type_ != FeatureType::Ground)
                        continue;
                    double weight = SURF_FACTOR_WEIGHT;
                    ceres::CostFunction *costFunction =
                            new ceres::AutoDiffCostFunction<FeatureCostFunctorNew, 1, 7, 7, 7>(
                                    new FeatureCostFunctorNew(featureCorPairs, weight));
                    problem.AddResidualBlock(costFunction, lossFunction, para_pose_[j-1], para_pose_[j], para_ex_pose_[0]);
                    count++;
                }
            }

            pair_2.first = localMapper_.frame_in_window_[j].groundPairsGroup_;
            pair_2.second = localMapper_.frame_in_window_[j].state_;
            cors.push_back(pair_2);
        }
        visualizer_.ShowCorrespondence(cors, localMapper_.extrinsic_parameters_);
        ROS_DEBUG("add %d lidar Feature Factor", count);
//        std::cout << count << " " << count_plane << " ";

        ROS_DEBUG("prepare for ceres: %fms", t_prepare.toc());

        // 2 --- run the solver
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;
//        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = CERES_NUM_ITERATIONS;
//    options.num_threads = CERES_SOLVER_THREADS;
        if (marginalization_flag_ == MARGIN_OLD)
            options.max_solver_time_in_seconds = CERES_SOLVER_TIME * 4.0 / 5.0;
        else
            options.max_solver_time_in_seconds = CERES_SOLVER_TIME;

        // 3 --- get the optimized results
        TicToc t_solver;
        ROS_DEBUG("begin solver");
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        ROS_DEBUG_STREAM(summary.BriefReport() << std::endl );
        ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
        ROS_DEBUG("ceres solver costs: %f ms", t_solver.toc());

//        std::cout << t_solver.toc() << " " << summary.jacobian_evaluation_time_in_seconds*1000 << " ";

        Para2States();

//   ROS_ERROR("After ceres");
//   for (int i = 0; i < WINDOW_SIZE + 1; i++)
//   {
//       State _state = localMapper_.frame_in_window_[i].state_;
//       ROS_DEBUG_STREAM("State: " << _state.timestamp_ - localMapper_.frame_in_window_[0].state_.timestamp_);
//       ROS_DEBUG_STREAM("  --  position: " << _state.position_.transpose());
//       ROS_DEBUG_STREAM("  --  rotation: " << Utility::R2ypr(_state.quaternion_.toRotationMatrix()).transpose());
//       ROS_DEBUG_STREAM("  --  linear velocity: " << _state.linear_velocity_.transpose());
//       ROS_DEBUG_STREAM("  --  angular velocity: " << _state.angular_velocity_.transpose());
//       ROS_DEBUG_STREAM("  --  bias gyr: " << _state.bias_g_.transpose());
//       ROS_DEBUG_STREAM("  --  bias acc: " << _state.bias_a_.transpose());
//   }

        // 4 --- marginalization
        TicToc t_whole_marginalization;
        if (marginalization_flag_ == MarginalizationFlag::MARGIN_OLD)
        {
            if(USE_IMU_FACTOR) {
                MarginalizationInfo *marginalization_info = new MarginalizationInfo();
                States2Para();

                if (last_marginalization_info_) {

                    std::vector<int> drop_id_set;
                    for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks_.size()); ++i) {

                        if (last_marginalization_parameter_blocks_[i] == para_pose_[0] ||
                            last_marginalization_parameter_blocks_[i] == para_velocity_bias_[0])
                            drop_id_set.push_back(i);
                    }
                    ROS_DEBUG("margin factor");
                    MarginalizationFactor *marginalization_factor = new MarginalizationFactor(
                            last_marginalization_info_);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                                   last_marginalization_parameter_blocks_,
                                                                                   drop_id_set);

                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }

                {
                    ROS_DEBUG("imu array[1] sum_dt %f", imu_preintegration_array_[1]->sum_dt_);
//                    if (imu_preintegration_array_[1]->sum_dt_ < 20.0) {
                    ROS_DEBUG("margin imu factor");
                    ImuFactor *imu_factor = new ImuFactor(imu_preintegration_array_[1]);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                                   std::vector<double *>{
                                                                                           para_pose_[0],
                                                                                           para_velocity_bias_[0],
                                                                                           para_pose_[1],
                                                                                           para_velocity_bias_[1]},
                                                                                   std::vector<int>{0, 1});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
//                    }
                }

                {
                    // for the lidar factor, I guess we don't need it in the marginalization process. Because we set fixed
                    // TODO

//            int surfFeaturesSize = localMapper_.frame_in_window_[1].surfPairsGroup_.size();
//            ROS_DEBUG("frame %d surfFeaturesSize: %d", 1, surfFeaturesSize);
//            for (int i = 0; i < surfFeaturesSize; ++i)
//            {
//                FeatureCorPairs featureCorPairs = localMapper_.frame_in_window_[1].surfPairsGroup_.at(i);
//                if (featureCorPairs.feature_type_ != FeatureType::Surf)
//                    continue;
//                double weight = EDGE_FACTOR_WEIGHT;
//                ceres::CostFunction *costFunction =
//                        new ceres::AutoDiffCostFunction<FeatureCostFunctorNew, 1, 7, 7, 7>(
//                                new FeatureCostFunctorNew(featureCorPairs, weight));
//                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(costFunction, lossFunction, std::vector<double *>{para_pose_[0], para_pose_[1], para_ex_pose_[0]}, std::vector<int>{0, 1});
//                marginalization_info->addResidualBlockInfo(residual_block_info);
//                count++;
//            }


                }

                TicToc t_pre_margin;
                marginalization_info->preMarginalize();
                ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

                TicToc t_margin;
                marginalization_info->marginalize();
                ROS_DEBUG("marginalization %f ms", t_margin.toc());

                // pre- sliding window
                std::unordered_map<long, double *> addr_shift;
                for (int j = 1; j <= WINDOW_SIZE; ++j) {
                    addr_shift[reinterpret_cast<long>(para_pose_[j])] = para_pose_[j - 1];
                    addr_shift[reinterpret_cast<long>(para_velocity_bias_[j])] = para_velocity_bias_[j - 1];
                }
                if (ESTIMATE_EXTRINSIC >= 0) {
                    addr_shift[reinterpret_cast<long>(para_ex_pose_[0])] = para_ex_pose_[0];
                }

                std::vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

                if (last_marginalization_info_)
                    delete last_marginalization_info_;
                last_marginalization_info_ = marginalization_info;
                last_marginalization_parameter_blocks_ = parameter_blocks;
            }
        }
        else { // marginlize second new
            if (USE_IMU_FACTOR) {

                if (last_marginalization_info_ &&
                    std::count(std::begin(last_marginalization_parameter_blocks_),
                               std::end(last_marginalization_parameter_blocks_), para_pose_[WINDOW_SIZE - 1])) {
                    MarginalizationInfo *marginalization_info = new MarginalizationInfo();
                    States2Para();
                    if (last_marginalization_info_) {
                        std::vector<int> drop_set;
                        for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks_.size()); i++) {
                            ROS_ASSERT(
                                    last_marginalization_parameter_blocks_[i] != para_velocity_bias_[WINDOW_SIZE - 1]);
                            if (last_marginalization_parameter_blocks_[i] == para_pose_[WINDOW_SIZE - 1])
                                drop_set.push_back(i);
                        }
                        // construct new marginlization_factor
                        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(
                                last_marginalization_info_);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                                       last_marginalization_parameter_blocks_,
                                                                                       drop_set);

                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }

                    TicToc t_pre_margin;
                    ROS_DEBUG("begin marginalization");
                    marginalization_info->preMarginalize();
                    ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

                    TicToc t_margin;
                    ROS_DEBUG("begin marginalization");
                    marginalization_info->marginalize();
                    ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

                    std::unordered_map<long, double *> addr_shift;
                    for (int i = 0; i <= WINDOW_SIZE; i++) {
                        if (i == WINDOW_SIZE - 1)
                            continue;
                        else if (i == WINDOW_SIZE) {
                            addr_shift[reinterpret_cast<long>(para_pose_[i])] = para_pose_[i - 1];
                            addr_shift[reinterpret_cast<long>(para_velocity_bias_[i])] = para_velocity_bias_[i - 1];
                        } else {
                            addr_shift[reinterpret_cast<long>(para_pose_[i])] = para_pose_[i];
                            addr_shift[reinterpret_cast<long>(para_velocity_bias_[i])] = para_velocity_bias_[i];
                        }
                    }
                    if (ESTIMATE_EXTRINSIC >= 0) {
                        addr_shift[reinterpret_cast<long>(para_ex_pose_[0])] = para_ex_pose_[0];
                    }

                    std::vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
                    if (last_marginalization_info_)
                        delete last_marginalization_info_;
                    last_marginalization_info_ = marginalization_info;
                    last_marginalization_parameter_blocks_ = parameter_blocks;
                }
            }
            ROS_DEBUG("whole marginalization costs: %f ms", t_whole_marginalization.toc());

            ROS_DEBUG("whole time for ceres: %f ms", t_whole.toc());
        }
    }

    void Estimator::States2Para()
    {
        // plugin in the initial guess for every parameters

        for (int i = 0; i < WINDOW_SIZE + 1; ++i)
        {

            // position+quaternion
            para_pose_[i][0] = localMapper_.frame_in_window_[i].state_.position_.x();
            para_pose_[i][1] = localMapper_.frame_in_window_[i].state_.position_.y();
            para_pose_[i][2] = localMapper_.frame_in_window_[i].state_.position_.z();
            para_pose_[i][3] = localMapper_.frame_in_window_[i].state_.quaternion_.x();
            para_pose_[i][4] = localMapper_.frame_in_window_[i].state_.quaternion_.y();
            para_pose_[i][5] = localMapper_.frame_in_window_[i].state_.quaternion_.z();
            para_pose_[i][6] = localMapper_.frame_in_window_[i].state_.quaternion_.w();
            // velocity
            para_velocity_bias_[i][0] = localMapper_.frame_in_window_[i].state_.linear_velocity_.x();
            para_velocity_bias_[i][1] = localMapper_.frame_in_window_[i].state_.linear_velocity_.y();
            para_velocity_bias_[i][2] = localMapper_.frame_in_window_[i].state_.linear_velocity_.z();
            // bias_acc + bias_gyr
            para_velocity_bias_[i][3] = localMapper_.frame_in_window_[i].state_.bias_a_.x();
            para_velocity_bias_[i][4] = localMapper_.frame_in_window_[i].state_.bias_a_.y();
            para_velocity_bias_[i][5] = localMapper_.frame_in_window_[i].state_.bias_a_.z();
            para_velocity_bias_[i][6] = localMapper_.frame_in_window_[i].state_.bias_g_.x();
            para_velocity_bias_[i][7] = localMapper_.frame_in_window_[i].state_.bias_g_.y();
            para_velocity_bias_[i][8] = localMapper_.frame_in_window_[i].state_.bias_g_.z();
        }

        // extrinsic parameters
        para_ex_pose_[0][0] = localMapper_.extrinsic_parameters_.position_.x();
        para_ex_pose_[0][1] = localMapper_.extrinsic_parameters_.position_.y();
        para_ex_pose_[0][2] = localMapper_.extrinsic_parameters_.position_.z();
        para_ex_pose_[0][3] = localMapper_.extrinsic_parameters_.quaternion_.x();
        para_ex_pose_[0][4] = localMapper_.extrinsic_parameters_.quaternion_.y();
        para_ex_pose_[0][5] = localMapper_.extrinsic_parameters_.quaternion_.z();
        para_ex_pose_[0][6] = localMapper_.extrinsic_parameters_.quaternion_.w();
    }

    void Estimator::Para2States()
    {
        // updates states from ceres param block after optimization
        for (int i = 0; i < WINDOW_SIZE + 1; ++i)
        {

            // position + quaternion
            localMapper_.frame_in_window_[i].state_.position_.x() = para_pose_[i][0];
            localMapper_.frame_in_window_[i].state_.position_.y() = para_pose_[i][1];
            localMapper_.frame_in_window_[i].state_.position_.z() = para_pose_[i][2];
            localMapper_.frame_in_window_[i].state_.quaternion_.x() = para_pose_[i][3];
            localMapper_.frame_in_window_[i].state_.quaternion_.y() = para_pose_[i][4];
            localMapper_.frame_in_window_[i].state_.quaternion_.z() = para_pose_[i][5];
            localMapper_.frame_in_window_[i].state_.quaternion_.w() = para_pose_[i][6];
            // velocity
            localMapper_.frame_in_window_[i].state_.linear_velocity_.x() = para_velocity_bias_[i][0];
            localMapper_.frame_in_window_[i].state_.linear_velocity_.y() = para_velocity_bias_[i][1];
            localMapper_.frame_in_window_[i].state_.linear_velocity_.z() = para_velocity_bias_[i][2];
            // bias_acc + bias_gyr
            localMapper_.frame_in_window_[i].state_.bias_a_.x() = para_velocity_bias_[i][3];
            localMapper_.frame_in_window_[i].state_.bias_a_.y() = para_velocity_bias_[i][4];
            localMapper_.frame_in_window_[i].state_.bias_a_.z() = para_velocity_bias_[i][5];
            localMapper_.frame_in_window_[i].state_.bias_g_.x() = para_velocity_bias_[i][6];
            localMapper_.frame_in_window_[i].state_.bias_g_.y() = para_velocity_bias_[i][7];
            localMapper_.frame_in_window_[i].state_.bias_g_.z() = para_velocity_bias_[i][8];
        }

        // extrinsic parameters
        localMapper_.extrinsic_parameters_.position_.x() = para_ex_pose_[0][0];
        localMapper_.extrinsic_parameters_.position_.y() = para_ex_pose_[0][1];
        localMapper_.extrinsic_parameters_.position_.z() = para_ex_pose_[0][2];
        localMapper_.extrinsic_parameters_.quaternion_.x() = para_ex_pose_[0][3];
        localMapper_.extrinsic_parameters_.quaternion_.y() = para_ex_pose_[0][4];
        localMapper_.extrinsic_parameters_.quaternion_.z() = para_ex_pose_[0][5];
        localMapper_.extrinsic_parameters_.quaternion_.w() = para_ex_pose_[0][6];
    }

    void Estimator::SlideWindow()
    {

        ROS_DEBUG("Sliding window");

        TicToc t_margin;
        if (frame_count_ == WINDOW_SIZE)
        {
            //relocalization
            if(FAST_RELOCALIZATION && globalMapper_.IsLoopClosed()){
                std::cout << "fast relocalization ----- －－－－－－－－－－－－－－　start" << std::endl;
                State updatedstate;
                kfs_mutex_.lock();
                globalMapper_.FastRelocalization(updatedstate);
                kfs_mutex_.unlock();
                UpdateFromRelocalization(updatedstate);
                PublishRelocalization();
                std::cout << "fast relocalization ----- －－－－－－－－－－－－－－－　end" << std::endl;
            }

            if (marginalization_flag_ == MARGIN_OLD)
            {
                Frame frame0 = localMapper_.frame_in_window_[0];
                localMapper_.Add2LocalMap(frame0.cloud_feature_.less_edge_feature_,
                                          frame0.cloud_feature_.less_surf_feature_,
                                          frame0.cloud_feature_.ground_feature_,
                                          frame0.cloud_feature_.plane_feature_,
                                          frame0.state_);

//                visualizer_.AddMarginOldCloud(frame0.state_, feature_cloud, ground_cloud, plane_cloud);

//                kfs_mutex_.lock();
                if(!globalMapper_.AddNewKF(KeyFrame(frame0))){
                    std::cout << "cannot add new kf" << std::endl;
                    UpdateFromPoseGraph();
                    frame0 = localMapper_.frame_in_window_[0];
                    globalMapper_.AddNewKF(KeyFrame(frame0));
                }
                globalMapper_.extrinsic_parameters_ = localMapper_.extrinsic_parameters_;
//                kfs_mutex_.unlock();

                localMapper_.SlideWindow(marginalization_flag_);

//                  add local map to local mapping after loop closure
                  PointCloudT closed_edge_map, closed_surf_map, closed_ground_map;
                  std::vector<PlaneFeature> closed_planes;
                 //  kfs_mutex_.lock();
                  globalMapper_.GetClosedMap(closed_edge_map, closed_surf_map, closed_ground_map, closed_planes, localMapper_.frame_in_window_[FIXED_SIZE].state_);
                 //  kfs_mutex_.unlock();

                  if(!closed_edge_map.points.empty() && !closed_surf_map.points.empty() && !closed_ground_map.points.empty()){
                      ROS_DEBUG("Add closed map");
                      localMapper_.AddClosedLocalMap(closed_edge_map, closed_surf_map, closed_ground_map, closed_planes);
                  }

                for (int i = 0; i < WINDOW_SIZE; i++)
                {
                    std::swap(imu_preintegration_array_[i], imu_preintegration_array_[i + 1]);
                    dt_buf_[i].swap(dt_buf_[i + 1]);
                    linear_acceleration_buf_[i].swap(linear_acceleration_buf_[i + 1]);
                    angular_velocity_buf_[i].swap(angular_velocity_buf_[i + 1]);
                }
//            ROS_DEBUG("slide window imu array[1] sum_dt %f",imu_preintegration_array_[1]->sum_dt_);
            }
            else
            {

                for (unsigned int i = 0; i < dt_buf_[frame_count_].size(); i++)
                {
                    double tmp_dt = dt_buf_[frame_count_][i];
                    Eigen::Vector3d tmp_linear_acceleration = linear_acceleration_buf_[frame_count_][i];
                    Eigen::Vector3d tmp_angular_velocity = angular_velocity_buf_[frame_count_][i];

                    ImuMeasurement tmp_imu_measurement(0.0, tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                    imu_preintegration_array_[frame_count_ - 1]->PushBack(tmp_imu_measurement);

                    dt_buf_[frame_count_ - 1].push_back(tmp_dt);
                    linear_acceleration_buf_[frame_count_ - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf_[frame_count_ - 1].push_back(tmp_angular_velocity);
                }

                localMapper_.SlideWindow(marginalization_flag_);
            }
            delete imu_preintegration_array_[WINDOW_SIZE];
            imu_preintegration_array_[WINDOW_SIZE] = new ImuPreintegration(last_acc_, last_gyr_,
                                                                           localMapper_.frame_in_window_[WINDOW_SIZE].state_.bias_a_,
                                                                           localMapper_.frame_in_window_[WINDOW_SIZE].state_.bias_g_);
            dt_buf_[WINDOW_SIZE].clear();
            linear_acceleration_buf_[WINDOW_SIZE].clear();
            angular_velocity_buf_[WINDOW_SIZE].clear();
        }

        last_state_ = localMapper_.frame_in_window_[frame_count_].state_;
    }

    void Estimator::Publish()
    {

        std::vector<State> local_map_state;
        std::vector<PointCloudT> local_map_feature;
        std::vector<PointCloudT> local_map_ground;
        std::vector<PointCloudT> local_map_plane;
        for (int i = 0; i < frame_count_; ++i) {

            local_map_state.push_back(localMapper_.frame_in_window_[i].state_);
            local_map_feature.push_back(localMapper_.frame_in_window_[i].GetTransformedCloud(localMapper_.extrinsic_parameters_));
            local_map_ground.push_back(localMapper_.frame_in_window_[i].GetTransformedGroundCloud(localMapper_.extrinsic_parameters_));
            local_map_plane.push_back(localMapper_.frame_in_window_[i].GetTransformedPlaneCloud(localMapper_.extrinsic_parameters_));
        }
        visualizer_.UpdateLocalWindow(local_map_state, local_map_feature, local_map_ground, local_map_plane);
        visualizer_.UpdateLocalMap(localMapper_.GetLocalMap());
        visualizer_.Publish();
    }

    void Estimator::UpdateFromPoseGraph() {

        // update using the result from pose graph optimization
        if (frame_count_ < WINDOW_SIZE)
            return;

        // update the pose in window
        State updated_latest_kf;
        kfs_mutex_.lock();
        updated_latest_kf = globalMapper_.GetUpdatePose();
        kfs_mutex_.unlock();

        if (updated_latest_kf.timestamp_ == 0)
            return;

        State old_state;
        State updated_state;
        int j;

        std::cout << updated_latest_kf.timestamp_ << std::endl;

        for (j=0; j < localMapper_.pose_deque_.size(); ++j) {
            std::cout << localMapper_.pose_deque_.at(j).timestamp_ - updated_latest_kf.timestamp_ << std::endl;
            if(std::abs(localMapper_.pose_deque_.at(j).timestamp_ - updated_latest_kf.timestamp_) <= 1e-6){
                old_state.resetPose(localMapper_.pose_deque_.at(j).getTransformation());
                updated_state.resetPose(updated_latest_kf.getTransformation());
                std::cout << j << std::endl;
                break;
            }
        }

        if (j==localMapper_.pose_deque_.size())
            return;


        for (int k = 0; k < WINDOW_SIZE+1; ++k) {
            auto rel_pose = old_state.getInverseTransformation() * localMapper_.frame_in_window_[k].state_.getTransformation();
            auto new_pose = updated_state.getTransformation() * rel_pose;
            Eigen::Matrix3d old_rot = localMapper_.frame_in_window_[k].state_.quaternion_.toRotationMatrix();
            std::cout << k << " - " <<  localMapper_.frame_in_window_[k].state_.position_.transpose() << " " << localMapper_.frame_in_window_[k].state_.getYPR().transpose() << " -> ";
            localMapper_.frame_in_window_[k].state_.resetPose(new_pose);
            std::cout << k << " - " <<  localMapper_.frame_in_window_[k].state_.position_.transpose() << " " << localMapper_.frame_in_window_[k].state_.getYPR().transpose() << std::endl;

            // update velocity represented in world frame
            Eigen::Matrix3d new_rot = localMapper_.frame_in_window_[k].state_.quaternion_.toRotationMatrix();
            Eigen::Vector3d old_velocity = localMapper_.frame_in_window_[k].state_.linear_velocity_;
            Eigen::Vector3d new_velocity = new_rot* old_rot.inverse() * old_velocity;
            localMapper_.frame_in_window_[k].state_.linear_velocity_ = new_velocity;

        }
//        std::cout << "updated" << std::endl;

        for (int m = 0; m < temp_states_high_freq_.size(); ++m) {
            auto rel_pose = old_state.getInverseTransformation() * temp_states_high_freq_.at(m).getTransformation();
            auto new_pose = updated_state.getTransformation() * rel_pose;
            Eigen::Matrix3d old_rot = temp_states_high_freq_.at(m).quaternion_.toRotationMatrix();
            temp_states_high_freq_.at(m).resetPose(new_pose);
            // update velocity represented in world frame
            Eigen::Matrix3d new_rot = temp_states_high_freq_.at(m).quaternion_.toRotationMatrix();
            Eigen::Vector3d old_velocity = temp_states_high_freq_.at(m).linear_velocity_;
            Eigen::Vector3d new_velocity = new_rot* old_rot.inverse() * old_velocity;
            temp_states_high_freq_.at(m).linear_velocity_ = new_velocity;
        }
//        std::cout << "updated" << std::endl;

        for (int l = 0; l < localMapper_.pose_deque_.size(); ++l) {
            auto rel_pose = old_state.getInverseTransformation() * localMapper_.pose_deque_.at(l).getTransformation();
            auto new_pose = updated_state.getTransformation() * rel_pose;
            Eigen::Matrix3d old_rot = localMapper_.pose_deque_.at(l).quaternion_.toRotationMatrix();
            localMapper_.pose_deque_.at(l).resetPose(new_pose);
            // update velocity represented in world frame
            Eigen::Matrix3d new_rot = localMapper_.pose_deque_.at(l).quaternion_.toRotationMatrix();
            Eigen::Vector3d old_velocity = localMapper_.pose_deque_.at(l).linear_velocity_;
            Eigen::Vector3d new_velocity = new_rot* old_rot.inverse() * old_velocity;
            localMapper_.pose_deque_.at(l).linear_velocity_ = new_velocity;
        }
        localMapper_.AssembleLocalMap();

//        std::cout << "update local map in the sliding window!!" << std::endl;

    }


    void Estimator::UpdateFromRelocalization(State & updated_state) {
        // update using the result from pose graph optimization
        if (frame_count_ < WINDOW_SIZE)
            return;
        // update the pose in window
        if (updated_state.timestamp_ == 0)
            return;

        State old_state;
        int j;
        for (j=0; j < localMapper_.pose_deque_.size(); ++j) {
            std::cout << localMapper_.pose_deque_.at(j).timestamp_ << std::endl;
            if(std::abs(localMapper_.pose_deque_.at(j).timestamp_ - updated_state.timestamp_) <= 1e-6){
                old_state.resetPose(localMapper_.pose_deque_.at(j).getTransformation());
                updated_state.resetPose(updated_state.getTransformation());
                std::cout << j << std::endl;
                break;
            }
        }
        if (j==localMapper_.pose_deque_.size())
            return;
        for (int k = 0; k < WINDOW_SIZE+1; ++k) {
            auto rel_pose = old_state.getInverseTransformation() * localMapper_.frame_in_window_[k].state_.getTransformation();
            auto new_pose = updated_state.getTransformation() * rel_pose;
            std::cout << k << " - " <<  localMapper_.frame_in_window_[k].state_.position_.transpose() << " " << localMapper_.frame_in_window_[k].state_.getYPR().transpose() << " -> ";
            localMapper_.frame_in_window_[k].state_.resetPose(new_pose);
            std::cout << k << " - " <<  localMapper_.frame_in_window_[k].state_.position_.transpose() << " " << localMapper_.frame_in_window_[k].state_.getYPR().transpose() << std::endl;
        }
        for (int m = 0; m < temp_states_high_freq_.size(); ++m) {
            auto rel_pose = old_state.getInverseTransformation() * temp_states_high_freq_.at(m).getTransformation();
            auto new_pose = updated_state.getTransformation() * rel_pose;
            temp_states_high_freq_.at(m).resetPose(new_pose);
        }

        for (int l = 0; l < localMapper_.pose_deque_.size(); ++l) {
            auto rel_pose = old_state.getInverseTransformation() * localMapper_.pose_deque_.at(l).getTransformation();
            auto new_pose = updated_state.getTransformation() * rel_pose;
            localMapper_.pose_deque_.at(l).resetPose(new_pose);
        }
        std::cout << "update KeyFrame in the sliding window after relocalization !!" << std::endl;

    }

    void Estimator::PublishRelocalization()
    {
        std::vector<State> local_map_state;
        for (int i = 0; i < frame_count_; ++i) {
            local_map_state.push_back(localMapper_.frame_in_window_[i].state_);
        }
        visualizer_.PublishRelocalization(local_map_state);
    }

} // namespace gp_lio