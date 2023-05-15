//
// Created by wchen on 2019/12/3.
//

#include <pluginlib/class_list_macros.h>
#include "gp_lio/nodelet/EstimatorNodelet.h"

//PLUGINLIB_EXPORT_CLASS(gp_lio::EstimatorNodelet, nodelet::Nodelet)

namespace gp_lio
{

//void EstimatorNodelet::onInit()
    EstimatorNodelet::EstimatorNodelet()
    {
//        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
//    NODELET_DEBUG("Initializing nodelet...");

        InitializeVariables();

//    nh_ = nodelet::Nodelet::getNodeHandle();
//    private_nh_ = nodelet::Nodelet::getPrivateNodeHandle();
        nh_ = ros::NodeHandle();
        private_nh_ = ros::NodeHandle("~");   // our imu :/imu/data    livox imu : /livox/imu_1HDDGAU00100751
        sub_imu_ = nh_.subscribe("/imu/data_xsens", 5000, &EstimatorNodelet::ImuCallback, this, ros::TransportHints().tcpNoDelay());
        sub_cloud_ = nh_.subscribe("/velodyne_points", 5000, &EstimatorNodelet::CloudCallback, this);
//
//        sub_imu_ = nh_.subscribe("/mti/sensor/imu", 5000, &EstimatorNodelet::ImuCallback, this, ros::TransportHints().tcpNoDelay());
//        sub_cloud_ = nh_.subscribe("/velodyne_points", 5000, &EstimatorNodelet::CloudCallback, this);

        estimator_.LoadConfig(private_nh_);

        std::thread *run(new std::thread(&EstimatorNodelet::Run, this));
        std::thread *command(new std::thread(&EstimatorNodelet::Command, this));

        ros::spin();
    }

    void EstimatorNodelet::InitializeVariables()
    {

        last_imu_time_ = 0.0;
        first_imu_ = true;
    }

    void EstimatorNodelet::Run()
    {
        if(REUSE_MAP){
            ROS_ERROR("ready ...");
            estimator_.LoadPosegraph();
            REUSE_MAP = false;
            ROS_ERROR("finished ...");
        }

        MeasurementsGroupType measurements;
        while (ros::ok())
        {

            std::unique_lock<std::mutex> locker(buffer_mutex_);
            while ((measurements = GroupMeasurements()).empty())
                buffer_con_.wait(locker);
            locker.unlock();

            // processing data in the estimator.
            // If the processing time is too long, more data will be stored in the buffer.
//            ROS_DEBUG("new group measurements");
//            for (size_t i = 0; i < measurements.size(); i++)
//            {
//                double cloud_time = measurements[i].second->header.stamp.toSec();
//                for (size_t j = 0; j < measurements[i].first.size(); j++)
//                {
//                    std::cout << (measurements[i].first[j]->header.stamp.toSec() - cloud_time) * 1000 << " ";
//                }
//                std::cout << std::endl;
//            }

            TicToc group_time;
            estimator_.ProcessMeasurements(measurements);
            State updated_state = estimator_.GetUpdatedState();

//            ROS_DEBUG_STREAM("State: " << updated_state.timestamp_ << ", cost " << group_time.toc() << "ms\n\n");
//            ROS_DEBUG_STREAM("  --  position: " << updated_state.position_.transpose());
//            ROS_DEBUG_STREAM("  --  rotation: " << Utility::R2ypr(updated_state.quaternion_.toRotationMatrix()).transpose());
//            ROS_DEBUG_STREAM("  --  linear velocity: " << updated_state.linear_velocity_.transpose());
//            ROS_DEBUG_STREAM("  --  angular velocity: " << updated_state.angular_velocity_.transpose() << "\n");
//            // UpdateStates(updated_state);
        }
    }

    void EstimatorNodelet::ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg)
    {

        if (imu_msg->header.stamp.toSec() <= last_imu_time_)
        {
            ROS_ERROR("imu message disordered!!!");
            return;
        }

        // store
        buffer_mutex_.lock();
        imu_buffer_.push(imu_msg);
        buffer_mutex_.unlock();
        buffer_con_.notify_one();

        // update the states in imu rate
        {
            std::lock_guard<std::mutex> lockGuard(state_mutex_);
            ImuPropagation(imu_msg);
            //TODO publish [tmp_p_, tmp_q_, tmp_linear_velocity_, tmp_angular_velocity_] in visulization class
        }
    }

    void EstimatorNodelet::CloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
    {

        buffer_mutex_.lock();
        cloud_buffer_.push(cloud_msg);
        buffer_mutex_.unlock();
        buffer_con_.notify_one();
    }

    void EstimatorNodelet::ImuPropagation(const sensor_msgs::ImuConstPtr &imu_msg)
    {

        double dx = imu_msg->linear_acceleration.x;
        double dy = imu_msg->linear_acceleration.y;
        double dz = imu_msg->linear_acceleration.z;
        Eigen::Vector3d linear_acceleration{dx, dy, dz};

        double rx = imu_msg->angular_velocity.x;
        double ry = imu_msg->angular_velocity.y;
        double rz = imu_msg->angular_velocity.z;
        Eigen::Vector3d angular_velocity{rx, ry, rz};

        if (first_imu_)
        {
            first_imu_ = false;
            last_imu_acc_ = linear_acceleration;
            last_imu_gyr_ = angular_velocity;
            return;
        }

        double current_time = imu_msg->header.stamp.toSec();
        double dt = current_time - last_imu_time_;

        // mid-point integration for states propagation
        Eigen::Vector3d mid_angular_velocity = 0.5 * (last_imu_gyr_ + angular_velocity) - tmp_state_.bias_g_;
        Eigen::Quaterniond updated_q = tmp_state_.quaternion_ * Utility::deltaQ(mid_angular_velocity * dt);

        Eigen::Vector3d last_acc_in_world = tmp_state_.quaternion_ * (last_imu_acc_ - tmp_state_.bias_a_) - tmp_gravity_;
        Eigen::Vector3d linear_acceleration_in_world = updated_q * (linear_acceleration - tmp_state_.bias_a_) - tmp_gravity_;
        Eigen::Vector3d mid_linear_acceleration = 0.5 * (last_acc_in_world + linear_acceleration_in_world);

        tmp_state_.timestamp_ = current_time;
        tmp_state_.quaternion_ = updated_q;
        tmp_state_.position_ = tmp_state_.position_ + dt * tmp_state_.linear_velocity_ + 0.5 * dt * dt * mid_linear_acceleration;
        tmp_state_.linear_velocity_ = tmp_state_.linear_velocity_ + dt * mid_linear_acceleration;
        tmp_state_.angular_velocity_ = angular_velocity - tmp_state_.bias_g_; // in body-frame

        // save for next propagation
        last_imu_acc_ = linear_acceleration;
        last_imu_gyr_ = angular_velocity;
        last_imu_time_ = imu_msg->header.stamp.toSec();
    }

    MeasurementsGroupType EstimatorNodelet::GroupMeasurements()
    {

        MeasurementsGroupType measurements;
        while (true)
        {

            // ROS_DEBUG_STREAM("buffer size: imu " << imu_buffer_.size() << ", cloud " << cloud_buffer_.size());
            if (imu_buffer_.empty() || cloud_buffer_.empty())
                return measurements;

            ROS_DEBUG("IMU : %f  LIDAR : %f",imu_buffer_.front()->header.stamp.toSec(),cloud_buffer_.front()->header.stamp.toSec());



            if (!(imu_buffer_.back()->header.stamp.toSec() > cloud_buffer_.front()->header.stamp.toSec()))
            {
                return measurements;
            }

            if (!(imu_buffer_.front()->header.stamp.toSec() < cloud_buffer_.front()->header.stamp.toSec()))
            {
//                ROS_WARN("IMU : %d  LIDAR : %d",imu_buffer_.front()->header.stamp.toSec(),cloud_buffer_.front()->header.stamp.toSec());
                ROS_DEBUG("throw the cloud!!!!");
                cloud_buffer_.pop();
                continue;
            }

            sensor_msgs::PointCloud2ConstPtr cloud_msg = cloud_buffer_.front();
            cloud_buffer_.pop();

            std::vector<sensor_msgs::ImuConstPtr> imu_msgs;
            while (imu_buffer_.front()->header.stamp.toSec() < cloud_msg->header.stamp.toSec())
            {
                imu_msgs.emplace_back(imu_buffer_.front());
                imu_buffer_.pop();
            }
            imu_msgs.emplace_back(imu_buffer_.front()); // only use for interpolation the imu measurement at the time of cloud measurments

            if (imu_msgs.empty())
            {
                ROS_WARN("no imu measurements between two cloud");
            }

            measurements.emplace_back(imu_msgs, cloud_msg);
        }
        return measurements;
    }

    void EstimatorNodelet::UpdateStates(State &state)
    {

        buffer_mutex_.lock();
        state_mutex_.lock();

        //TODO

        buffer_mutex_.unlock();
        buffer_mutex_.unlock();
    }

    void EstimatorNodelet::Command() {
//        if (!LOOP_CLOSURE)
//            return;
        while (1) {
            //按s保存位姿图并关闭程序
            char c = std::getchar();
            if (c == 's') {
                std::cout << "get commmand s ..." <<std::endl;
                estimator_.SavePosegraph();
                std::printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
                std::printf("program shutting down...\n");
                ros::shutdown();
            }
            //按n开始一个新的图像序列
            if (c == 'n') {
//                new_sequence();
            }
            std::chrono::milliseconds dura(5);
            std::this_thread::sleep_for(dura);

        }
    }
} // namespace gp_lio
