//
// Created by ss on 2021-07-05.
//

#include "IMUpreintegration.h"
#include <glog/logging.h>

namespace gp_lio{
    IMUpreintegration::IMUpreintegration():nh_("~") {

        extrinsic_maritx4_ = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond exq(extrinsic_maritx4_.block<3,3>(0,0));
        Eigen::Vector3d ext = extrinsic_maritx4_.block<3,1>(0,3);
        lidar2Imu_ = gtsam::Pose3(gtsam::Rot3(exq),gtsam::Point3(ext.x(),ext.y(),ext.z()));
        imu2Lidar_ = lidar2Imu_.inverse();
        lio_opt_initialized_ = false;

        subImu_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data_xsens",100,&IMUpreintegration::ImuCallBack,this);
        subPriorOdometry_ = nh_.subscribe<nav_msgs::Odometry>("/localization/odom",100,&IMUpreintegration::PriroOdometeyCallBack,this);
        pub_li_odom_ptr_ = std::make_shared<OdometryPublisher>(nh_,"/localization/li_odom","/map","lidar",100);
        pubImuOdometry_ = nh_.advertise<nav_msgs::Odometry>("/localization/Imu_incremental", 2000);

        // imu
        lastImuTime_opt_ = -1;
        imuAccNoise_ = 0.1 ;
        imuGyrNoise_ = 0.03;
        imuAccBiasN_ = 0.02;
        imuGyrBiasN_ = 4.0e-6;
        imuGravity_ = 9.81007;
        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity_);
        p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise_, 2);             // 加速度计的白噪声
        p->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise_, 2);                 // 陀螺仪的白噪声
        p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2);                  // 通过速度积分位置信息引入的噪声
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished()); // 初始化imu偏置信息

        priorPoseNoise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // 初始化位姿的噪声
        priorVelNoise_ = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);                                                               // 初始化速度的噪声
        priorBiasNoise_ = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);                                                             // 初始化偏置的噪声
        correctionNoise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());   // rad,rad,rad,m, m, m
        correctionNoise2_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());                 // rad,rad,rad,m, m, m
        noiseModelBetweenBias_ = (gtsam::Vector(6) << imuAccBiasN_, imuAccBiasN_, imuAccBiasN_, imuGyrBiasN_, imuGyrBiasN_, imuGyrBiasN_).finished();

        // 根据上面的参数，定义两个imu预积分器，一个用于imu信息处理线程，一个用于优化线程
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization


        std::thread*command(new std::thread(&IMUpreintegration::Command,this));
    }
    IMUpreintegration::~IMUpreintegration() {}

    void IMUpreintegration::run() {

        while (HasData()){   //data buffer is not empty
            if(!ValidData())
                continue;
            if(UpdatePreintegration()){
                PublishData();
            }

        }

    }

    bool IMUpreintegration::HasData() {
        return !(prior_odom_buffer_.empty() || imu_opt_buffer_.empty() || imu_odom_buffer_.empty());
    }

    bool IMUpreintegration::ValidData() {
        mutex_.lock();
        current_prior_odom_pose_ = prior_odom_buffer_.front();

        prior_odom_buffer_.pop_front();
        mutex_.unlock();
        return true;
    }

    bool IMUpreintegration::UpdatePreintegration() {

        if(imu_opt_buffer_.empty())
            return false;
        mutex_.lock();
        curPriorOdomTime_ = current_prior_odom_pose_->header.stamp.toSec();
        // 转换消息数据为gtsam的3d位姿信息
        float p_x = current_prior_odom_pose_->pose.pose.position.x;
        float p_y = current_prior_odom_pose_->pose.pose.position.y;
        float p_z = current_prior_odom_pose_->pose.pose.position.z;
        float r_x = current_prior_odom_pose_->pose.pose.orientation.x;
        float r_y = current_prior_odom_pose_->pose.pose.orientation.y;
        float r_z = current_prior_odom_pose_->pose.pose.orientation.z;
        float r_w = current_prior_odom_pose_->pose.pose.orientation.w;
        bool degenerate = (int)current_prior_odom_pose_->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPriorPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));
        // 0. initialize system
        if(lio_opt_initialized_ == false)
        {
            resetOptimization();  // 初始化isam2优化器及非线性因子图
            // pop old IMU message
            while (!imu_opt_buffer_.empty())
            {
                if(imu_opt_buffer_.front()->header.stamp.toSec() < curPriorOdomTime_ - delta_t_)
                {
                    lastImuTime_opt_ = imu_opt_buffer_.front()->header.stamp.toSec();
                    imu_opt_buffer_.pop_front();
                }
                else
                    break;
            }
            //initial pose
            prevPose_ = lidarPriorPose.compose(lidar2Imu_);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0),prevPose_,priorPoseNoise_);
            graphFactors_li_.add(priorPose);

            //initial velocity
            prevVel_ = gtsam::Vector3(0,0,0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0),prevVel_,priorVelNoise_);
            graphFactors_li_.add(priorVel);

            //initial bias
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise_);
            graphFactors_li_.add(priorBias);

            // add values
            initialEstimateValues_li_.insert(X(0), prevPose_);
            initialEstimateValues_li_.insert(V(0), prevVel_);
            initialEstimateValues_li_.insert(B(0), prevBias_);
            // optimize once
            isam_li_.update(graphFactors_li_, initialEstimateValues_li_);
            graphFactors_li_.resize(0);
            initialEstimateValues_li_.clear();
            isamCurrentEstimate_li_ = isam_li_.calculateEstimate();

//            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

            key = 1;
            mutex_.unlock();
            lio_opt_initialized_ = true;
            LOG(INFO) << "IMUpretegration initialized finished";
            return true;
        }

        // reset graph for speed
        // 当isam2规模太大时，进行边缘化，重置优化器和因子图
        if(key == 100)
        {
            // get updated noise before reset
            // 获取最新关键帧的协方差
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(isam_li_.marginalCovariance(X(key - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise = gtsam::noiseModel::Gaussian::Covariance(isam_li_.marginalCovariance(V(key - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(isam_li_.marginalCovariance(B(key - 1)));
            // reset graph
            // 重置isam2优化器和因子图
            resetOptimization();
            // 按最新关键帧的协方差将位姿、速度、偏置因子添加到因子图中
            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors_li_.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors_li_.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors_li_.add(priorBias);
            // add values
            // 并用最新关键帧的位姿、速度、偏置初始化对应的因子
            initialEstimateValues_li_.insert(X(0), prevPose_);
            initialEstimateValues_li_.insert(V(0), prevVel_);
            initialEstimateValues_li_.insert(B(0), prevBias_);
            // optimize once
            // 并将最新初始化的因子图更新到重置的isam2优化器中
            isam_li_.update(graphFactors_li_, initialEstimateValues_li_);
            graphFactors_li_.resize(0);
            initialEstimateValues_li_.clear();
            isamCurrentEstimate_li_ = isam_li_.calculateEstimate();

            key = 1; // 重置关键帧数量
        }

        // 1. integrate imu data and optimize
        while (!imu_opt_buffer_.empty())
        {
            // pop and integrate imu data that is between two optimizations
            // 对相邻两次优化之间的imu帧进行积分，并移除
            sensor_msgs::ImuConstPtr thisImu = imu_opt_buffer_.front();
            double imuTime = thisImu->header.stamp.toSec();
            if(imuTime < curPriorOdomTime_ - delta_t_)
            {
                double dt = (lastImuTime_opt_ < 0) ? (1.0/200):(imuTime - lastImuTime_opt_);
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), dt);
                lastImuTime_opt_ = imuTime;
                imu_opt_buffer_.pop_front();
            }
            else
                break;
        }
//        mutex_.unlock();

        // add imu factor to graph
        // 将imu因子添加到因子图中
        const gtsam::PreintegratedImuMeasurements &preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu); // 该因子涉及的优化变量包括:上一帧和当前帧的位姿和速度，上一帧的偏置，相邻两关键帧之间的预积分结果
        graphFactors_li_.add(imu_factor);
        // add imu bias between factor
        // 将imu偏置因子添加到因子图中
        graphFactors_li_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                                 gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias_)));
        // add pose factor
        // 添加当前关键帧位姿因子
        gtsam::Pose3 curPose = lidarPriorPose.compose(lidar2Imu_);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2_ : correctionNoise_);
        graphFactors_li_.add(pose_factor);
        // insert predicted values
        // 设置当前关键帧位姿因子、速度因子和偏置因子的初始值
        gtsam::NavState propState_ = imuIntegratorOpt_->                                                                                                                                                                   predict(prevState_, prevBias_);
        initialEstimateValues_li_.insert(X(key), propState_.pose());
        initialEstimateValues_li_.insert(V(key), propState_.v());
        initialEstimateValues_li_.insert(B(key), prevBias_);
        // optimize
        // 将最新关键帧相关的因子图更新到isam2优化器中，并进行优化
        isam_li_.update(graphFactors_li_, initialEstimateValues_li_);
        isam_li_.update();
        graphFactors_li_.resize(0);
        initialEstimateValues_li_.clear();
        // Overwrite the beginning of the preintegration for the next step.
        // 获取当前关键帧的优化结果，并将结果置为先前值
        gtsam::Values result = isam_li_.calculateEstimate();
        isamCurrentEstimate_li_ = isam_li_.calculateEstimate();
        prevPose_ = result.at<gtsam::Pose3>(X(key));
        prevVel_ = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
        // Reset the optimization preintegration object.
        // 利用优化后的imu偏置信息重置imu预积分对象
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        // check optimization
        // 对优化结果进行失败检测: 当速度和偏置太大时，则认为优化失败
        //...
        if(failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            return false;
        }

        // 2. after optiization, re-propagate imu odometry preintegration
        // 2. 优化后，重新对imu里程计进行预积分
        // 利用优化结果更新prev状态
        //...
        prevStateOdom_ = prevState_;
        prevBiasOdom_ = prevBias_;

        //first pop imu message older than current correction data
        double imuTime2 = imu_odom_buffer_.front()->header.stamp.toSec();
        double lastImuTime = -1;
        while (!imu_odom_buffer_.empty() && imuTime2 < curPriorOdomTime_ - delta_t_)
        {
            lastImuTime = imuTime2 = imu_odom_buffer_.front()->header.stamp.toSec();
            imu_odom_buffer_.pop_front();
        }
        //repropogate
        if(!imu_odom_buffer_.empty())
        {
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom_);
            for(int i = 0; i < (int)imu_odom_buffer_.size(); ++i)
            {
                sensor_msgs::ImuConstPtr thisImu = imu_odom_buffer_[i];
                double imuTime3 = thisImu->header.stamp.toSec();
                double dt = (lastImuTime < 0) ? (1.0/200.0) : (imuTime3 -lastImuTime);
                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), dt);
                lastImuTime = imuTime3;
            }

        }
        mutex_.unlock();



        ++key;
        doneFirstOpt_ = true;
        return true;

    }

    bool IMUpreintegration::failureDetection(const gtsam::Vector3 &velCur,
                                             const gtsam::imuBias::ConstantBias &biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        // 当速度太大，则认为失败
        if (vel.norm() > 30)
        {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        // 当偏置太大，则认为失败
        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }
    void IMUpreintegration::resetOptimization()
    {
        // 重置isam2优化器
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        isam_li_ = gtsam::ISAM2(optParameters);

        // 重置初始化非线性因子图
        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors_li_ = newGraphFactors;

        gtsam::Values NewGraphValues;
        initialEstimateValues_li_ = NewGraphValues;
    }

    void IMUpreintegration::resetParams()
    {
        lastImuTime_opt_ = -1;
        doneFirstOpt_ = false;
        lio_opt_initialized_ = false;
    }

    bool IMUpreintegration::PublishData() {
        unsigned size = isamCurrentEstimate_li_.size()/3;
        if(!size)
        {
            return false;
        }
        gtsam::Pose3 li_pose = isamCurrentEstimate_li_.at<gtsam::Pose3>(X(size-1)).compose(imu2Lidar_);
        gtsam::Vector3 li_velocity = isamCurrentEstimate_li_.at<gtsam::Vector3>(V(size-1));
        li_velocity_.x() = li_velocity.x();
        li_velocity_.y() = li_velocity.y();
        li_velocity_.z() = li_velocity.z();
        Eigen::Quaternionf q;
        q.x() = li_pose.rotation().toQuaternion().x();
        q.y() = li_pose.rotation().toQuaternion().y();
        q.z() = li_pose.rotation().toQuaternion().z();
        q.w() = li_pose.rotation().toQuaternion().w();
        Eigen::Matrix3f R = q.normalized().toRotationMatrix();
        Eigen::Vector3f t;
        t.x() = li_pose.translation().x();
        t.y() = li_pose.translation().y();
        t.z() = li_pose.translation().z();
        li_odometry_.block<3,3>(0,0) = R;
        li_odometry_.block<3,1>(0,3) = t;

        Eigen::Quaternionf tmp_q(li_odometry_.block<3,3>(0,0));
        Eigen::Vector3f tmp_p(li_odometry_.topRightCorner(3,1));
        double tmp_t = curPriorOdomTime_;
        std::array<float,7> tmp_odom{tmp_p.x(),tmp_p.y(),tmp_p.z(),
                                     tmp_q.x(),tmp_q.y(),tmp_q.z(),tmp_q.w()};
        odom_li_.push_back(std::make_pair(tmp_t,tmp_odom));

        pub_li_odom_ptr_->Publish(li_odometry_,li_velocity_,curPriorOdomTime_);
        return true;
    }

    void IMUpreintegration::ImuCallBack(const sensor_msgs::ImuConstPtr &imu_raw) {
        mutex_.lock();
        imu_opt_buffer_.push_back(imu_raw);
        imu_odom_buffer_.push_back(imu_raw);
        mutex_.unlock();

        if(doneFirstOpt_ == false)
            return;

        double imuTime = imu_raw->header.stamp.toSec();
        double dt = (lastImuTime_imu_ < 0) ? (1.0/200.0):(imuTime-lastImuTime_imu_);
        lastImuTime_imu_ = imuTime;
//        std::cout << "ImuCallBack-------> dt:" << dt << std::endl;
        // 记录imu的测量信息
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(imu_raw->linear_acceleration.x, imu_raw->linear_acceleration.y, imu_raw->linear_acceleration.z),
                                                gtsam::Vector3(imu_raw->angular_velocity.x, imu_raw->angular_velocity.y, imu_raw->angular_velocity.z), dt);

        // predict odometry
        // 利用上一时刻的imu里程计状态信息PVQ和偏置信息，预积分当前时刻imu里程计状态信息PVQ
//        Eigen::Vector3f ba(0, 0, 0);
//        Eigen::Vector3f bg(0, 0, 0);
//        prevBiasOdom_.identity();
//        std::cout<<"ImuCallBack---> "<<prevBiasOdom_.accelerometer().x()<<" "<<prevBiasOdom_.accelerometer().y()<<" "<<prevBiasOdom_.accelerometer().z()
//                                     <<" "<< prevBiasOdom_.gyroscope().x()<<" "<< prevBiasOdom_.gyroscope().y()<<" "<< prevBiasOdom_.gyroscope().z()<<std::endl;
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom_, prevBiasOdom_);

        // publish odometry
        nav_msgs::Odometry odometry;
        odometry.header.stamp = imu_raw->header.stamp;
        odometry.header.frame_id = "/map";
        odometry.child_frame_id = "lidar";

        // transform imu pose to ldiar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar_); // 获得估计的雷达位姿信息

//        odometry.pose.pose.position.x = prevStateOdom_.position().x();
//        odometry.pose.pose.position.y = prevStateOdom_.position().y();
//        odometry.pose.pose.position.z = prevStateOdom_.position().z();
//        odometry.pose.pose.orientation.x = prevStateOdom_.quaternion().x();
//        odometry.pose.pose.orientation.y = prevStateOdom_.quaternion().y();
//        odometry.pose.pose.orientation.z = prevStateOdom_.quaternion().z();
//        odometry.pose.pose.orientation.w = prevStateOdom_.quaternion().w();

//        // 发布通过imu估计的雷达里程计信息(后面都称为imu里程计信息)
        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();

        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = imu_raw->angular_velocity.x + prevBiasOdom_.gyroscope().x();
        odometry.twist.twist.angular.y = imu_raw->angular_velocity.y + prevBiasOdom_.gyroscope().y();
        odometry.twist.twist.angular.z = imu_raw->angular_velocity.z + prevBiasOdom_.gyroscope().z();
        pubImuOdometry_.publish(odometry);
    }

    void IMUpreintegration::PriroOdometeyCallBack(const nav_msgs::OdometryConstPtr &odomMsg) {
        mutex_.lock();
        prior_odom_buffer_.push_back(odomMsg);
        mutex_.unlock();
    }

    void IMUpreintegration::Command() {
        while (1){
            char c = std::getchar();
            if('s'==c){
                std::cout <<"get command s ..."<<std::endl;
//                std::string dir_path = MAP_PATH + "/odom1";
                std::string dir_path = "/home/luo/work/map/work_ws/src/localization/result/odom1";
                std::string file_path = dir_path + "/odom_li.txt";
                if(file_manager1_.CreatDirectory(dir_path)){
                    std::FILE* ofile_odom = std::fopen(file_path.c_str(),"w");
                    ROS_ERROR("ready to save odom datas, please waite ...");
                    for(auto & odom:odom_li_){
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



int main(int argc, char** argv){

    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;
    ros::init(argc,argv,"IMUpreintegration_node");
    gp_lio::IMUpreintegration ImuP;
    ros::Rate rate(200);
    while (ros::ok()){
        ros::spinOnce();
        ImuP.run();
        rate.sleep();
    }

    return 0;

}