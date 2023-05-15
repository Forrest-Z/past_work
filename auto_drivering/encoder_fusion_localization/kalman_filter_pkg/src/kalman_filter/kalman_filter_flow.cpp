/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-18 19:38:02
 * @LastEditors: luo
 * @LastEditTime: 2022-01-08 15:46:29
 */

#include "kalman_filter_pkg/kalman_filter/kalman_filter_flow.hpp"

namespace KalmanFilter
{

KalmanFilterFlow::KalmanFilterFlow(ros::NodeHandle& nh) 
:nh_(nh)
{
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh_, "/velodyne_points", 100000);
    // cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh_, "/transfered_velodyne_points", 100000);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh_, "/imu/data_xsens", 100000);
    odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh_, "/synced_odom", 100000);
    encoder_sub_ptr_ = std::make_shared<EncoderSubscriber>(nh_, "/IMCU_INFO", 100000);

    
    // cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh_, "/synced_cloud", 100000);
    // imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh_, "/synced_imu", 100000);
    // odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh_, "/synced_odom", 100000);

    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh_, "/global_map", "map", 100000);
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh_, "/local_map", "/map", 100000);

    matching_ptr_ = std::make_shared<Matching>();

    odom_publish_ptr_ = std::make_shared<OdometryPublisher>(nh_, "/kalman_filter_odom", "/map", "/lidar", 1000);
    matching_odom_publish_ptr_ = std::make_shared<OdometryPublisher>(nh_, "/kalman_matching_odom", "/map", "/lidar_odom", 1000);
    encoder_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh_, "/encoder_odom", "/map", "/encoder_odom", 100);


    gyro_noise_ = 0.01;
    acc_noise_ = 0.28;
    gyro_bias_noise_ = 0.0000001;
    acc_bias_noise_ = 0.02;
    v_noise_ = 0.01;

    laser_pose_noise_ = 2.0 * kDegreeToRatian;
    laser_position_noise_ = 0.005;


}

KalmanFilterFlow::~KalmanFilterFlow()
{

}

bool KalmanFilterFlow::Run()
{
    if(!ReadData())
        return false;

    if(!InitCalibration())
        return false;

    SetInitPose();

    ValidData();
    // while()
    {
        // CalculateEncoderOdom();

        Filter();
        PublishData();
    }

    imu_data_buff_.clear();
    cloud_data_buff_.clear();
    odom_data_buff_.clear();
    encoder_data_buff_.clear();

}

bool KalmanFilterFlow::ReadData()
{   

    // std::deque<IMUData> unsynced_imu;
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    encoder_sub_ptr_->ParseData(encoder_data_buff_);
    imu_sub_ptr_->ParseData(imu_data_buff_);
    odom_sub_ptr_->ParseData(odom_data_buff_);


    // std::cout << " encoder_data_buff_ size = " << encoder_data_buff_.size() << std::endl;
    if(cloud_data_buff_.empty())
        return false;

    // double synced_time = cloud_data_buff_.front().time;
    // bool valid_imu = IMUData::SyncData(unsynced_imu, imu_data_buff_, synced_time);


    std::cout << " cloud size = " << cloud_data_buff_.size() << std::endl;
    std::cout << " imu size = " << imu_data_buff_.size() << std::endl;
    std::cout << " encoder size = " << encoder_data_buff_.size() << std::endl;
    std::cout << " odom size = " << odom_data_buff_.size() << std::endl;

    static bool sensor_inited = false;
    if(!sensor_inited)
    {
        if(imu_data_buff_.front().time > cloud_data_buff_.front().time  || 
        encoder_data_buff_.front().time > cloud_data_buff_.front().time)
        {
            cloud_data_buff_.pop_front();

        }

        sensor_inited = true;
    }

    // std::cout << "ReadData = " << sensor_inited << std::endl;
    return sensor_inited;

}

bool KalmanFilterFlow::InitCalibration()
{
    static bool calibration = false;
    // std::cout << "InitCalibration 00" << std::endl;
    if(!calibration)
    {
        // lidar_to_imu_(2, 3) = 0.3330773; 

        encoder_to_imu_(0, 3) = -0.5;
        encoder_to_imu_(2, 3) = 2.3;

        calibration = true;
    }

    std::cout << "InitCalibration 11 calibration = " << calibration << std::endl;
    return calibration;
    
}


bool KalmanFilterFlow::SetInitPose()
{
    static bool pose_inited = false;
    std::cout << "SetInitPose " << std::endl;
    if(pose_inited)
    {
        return true;
    }

    std::cout << "SetInitPose 00" << std::endl;
 
    std::cout << "SetInitPose 11" << std::endl;
    if(!pose_inited && !odom_data_buff_.empty())
    {
        has_inited_ = true;
        pose_inited = true;

        init_pose_ = odom_data_buff_.back().pose;
        

        matching_ptr_->SetInitPose(init_pose_);
        // kalman_filter_process_ptr_->SetInitPose(init_pose_);

        position_x = init_pose_(0, 3);
        position_y = init_pose_(1, 3);
        Eigen::Vector3d vec_angle = init_pose_.block<3, 3>(0, 0).cast<double>().eulerAngles(0, 1, 2);
        // float angle_z = vec_angle[0];
        // pose_yaw = (double) angle_z;
        pose_yaw = vec_angle[2];

        std::cout << " p_x = " << position_x << std::endl;
        std::cout << " p_y = " << position_y << std::endl;
        std::cout << " p_yaw = " << pose_yaw << std::endl;
        
        state_.position = Eigen::Vector3d(init_pose_(0, 3), init_pose_(1, 3), init_pose_(2, 3));
        state_.velocity = Eigen::Vector3d::Zero();
        state_.pose = Eigen::Quaterniond(init_pose_.block<3, 3>(0, 0).cast<double>());
        // state_.pose = init_pose_.block<3, 3>(0, 0).cast<double>();
        state_.ba.setZero();
        state_.bg.setZero();

        error_state_.x.setZero();
        error_state_.p.setZero();
        error_state_.p.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-2;
        error_state_.p.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-3;
        error_state_.p.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 1e-4;
        error_state_.p.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 1e-4;
        error_state_.p.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 1e-6;
    }

    std::cout << "SetInitPose 33" << std::endl;
    return has_inited_;
    
}

bool KalmanFilterFlow::ValidData()
{
    std::cout << "ValidData 00" << std::endl;

    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_odom_data_ = odom_data_buff_.front();
    current_encoder_data_ = encoder_data_buff_.front();

    std::cout << "ValidData 11" << std::endl;

    double synced_time = current_cloud_data_.time - current_imu_data_.time;
    double synced_encoder_time = current_cloud_data_.time - current_encoder_data_.time;

    std::cout << "ValidData 22"<< synced_time << ", synced_encoder_time = " << synced_time << std::endl;

    if(synced_time < -0.05 || synced_encoder_time < -0.05)
    {
        cloud_data_buff_.pop_front();
        return false;
    }

    if(synced_time > 0.05)
    {
        imu_data_buff_.pop_front();
        return false;
    }

    if(synced_encoder_time > 0.05)
    {
        encoder_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    odom_data_buff_.pop_front();
    encoder_data_buff_.pop_front();
    
    return true;

}

Eigen::Matrix4f KalmanFilterFlow::GetPose()
{
    Eigen::Matrix4f pose;
    pose.block<3, 3>(0, 0) = state_.pose.cast<float>().toRotationMatrix();
    pose(0, 3) = state_.position(0);
    pose(1, 3) = state_.position(1);
    pose(2, 3) = state_.position(2);

    return pose;
    
}

bool KalmanFilterFlow::CalculateEncoderOdom(EncoderData& encoder_msg, IMUData& imu_msg, double dt)
{
    
}

bool KalmanFilterFlow::CalculateEncoderPosition(EncoderData& encoder_msg, IMUData& imu_msg, double dt)
{
    encoder_msg.GetStatus();
    encoder_msg.CalculateNLeft();
    encoder_msg.CalculateNRight();
    encoder_msg.CalculateVelo();

    // std::cout<< "velo_left = " << encoder_msg.velo_left << std::endl;
    // std::cout<< "velo_right = " << encoder_msg.velo_right << std::endl;

    double delta_left_p = encoder_msg.velo_left * dt;
    double delta_right_p = encoder_msg.velo_right * dt;

    // std::cout<< "delta_left_p = " << delta_left_p << std::endl;
    // std::cout<< "delta_right_p = " << delta_right_p << std::endl;

    double delta_s = 0.5 * ( delta_left_p + delta_right_p);
    double delta_th = encoder_msg.w_encoder * dt;

    // double imu_x = imu_msg.angular_vel.x * dt;
    // double imu_y = imu_msg.angular_vel.y * dt;
    double imu_z = imu_msg.angular_vel.z * dt;
    
    pose_yaw += imu_z;
    // pose_yaw += delta_th;

    // delta_encoder_x += delta_s * std::cos(pose_yaw + delta_th);
    // delta_encoder_y += delta_s * std::sin(pose_yaw + delta_th);
    // encoder_position[2] = 0.0;

    position_x += delta_s * std::cos(pose_yaw + delta_th);
    position_y += delta_s * std::sin(pose_yaw + delta_th);
    double z = 0.0;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_yaw);
    Eigen::Quaterniond qua;
    qua.x() = odom_quat.x;
    qua.y() = odom_quat.y;
    qua.z() = odom_quat.z;
    qua.w() = odom_quat.w;

    Eigen::Matrix3d r_mat = qua.toRotationMatrix();


    std::cout<< "position_x = " << position_x << std::endl;
    std::cout<< "position_y = " << position_y << std::endl;
    std::cout<< "pose_yaw = " << pose_yaw << std::endl;
    std::cout<< "-----------v------------  = " << encoder_msg.v_encoder << std::endl;


    Eigen::Matrix4f encoder_pose = Eigen::Matrix4f::Identity();
    encoder_pose.block<3, 3>(0, 0) = r_mat.cast<float>();
    // encoder_pose(0, 3) = delta_encoder_x;
    // encoder_pose(1, 3) = delta_encoder_y;
    encoder_pose(0, 3) = position_x;
    encoder_pose(1, 3) = position_y;
    encoder_pose(2, 3) = z;

    // encoder_odom = encoder_pose.cast<float>() * encoder_to_imu_;
    encoder_odom = encoder_pose.cast<float>();
}


bool KalmanFilterFlow::Filter()
{
    // std::cout << "Filter 00" << std::endl;
   
    // Predict();


    PredictProcess();
    // std::cout << "Filter 11" << std::endl;

    // Correct();

    // CorrectProcess();
    CorrectFromEncoder();
    // std::cout << "Filter 22" << std::endl;

    return true;

}

bool KalmanFilterFlow::PredictProcess()
{
    if(!has_inited_)
    {
        return false;
    }

    if(imu_data_buff_.size() == 0)
    {
        return false;
    }

    if(encoder_data_buff_.size() == 0)
    {
        return false;
    }

    Eigen::Vector3d pp = state_.position;
    Eigen::Vector3d vv = state_.velocity;
    Eigen::Quaterniond qq = state_.pose;
    Eigen::Vector3d gn(0, 0, -9.809432933396721);
    Eigen::Vector3d wb;
    double encoder_vv = 0.0;

    int data_size = imu_data_buff_.size() > encoder_data_buff_.size() ? encoder_data_buff_.size() : imu_data_buff_.size();
    
    for(int i = 1; i < data_size; i++)
    {
        ///姿态解算
        double delta_time = imu_data_buff_[i].time - imu_data_buff_[i - 1].time;

        CalculateEncoderPosition(encoder_data_buff_[i], imu_data_buff_[i], delta_time);

        encoder_vv += encoder_data_buff_[i].v_encoder;
        // encoder_position[0] += delta_encoder_x;
        // encoder_position[1] += delta_encoder_y;

        //角度跟新(四元素)
        wb [0] = 0.5 * imu_data_buff_[i-1].angular_vel.x +  0.5 * imu_data_buff_[i].angular_vel.x;
        wb [1] = 0.5 * imu_data_buff_[i-1].angular_vel.y +  0.5 * imu_data_buff_[i].angular_vel.y;
        wb [2] = 0.5 * imu_data_buff_[i-1].angular_vel.z +  0.5 * imu_data_buff_[i].angular_vel.z;

        wb = wb - state_.bg;
        wb = wb * delta_time;//等效旋转矢量
        double angular_delta_mag = wb.norm();//等效旋转矢量大小,即旋转角度的大小
        Eigen::Vector3d angular_delta_dir = wb.normalized();//归一化

        double angular_delta_cos = std::cos(angular_delta_mag / 2.0);//四元素实部
        double angular_delta_sin = std::sin(angular_delta_mag / 2.0);//四元素虚部

        //四元素q:body坐标系中k-1时刻到k时刻,角度增量
        Eigen::Quaterniond dq(angular_delta_cos,
                                angular_delta_sin * angular_delta_dir.x(),
                                angular_delta_sin * angular_delta_dir.y(),
                                angular_delta_sin * angular_delta_dir.z());
        
        Eigen::Quaterniond qq2 = (qq * dq).normalized();

        //计算线加速度真值
        Eigen::Vector3d f1(imu_data_buff_[i-1].linear_acc.x, imu_data_buff_[i-1].linear_acc.y, imu_data_buff_[i-1].linear_acc.z);
        f1 = f1 - state_.ba;
        Eigen::Vector3d f2(imu_data_buff_[i].linear_acc.x, imu_data_buff_[i].linear_acc.y, imu_data_buff_[i].linear_acc.z);
        f2 = f2 - state_.ba;

        //速度跟新(中值法)
        Eigen::Vector3d vv2 = vv + delta_time * (0.5 * (qq * f1 + qq2 * f2) + gn);
        
        //位置跟新(中值法)
        Eigen::Vector3d pp2 = pp + 0.5 * delta_time * (vv + vv2);

        pp = pp2;
        vv = vv2;
        qq = qq2;

    }

    encoder_position[0] = delta_encoder_x / data_size;
    encoder_position[1] = delta_encoder_y / data_size;

    encoder_velo = encoder_vv / data_size;
    // std::cout << "encoder pose = " << std::endl << encoder_odom << std::endl;
    // std::cout << "-----------encoder_velo--------- = " << encoder_velo << std::endl;
    // std::cout << "-----------vvvvv--------- = " << vv.norm() << std::endl;
    // std::cout << "-----------vvvvv----x----- = " << vv[0] << std::endl;
    // std::cout << "-----------vvvvv----y----- = " << vv[1] << std::endl;
    // std::cout << "-----------vvvvv----z----- = " << vv[2] << std::endl;
    // std::cout << "Predict 22" << std::endl;
    state_.position = pp;
    state_.velocity = vv;
    state_.pose = qq;

    // state_.position(2, 0) = 0;
    // state_.velocity(2, 0) = 0;

    // std::cout << "Predict 22---1" << std::endl;
    Eigen::Matrix<double, 15, 15> Ft = Eigen::Matrix<double, 15, 15>::Zero();
    Ft.block<3, 3>(0, 3) = Eigen::Matrix<double, 3, 3>::Identity();

    Eigen::Matrix<double, 3, 3> temp = Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Vector3d ff(imu_data_buff_.back().linear_acc.x, imu_data_buff_.back().linear_acc.y, imu_data_buff_.back().linear_acc.z);

    // std::cout << "Predict 33" << std::endl;
    ff = ff - state_.ba;
    // temp << 0, -ff[2], ff[1], ff[2], 0, -ff[0], -ff[1], ff[0], 0;
    temp = GetSkewMatrix(ff);
    temp= -qq.toRotationMatrix() * temp;
    Ft.block<3, 3>(3, 6) = temp;
    Ft.block<3, 3>(3, 9) = -qq.toRotationMatrix();
    // std::cout << "Predict 44" << std::endl;
    temp.setZero();
    Eigen::Vector3d wbb;
    wbb[0] = imu_data_buff_.back().angular_vel.x;
    wbb[1] = imu_data_buff_.back().angular_vel.y;
    wbb[2] = imu_data_buff_.back().angular_vel.z;
    wbb = wbb - state_.bg;
    // temp << 0, -wb(2), wb(1), wb(2), 0, -wb(0), -wb(1), wb(0), 0;
    temp = GetSkewMatrix(wbb);
    Ft.block<3, 3>(6, 6) = -temp;
    Ft.block<3, 3>(6, 12) = -Eigen::Matrix<double, 3, 3>::Identity();

    double T = imu_data_buff_.back().time - imu_data_buff_.front().time;
    Ft = Eigen::Matrix<double, 15, 15>::Identity() + Ft * T;

    Eigen::Matrix<double, 15, 12> Bt = Eigen::Matrix<double, 15, 12>::Zero();
    Bt.block<3, 3>(3, 0) = qq.toRotationMatrix();
    Bt.block<3, 3>(6, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    Bt.block<3, 3>(9, 6) = Eigen::Matrix<double, 3, 3>::Identity();
    Bt.block<3, 3>(12, 9) = Eigen::Matrix<double, 3, 3>::Identity();
    Bt = Bt * T;

    // std::cout << "Predict 66" << std::endl;
    Eigen::Matrix<double, 12, 1> ww = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Vector3d noise_gyro;
    noise_gyro<< gyro_noise_,gyro_noise_,gyro_noise_;
    Eigen::Vector3d noise_acc;
    noise_acc<< acc_noise_,acc_noise_,acc_noise_;
    Eigen::Vector3d noise_bias_gyro;
    noise_bias_gyro<< gyro_bias_noise_,gyro_bias_noise_,gyro_bias_noise_;
    Eigen::Vector3d noise_bias_acc;
    noise_bias_acc<< acc_bias_noise_,acc_bias_noise_,acc_bias_noise_;
    ww.head(3) = noise_acc;
    ww.block<3,1>(3,0)=noise_gyro;
    ww.block<3,1>(6,0)=noise_bias_acc;
    ww.tail(3) =noise_bias_gyro;

    // ww(2, 0) = 0;
    // ww(5, 0) = 0;
    // ww(8, 0) = 0;
    // ww(11, 0) = 0;

    error_state_.x = Ft * error_state_.x + Bt * ww;
    // error_state_.x(2, 0) = 0;
    // error_state_.x(5, 0) = 0;
    // std::cout << " error_state_  = " << std::endl << error_state_.x << std::endl; 

    Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Identity();
    Q.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity() * acc_noise_ * acc_noise_ ;
    Q.block<3, 3>(3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * gyro_noise_ *  gyro_noise_;
    Q.block<3, 3>(6, 6) = Eigen::Matrix<double, 3, 3>::Identity() * acc_bias_noise_ * acc_bias_noise_;
    Q.block<3, 3>(9, 9) = Eigen::Matrix<double, 3, 3>::Identity() * gyro_bias_noise_* gyro_bias_noise_;

    
    error_state_.p = Ft * error_state_.p * Ft.transpose() + Bt * Q * Bt.transpose();

    // std::cout << "Predict 88" << std::endl;
    return true;

}

bool KalmanFilterFlow::CorrectFromEncoder()
{

    Eigen::Matrix4d predicted_pose = Eigen::Matrix4d::Identity();
    predicted_pose.block<3, 1>(0, 3) = state_.position + error_state_.x.block<3, 1>(0, 0);
    // predicted_pose(2, 3) = 0.0;

    Eigen::Vector3d error_q = error_state_.x.block<3, 1>(6, 0);
    double error_q_norm = error_q.norm();
    if(error_q_norm != 0)
    {
        error_q = error_q / error_q_norm;
        error_q = error_q * std::sin(error_q_norm / 2);
    }
    Eigen::Quaterniond temp_q(std::cos(error_q_norm / 2), error_q[0], error_q[1], error_q[2]);
    predicted_pose.block<3, 3>(0, 0) = ((state_.pose * temp_q).normalized()).toRotationMatrix();

    // matching_ptr_->Update(current_cloud_data_, laser_odometry);
    Eigen::Matrix4f laser_odometry = Eigen::Matrix4f::Identity();
    matching_odom = odom_data_buff_.front().pose.cast<float>();
    laser_odometry = matching_odom;

    Eigen::Matrix<double, 9, 1> Yk;
    Yk.block<3, 1>(0, 0) = predicted_pose.block<3, 1>(0, 3).cast<double>() - laser_odometry.block<3, 1>(0, 3).cast<double>();

    Eigen::Matrix3d delta_R = predicted_pose.block<3, 3>(0, 0) * laser_odometry.block<3, 3>(0, 0).cast<double>().transpose();
    delta_R = delta_R - Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Vector3d delta_pose(delta_R(1, 2), delta_R(2, 0), delta_R(0, 1));
    Yk(6, 0) = delta_pose[0];
    Yk(7, 0) = delta_pose[1];
    Yk(8, 0) = delta_pose[2];

    Eigen::Vector3d velo(encoder_velo, 0, 0);
    Eigen::Vector3d delta_v = state_.velocity - encoder_to_imu_.block<3, 3>(0, 0).cast<double>() * velo;
    Yk.block<3, 1>(3, 0) = delta_v;

    Eigen::Matrix<double, 9, 15> Gk = Eigen::Matrix<double, 9, 15>::Zero();
    Gk.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();//p
    Gk.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();//theta
    Gk.block<3, 3>(3, 3) = encoder_to_imu_.block<3, 3>(0, 0).cast<double>();//v
    Gk.block<3, 3>(3, 6) = GetSkewMatrix(velo);//v

    Eigen::Matrix<double, 9, 9> Ck = Eigen::Matrix<double, 9, 9>::Identity();
    Eigen::Matrix<double, 9, 9> Rk = Eigen::Matrix<double, 9, 9>::Identity();
    Rk.block<3, 3>(0, 0) = laser_position_noise_ * laser_position_noise_ *  Eigen::Matrix3d::Identity();
    Rk.block<3, 3>(3, 3) = v_noise_ * v_noise_ * Eigen::Matrix3d::Identity();
    Rk.block<3, 3>(6, 6) = laser_position_noise_ * laser_position_noise_ * Eigen::Matrix3d::Identity();

    const Eigen::MatrixXd Kk = error_state_.p * Gk.transpose() * (Gk * error_state_.p * Gk.transpose() + Ck * Rk * Ck.transpose()).inverse();

    error_state_.p = (Eigen::Matrix<double, 15, 15>::Identity() - Kk * Gk) * error_state_.p;
    error_state_.x = error_state_.x + Kk * (Yk - Gk * error_state_.x);

    // state_.position[0] = encoder_position[0];
    // state_.position[1] = encoder_position[1];
    
    state_.position = state_.position - error_state_.x.block<3, 1>(0, 0);
    state_.velocity = state_.velocity - error_state_.x.block<3, 1>(3, 0);
    Eigen::Vector3d dphi_dir = error_state_.x.block<3, 1>(6, 0);
    double dphi_norm = dphi_dir.norm();

    if(dphi_norm != 0)
    {
        dphi_dir = dphi_dir / dphi_norm;
        dphi_dir = dphi_dir * std::sin(dphi_norm / 2);
    }
    Eigen::Quaterniond temp2(std::cos(dphi_norm / 2), dphi_dir[0], dphi_dir[1], dphi_dir[2]);
    state_.pose = temp2 * state_.pose;

    state_.ba = state_.ba - error_state_.x.block<3, 1>(9, 0);
    state_.bg = state_.bg - error_state_.x.block<3, 1>(12, 0);    

    Eigen::Matrix4f odom_result = Eigen::Matrix4f::Identity();
    odom_result.block<3, 3>(0, 0) = state_.pose.toRotationMatrix().cast<float>();
    odom_result.block<3, 1>(0, 3) = state_.position.cast<float>();

    odom_result = odom_result * lidar_to_imu_.inverse();

    std::cout << " odom_result  = " << std::endl << odom_result << std::endl;
    kalman_filter_odom = odom_result;


    error_state_.x.setZero();

    return true;
    
}

bool KalmanFilterFlow::CorrectProcess()
{

    Eigen::Matrix4d predicted_pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4f laser_odometry = Eigen::Matrix4f::Identity();
    // matching_ptr_->Update(current_cloud_data_, laser_odometry);

    matching_odom = odom_data_buff_.front().pose.cast<float>();
    laser_odometry = matching_odom;
   
    predicted_pose.block<3, 1>(0, 3) = state_.position + error_state_.x.block<3, 1>(0, 0);
    // predicted_pose(2, 3) = 0.0;

    Eigen::Vector3d error_q = error_state_.x.block<3, 1>(6, 0);
    double error_q_norm = error_q.norm();
    if(error_q_norm != 0)
    {
        error_q = error_q / error_q_norm;
        error_q = error_q * std::sin(error_q_norm / 2);
    }
    Eigen::Quaterniond temp_q(std::cos(error_q_norm / 2), error_q[0], error_q[1], error_q[2]);
    predicted_pose.block<3, 3>(0, 0) = ((state_.pose * temp_q).normalized()).toRotationMatrix();

    Eigen::Matrix<double, 6, 1> Yk;
    Yk.block<3, 1>(0, 0) = predicted_pose.block<3, 1>(0, 3).cast<double>() - laser_odometry.block<3, 1>(0, 3).cast<double>();
    Eigen::Matrix3d delta_R = predicted_pose.block<3, 3>(0, 0) * laser_odometry.block<3, 3>(0, 0).cast<double>().transpose();
    delta_R = delta_R - Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Vector3d delta_pose(delta_R(1, 2), delta_R(2, 0), delta_R(0, 1));
    Yk(3, 0) = delta_pose[0];
    Yk(4, 0) = delta_pose[1];
    Yk(5, 0) = delta_pose[2];

    Eigen::Matrix<double, 6, 15> Gk = Eigen::Matrix<double, 6, 15>::Zero();
    Gk.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    Gk.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 6, 6> Ck = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 6> Rk = Eigen::Matrix<double, 6, 6>::Identity();
    Rk.block<3, 3>(0, 0) = laser_position_noise_ * laser_position_noise_ *  Eigen::Matrix3d::Identity();
    Rk.block<3, 3>(3, 3) = laser_position_noise_ * laser_position_noise_ * Eigen::Matrix3d::Identity();
    // Rk.block<3, 3>(0, 0) *= acc_noise_;
    // Rk.block<3, 3>(3, 3) *= gyro_noise_;

    const Eigen::MatrixXd Kk = error_state_.p * Gk.transpose() * (Gk * error_state_.p * Gk.transpose() + Ck * Rk * Ck.transpose()).inverse();

    error_state_.p = (Eigen::Matrix<double, 15, 15>::Identity() - Kk * Gk) * error_state_.p;
    error_state_.x = error_state_.x + Kk * (Yk - Gk * error_state_.x);
    // error_state_.x(2, 0) = 0;
    // error_state_.x(5, 0) = 0;

    state_.position = state_.position - error_state_.x.block<3, 1>(0, 0);
    state_.velocity = state_.velocity - error_state_.x.block<3, 1>(3, 0);
    Eigen::Vector3d dphi_dir = error_state_.x.block<3, 1>(6, 0);
    double dphi_norm = dphi_dir.norm();

    if(dphi_norm != 0)
    {
        dphi_dir = dphi_dir / dphi_norm;
        dphi_dir = dphi_dir * std::sin(dphi_norm / 2);
    }
    Eigen::Quaterniond temp2(std::cos(dphi_norm / 2), dphi_dir[0], dphi_dir[1], dphi_dir[2]);
    state_.pose = temp2 * state_.pose;

    state_.ba = state_.ba - error_state_.x.block<3, 1>(9, 0);
    state_.bg = state_.bg - error_state_.x.block<3, 1>(12, 0);    

    Eigen::Matrix4f odom_result = Eigen::Matrix4f::Identity();
    odom_result.block<3, 3>(0, 0) = state_.pose.toRotationMatrix().cast<float>();
    odom_result.block<3, 1>(0, 3) = state_.position.cast<float>();

    odom_result = odom_result * lidar_to_imu_.inverse();

    std::cout << " odom_result  = " << std::endl << odom_result << std::endl;
    kalman_filter_odom = odom_result;


    error_state_.x.setZero();

    return true;
}



bool KalmanFilterFlow::PublishData()
{
    
    odom_publish_ptr_->Publish(kalman_filter_odom, current_cloud_data_.time);
    matching_odom_publish_ptr_->Publish(matching_odom, current_cloud_data_.time);
    encoder_odom_pub_ptr_->Publish(encoder_odom, current_cloud_data_.time);

    CloudData::CloudPointTPtr global_map;
    CloudData::CloudPointTPtr local_map;
    matching_ptr_->GetGlobalMap(global_map);
    local_map = matching_ptr_->GetLocalMap();

    global_map_pub_ptr_->Publish(global_map);
    local_map_pub_ptr_->Publish(local_map);

    return true;
}



}