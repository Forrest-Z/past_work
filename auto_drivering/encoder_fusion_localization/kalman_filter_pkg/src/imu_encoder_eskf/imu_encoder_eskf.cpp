/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-26 14:51:14
 * @LastEditors: luo
 * @LastEditTime: 2022-01-11 23:06:39
 */
#include "kalman_filter_pkg/imu_encoder_eskf/imu_encoder_eskf.hpp"

namespace KalmanFilter
{

ImuEncoderESKF::ImuEncoderESKF(ros::NodeHandle& nh)
:nh_(nh)
{

    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh_, "/imu/data_xsens", 100000);
    
    encoder_sub_ptr_ = std::make_shared<EncoderSubscriber>(nh_, "/IMCU_INFO", 100000);

    fusion_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh_, "/fusion_pose", "/map", "/fusion_pose", 100);



    gyro_noise_ = 0.01;
    acc_noise_ = 0.28;
    gyro_bias_noise_ = 0.0000001;
    acc_bias_noise_ = 0.02;

    encoder_position_noise_ = 0.005;
    v_noise_ = 0.01;


}


ImuEncoderESKF::~ImuEncoderESKF()
{


}

bool ImuEncoderESKF::Run()
{
    if(!ReadData())
        return false;

    if(!Calibration())
        return false;

    if(!HasData())
        return false;

    if(!ValidData())
        return false;
    
    if(!SetInitPose())
        return false;

    Filter();

    PublishData();

    imu_data_buff_.clear();
    encoder_data_buff_.clear();

}

bool ImuEncoderESKF::ReadData()
{
    encoder_sub_ptr_->ParseData(encoder_data_buff_);

    if(encoder_data_buff_.size() == 0)
        return false;

    double synce_time = encoder_data_buff_.front().time;


    // std::deque<IMUData> unsynce_imu_data;
    // imu_sub_ptr_->ParseData(unsynce_imu_data);
    imu_sub_ptr_->ParseData(imu_data_buff_);


    // std::cout << " imu_time = " << imu_data_buff_.front().time<< std::endl;
    // std::cout << " encoder_time = " << encoder_data_buff_.front().time<< std::endl;

    // std::cout << " imu_size = " << imu_data_buff_.size() << std::endl;
    // std::cout << " encoder_size = " << encoder_data_buff_.size() << std::endl;

    // bool synce_imu = IMUData::SyncData(unsynce_imu_data, imu_data_buff_, synce_time);

    // std::cout << " synce_imu = " << synce_imu<< std::endl;

    // static bool sensor_init = false;
    
    // if(!sensor_init)
    // {
    //     if(!synce_imu)
    //     {
    //         encoder_data_buff_.pop_front();
    //         return false;
    //     }

    //     sensor_init = true;
    // }

    // std::cout << " imu_size = " << imu_data_buff_.size() << std::endl;
    // std::cout << " encoder_size = " << encoder_data_buff_.size() << std::endl;
    // std::cout << " sensor_init = " << sensor_init << std::endl;
    // return sensor_init;

    return true;
}

bool ImuEncoderESKF::Calibration()
{
    static bool has_calibration = false;

    if(!has_calibration)
    {
        encoder_to_imu_ = Eigen::Matrix4f::Identity();

        encoder_to_imu_(0, 3) = 0.5;
        encoder_to_imu_(2, 3) = 2.5;

        // state_.position = Eigen::Vector3d(init_pose_(0, 3), init_pose_(1, 3), init_pose_(2, 3));
        // state_.velocity.setZero();
        // Eigen::Quaterniond q(init_pose_.block<3, 3>(0, 0).cast<double>());
        // state_.pose = q;
        // state_.ba.setZero();
        // state_.bg.setZero();

        // error_state_.x.setZero();
        // error_state_.p.setZero();

        has_calibration = true;
    }
    // std::cout << " Calibration 11" << std::endl;
    return has_calibration;
}

bool ImuEncoderESKF::HasData()
{
    if(imu_data_buff_.size() == 0)
        return false;

    if(encoder_data_buff_.size() == 0)
        return false;

    // std::cout << " HasData 11" << std::endl;
    return true;
}

bool ImuEncoderESKF::ValidData()
{
    current_encoder_data_ = encoder_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();

    double diff_time = current_encoder_data_.time - current_imu_data_.time;

    if(diff_time < -0.005)
    {
        encoder_data_buff_.pop_front();
        return false;
    }

    if(diff_time > 0.005)
    {
        imu_data_buff_.pop_front();
        return false;
    }

    encoder_data_buff_.pop_front();
    imu_data_buff_.pop_front();

    // std::cout << " ValidData 11" << std::endl;
    return true;

}

bool ImuEncoderESKF::SetInitPose()
{
     static bool pose_inited = false;
    // std::cout << "SetInitPose " << std::endl;
    if(pose_inited)
    {
        return true;
    }

    // std::cout << "SetInitPose 00" << std::endl;
 
    // std::cout << "SetInitPose 11" << std::endl;
    if(!pose_inited )
    {
        pose_inited = true;

        has_inited = true;

        init_pose_ = Eigen::Matrix4f::Identity();
        
        // matching_ptr_->SetInitPose(init_pose_);
        // kalman_filter_process_ptr_->SetInitPose(init_pose_);

        position_x = init_pose_(0, 3);
        position_y = init_pose_(1, 3);
        
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

    // std::cout << "SetInitPose 33" << std::endl;

    return pose_inited;
    
}

bool ImuEncoderESKF::Filter()
{

    std::cout << " encoder_data_buff_ size = " << encoder_data_buff_.size() << std::endl;
    std::cout << " imu_data_buff_ size = " << imu_data_buff_.size() << std::endl;

    if (!Predict())
        return false;

    Correct();

    PublishData();


}


bool ImuEncoderESKF::CalculateEncoderPosition(EncoderData& encoder_msg, IMUData& imu_msg, double dt)
{
    encoder_msg.GetStatus();
    encoder_msg.CalculateNLeft();
    encoder_msg.CalculateNRight();
    encoder_msg.CalculateVelo();

    std::cout<< "velo_left = " << encoder_msg.velo_left << std::endl;
    std::cout<< "velo_right = " << encoder_msg.velo_right << std::endl;
    std::cout<< "dt  = " << dt << std::endl;

    double delta_left_p = encoder_msg.velo_left * dt;
    double delta_right_p = encoder_msg.velo_right * dt;

    // std::cout<< "delta_left_p = " << delta_left_p << std::endl;
    // std::cout<< "delta_right_p = " << delta_right_p << std::endl;

    double delta_s = 0.5 * ( delta_left_p + delta_right_p);
    double delta_th = encoder_msg.w_encoder * dt;

    double imu_z = imu_msg.angular_vel.z * dt;
    pose_yaw += imu_z;
    // pose_yaw += delta_th;

    encoder_velo += encoder_msg.v_encoder;

    position_x += delta_s * std::cos(pose_yaw);
    position_y += delta_s * std::sin(pose_yaw);
    double z = 0.0;

    // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_yaw);
    // Eigen::Quaterniond qua;
    // qua.x() = odom_quat.x;
    // qua.y() = odom_quat.y;
    // qua.z() = odom_quat.z;
    // qua.w() = odom_quat.w;

    // Eigen::Matrix3d r_mat = qua.toRotationMatrix();


    // std::cout<< "position_x = " << position_x << std::endl;
    // std::cout<< "position_y = " << position_y << std::endl;
    // std::cout<< "pose_yaw = " << pose_yaw << std::endl;
    std::cout<< "-----------v------------  = " << encoder_msg.v_encoder << std::endl;

}



bool ImuEncoderESKF::Predict()
{
    std::cout << "Predict Predict" << std::endl;
    // static bool is_first = false;

    if(!has_last_imu_data_)
    {
        last_imu_data_ = current_imu_data_;
        has_last_imu_data_ = true;

        return  false;
    }

    if(imu_data_buff_.size() == 0)
    {
        return false;
    }

    if(encoder_data_buff_.size() == 0)
    {
        return false;
    }


    // std::cout << "Predict " << std::endl;
    Eigen::Vector3d g(0.0, 0.0, -9.809432933396721);

    Eigen::Vector3d position = state_.position;
    Eigen::Vector3d velocity = state_.velocity;
    Eigen::Quaterniond pose = state_.pose;
    Eigen::Vector3d w;
    double encoder_vv = 0.0;

    // std::cout << "Predict 0000" << std::endl;

    int data_size = imu_data_buff_.size() > encoder_data_buff_.size() ? encoder_data_buff_.size() : imu_data_buff_.size();

    // if(data_size == 1)
    // {
    //     double dt = current_imu_data_.time - last_imu_data_.time;

    //     CalculateEncoderPosition(current_encoder_data_, current_imu_data_, dt);
        
    //     encoder_vv += current_encoder_data_.v_encoder;
    //     // std::cout << "Predict 11 dt " << dt << std::endl;
    //     //pose

    //     w[0] = 0.5 * ( current_imu_data_.angular_vel.x + last_imu_data_.angular_vel.x);
    //     w[1] = 0.5 * ( current_imu_data_.angular_vel.y + last_imu_data_.angular_vel.y);
    //     w[2] = 0.5 * ( current_imu_data_.angular_vel.z + last_imu_data_.angular_vel.z);
    //     w = w - state_.bg;
    //     w = w * dt;
    //     double angle_delta = w.norm();
    //     Eigen::Vector3d angle_delta_dir = w.normalized();

    //     double angle_delta_cos = std::cos(angle_delta / 2.0);
    //     double angle_delta_sin = std::sin(angle_delta / 2.0);

    //     Eigen::Quaterniond dq( angle_delta_cos,
    //                             angle_delta_sin * angle_delta_dir.x(),
    //                             angle_delta_sin * angle_delta_dir.y(),
    //                             angle_delta_sin * angle_delta_dir.z());

    //     Eigen::Quaterniond predict_pose = (pose * dq).normalized();

    //     // std::cout << "Predict pose " << std::endl;
    //     //velocity
    //     Eigen::Vector3d curr_acc(current_imu_data_.linear_acc.x, current_imu_data_.linear_acc.y, current_imu_data_.linear_acc.z);
    //     curr_acc = curr_acc - state_.ba;

    //     Eigen::Vector3d last_acc(last_imu_data_.linear_acc.x, last_imu_data_.linear_acc.y, last_imu_data_.linear_acc.z);
    //     last_acc = last_acc - state_.ba;

    //     // Eigen::Vector3d predict_velocity = position + dt * ( 0.5 * (pose * last_acc + predict_pose * curr_acc ) + g); 
    //     Eigen::Vector3d predict_velocity = position + velocity * dt + 0.5 * ( pose.toRotationMatrix() * curr_acc + g) * dt * dt;

    //     // std::cout << "Predict position " << std::endl;
    //     //position
    //     // Eigen::Vector3d predict_position = velocity + 0.5 * dt * ( velocity + predict_velocity);
    //     Eigen::Vector3d predict_position = velocity + ( pose.toRotationMatrix() * curr_acc + g) * dt;

    //     // last_imu_data_ = current_imu_data_;

    //     position = predict_position;
    //     pose = predict_pose;
    //     velocity = predict_velocity;

    // }else 
    // {
        for(int i = 1; i < data_size; i++)
        {
            ///姿态解算
            // std::cout << "Predict 00" << std::endl;
            double dt = imu_data_buff_[i].time - imu_data_buff_[i - 1].time;

            CalculateEncoderPosition(encoder_data_buff_[i], imu_data_buff_[i], dt);
            
            encoder_vv += encoder_data_buff_[i].v_encoder;
            // std::cout << "Predict 11 dt " << dt << std::endl;
            //pose
    
            w[0] = 0.5 * ( imu_data_buff_[i].angular_vel.x + imu_data_buff_[i-1].angular_vel.x);
            w[1] = 0.5 * ( imu_data_buff_[i].angular_vel.y + imu_data_buff_[i-1].angular_vel.y);
            w[2] = 0.5 * ( imu_data_buff_[i].angular_vel.z + imu_data_buff_[i-1].angular_vel.z);
            w = w - state_.bg;
            w = w * dt;
            double angle_delta = w.norm();
            Eigen::Vector3d angle_delta_dir = w.normalized();

            double angle_delta_cos = std::cos(angle_delta / 2.0);
            double angle_delta_sin = std::sin(angle_delta / 2.0);

            Eigen::Quaterniond dq( angle_delta_cos,
                                    angle_delta_sin * angle_delta_dir.x(),
                                    angle_delta_sin * angle_delta_dir.y(),
                                    angle_delta_sin * angle_delta_dir.z());

            Eigen::Quaterniond predict_pose = (pose * dq).normalized();

            // std::cout << "Predict pose " << std::endl;
            //velocity
            Eigen::Vector3d curr_acc(imu_data_buff_[i].linear_acc.x, imu_data_buff_[i].linear_acc.y, imu_data_buff_[i].linear_acc.z);
            curr_acc = curr_acc - state_.ba;

            Eigen::Vector3d last_acc(imu_data_buff_[i-1].linear_acc.x, imu_data_buff_[i-1].linear_acc.y, imu_data_buff_[i-1].linear_acc.z);
            last_acc = last_acc - state_.ba;

            Eigen::Vector3d predict_velocity = position + dt * ( 0.5 * (pose * last_acc + predict_pose * curr_acc ) + g); 
            // Eigen::Vector3d predict_velocity = position + velocity * dt + 0.5 * ( pose.toRotationMatrix() * curr_acc + g) * dt * dt;

            // std::cout << "Predict position " << std::endl;
            //position
            Eigen::Vector3d predict_position = velocity + 0.5 * dt * ( velocity + predict_velocity);
            // Eigen::Vector3d predict_position = velocity + ( pose.toRotationMatrix() * curr_acc + g) * dt;

            // last_imu_data_ = current_imu_data_;

            position = predict_position;
            pose = predict_pose;
            velocity = predict_velocity;
        }
    // }

    position_x = position_x / data_size;
    position_y = position_y / data_size;

    encoder_velo = encoder_vv / data_size;

    state_.position = position;
    state_.pose = pose;
    state_.velocity = velocity;


    last_imu_data_ = current_imu_data_;
    // std::cout << "Predict p " << std::endl;
    Eigen::Matrix<double, 15, 15> Ft = Eigen::Matrix<double, 15, 15>::Zero();


    //p
    Ft.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(); 

    // std::cout << "Predict v " << std::endl;
    //v
    Eigen::Vector3d acc_back(imu_data_buff_.back().linear_acc.x, imu_data_buff_.back().linear_acc.y, imu_data_buff_.back().linear_acc.z);
    acc_back = acc_back - state_.ba;
    Eigen::Matrix3d temp = Eigen::Matrix3d::Zero();
    temp = GetSkewMatrix(acc_back);
    temp = -state_.pose.toRotationMatrix() * temp;
    Ft.block<3, 3>(3, 6) = temp;
    Ft.block<3, 3>(3, 9) = -state_.pose.toRotationMatrix();

    // std::cout << "Predict q " << std::endl;
    //q
    Eigen::Vector3d w_angle(imu_data_buff_.back().angular_vel.x, imu_data_buff_.back().angular_vel.y, imu_data_buff_.back().angular_vel.z);
    w_angle = w_angle - state_.bg;
    Eigen::Matrix3d angle_temp = Eigen::Matrix3d::Zero();
    angle_temp = GetSkewMatrix(w_angle);
    Ft.block<3, 3>(6, 6) = -angle_temp;
    Ft.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity();

    double T = imu_data_buff_.back().time - imu_data_buff_.front().time;
    Ft = Eigen::Matrix<double, 15, 15>::Identity() + Ft * T;

    // std::cout << "Predict bt " << std::endl;
    Eigen::Matrix<double, 15, 12> Bt = Eigen::Matrix<double, 15, 12>::Zero();
    Bt.block<3, 3>(3, 0) = pose.toRotationMatrix();
    Bt.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();
    Bt.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();
    Bt.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();

    Bt = Bt * T;

    // std::cout << "Predict ww " << std::endl;
    Eigen::Matrix<double, 12, 1> ww = Eigen::Matrix<double, 12, 1>::Zero();

    Eigen::Vector3d ba_noise;
    ba_noise << acc_noise_, acc_noise_, acc_noise_;

    Eigen::Vector3d bg_noise;
    bg_noise << gyro_noise_, gyro_noise_, gyro_noise_;

    Eigen::Vector3d acc_bias_noise;
    acc_bias_noise << acc_bias_noise_, acc_bias_noise_, acc_bias_noise_;

    Eigen::Vector3d gyro_bias_noise;
    gyro_bias_noise << gyro_bias_noise_, gyro_bias_noise_, gyro_bias_noise_;

    ww.block<3, 1>(0, 0) = ba_noise;
    ww.block<3, 1>(3, 0) = bg_noise;
    ww.block<3, 1>(6, 0) = acc_bias_noise;
    ww.block<3, 1>(9, 0) = gyro_bias_noise;

    error_state_.x = Ft * error_state_.x + Bt * ww;

    // std::cout << "Predict Qt " << std::endl;
    Eigen::Matrix<double, 12, 12> Qt = Eigen::Matrix<double, 12, 12>::Identity();
    Qt.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity() * acc_noise_ * acc_noise_;
    Qt.block<3, 3>(3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * gyro_noise_ * gyro_noise_;
    Qt.block<3, 3>(6, 6) = Eigen::Matrix<double, 3, 3>::Identity() * acc_bias_noise_ * acc_bias_noise_;
    Qt.block<3, 3>(9, 9) = Eigen::Matrix<double, 3, 3>::Identity() * gyro_bias_noise_ * gyro_bias_noise_;

    error_state_.p = Ft * error_state_.p * Ft.transpose() + Bt * Qt * Bt.transpose();
    // std::cout << "Predict end " << std::endl;
    return true;

}

bool ImuEncoderESKF::Correct()
{

    // std::cout << "Correct 00 " << std::endl;
    Eigen::Matrix4d pose_from_imu = Eigen::Matrix4d::Identity();
    pose_from_imu.block<3, 1>(0, 3) = state_.position + error_state_.x.block<3, 1>(0, 0);

    // std::cout << "Correct 11 " << std::endl;
    Eigen::Vector3d error_q = error_state_.x.block<3, 1>(6, 0);
    double error_q_norm = error_q.norm();
    if(error_q_norm != 0)
    {
        error_q = error_q / error_q_norm;
        error_q = error_q * std::sin(error_q_norm / 2);
    }
    Eigen::Quaterniond temp_q(std::cos(error_q_norm / 2), error_q[0], error_q[1], error_q[2]);
    pose_from_imu.block<3, 3>(0, 0) = ((state_.pose * temp_q).normalized()).toRotationMatrix();

    Eigen::Vector3d p_mean(position_x, position_y, 0);
    
    // std::cout << "Correct 11 " << std::endl;
    Eigen::Matrix<double, 9, 1> Yk;
    // Yk.block<3, 1>(0, 3) = p_mean - pose_from_imu.block<3, 1>(0, 3);
    Yk(0, 0) = p_mean[0] - pose_from_imu(0, 3);
    Yk(1, 0) = p_mean[1] - pose_from_imu(1, 3);
    Yk(2, 0) = 0.0;
// std::cout << "Correct 2222---------1 " << std::endl;
    geometry_msgs::Quaternion encoder_yaw_q = tf::createQuaternionMsgFromYaw(pose_yaw);
    Eigen::Quaterniond encoder_q;
    encoder_q.w() = encoder_yaw_q.w;
    encoder_q.x() = encoder_yaw_q.x;
    encoder_q.y() = encoder_yaw_q.y;
    encoder_q.z() = encoder_yaw_q.z;

    // std::cout << "Correct 22222-----2 " << std::endl;
    Eigen::Matrix<double, 3, 3> encoder_matrix = encoder_q.toRotationMatrix();
    Eigen::Matrix<double, 3, 3> delta_matrix = pose_from_imu.block<3, 3>(0, 0) * encoder_matrix.transpose();
    delta_matrix = delta_matrix - Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Vector3d delta_angle(delta_matrix(1, 2), delta_matrix(2, 0), delta_matrix(0, 1));
    Yk(6, 0) = delta_angle[0];
    Yk(7, 0) = delta_angle[1];
    Yk(8, 0) = delta_angle[2];

    // std::cout << "Correct 22 " << std::endl;
    Eigen::Vector3d encoder_v(encoder_velo, 0.0, 0.0);
    Eigen::Vector3d delta_velo = state_.velocity - encoder_to_imu_.block<3, 3>(0, 0).cast<double>() * encoder_v;

    // std::cout << "Correct 22222222 " << std::endl;
    Yk.block<3, 1>(3, 0) = delta_velo;

    // std::cout << "Correct Gt " << std::endl;
    Eigen::Matrix<double, 9, 15> Gt = Eigen::Matrix<double, 9, 15>::Zero();
    Gt.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
    Gt.block<3, 3>(6, 6) = Eigen::Matrix<double, 3, 3>::Identity();
    Gt.block<3, 3>(3, 3) = encoder_to_imu_.block<3, 3>(0, 0).cast<double>();
    Eigen::Matrix3d v_matrix = GetSkewMatrix(encoder_v);
    Gt.block<3, 3>(3, 6) = v_matrix;
    
    // std::cout << "Correct 33 " << std::endl;
    Eigen::Matrix<double, 9, 9> Ct = Eigen::Matrix<double, 9, 9>::Identity();
    Eigen::Matrix<double, 9,  9> Rt = Eigen::Matrix<double, 9, 9>::Identity();
    Rt.block<3, 3>(0, 0) = encoder_position_noise_ * encoder_position_noise_ *  Eigen::Matrix3d::Identity();
    Rt.block<3, 3>(3, 3) = v_noise_ * v_noise_ * Eigen::Matrix3d::Identity();
    Rt.block<3, 3>(6, 6) = encoder_position_noise_ * encoder_position_noise_ * Eigen::Matrix3d::Identity();

    const Eigen::MatrixXd Kk = error_state_.p * Gt.transpose() * ( Gt * error_state_.p * Gt.transpose() + Ct * Rt * Ct.transpose()).inverse();

    error_state_.p = (Eigen::Matrix<double, 15, 15>::Identity() - Kk * Gt) * error_state_.p;
    error_state_.x = error_state_.x + Kk * ( Yk - Gt * error_state_.x);

    state_.position = state_.position - error_state_.x.block<3, 1>(0, 0);
    state_.velocity = state_.velocity - error_state_.x.block<3, 1>(3, 0);
    // std::cout << "Correct 44 " << std::endl;
    Eigen::Vector3d angle_dir = error_state_.x.block<3, 1>(6, 0);
    double angle_size = angle_dir.norm();

    if(angle_size != 0)
    {
        angle_dir = angle_dir / angle_size;
        angle_dir = angle_dir * std::sin(angle_size / 2.0);

    }
    
    // std::cout << "Correct 55 " << std::endl;
    Eigen::Quaterniond angle_qua(std::cos(angle_size / 2.0), angle_dir[0], angle_dir[1], angle_dir[2]);
    state_.pose = angle_qua * state_.pose;

    state_.ba = state_.ba - error_state_.x.block<3, 1>(9, 0);
    state_.bg = state_.bg - error_state_.x.block<3, 1>(12, 0);
    
    Eigen::Matrix4f odom_result = Eigen::Matrix4f::Identity();
    odom_result.block<3, 3>(0, 0) = state_.pose.toRotationMatrix().cast<float>();
    odom_result.block<3, 1>(0, 3) = state_.position.cast<float>();

    odom_result = odom_result * encoder_to_imu_.inverse();

    fusion_pose_ = odom_result;

    error_state_.x.setZero();
    // std::cout << "Correct 66 " << std::endl;
    return true;

}

bool ImuEncoderESKF::PublishData()
{
    fusion_odom_pub_ptr_->Publish(fusion_pose_, current_encoder_data_.time);

    return false;
   
}


}