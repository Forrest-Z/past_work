/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-22 10:19:53
 * @LastEditors: luo
 * @LastEditTime: 2022-01-18 15:38:39
 */
#include "kalman_filter_pkg/imu_pre_integrator/imu_pre_integrator.hpp"

namespace KalmanFilter
{

IMUPreIntegrator::IMUPreIntegrator(const YAML::Node& node)
{
    EARTH.GRAVITY_MAGNITUDE = node["earth"]["gravity_magnitude"].as<double>();
    COV.MEASUREMENT.ACCEL = node["covariance"]["measurement"]["accel"].as<double>();
    COV.MEASUREMENT.GYRO = node["covariance"]["measurement"]["gyro"].as<double>();
    COV.RANDOM_WALK.ACCEL = node["covariance"]["random_walk"]["accel"].as<double>();
    COV.RANDOM_WALK.GYRO =  node["covariance"]["random_walk"]["accel"].as<double>();
    // prompt:
    std::cout << std::endl 
              << "IMU Pre-Integration params:" << std::endl
              << "\tgravity magnitude: " << EARTH.GRAVITY_MAGNITUDE << std::endl
              << std::endl
              << "\tprocess noise:" << std::endl
              << "\t\tmeasurement:" << std::endl
              << "\t\t\taccel.: " << COV.MEASUREMENT.ACCEL << std::endl
              << "\t\t\tgyro.: " << COV.MEASUREMENT.GYRO << std::endl
              << "\t\trandom_walk:" << std::endl
              << "\t\t\taccel.: " << COV.RANDOM_WALK.ACCEL << std::endl
              << "\t\t\tgyro.: " << COV.RANDOM_WALK.GYRO << std::endl
              << std::endl;


    state.g = Eigen::Vector3d(0.0, 0.0, EARTH.GRAVITY_MAGNITUDE);

//  // b. process noise:
//     Q_.block<3, 3>(INDEX_M_ACC_PREV, INDEX_M_ACC_PREV) = Q_.block<3, 3>(INDEX_M_ACC_CURR, INDEX_M_ACC_CURR) = COV.MEASUREMENT.ACCEL * Eigen::Matrix3d::Identity();
//     Q_.block<3, 3>(INDEX_M_GYR_PREV, INDEX_M_GYR_PREV) = Q_.block<3, 3>(INDEX_M_GYR_CURR, INDEX_M_GYR_CURR) = COV.MEASUREMENT.GYRO * Eigen::Matrix3d::Identity();
//     Q_.block<3, 3>(INDEX_R_ACC_PREV, INDEX_R_ACC_PREV) = COV.RANDOM_WALK.ACCEL * Eigen::Matrix3d::Identity();
//     Q_.block<3, 3>(INDEX_R_GYR_PREV, INDEX_R_GYR_PREV) = COV.RANDOM_WALK.GYRO * Eigen::Matrix3d::Identity();

//     // c. process equation, state propagation:
//     F_.block<3, 3>(INDEX_ALPHA,  INDEX_BETA) =  Eigen::Matrix3d::Identity();
//     F_.block<3, 3>(INDEX_THETA,   INDEX_B_G) = -Eigen::Matrix3d::Identity();

//     // d. process equation, noise input:
//     B_.block<3, 3>(INDEX_THETA, INDEX_M_GYR_PREV) = B_.block<3, 3>(INDEX_THETA, INDEX_M_GYR_CURR) = 0.50 * Eigen::Matrix3d::Identity();
//     B_.block<3, 3>(INDEX_B_A, INDEX_R_ACC_PREV) = B_.block<3, 3>(INDEX_B_G, INDEX_R_GYR_PREV) = Eigen::Matrix3d::Identity();
  
    Q_.block<3, 3>(0, 0) = COV.MEASUREMENT.ACCEL * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(6, 6) = COV.MEASUREMENT.ACCEL * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(3, 3) = COV.MEASUREMENT.GYRO * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(9, 9) = COV.MEASUREMENT.GYRO * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(12, 12) = COV.RANDOM_WALK.ACCEL * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(15, 15) = COV.RANDOM_WALK.GYRO * Eigen::Matrix3d::Identity();

    F_.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
    F_.block<3, 3>(3, 12) = Eigen::Matrix3d::Identity();

    B_.block<3, 3>(3, 3) = 0.50 * Eigen::Matrix3d::Identity();
    B_.block<3, 3>(3, 9) =  0.50 * Eigen::Matrix3d::Identity();
    B_.block<3, 3>(9, 12) = Eigen::Matrix3d::Identity();
    B_.block<3, 3>(12, 15) = Eigen::Matrix3d::Identity();

}

IMUPreIntegrator::~IMUPreIntegrator()
{


}

bool IMUPreIntegrator::Init(const IMUData& init_imu_data)
{
    ResetState(init_imu_data);

    is_inited_ = true;

    return true;

}


bool IMUPreIntegrator::Update(const IMUData& imu_data)
{
    if(imu_data_buff_.front().time < imu_data.time)
    {
        imu_data_buff_.push_back(imu_data);

        UpdateState();

        imu_data_buff_.pop_front();
    }

    return true;

}



bool IMUPreIntegrator::Reset(const IMUData& init_imu_data, IMUPreIntrgration& imu_pre_integration)
{
    Update(init_imu_data);
    std::cout << "--------------------IMUPreIntegrator::Reset--init_imu_data.time_= " << init_imu_data.time << std::endl;
    std::cout << "--------------------IMUPreIntegrator::Reset time_= " << time_ << std::endl;

    imu_pre_integration.T = init_imu_data.time - time_;



    imu_pre_integration.g = state.g;

    imu_pre_integration.alpha_ij = state.alpha_ij;
    imu_pre_integration.theta_ij = state.theta_ij;
    imu_pre_integration.beta_ij = state.beta_ij;
    imu_pre_integration.b_a_i = state.b_a_i;
    imu_pre_integration.b_g_i = state.b_g_i;

    imu_pre_integration.P = P_;
    imu_pre_integration.J = J_;
    std::cout << "~~~~~~~~~~~~~MUPreIntegrator::---PPPPPPPPPPPP--111---" <<std::endl<< imu_pre_integration.P << std::endl;

    ResetState(init_imu_data);

    std::cout << "~~~~~~~~~~~~~~~~IMUPreIntegrator::---PPPPPPPPPPPP--222---"<<std::endl << imu_pre_integration.P << std::endl;

    return true;

}

void IMUPreIntegrator::ResetState(const IMUData& init_imu_data)
{
    time_ = init_imu_data.time;

    std::cout << "IMUPreIntegrator::ResetState----time = " << time_ << std::endl;

    state.alpha_ij = Eigen::Vector3d::Zero();

    state.theta_ij = Sophus::SO3d();

    state.beta_ij = Eigen::Vector3d::Zero();

    state.b_a_i = Eigen::Vector3d(init_imu_data.accel_bias.x, init_imu_data.accel_bias.y, init_imu_data.accel_bias.z);
    state.b_g_i = Eigen::Vector3d(init_imu_data.gyro_bias.x, init_imu_data.gyro_bias.y, init_imu_data.gyro_bias.z);

    P_ = MatrixP::Identity();
    // P_ = MatrixP::Zero();

    J_ = MatrixJ::Identity();

    imu_data_buff_.clear();

    imu_data_buff_.push_back(init_imu_data);

}


void IMUPreIntegrator::UpdateState()
{
    static double T = 0.0;

    static Eigen::Vector3d w_mid = Eigen::Vector3d::Zero();
    static Eigen::Vector3d a_mid = Eigen::Vector3d::Zero();

    static Sophus::SO3d prev_theta_ij = Sophus::SO3d();
    static Sophus::SO3d curr_theta_ij = Sophus::SO3d();
    static Sophus::SO3d d_theta_ij = Sophus::SO3d();


    static Eigen::Matrix3d dR_inv = Eigen::Matrix3d::Zero();
    static Eigen::Matrix3d prev_R = Eigen::Matrix3d::Zero();
    static Eigen::Matrix3d curr_R = Eigen::Matrix3d::Zero();
    static Eigen::Matrix3d prev_R_a_hat = Eigen::Matrix3d::Zero();
    static Eigen::Matrix3d curr_R_a_hat = Eigen::Matrix3d::Zero();

    const IMUData &pre_imu_data = imu_data_buff_.at(0);
    const IMUData &curr_imu_data = imu_data_buff_.at(1);

    T = curr_imu_data.time - pre_imu_data.time;

    std::cout << "----IMUPreIntegrator::UpdateState-----ttttttttttttttttt----------- " << T << std::endl;

    const Eigen::Vector3d prev_w(pre_imu_data.angular_vel.x - state.b_g_i.x(),
                                    pre_imu_data.angular_vel.y - state.b_g_i.y(),
                                    pre_imu_data.angular_vel.z - state.b_g_i.z());

    const Eigen::Vector3d curr_w(curr_imu_data.angular_vel.x - state.b_g_i.x(),
                                    curr_imu_data.angular_vel.y - state.b_g_i.y(),
                                    curr_imu_data.angular_vel.z - state.b_g_i.z());

    const Eigen::Vector3d prev_a(pre_imu_data.linear_acc.x - state.b_a_i.x(),
                                    pre_imu_data.linear_acc.y - state.b_a_i.y(),
                                    pre_imu_data.linear_acc.z - state.b_a_i.z());

    const Eigen::Vector3d curr_a(curr_imu_data.linear_acc.x - state.b_a_i.x(),
                                    curr_imu_data.linear_acc.y - state.b_a_i.y(),
                                    curr_imu_data.linear_acc.z - state.b_a_i.z());


    // //
    // // a. update mean:
    // //
    // // 1. get w_mid:
    // w_mid = 0.5 * ( prev_w + curr_w );
    // // 2. update relative orientation, so3:
    // prev_theta_ij = state.theta_ij;
    // d_theta_ij = Sophus::SO3d::exp(w_mid * T);
    // state.theta_ij = state.theta_ij * d_theta_ij;
    // curr_theta_ij = state.theta_ij;
    // // 3. get a_mid:
    // a_mid = 0.5 * ( prev_theta_ij * prev_a + curr_theta_ij * curr_a );
    // // 4. update relative translation:
    // state.alpha_ij += (state.beta_ij + 0.5 * a_mid * T) * T;
    // // 5. update relative velocity:
    // state.beta_ij += a_mid * T;

    // //
    // // b. update covariance:
    // //
    // // 1. intermediate results:
    // dR_inv = d_theta_ij.inverse().matrix();
    // prev_R = prev_theta_ij.matrix();
    // curr_R = curr_theta_ij.matrix();
    // prev_R_a_hat = prev_R * Sophus::SO3d::hat(prev_a);
    // curr_R_a_hat = curr_R * Sophus::SO3d::hat(curr_a);

    // //
    // // 2. set up F:
    // //  static const int DIM_NOISE = 18;

    // // static const int INDEX_ALPHA = 0;
    // // static const int INDEX_THETA = 3;
    // // static const int INDEX_BETA = 6;
    // // static const int INDEX_B_A = 9;
    // // static const int INDEX_B_G = 12;

    // // static const int INDEX_M_ACC_PREV = 0;
    // // static const int INDEX_M_GYR_PREV = 3;
    // // static const int INDEX_M_ACC_CURR = 6;
    // // static const int INDEX_M_GYR_CURR = 9;
    // // static const int INDEX_R_ACC_PREV = 12;
    // // static const int INDEX_R_GYR_PREV = 15;
    // //
    // // F12 & F32:
    // F_.block<3, 3>(INDEX_ALPHA, INDEX_THETA) = F_.block<3, 3>(INDEX_BETA, INDEX_THETA) = -0.50 * (prev_R_a_hat + curr_R_a_hat * dR_inv);
    // F_.block<3, 3>(INDEX_ALPHA, INDEX_THETA) = 0.50 * T * F_.block<3, 3>(INDEX_ALPHA, INDEX_THETA);
    // // F14 & F34:
    // F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_A) = F_.block<3, 3>(INDEX_BETA,   INDEX_B_A) = -0.50 * (prev_R + curr_R);
    // F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_A) = 0.50 * T * F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_A);
    // // F15 & F35:
    // F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_G) = F_.block<3, 3>(INDEX_BETA,   INDEX_B_G) = +0.50 * T * curr_R_a_hat;
    // F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_G) = 0.50 * T * F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_G);
    // // F22:
    // F_.block<3, 3>(INDEX_THETA, INDEX_THETA) = -Sophus::SO3d::hat(w_mid);

    // //
    // // 3. set up G:
    // //
    // // G11 & G31:
    // B_.block<3, 3>(INDEX_ALPHA, INDEX_M_ACC_PREV) = B_.block<3, 3>(INDEX_BETA, INDEX_M_ACC_PREV) = +0.50 * prev_R;
    // B_.block<3, 3>(INDEX_ALPHA, INDEX_M_ACC_PREV) = 0.50 * T * B_.block<3, 3>(INDEX_ALPHA, INDEX_M_ACC_PREV);
    // // G12 & G32:
    // B_.block<3, 3>(INDEX_ALPHA, INDEX_M_GYR_PREV) = B_.block<3, 3>(INDEX_BETA, INDEX_M_GYR_PREV) = -0.25 * T * curr_R_a_hat;
    // B_.block<3, 3>(INDEX_ALPHA, INDEX_M_GYR_PREV) = 0.50 * T * B_.block<3, 3>(INDEX_ALPHA, INDEX_M_GYR_PREV);
    // // G13 & G33:
    // B_.block<3, 3>(INDEX_ALPHA, INDEX_M_ACC_CURR) = B_.block<3, 3>(INDEX_BETA, INDEX_M_ACC_CURR) = 0.5 * curr_R;
    // B_.block<3, 3>(INDEX_ALPHA, INDEX_M_ACC_CURR) = 0.50 * T * B_.block<3, 3>(INDEX_ALPHA, INDEX_M_ACC_CURR);
    // // G14 & G34:
    // B_.block<3, 3>(INDEX_ALPHA, INDEX_M_GYR_CURR) = B_.block<3, 3>(INDEX_BETA, INDEX_M_GYR_CURR) = -0.25 * T * curr_R_a_hat;
    // B_.block<3, 3>(INDEX_ALPHA, INDEX_M_GYR_CURR) = 0.50 * T * B_.block<3, 3>(INDEX_ALPHA, INDEX_M_GYR_CURR);

    // // 4. update P_:
    // MatrixF F = MatrixF::Identity() + T * F_;
    // MatrixB B = T * B_;

    // P_ = F*P_*F.transpose() + B*Q_*B.transpose();

    // // 
    // // c. update Jacobian:
    // //
    // J_ = F * J_;


    // std::cout << "******************imu_integrator_J*********B******* = " << std::endl << B << std::endl;
    // std::cout << "******************imu_integrator_J*********F_******* = " << std::endl << F_ << std::endl;
    // std::cout << "******************imu_integrator_J*********P_******* = " << std::endl << P_ << std::endl;
    // std::cout << "******************imu_integrator_J*********J_******* = " << std::endl << J_ << std::endl;
    // // a. update mean:
    // // 1. get w_mid:
    // w_mid = 0.5 * ( prev_w + curr_w );
    // // 2. update relative orientation, so3:
    // prev_theta_ij = state.theta_ij;
    // d_theta_ij = Sophus::SO3d::exp(w_mid * T);
    // state.theta_ij = state.theta_ij * d_theta_ij;
    // curr_theta_ij = state.theta_ij;
    // // 3. get a_mid:
    // a_mid = 0.5 * ( prev_theta_ij * prev_a + curr_theta_ij * curr_a );
    // // 4. update relative translation:
    // state.alpha_ij += (state.beta_ij + 0.5 * a_mid * T) * T;
    // // 5. update relative velocity:
    // state.beta_ij += a_mid * T;

    // //
    // // b. update covariance:
    // //
    // // 1. intermediate results:
    // dR_inv = d_theta_ij.inverse().matrix();
    // prev_R = prev_theta_ij.matrix();
    // curr_R = curr_theta_ij.matrix();
    // prev_R_a_hat = prev_R * Sophus::SO3d::hat(prev_a);
    // curr_R_a_hat = curr_R * Sophus::SO3d::hat(curr_a);


    // // 2. set up F:
    // //
    // // F12 & F32:
    // F_.block<3, 3>(INDEX_ALPHA, INDEX_THETA) = F_.block<3, 3>(INDEX_BETA, INDEX_THETA) = -0.50 * (prev_R_a_hat + curr_R_a_hat * dR_inv);
    // F_.block<3, 3>(INDEX_ALPHA, INDEX_THETA) = 0.50 * T * F_.block<3, 3>(INDEX_ALPHA, INDEX_THETA);
    // // F14 & F34:
    // F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_A) = F_.block<3, 3>(INDEX_BETA,   INDEX_B_A) = -0.50 * (prev_R + curr_R);
    // F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_A) = 0.50 * T * F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_A);
    // // F15 & F35:
    // F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_G) = F_.block<3, 3>(INDEX_BETA,   INDEX_B_G) = +0.50 * T * curr_R_a_hat;
    // F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_G) = 0.50 * T * F_.block<3, 3>(INDEX_ALPHA,   INDEX_B_G);
    // // F22:
    // F_.block<3, 3>(INDEX_THETA, INDEX_THETA) = -Sophus::SO3d::hat(w_mid);

    // //
    // // 3. set up G:
    // //
    // // G11 & G31:
    // B_.block<3, 3>(INDEX_ALPHA, INDEX_M_ACC_PREV) = B_.block<3, 3>(INDEX_BETA, INDEX_M_ACC_PREV) = +0.50 * prev_R;
    // B_.block<3, 3>(INDEX_ALPHA, INDEX_M_ACC_PREV) = 0.50 * T * B_.block<3, 3>(INDEX_ALPHA, INDEX_M_ACC_PREV);
    // // G12 & G32:
    // B_.block<3, 3>(INDEX_ALPHA, INDEX_M_GYR_PREV) = B_.block<3, 3>(INDEX_BETA, INDEX_M_GYR_PREV) = -0.25 * T * curr_R_a_hat;
    // B_.block<3, 3>(INDEX_ALPHA, INDEX_M_GYR_PREV) = 0.50 * T * B_.block<3, 3>(INDEX_ALPHA, INDEX_M_GYR_PREV);
    // // G13 & G33:
    // B_.block<3, 3>(INDEX_ALPHA, INDEX_M_ACC_CURR) = B_.block<3, 3>(INDEX_BETA, INDEX_M_ACC_CURR) = 0.5 * curr_R;
    // B_.block<3, 3>(INDEX_ALPHA, INDEX_M_ACC_CURR) = 0.50 * T * B_.block<3, 3>(INDEX_ALPHA, INDEX_M_ACC_CURR);
    // // G14 & G34:
    // B_.block<3, 3>(INDEX_ALPHA, INDEX_M_GYR_CURR) = B_.block<3, 3>(INDEX_BETA, INDEX_M_GYR_CURR) = -0.25 * T * curr_R_a_hat;
    // B_.block<3, 3>(INDEX_ALPHA, INDEX_M_GYR_CURR) = 0.50 * T * B_.block<3, 3>(INDEX_ALPHA, INDEX_M_GYR_CURR);

    // // 4. update P_:
    // MatrixF F = MatrixF::Identity() + T * F_;
    // MatrixB B = T * B_;

    // std::cout << "******************imu_integrator_J*********F_******* = " << std::endl << F_ << std::endl;
    // P_ = F*P_*F.transpose() + B*Q_*B.transpose();
    // std::cout << "******************imu_integrator_J*********P_******* = " << std::endl << P_ << std::endl;
    // //     // std::cout << "******************imu_integrator_J*********J_******* = " << std::endl << J_ << std::endl;
    // // c. update Jacobian:
    // //
    // J_ = F * J_;


    // std::cout << "******************imu_integrator_J*********J_******* = " << std::endl << J_ << std::endl;
    // // std::cout << "******************imu_integrator_J**************** = " << std::endl << J_ << std::endl;
    // // std::cout << "******************imu_integrator_J**************** = " << std::endl; 
    // // std::cout << "******************imu_integrator_********theta_ij******** = " << state.theta_ij << std::endl;
    // // std::cout << "******************imu_integrator_J*******alpha_ij********* = " << state.alpha_ij << std::endl;
    // // std::cout << "******************imu_integrator_J*******beta_ij********* = " << state.beta_ij << std::endl;



    //预积分 公式13-17
    w_mid = 0.5 * (prev_w + curr_w);

    prev_theta_ij = state.theta_ij;
    d_theta_ij = Sophus::SO3d::exp(w_mid * T);
    state.theta_ij = state.theta_ij * d_theta_ij;
    curr_theta_ij = state.theta_ij;

    a_mid = 0.5 * (prev_theta_ij * prev_a + curr_theta_ij * curr_a);
    state.alpha_ij += (state.beta_ij + 0.5 * a_mid * T) * T;

    state.beta_ij += a_mid * T;

    dR_inv = d_theta_ij.inverse().matrix();
    prev_R = prev_theta_ij.matrix();
    curr_R = curr_theta_ij.matrix();
    prev_R_a_hat = prev_R * Sophus::SO3d::hat(prev_a);
    curr_R_a_hat = curr_R * Sophus::SO3d::hat(curr_a);

    // static const int INDEX_ALPHA = 0;
    // static const int INDEX_THETA = 3;
    // static const int INDEX_BETA = 6;
    // static const int INDEX_B_A = 9;
    // static const int INDEX_B_G = 12;

    // static const int INDEX_M_ACC_PREV = 0;
    // static const int INDEX_M_GYR_PREV = 3;
    // static const int INDEX_M_ACC_CURR = 6;
    // static const int INDEX_M_GYR_CURR = 9;
    // static const int INDEX_R_ACC_PREV = 12;
    // static const int INDEX_R_GYR_PREV = 15;
    

    //d_alpha
    F_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); 
    F_.block<3, 3>(0, 3) = -0.25 * T * T * (prev_R_a_hat + curr_R_a_hat * dR_inv);
    F_.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * T;
    F_.block<3, 3>(0, 9) = -0.25 * T * T * (prev_R + curr_R);
    F_.block<3, 3>(0, 12) =  0.25 * T * T * T * curr_R_a_hat;

    //d_theta
    F_.block<3, 3>(3, 3) = -Sophus::SO3d::hat(w_mid);
    F_.block<3, 3>(3, 12) = -Eigen::Matrix3d::Identity() * T;

    //d_beta
    F_.block<3, 3>(6, 3) = -0.50 * (prev_R_a_hat + curr_R_a_hat * dR_inv);
    F_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity(); 
    F_.block<3, 3>(6, 9) = -0.50 * (prev_R + curr_R);
    F_.block<3, 3>(6, 12) = 0.50 * T * curr_R_a_hat;

    F_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity(); 

    F_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity(); 

    //set up G
    B_.block<3, 3>(0, 0) = 0.25 * T * T * prev_R;
    B_.block<3, 3>(0, 3) = -0.125 * T * T * curr_R_a_hat;
    B_.block<3, 3>(0, 9) = 0.25 * T * curr_R_a_hat;
    B_.block<3, 3>(0, 12) = -0.125 * T * T * curr_R_a_hat;

    B_.block<3, 3>(3, 3) = 0.25 * Eigen::Matrix3d::Identity() * T;
    B_.block<3, 3>(3, 9) = 0.25 * Eigen::Matrix3d::Identity() * T;

    B_.block<3, 3>(6, 0) = 0.5 * T * prev_R;
    B_.block<3, 3>(6, 3) = -0.25 * T * curr_R_a_hat;
    B_.block<3, 3>(6, 6) = 0.5 * T * curr_R;
    B_.block<3, 3>(6, 9) = -0.25 * T * curr_R_a_hat;

    B_.block<3, 3>(9, 12) = Eigen::Matrix3d::Identity() * T;
    B_.block<3, 3>(12, 15) = Eigen::Matrix3d::Identity() * T;


    MatrixF F = MatrixF::Identity() + T * F_;

    MatrixB B = T * B_;

    P_ = F * P_ * F.transpose() + B * Q_ * B.transpose();


    J_ = F * J_;

    // std::cout << "******************imu_integrator_J*********B******* = " << std::endl << B << std::endl;
    // std::cout << "******************imu_integrator_J*********F_******* = " << std::endl << F_ << std::endl;
    std::cout << "******************imu_integrator_J*********P_******* = " << std::endl << P_ << std::endl;
    // std::cout << "******************imu_integrator_J*********J_******* = " << std::endl << J_ << std::endl;


}



}