/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-18 19:38:12
 * @LastEditors: luo
 * @LastEditTime: 2021-11-19 17:42:14
 */
#include "kalman_filter_pkg/kalman_filter/kalman_filter.hpp"


namespace KalmanFilter
{

KalmanFilterProcess::KalmanFilterProcess()
{

    acc_noise_ = 1e-2;
    gyro_noise_ = 1e-4;

}
KalmanFilterProcess::~KalmanFilterProcess()
{


}

bool KalmanFilterProcess::SetInitPose(const Eigen::Matrix4f& init_pose)
{
    has_inited_ = true;
    
    state_.position = Eigen::Vector3d(init_pose_(0, 3), init_pose_(1, 3), init_pose_(2, 3));
    state_.velocity = Eigen::Vector3d::Zero();
    state_.pose = Eigen::Quaterniond(init_pose_.block<3, 3>(0, 0).cast<double>());
    state_.ba.setZero();
    state_.bg.setZero();

    error_state_.x.setZero();
    error_state_.p.setZero();
    error_state_.p.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-2;
    error_state_.p.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-3;
    error_state_.p.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 1e-4;
    error_state_.p.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 1e-4;
    error_state_.p.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 1e-6;

    return true;
    
}

bool KalmanFilterProcess::Predict(const std::deque<IMUData>& deque_imu_data)
{
    Eigen::Vector3d pp = state_.position;
    Eigen::Vector3d vv = state_.velocity;
    Eigen::Quaterniond qq = state_.pose;

    double w = 7.27220521664304e-05;//地球自转
    double L = 48.9825093568 * kDegreeToRatian;
    Eigen::Vector3d gn(0, 0, -9.809432933396721);
    // Eigen::Vector3d w_ie_n(0, w * std::cos())

    State last_state = state_;
    for(int i = 1; i < deque_imu_data.size(); ++i)
    {
        double dt = deque_imu_data[i].time - deque_imu_data[i-1].time;
        double dt_2 = dt * dt;

        if(dt < 0.f)
        {
            std::cout << "time diff = " << dt << std::endl;

            return false;
        }
        Eigen::Vector3d wb;
        wb[0] = 0.5 * (deque_imu_data[i].angular_vel.x + deque_imu_data[i - 1].angular_vel.x);
        wb[1] = 0.5 * (deque_imu_data[i].angular_vel.y + deque_imu_data[i - 1].angular_vel.y);
        wb[2] = 0.5 * (deque_imu_data[i].angular_vel.z + deque_imu_data[i - 1].angular_vel.z);
        wb = wb + state_.bg;
        wb = wb * dt;

        double angle = wb.norm();
        Eigen::Quaterniond qb(1, 0, 0, 0);
        qb = Eigen::Quaterniond(std::cos(angle / 2), wb[0], wb[1], wb[2]);

        Eigen::Quaterniond qq2 = qq * qb;
        Eigen::Vector3d f1(deque_imu_data[i-1].linear_acc.x,
                            deque_imu_data[i-1].linear_acc.y,
                            deque_imu_data[i-1].linear_acc.z
                            );
        f1 = f1 + state_.ba;

        Eigen::Vector3d f2(deque_imu_data[i].linear_acc.x,
                            deque_imu_data[i].linear_acc.y,
                            deque_imu_data[i].linear_acc.z
                            );
        f2 = f2 + state_.ba;

        Eigen::Vector3d vv2 = vv + dt * (0.5 * (qq * f1 + qq2 * f2) + gn);
        Eigen::Vector3d pp2 = pp + 0.5 * dt * (vv + vv2);

        pp = pp2;
        vv = vv2;
        qq = qq2;

    }

    state_.position = pp;
    state_.velocity = vv;
    state_.pose = qq;

    Eigen::Matrix<double, 15, 15> Ft = Eigen::Matrix<double, 15, 15>::Zero();
    Ft.block<3, 3>(0, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 3, 3> temp = Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Vector3d ff(deque_imu_data.back().linear_acc.x, deque_imu_data.back().linear_acc.y, deque_imu_data.back().linear_acc.z);

    ff = qq * ff;
    temp(0, 1) = -ff[2];
    temp(0, 2) = ff[1];
    temp(1, 0) = ff[2];
    temp(1, 2) = -ff[0];
    temp(2, 0) = -ff[1];
    temp(2, 1) = ff[0];
    Ft.block<3, 3>(3, 6) = temp;
    Ft.block<3, 3>(3, 12) = qq.toRotationMatrix();

    temp.setZero();
    temp(0, 1) = w * std::sin(L);
    temp(0, 2) = -w * std::cos(L);
    temp(1, 0) = -w * std::sin(L);
    temp(2, 0) = w * std::cos(L);
    Ft.block<3, 3>(6, 6) = temp;
    Ft.block<3, 3>(6, 9) = -Ft.block<3, 3>(3, 12);
    Eigen::Matrix<double, 15, 6> Bt = Eigen::Matrix<double, 15, 6>::Zero();
    Bt.block<3, 3>(3, 3) = Ft.block<3, 3>(3, 12);
    Bt.block<3, 3>(6, 0) = Ft.block<3, 3>(6, 9);

    double T = deque_imu_data.back().time - deque_imu_data.back().time;
    Ft = Eigen::Matrix<double, 15, 15>::Identity() + Ft * T;
    Bt = Bt * T;

    Eigen::Matrix<double, 6, 1> ww = Eigen::Matrix<double, 6, 1>::Zero();
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::normal_distribution<double> distribution(0.0, 1.0);
    Eigen::Vector3d noise_gyro(distribution(generator), distribution(generator), distribution(generator));
    Eigen::Vector3d noise_acc(distribution(generator), distribution(generator), distribution(generator));

    noise_gyro = noise_gyro * gyro_noise_;
    noise_acc = noise_acc * acc_noise_;
    ww.head(3) = noise_gyro;
    ww.tail(3) = noise_acc;

    error_state_.x = Ft * error_state_.x + Bt * ww;
    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Identity();
    Q.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity() * gyro_noise_ * gyro_noise_;
    Q.block<3, 3>(3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * acc_noise_ * acc_noise_;

    error_state_.p = Ft * error_state_.p * Ft.transpose() + Bt * Q * Bt.transpose();

    return true;

}

bool KalmanFilterProcess::Correct()
{
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = state_.pose.toRotationMatrix().cast<float>();
    pose.block<3, 1>(0, 3) = state_.position.cast<float>();
    

}


}