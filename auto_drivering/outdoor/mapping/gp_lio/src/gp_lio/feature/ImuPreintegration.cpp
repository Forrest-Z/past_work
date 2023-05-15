//
// Created by wchen on 2019/12/3.
//

#include "gp_lio/feature/ImuPreintegration.h"

namespace gp_lio
{

ImuPreintegration::ImuPreintegration(const Eigen::Vector3d &acc_0, const Eigen::Vector3d &gyr_0,
                                     const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg)
    : acc_0_(acc_0), gyr_0_(gyr_0), linearized_acc_(acc_0), linearized_gyr_(gyr_0),
      linearized_ba_(linearized_ba), linearized_bg_(linearized_bg), sum_dt_(0.0),
      delta_p_(Eigen::Vector3d::Zero()), delta_q_(Eigen::Quaterniond::Identity()), delta_v_(Eigen::Vector3d::Zero()),
      jacobian_(Eigen::Matrix<double, 15, 15>::Identity()), covariance_(Eigen::Matrix<double, 15, 15>::Zero())
{

    noise_ = Eigen::Matrix<double, 18, 18>::Zero();
    noise_.block<3, 3>(0, 0) = (ACC_NOISE_STD * ACC_NOISE_STD) * Eigen::Matrix3d::Identity();
    noise_.block<3, 3>(3, 3) = (GYR_NOISE_STD * GYR_NOISE_STD) * Eigen::Matrix3d::Identity();
    noise_.block<3, 3>(6, 6) = (ACC_NOISE_STD * ACC_NOISE_STD) * Eigen::Matrix3d::Identity();
    noise_.block<3, 3>(9, 9) = (GYR_NOISE_STD * GYR_NOISE_STD) * Eigen::Matrix3d::Identity();
    noise_.block<3, 3>(12, 12) = (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
    noise_.block<3, 3>(15, 15) = (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();

    buffer_.reserve(200);
}

void ImuPreintegration::PushBack(ImuMeasurement &imu_measurement)
{
//    ROS_DEBUG_STREAM("buffer size: " << buffer_.size());
    buffer_.emplace_back(imu_measurement);
    Propagate(imu_measurement.dt_, imu_measurement.getAcc(), imu_measurement.getGyr());
}

std::vector<ImuMeasurement> ImuPreintegration::GetIMUBuffer()
{
    return buffer_;
}

void ImuPreintegration::Propagate(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
{

    dt_ = dt;
    acc_1_ = acc;
    gyr_1_ = gyr;

    Eigen::Vector3d updated_delta_p;
    Eigen::Quaterniond updated_delta_q;
    Eigen::Vector3d updated_delta_v;
    Eigen::Vector3d updated_linearized_ba;
    Eigen::Vector3d updated_linearized_bg;

    MidPointIntegration(dt, acc_0_, gyr_0_, acc, gyr,
                        delta_p_, delta_q_, delta_v_, linearized_ba_, linearized_bg_,
                        updated_delta_p, updated_delta_q, updated_delta_v, updated_linearized_ba,
                        updated_linearized_bg, true);

    delta_p_ = updated_delta_p;
    delta_q_ = updated_delta_q;
    delta_q_.normalize();
    delta_v_ = updated_delta_v;
    linearized_ba_ = updated_linearized_ba;
    linearized_bg_ = updated_linearized_bg;

    sum_dt_ += dt;
    acc_0_ = acc_1_;
    gyr_0_ = gyr_1_;

    // ROS_DEBUG_STREAM("delta p:" << delta_p_.transpose()
    //                             << ", dt: " << dt_ << " acc_1: " << acc_1_.transpose()
    //                             << ", gyr_1: " << gyr_1_.transpose());
}

void ImuPreintegration::Repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
{
    sum_dt_ = 0.0;
    acc_0_ = linearized_acc_;
    gyr_0_ = linearized_gyr_;
    delta_p_.setZero();
    delta_q_.setIdentity();
    delta_v_.setZero();
    linearized_ba_ = _linearized_ba;
    linearized_bg_ = _linearized_bg;
    jacobian_.setIdentity();
    covariance_.setZero();
    for (int i = 0; i < static_cast<int>(buffer_.size()); i++)
        Propagate(buffer_[i].dt_, buffer_[i].getAcc(), buffer_[i].getGyr());
}

void ImuPreintegration::MidPointIntegration(double &dt, const Eigen::Vector3d &acc_0, const Eigen::Vector3d &gyr_0,
                                            const Eigen::Vector3d &acc_1, const Eigen::Vector3d &gyr_1,
                                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg,
                                            bool update_jacobian)
{

    // ROS_INFO("midpoint integration");
    Eigen::Vector3d un_acc_0 = delta_q * (acc_0 - linearized_ba);
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + gyr_1) - linearized_bg;
    result_delta_q = delta_q * Eigen::Quaterniond(1, un_gyr(0) * dt / 2, un_gyr(1) * dt / 2, un_gyr(2) * dt / 2);
    Eigen::Vector3d un_acc_1 = result_delta_q * (acc_1 - linearized_ba);
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    result_delta_p = delta_p + delta_v * dt + 0.5 * un_acc * dt * dt;
    result_delta_v = delta_v + un_acc * dt;
    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;

    if (update_jacobian)
    {
        Eigen::Vector3d w_x = 0.5 * (gyr_0 + gyr_1) - linearized_bg;
        Eigen::Vector3d a_0_x = acc_0 - linearized_ba;
        Eigen::Vector3d a_1_x = acc_1 - linearized_ba;
        Eigen::Matrix3d R_w_x, R_a_0_x, R_a_1_x;

        R_w_x << 0, -w_x(2), w_x(1),
            w_x(2), 0, -w_x(0),
            -w_x(1), w_x(0), 0;
        R_a_0_x << 0, -a_0_x(2), a_0_x(1),
            a_0_x(2), 0, -a_0_x(0),
            -a_0_x(1), a_0_x(0), 0;
        R_a_1_x << 0, -a_1_x(2), a_1_x(1),
            a_1_x(2), 0, -a_1_x(0),
            -a_1_x(1), a_1_x(0), 0;

        Eigen::MatrixXd F = Eigen::MatrixXd::Zero(15, 15);
        F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * dt * dt +
                              -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt * dt;
        F.block<3, 3>(0, 6) = Eigen::MatrixXd::Identity(3, 3) * dt;
        F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * dt * dt;
        F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * dt * dt * -dt;
        F.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() - R_w_x * dt;
        F.block<3, 3>(3, 12) = -1.0 * Eigen::MatrixXd::Identity(3, 3) * dt;
        F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * dt +
                              -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt;
        F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
        F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * dt;
        F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * dt * -dt;
        F.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
        F.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();
        //cout<<"A"<<endl<<A<<endl;

        Eigen::MatrixXd V = Eigen::MatrixXd::Zero(15, 18);
        V.block<3, 3>(0, 0) = 0.25 * delta_q.toRotationMatrix() * dt * dt;
        V.block<3, 3>(0, 3) = 0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x * dt * dt * 0.5 * dt;
        V.block<3, 3>(0, 6) = 0.25 * result_delta_q.toRotationMatrix() * dt * dt;
        V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);
        V.block<3, 3>(3, 3) = 0.5 * Eigen::MatrixXd::Identity(3, 3) * dt;
        V.block<3, 3>(3, 9) = 0.5 * Eigen::MatrixXd::Identity(3, 3) * dt;
        V.block<3, 3>(6, 0) = 0.5 * delta_q.toRotationMatrix() * dt;
        V.block<3, 3>(6, 3) = 0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x * dt * 0.5 * dt;
        V.block<3, 3>(6, 6) = 0.5 * result_delta_q.toRotationMatrix() * dt;
        V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);
        V.block<3, 3>(9, 12) = Eigen::MatrixXd::Identity(3, 3) * dt;
        V.block<3, 3>(12, 15) = Eigen::MatrixXd::Identity(3, 3) * dt;

        jacobian_ = F * jacobian_;
        covariance_ = F * covariance_ * F.transpose() + V * noise_ * V.transpose();
    }
}

Eigen::Matrix<double, 15, 1>
ImuPreintegration::Evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi,
                            const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi, const Eigen::Vector3d &Pj,
                            const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj,
                            const Eigen::Vector3d &Bgj)
{

    // ROS_DEBUG_STREAM("delta_p: " << delta_p_.transpose()
    //                              << ", delta_q: " << Utility::R2ypr(delta_q_.toRotationMatrix()).transpose()
    //                              << ", delta_v: " << delta_v_.transpose());

    Eigen::Matrix<double, 15, 1> residuals;

    Eigen::Matrix3d dp_dba = jacobian_.block<3, 3>(StateOrder::O_P, StateOrder::O_BA);
    Eigen::Matrix3d dp_dbg = jacobian_.block<3, 3>(StateOrder::O_P, StateOrder::O_BG);

    Eigen::Matrix3d dq_dbg = jacobian_.block<3, 3>(StateOrder::O_R, StateOrder::O_BG);

    Eigen::Matrix3d dv_dba = jacobian_.block<3, 3>(StateOrder::O_V, StateOrder::O_BA);
    Eigen::Matrix3d dv_dbg = jacobian_.block<3, 3>(StateOrder::O_V, StateOrder::O_BG);

    Eigen::Vector3d dba = Bai - linearized_ba_;
    Eigen::Vector3d dbg = Bgi - linearized_bg_;

    Eigen::Quaterniond corrected_delta_q = delta_q_ * Utility::deltaQ(dq_dbg * dbg);
    Eigen::Vector3d corrected_delta_v = delta_v_ + dv_dba * dba + dv_dbg * dbg;
    Eigen::Vector3d corrected_delta_p = delta_p_ + dp_dba * dba + dp_dbg * dbg;

    residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * GRAVITY * sum_dt_ * sum_dt_ + Pj - Pi - Vi * sum_dt_) - corrected_delta_p;
    residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
    residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (GRAVITY * sum_dt_ + Vj - Vi) - corrected_delta_v;
    residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
    residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;

    return residuals;
}

} // namespace gp_lio