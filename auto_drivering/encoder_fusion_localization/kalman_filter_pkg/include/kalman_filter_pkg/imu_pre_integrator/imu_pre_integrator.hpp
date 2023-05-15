/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-22 10:19:04
 * @LastEditors: luo
 * @LastEditTime: 2022-01-14 16:11:58
 */
#pragma once

#include <iostream>
#include <ros/ros.h>
#include <deque>
#include <mutex>

#include <sophus/so3.hpp>

#include "kalman_filter_pkg/sensor_data/imu_data.hpp"
#include "kalman_filter_pkg/imu_pre_integrator/pre_integrator.hpp"
// #include ""

namespace KalmanFilter
{
class IMUPreIntegrator: public PreIntegrator
{
public:
    static const int DIM_STATE = 15;
    
    typedef Eigen::Matrix<double, 15, 15> MatrixP;
    typedef Eigen::Matrix<double, 15, 15> MatrixJ;
    typedef Eigen::Matrix<double, 15, 1> Vector15d;

    static const int INDEX_P = 0;
    static const int INDEX_R = 3;
    static const int INDEX_V = 6;
    static const int INDEX_A = 9;
    static const int INDEX_G = 12;

    struct IMUPreIntrgration
    {
        double T;

        Eigen::Vector3d g;

        Eigen::Vector3d alpha_ij;

        Sophus::SO3d theta_ij;

        Eigen::Vector3d beta_ij;

        Eigen::Vector3d b_a_i;

        Eigen::Vector3d b_g_i;

        MatrixP P;

        MatrixJ J;

        double GetT(void) const {return T;}

        Eigen::Vector3d GetGravity(void) const { return g;}

        Vector15d GetMeasurement(void) const 
        {
            Vector15d measurement = Vector15d::Zero();

            measurement.block<3, 1>(INDEX_P, 0) = alpha_ij;
            measurement.block<3, 1>(INDEX_R, 0) = theta_ij.log();
            measurement.block<3, 1>(INDEX_V, 0) = beta_ij;

            return measurement;
        }

        Eigen::MatrixXd GetInformation() const 
        {
            // return P;
            std::cout << "GetInformation--111--PPPPPPPPPPPPPPPPPPP = " << std::endl << P << std::endl;
            std::cout << "GetInformation--222--PPPPPPPPPPPPPPPPPPP = " << std::endl << P.inverse() << std::endl;
            
            return P.inverse();
        }

        Eigen::MatrixXd GetJacoBian() const
        {
            return J;
        }
    };

    IMUPreIntegrator(const YAML::Node& node);
    ~IMUPreIntegrator();

    bool Init(const IMUData& init_imu_data);

    bool Update(const IMUData& imu_data);

    bool Reset(const IMUData& init_imu_data, IMUPreIntrgration& imu_pre_integration);


private:

    static const int DIM_NOISE = 18;

    static const int INDEX_ALPHA = 0;
    static const int INDEX_THETA = 3;
    static const int INDEX_BETA = 6;
    static const int INDEX_B_A = 9;
    static const int INDEX_B_G = 12;

    static const int INDEX_M_ACC_PREV = 0;
    static const int INDEX_M_GYR_PREV = 3;
    static const int INDEX_M_ACC_CURR = 6;
    static const int INDEX_M_GYR_CURR = 9;
    static const int INDEX_R_ACC_PREV = 12;
    static const int INDEX_R_GYR_PREV = 15;

    typedef Eigen::Matrix<double, 15, 15> MatrixF;
    typedef Eigen::Matrix<double, 15, 18> MatrixB;
    typedef Eigen::Matrix<double, 18, 18> MatrixQ;

    std::deque<IMUData> imu_data_buff_;

    struct{
        double GRAVITY_MAGNITUDE;
    } EARTH;

    struct {
        struct 
        {
            double ACCEL;
            double GYRO;
        } RANDOM_WALK;

        struct 
        {
            double ACCEL;
            double GYRO;
        } MEASUREMENT;
        
    } COV;

    struct {
        Eigen::Vector3d g;

        Eigen::Vector3d alpha_ij;

        Sophus::SO3d theta_ij;

        Eigen::Vector3d beta_ij;

        Eigen::Vector3d b_a_i;

        Eigen::Vector3d b_g_i;
    } state;

    MatrixP P_ = MatrixP::Zero();

    MatrixJ J_ = MatrixJ::Identity();

    MatrixQ Q_ = MatrixQ::Zero();

    MatrixF F_ = MatrixF::Zero();

    MatrixB B_ = MatrixB::Zero();

    void ResetState(const IMUData& init_imu_data);

    void UpdateState();

    // state GetCurrState(){return state;};

};   
} // namespace KalmanFilter
