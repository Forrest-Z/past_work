//
// Created by wchen on 2019/12/3.
//

#ifndef SRC_IMUPREINTEGRATIONBASE_H
#define SRC_IMUPREINTEGRATIONBASE_H

#include <vector>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gp_lio/utility/Utility.h"
//#include "gp_lio/estimator/Parameters.h"

namespace gp_lio
{
class ImuPreintegration
{

public:
    ImuPreintegration() = delete;
    ImuPreintegration(const Eigen::Vector3d &acc_0,
                      const Eigen::Vector3d &gyr_0,
                      const Eigen::Vector3d &linearized_ba,
                      const Eigen::Vector3d &linearized_bg);

    void PushBack(ImuMeasurement &imu_measurement);

    Eigen::Matrix<double, 15, 1> Evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                          const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj);

    std::vector<ImuMeasurement> GetIMUBuffer();

private:
    void Propagate(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr);

    void MidPointIntegration(double &dt,
                             const Eigen::Vector3d &acc_0, const Eigen::Vector3d &gyr_0,
                             const Eigen::Vector3d &acc_1, const Eigen::Vector3d &gyr_1,
                             const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                             const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                             Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                             Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian);
    
    void Repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg);

private:
    std::vector<ImuMeasurement> buffer_;

    double dt_;
    Eigen::Vector3d acc_0_, acc_1_;
    Eigen::Vector3d gyr_0_, gyr_1_;
    const Eigen::Vector3d linearized_acc_;
    const Eigen::Vector3d linearized_gyr_;

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix<double, 18, 18> noise_;
    Eigen::Matrix<double, 15, 15> jacobian_;
    Eigen::Matrix<double, 15, 15> covariance_;

    double sum_dt_;

    // imu preintegration term
    Eigen::Vector3d delta_p_;
    Eigen::Quaterniond delta_q_;
    Eigen::Vector3d delta_v_;
    Eigen::Vector3d linearized_ba_;
    Eigen::Vector3d linearized_bg_;
};
} // namespace gp_lio
#endif //SRC_IMUPREINTEGRATIONBASE_H
