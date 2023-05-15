//
// Created by wchen on 2019/12/3.
//

#ifndef SRC_IMUFACTOR_H
#define SRC_IMUFACTOR_H

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "gp_lio/feature/ImuPreintegration.h"

namespace gp_lio{
// 15 for residuals_ (deltaP, deltaR, deltaV, deltaBias_a, deltaBias_g)
// 7 for pose (position+quaternion)
// 9 for velocity + bias
    class ImuFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9>{

    public:
        ImuFactor()=delete;

        ImuFactor(ImuPreintegration* imu_preintegration);

        virtual bool Evaluate(double const* const* parameters,
                              double* residuals,
                              double** jacobians) const;

    private:

        ImuPreintegration* imu_preintegration_;

    };
}
#endif //SRC_IMUFACTOR_H
