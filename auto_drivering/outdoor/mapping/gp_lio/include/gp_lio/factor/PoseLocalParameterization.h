//
// Created by wchen on 2019/12/27.
//

#ifndef GP_LIO_POSELOCALPARAMETERIZATION_H
#define GP_LIO_POSELOCALPARAMETERIZATION_H

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

#include "gp_lio/utility/Utility.h"

namespace gp_lio{

    class PoseLocalParameterization : public ceres::LocalParameterization
    {
        virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
        virtual bool ComputeJacobian(const double *x, double *jacobian) const;
        virtual int GlobalSize() const { return 7; };
        virtual int LocalSize() const { return 6; };
    };

}



#endif //GP_LIO_POSELOCALPARAMETERIZATION_H
