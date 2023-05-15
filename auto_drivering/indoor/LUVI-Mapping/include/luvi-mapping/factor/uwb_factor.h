//
// Created by xc on 2021/4/6.
//

#ifndef ALOAM_VELODYNE_WUB_FACTOR_H
#define ALOAM_VELODYNE_WUB_FACTOR_H

#include <ceres/ceres.h>

struct UWBRangeCost{
    UWBRangeCost(double x,double y,double z,double dis):x_(x),y_(y),z_(z),d_(dis){};
    template <typename T>
    bool operator()(const T* const xyz,T*residual)const{
        residual[0] = ceres::abs(ceres::sqrt((xyz[0] - x_) * (xyz[0] - x_) + (xyz[1] - y_) * (xyz[1] - y_) + (xyz[2] - z_) * (xyz[2] - z_))) - d_;
        return true;
    }
    const double x_,y_,z_,d_;
};






#endif //ALOAM_VELODYNE_WUB_FACTOR_H
