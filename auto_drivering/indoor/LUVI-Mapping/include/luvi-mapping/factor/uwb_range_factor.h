//
// Created by xc on 2021/3/23.
//

#ifndef ALOAM_VELODYNE_UWB_RANGE_FACTOR_H
#define ALOAM_VELODYNE_UWB_RANGE_FACTOR_H

#include <gtsam/nonlinear/expressions.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>

using namespace gtsam;

class UwbRangeFactor:public NoiseModelFactor2<Pose3,Point3>{

public:
    UwbRangeFactor(Key i,Key j,double distance,const SharedNoiseModel& model):
        NoiseModelFactor2<Pose3,Point3>(model,i,j),distance_m_(distance){}
        Vector evaluateError(const Pose3& pose,const Point3& landk,boost::optional<Matrix&> H1 = boost::none,
                boost::optional<Matrix&> H2 = boost::none) const{
        if(H1){
//            double dis_pridict = (pose.x()-landk.x()) * (pose.x()-landk.x()) + (pose.y()-landk.y()) * (pose.y()-landk.y()) + (pose.z()-landk.z()) * (pose.z()-landk.z());
            (*H1) = (Matrix(1,6) << (pose.x()-landk.x()) * (pose.x()-landk.x()) + (pose.y()-landk.y()) * (pose.y()-landk.y()) + (pose.z()-landk.z()) * (pose.z()-landk.z()) * (pose.x() - landk.x()),
                    (pose.x()-landk.x()) * (pose.x()-landk.x()) + (pose.y()-landk.y()) * (pose.y()-landk.y()) + (pose.z()-landk.z()) * (pose.z()-landk.z()) * (pose.y() - landk.y()),
                    (pose.x()-landk.x()) * (pose.x()-landk.x()) + (pose.y()-landk.y()) * (pose.y()-landk.y()) + (pose.z()-landk.z()) * (pose.z()-landk.z()) * (pose.z() - landk.z()),0,0,0).finished();
        }
        if(H2){
//            double dis_pridict = (pose.x()-landk.x()) * (pose.x()-landk.x()) + (pose.y()-landk.y()) * (pose.y()-landk.y()) + (pose.z()-landk.z()) * (pose.z()-landk.z());
            (*H2) = (Matrix(1,3) << (pose.x()-landk.x()) * (pose.x()-landk.x()) + (pose.y()-landk.y()) * (pose.y()-landk.y()) + (pose.z()-landk.z()) * (pose.z()-landk.z()) * (landk.x()-pose.x()),
                    (pose.x()-landk.x()) * (pose.x()-landk.x()) + (pose.y()-landk.y()) * (pose.y()-landk.y()) + (pose.z()-landk.z()) * (pose.z()-landk.z()) * (landk.y() - pose.y()),
                    (pose.x()-landk.x()) * (pose.x()-landk.x()) + (pose.y()-landk.y()) * (pose.y()-landk.y()) + (pose.z()-landk.z()) * (pose.z()-landk.z()) * (landk.z()-pose.z())).finished();
        }
        double error = std::abs(std::sqrt((pose.x()-landk.x()) * (pose.x()-landk.x()) +
                                          (pose.y()-landk.y()) * (pose.y()-landk.y()) + (pose.z()-landk.z()) * (pose.z()-landk.z()))) - distance_m_ ;
        return (Vector(1) << error).finished();

    };

private:
    double distance_m_;

};



#endif //ALOAM_VELODYNE_UWB_RANGE_FACTOR_H
