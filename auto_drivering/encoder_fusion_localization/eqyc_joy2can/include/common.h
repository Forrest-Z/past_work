//
// Created by ss on 2021-09-07.
//

#ifndef LOCALIZATION_COMMON_H
#define LOCALIZATION_COMMON_H

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// std
#include <vector>
#include <iostream>

namespace GCT{


    struct Robotics
    {
        double x, y;
        double siny, cosy;
        double bot_f_x, bot_f_y;
        double bot_vel,bot_theta;
    };

    double max(double a, double b)
    {
        if(a > b)
            return a;
        else
            return b;
    }
    double min(double a, double b)
    {
        if(a > b)
            return b;
        else
            return a;
    }

    int sign(double x)
    {
        if (x < 0)
            return -1;
        else
            return 1;
    }
    double calc_dis(double x1, double y1, double x2, double y2)
    {
        return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
    }

}



#endif //LOCALIZATION_COMMON_H
