//
// Created by wchen on 2019/11/28.
//

#ifndef SRC_SAMPLENODELET_H
#define SRC_SAMPLENODELET_H

// -> Google coding style: #include
// 1. Related header;
// 2. C headers;
// 3. C++ headers;
// 4. other Library headers;
// 5. this package headers;

// C++ headers
#include <iostream>
#include <vector>
// other library headers: ROS
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/time.h>
// other library headers: Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// other library headers: GTSAM
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/RangeFactor.h>

//  this package headers
#include "gp_lio/utility/Utility.h"

namespace gp_lio{

    struct Point{
        Eigen::Vector3d position;
        ros::Time timestamp;
    };

    class SampleNodelet: public nodelet::Nodelet
    {
    public:
        virtual void onInit();

        void FunctionSample(const double &inputs, double &outputs);

    private:

        bool LoadConfig();

        void Run();

        void Publish();

    private:

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Publisher pub_;

    };

}

#endif //SRC_SAMPLENODELET_H
