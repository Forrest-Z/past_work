//
// Created by xc on 2020/12/6.
//

#ifndef GP_LIO_TF_BROADCASTER_H
#define GP_LIO_TF_BROADCASTER_H

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

namespace gp_lio {
    class TFBroadCaster {
    public:
        TFBroadCaster(std::string frame_id, std::string child_frame_id);
        TFBroadCaster() = default;
        void SendTransform(Eigen::Matrix4f pose, double time);
    protected:
        tf::StampedTransform transform_;
        tf::TransformBroadcaster broadcaster_;
    };
}

#endif //GP_LIO_TF_BROADCASTER_H
