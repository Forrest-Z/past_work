//
// Created by wchen on 2019/12/3.
//

#ifndef SRC_KEYFRAME_H
#define SRC_KEYFRAME_H

#include "gp_lio/mapper/Frame.h"
#include "gp_lio/utility/Utility.h"

#include <pcl/filters/random_sample.h>

namespace gp_lio{

    class KeyFrame : public Frame {

    public:

        KeyFrame();

        KeyFrame(Frame frame);

        ~KeyFrame();

        void ConstructFeatureCloud();

        pcl::PointCloud<pcl::PointXYZ> GetTransformedFeatureCloud(ExternalParameters& extrinsic_parameters);
        pcl::PointCloud<pcl::PointXYZI> GetTransformedAllCloud(ExternalParameters& extrinsic_parameters);

    public:

        pcl::PointCloud<pcl::PointXYZ> feature_cloud_;
        DescriptorVector descriptor_;

        Eigen::Vector3d gnss_pose_;
        Eigen::Vector4d gnss_xyza_;
        bool has_prior;
        // reuse map
        int sequence_;
        int loop_index_;
        int index_;
        bool has_loop;

    };
}



#endif //SRC_KEYFRAME_H
