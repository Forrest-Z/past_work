//
// Created by wchen on 2019/12/3.
//

#ifndef SRC_KEYFRAME_H
#define SRC_KEYFRAME_H

#include "Frame.h"
#include "Utility.h"

#include <pcl/filters/random_sample.h>

namespace gp_lio{

    struct KeyFrame{
        pcl::PointCloud<pcl::PointXYZI> GetTansformedCloud(){
            pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
            pcl::PointCloud<pcl::PointXYZI> cloud;
            Eigen::Matrix4d trans;
            trans.setIdentity();
            trans.block<3,3>(0,0) = quaternion_.toRotationMatrix();
            trans.block<3,1>(0,3) = position_;
            cloud += corner_cloud_;
            cloud += surf_cloud_;
            pcl::transformPointCloud(cloud,transformed_cloud,trans);
            return transformed_cloud;
        }
        double timestamp_;
        pcl::PointCloud<pcl::PointXYZI> corner_cloud_, surf_cloud_;
        Eigen::Vector3d position_;
        Eigen::Quaterniond quaternion_;
    };

}



#endif //SRC_KEYFRAME_H
