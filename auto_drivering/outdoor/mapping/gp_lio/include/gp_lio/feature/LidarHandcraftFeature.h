//
// Created by wchen on 2020/7/10.
//

#define PCL_NO_PRECOMPILE //for custom PointT

#ifndef GP_LIO_LIDARHANDCRAFTFEATURE_H
#define GP_LIO_LIDARHANDCRAFTFEATURE_H

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <pcl_conversions/pcl_conversions.h>

struct PointxyzWithHandcraftFeature
{
    PCL_ADD_POINT4D;
    float c;
    float o;
    float l;
    float e;
    float d;
    float s2;
    float l2;
    float dz;
    float vz;
    float v;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointxyzWithHandcraftFeature,
                                   (float, x, x)
                                           (float, y, y)
                                           (float, z, z)
                                           (float, c, c)
                                           (float, o, o)
                                           (float, l, l)
                                           (float, e, e)
                                           (float, d, d)
                                           (float, s2, s2)
                                           (float, l2, l2)
                                           (float, dz, dz)
                                           (float, vz, vz)
                                           (float, v, v)
)

namespace gp_lio{

    class LidarHandcraftFeature {

    public:
        LidarHandcraftFeature(pcl::PointCloud<pcl::PointXYZ>& feature_cloud);

        ~LidarHandcraftFeature();

        void generate_feature(pcl::PointCloud<PointxyzWithHandcraftFeature>::Ptr& features,int k_start,int k_end,int k_step);

    private:

        double CalculateEntropy(Eigen::Vector3d& eigen);

        PointxyzWithHandcraftFeature CalculateHandcraftFeatures(int i, std::vector<int>& nbrs_index, Eigen::Vector3d& eigen3d,  Eigen::Vector2d& eigen2d, Eigen::Matrix3d& vector);


    private:

        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;

    };


}



#endif //GP_LIO_LIDARHANDCRAFTFEATURE_H
