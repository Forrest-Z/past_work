//
// Created by wchen on 2020/10/19.
//

#include "gp_lio/mapper/LocalMapMatcher.h"

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

int
main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    std::string OUTPUT_PATH = "/home/wchen/Projects/Code/BnB/Demo---Practical-optimal-registration-of-terrestrial-LiDAR-scan-pairs-master/data/my_data/";
    int loop_count = 0;
    int ref_idx = 17;
    int query_idx = 821;
    std::string ref_ground_save_file = OUTPUT_PATH + "loop_" + std::to_string(loop_count) + "_target_ground_" + std::to_string(ref_idx) + ".pcd";
    pcl::io::loadPCDFile(ref_ground_save_file, *cloud);

    std::cerr << "Point cloud data: " << cloud->size () << " points" << std::endl;
    for (const auto& point: *cloud)
        std::cerr << "    " << point.x << " "
                  << point.y << " "
                  << point.z << " "
                  << point.intensity << std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    for (std::size_t i = 0; i < inliers->indices.size (); ++i)
        for (const auto& idx: inliers->indices)
            std::cerr << idx << "    " << cloud->points[idx].x << " "
                      << cloud->points[idx].y << " "
                      << cloud->points[idx].z << std::endl;

    return (0);
}