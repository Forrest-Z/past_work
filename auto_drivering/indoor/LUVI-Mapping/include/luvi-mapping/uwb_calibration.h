//
// Created by xc on 2021/4/6.
//

#ifndef ALOAM_VELODYNE_UWB_CALIBRATION_H
#define ALOAM_VELODYNE_UWB_CALIBRATION_H

#include <sys/file.h>
#include <iostream>
#include <thread>
#include <stdio.h>
#include <queue>
#include <vector>
#include <map>
#include <array>
#include <string>
// ros
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "nlink_parser/LinktrackNode2.h"
#include "nlink_parser/LinktrackNodeframe2.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"

//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// ceres
#include <ceres/ceres.h>
// local
#include "luvi-mapping/factor/uwb_factor.h"
#include "luvi-mapping/utility/circleIntersection.h"
namespace LIRO{
    class UwbCalibration{
    public:
        UwbCalibration();
        ~UwbCalibration();
        void Run();

    private:
        void UwbDataFlagCallback(std_msgs::Bool flag);
        void Command();
        bool LoadData();
        void SaveDatas();
        void GuassUwbInitial();
        bool IsCurve(Point& p1, Point& p2, Point& p3);
        double Distance(Point p,Point q);
        void Optimization();
        void AnchorsVisualization();
        void AnchorMeasurmentsVisualization();

    public:

    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_anchor_initial_;
        ros::Publisher pub_anchor_optimized_;
        ros::Publisher pub_anchor_measurments_;
        ros::Publisher pub_anchor_outliners_measurments_;
        ros::Subscriber sub_uwb_data_flag_;

        std::string uwb_data_path_;
        std::string save_uwb_path_;
        std::map<int,std::vector<std::array<double,4>>> uwb_data_;
        std::map<int,std::array<double,3>> uwb_initial_;
        std::map<int,std::array<double,3>> uwb_optimized_;
        pcl::PointCloud<pcl::PointXYZI> anchors_initial_p_,anchors_settled_p_;
        std::string data_dir_path_;
        bool msgs_flag;
        visualization_msgs::MarkerArray uwb_measurments_msgs_,uwb_measurments_outliners_msgs_;
        int anchor_number_;
        std::map<int,double> mid_dis_, front_dis_, back_dis_;
    };
}


#endif //ALOAM_VELODYNE_UWB_CALIBRATION_H
