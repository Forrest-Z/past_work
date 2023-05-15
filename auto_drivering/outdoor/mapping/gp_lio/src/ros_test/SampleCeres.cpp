//
// Created by wchen on 2019/12/2.
//

#include "ros_test/SampleCeres.h"
#include <cmath>
#include "gp_lio/utility/Utility.h"

double testStartEnd(const double &y_start, const double &x_start,
        const double &y_end, const double &x_end){

    double start = -atan2(y_start, x_start);
    double end = -atan2(y_end, x_end)+2*M_PI;

    if (end - start > 3 * M_PI) {
        end -= 2 * M_PI;
    } else if (end - start < M_PI)
        end += 2 * M_PI;
    return (end - start);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "SampleCeres");
    ros::NodeHandle node("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    // test eigen affine3d, matrix4d
    Eigen::Vector3d ypr({30/180*M_PI, 60/180*M_PI, 0});
    Eigen::Matrix3d rot = gp_lio::Utility::ypr2R(ypr);
    Eigen::Vector3d t({1, 3, 2});
    Eigen::Vector3d point({-35, 2.5, -1});

    Eigen::Vector3d point1 = rot*point + t;

    Eigen::Matrix4d T;
    T.setIdentity();
    T.block<3, 3>(0, 0) = rot;
    T.block<3, 1>(0, 3) = t;

    Eigen::Affine3d aff_T;
    aff_T.matrix() = T;
    Eigen::Vector3d point2 = aff_T*point;

    ROS_DEBUG_STREAM("T: \n" << T << "\n aff_T: \n" << aff_T.matrix() << "\n point: " << point.transpose() << "\n point1: " << point1.transpose() << "\n point2: " << point2.transpose() );

    double diff1 = testStartEnd(0, 1, -0.1, 1);
    std::cout << "end-start = " << diff1 * 180.0 / M_PI << std::endl;

    std::cout << "test end" << std::endl << std::endl;

    double initial_x = 0;

    double solving_time = gp_lio::SampleCeres::FirstExample(initial_x);

    std::cout << "solving time: " << solving_time << std::endl;

    return 0;

}
