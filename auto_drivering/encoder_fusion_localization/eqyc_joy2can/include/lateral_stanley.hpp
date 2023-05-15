
//ros
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

//std
#include <time.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <mutex>

//eigen
#include "Eigen/Core"
#include "Eigen/Dense"

//#include "usr/include/eigen3/Eigen/Core"
//#include "usr/include/eigen3/Eigen/Dense"

//local
#include "common.h"



//#include <std_msgs/Int64.h>

namespace GCT{
    class Control{
    public:
        Control();
        ~Control();

        void Run();

    private:
        void OdomCallback(const nav_msgs::OdometryConstPtr &odom_msg);
        void PathCallback(const nav_msgs::PathConstPtr &path_msg);

        double normalize_angle(double angle);
        void find_nearest_in_global_path(std::vector<Eigen::Vector3d>, double x, double y, double& min_x, double& min_y, double& min_dis, int& min_id);


    public:

    private:

        ros::NodeHandle nh_;
        ros::Subscriber sub_odom_;
        ros::Subscriber sub_path_;

        ros::Publisher pub_cmd_delta_;
        ros::Publisher pub_cross_error_;

        Robotics bot_;
        std::vector<Eigen::Vector3d> gp_v_;

        std::mutex mux_;

        double K_V;  //gain parameter
        double ALPHA;
        double WHEELBASE;  //in meters
        double steer;         //global
        int    m = 0;            //global
        double ep_max = 0.0;  //global
        double ep_sum = 0.0;  //global
        double ep_avg = 0.0;  //global

        double ep;
        double cp;
        double bot_theta1;

        double path_length;

    };
}

