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

//eigen
#include "Eigen/Core"
#include "Eigen/Dense"




namespace GCT{
    class Publish{
    public:
        Publish();
        ~Publish();

        void Run();

    private:
        void LoadPath();
        void SavePath();


    public:

    private:

        ros::NodeHandle nh_;
        ros::Publisher pub_globalPath_;

        std::vector<std::pair<Eigen::Vector3d,Eigen::Quaterniond>> path_msg_buffer_;
        nav_msgs::Path path_msg;

    };
}




