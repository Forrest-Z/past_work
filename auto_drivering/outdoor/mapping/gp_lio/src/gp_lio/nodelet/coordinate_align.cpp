//
// Created by wchen on 2020/6/21.
//
#include <string>
#include <stdlib.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <ostream>
#include <sstream>

using namespace std;

nav_msgs::Path getPathGp_lio(std::string file_path){
    nav_msgs::Path path;
    path.header.frame_id = "world";

    ifstream fin(file_path.c_str());

    string i_line;

    if(fin.is_open()){

        while (getline(fin, i_line)){

            std::vector<double> pose_v;

            stringstream ss(i_line);

            string splited;

            while(ss >> splited){
                double data = atof(splited.c_str());
                pose_v.push_back(data);
//                std::cout <<  std::fixed << std::setprecision(6) << data << " ";
            }
//            std::cout << std::endl;

            geometry_msgs::PoseStamped pose;

            pose.header.frame_id = "world";
            pose.header.stamp = ros::Time(pose_v.at(0));
            pose.pose.position.x = pose_v.at(3);
            pose.pose.position.y = pose_v.at(1);
            pose.pose.position.z = pose_v.at(2);
            pose.pose.orientation.x = pose_v.at(6);
            pose.pose.orientation.y = pose_v.at(5);
            pose.pose.orientation.z = pose_v.at(4);
            pose.pose.orientation.w = pose_v.at(7);

            path.poses.push_back(pose);
        }
        fin.close();

        return path;

    } else{
        std::cerr << "cannot read file " << file_path << std::endl;
        exit(0);
    }


};

nav_msgs::Path getPathGroundTruth(std::string file_path){
    nav_msgs::Path path;
    path.header.frame_id = "world";

    ifstream fin(file_path.c_str());

    string i_line;

    if(fin.is_open()){

        while (getline(fin, i_line)){

            std::vector<double> pose_v;

            stringstream ss(i_line);

            string splited;

            while(ss >> splited){
                double data = atof(splited.c_str());
                pose_v.push_back(data);
                std::cout <<  std::fixed << std::setprecision(6) << data << " ";
            }
            std::cout << std::endl;

            geometry_msgs::PoseStamped pose;

            pose.header.frame_id = "world";
            pose.header.stamp = ros::Time(pose_v.at(0));
            pose.pose.position.x = pose_v.at(1);
            pose.pose.position.y = pose_v.at(2);
            pose.pose.position.z = pose_v.at(3);
            pose.pose.orientation.x = pose_v.at(4);
            pose.pose.orientation.y = pose_v.at(5);
            pose.pose.orientation.z = pose_v.at(6);
            pose.pose.orientation.w = pose_v.at(7);

            path.poses.push_back(pose);
        }
        fin.close();

        return path;

    } else{
        std::cerr << "cannot read file " << file_path << std::endl;
        exit(0);
    }


};

nav_msgs::Path getPathLOAM(std::string file_path){
    nav_msgs::Path path;
    path.header.frame_id = "world";

    ifstream fin(file_path.c_str());

    string i_line;

    if(fin.is_open()){

        while (getline(fin, i_line)){

            std::vector<double> pose_v;

            stringstream ss(i_line);

            string splited;

            while(ss >> splited){
                double data = atof(splited.c_str());
                pose_v.push_back(data);
                std::cout <<  std::fixed << std::setprecision(6) << data << " ";
            }
            std::cout << std::endl;

            geometry_msgs::PoseStamped pose;

            pose.header.frame_id = "world";
            pose.header.stamp = ros::Time(pose_v.at(0));
            pose.pose.position.x = pose_v.at(3);
            pose.pose.position.y = - pose_v.at(1);
            pose.pose.position.z = - pose_v.at(2);
            pose.pose.orientation.x = pose_v.at(6);
            pose.pose.orientation.y = pose_v.at(4);
            pose.pose.orientation.z = pose_v.at(5);
            pose.pose.orientation.w = pose_v.at(7);

            path.poses.push_back(pose);
        }
        fin.close();

        return path;

    } else{
        std::cerr << "cannot read file " << file_path << std::endl;
        exit(0);
    }


};

nav_msgs::Path getPathLego_LOAM(std::string file_path){
    nav_msgs::Path path;
    path.header.frame_id = "world";

    ifstream fin(file_path.c_str());

    string i_line;

    if(fin.is_open()){

        while (getline(fin, i_line)){

            std::vector<double> pose_v;

            stringstream ss(i_line);

            string splited;

            while(ss >> splited){
                double data = atof(splited.c_str());
                pose_v.push_back(data);
                std::cout <<  std::fixed << std::setprecision(6) << data << " ";
            }
            std::cout << std::endl;

            geometry_msgs::PoseStamped pose;

            pose.header.frame_id = "world";
            pose.header.stamp = ros::Time(pose_v.at(0));
            pose.pose.position.x = pose_v.at(3);
            pose.pose.position.y = - pose_v.at(1);
            pose.pose.position.z = - pose_v.at(2);
            pose.pose.orientation.x = pose_v.at(6);
            pose.pose.orientation.y = pose_v.at(4);
            pose.pose.orientation.z = pose_v.at(5);
            pose.pose.orientation.w = pose_v.at(7);

            path.poses.push_back(pose);
        }
        fin.close();

        return path;

    } else{
        std::cerr << "cannot read file " << file_path << std::endl;
        exit(0);
    }


};


void writetofile(std::string file_path, nav_msgs::Path path){

    ofstream fout(file_path);
    if(fout.is_open()){

        for (int i = 0; i < path.poses.size(); ++i) {

            auto i_pose = path.poses.at(i);

            fout << std::fixed << std::setprecision(6)
                 << i_pose.header.stamp.toSec() << " "
                 << i_pose.pose.position.x << " "
                 << i_pose.pose.position.y << " "
                 << i_pose.pose.position.z << " "
                 << i_pose.pose.orientation.x << " "
                 << i_pose.pose.orientation.y << " "
                 << i_pose.pose.orientation.z << " "
                 << i_pose.pose.orientation.w << std::endl;

        }

        fout.close();

    }

}

int main(int argc, char **argv){

    ros::init(argc, argv, "coordinate_align");

    string gp_lio_file = "/home/wchen/Projects/Code/catkin_ws/src/gp_lio/result/evo/evo-master/my_data/gp_lio.txt";
    auto gp_lio_path = getPathGp_lio(gp_lio_file);

    string groundtruth_file = "/home/wchen/Projects/Code/catkin_ws/src/gp_lio/result/evo/evo-master/my_data/ground_truth.txt";
    auto ground_truth_path = getPathGroundTruth(groundtruth_file);

    string lego_loam_file = "/home/wchen/Projects/Code/catkin_ws/src/gp_lio/result/evo/evo-master/my_data/lego_loam.txt";
    auto lego_loam_path = getPathLego_LOAM(lego_loam_file);

    string loam_file = "/home/wchen/Projects/Code/catkin_ws/src/gp_lio/result/evo/evo-master/my_data/loam.txt";
    auto loam_path = getPathLOAM(loam_file);

    string new_gp_lio_file = "/home/wchen/Projects/Code/catkin_ws/src/gp_lio/result/evo/evo-master/my_data/aligned_gp_lio.txt";
    string new_ground_truth_file = "/home/wchen/Projects/Code/catkin_ws/src/gp_lio/result/evo/evo-master/my_data/aligned_ground_truth.txt";
    string new_lego_loam_file = "/home/wchen/Projects/Code/catkin_ws/src/gp_lio/result/evo/evo-master/my_data/aligned_lego_loam.txt";
    string new_loam_file = "/home/wchen/Projects/Code/catkin_ws/src/gp_lio/result/evo/evo-master/my_data/aligned_loam.txt";

    writetofile(new_gp_lio_file, gp_lio_path);
    writetofile(new_ground_truth_file, ground_truth_path);
    writetofile(new_lego_loam_file, lego_loam_path);
    writetofile(new_loam_file, loam_path);

    ros::NodeHandle nh;

    ros::Publisher pub_gp_lio = nh.advertise<nav_msgs::Path>("gp_lio", 1);
    ros::Publisher pub_gt = nh.advertise<nav_msgs::Path>("gt", 1);
    ros::Publisher pub_lg = nh.advertise<nav_msgs::Path>("lg", 1);
    ros::Publisher pub_l = nh.advertise<nav_msgs::Path>("l", 1);

    ros::Rate rate(10);

    while (ros::ok()){

        std::cout << "publish " << gp_lio_path.poses.size() << std::endl;
        pub_gp_lio.publish(gp_lio_path);
        pub_gt.publish(ground_truth_path);
        pub_lg.publish(lego_loam_path);
        pub_l.publish(loam_path);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}