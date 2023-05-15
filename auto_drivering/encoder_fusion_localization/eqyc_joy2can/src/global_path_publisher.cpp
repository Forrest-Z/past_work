
#include "global_path_publisher.hpp"


namespace GCT{
    Publish::Publish():nh_("~"){

        pub_globalPath_ = nh_.advertise<nav_msgs::Path>("/GlobalPath", 1);

        LoadPath();
        SavePath();

    }

    Publish::~Publish() {

    }

    void Publish::LoadPath() {
        std::string map_save_dir_ = "/home/hw/catkin_ws_tractor/src/eqyc_joy2can/src";
        std::string pose_graph_file = map_save_dir_ + "/odom3.txt";
        std::FILE* ifile_pg;
        ifile_pg = std::fopen(pose_graph_file.c_str(), "r");
        if (ifile_pg == NULL) {
            std::cout << "! Open [ " << pose_graph_file << " ] failed ..." << std::endl;
            return;
        }

        std::pair<Eigen::Vector3d,Eigen::Quaterniond> pose;
        int i;
        double t, px, py, pz, qx, qy, qz, qw;
        while(std::fscanf(ifile_pg,"%lf %lf %lf %lf %lf %lf %lf", &px, &py, &pz, &qx, &qy, &qz, &qw) !=EOF){

            pose.first.x() = px;
            pose.first.y() = py;
            pose.first.z() = pz;
            pose.second.x() = qx;
            pose.second.y() = qy;
            pose.second.z() = qz;
            pose.second.w() = qw;
            path_msg_buffer_.push_back(pose);
        }
        std::fclose(ifile_pg);
        std::cout << "global path publish finished ... "<<std::endl;

        return;
    }

    void Publish::SavePath(){
        path_msg.header.frame_id = "map";
        for (unsigned int i = 0; i < path_msg_buffer_.size() ;i++)
        {
            geometry_msgs::PoseStamped location;

            location.pose.position.x = path_msg_buffer_[i].first.x();
            location.pose.position.y = path_msg_buffer_[i].first.y();
            location.pose.orientation.x = path_msg_buffer_[i].second.x();
            location.pose.orientation.y = path_msg_buffer_[i].second.y();
            location.pose.orientation.z = path_msg_buffer_[i].second.z();
            location.pose.orientation.w = path_msg_buffer_[i].second.w();
            path_msg.poses.push_back(location);
        }
        return ;
    }


    void Publish::Run() {

        pub_globalPath_.publish(path_msg);
        return;
    }



}



int main(int argc, char** argv){
    ros::init(argc,argv,"GlobalPathPublisher");
    GCT::Publish PUB;
    ros::Rate rate(200);
    while(ros::ok()){
        ros::spinOnce();
        PUB.Run();
        rate.sleep();
    }
    return 0;
}
