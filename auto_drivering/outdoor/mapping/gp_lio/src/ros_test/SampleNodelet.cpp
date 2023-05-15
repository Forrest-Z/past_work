//
// Created by wchen on 2019/11/28.
//

// related header
#include "ros_test/SampleNodelet.h"

PLUGINLIB_EXPORT_CLASS(gp_lio::SampleNodelet, nodelet::Nodelet)

static int WINDOW_SIZE=3;

namespace gp_lio{

    void SampleNodelet::onInit() {
        NODELET_DEBUG("Initializing nodelet...");

        nh_ = nodelet::Nodelet::getNodeHandle();

        private_nh_ = nodelet::Nodelet::getPrivateNodeHandle();

        pub_ = nh_.advertise<std_msgs::String>("/pub_string", 1);

        ROS_INFO("Nodelet is OK for test!!");

        if (!LoadConfig()){
            ROS_ERROR("Fail to load config!!!");
            exit(0);
        }

        Run();
    }

    bool SampleNodelet::LoadConfig() {

        ROS_INFO("loading config...");
        return true;
    }

    void SampleNodelet::Run() {

        //-> google coding style
//        // test R2ypr
//        Eigen::Matrix3d r;
//        r.setIdentity();
//        Eigen::Vector3d ypr = Utility::R2ypr(r);
//        std::cout << "ypr: " << ypr.transpose() << std::endl;

        // test point structure;
        Point point;
        point.timestamp = ros::Time::now();
        point.position = Eigen::Vector3d({0.0,0.0,0.0});

        std::vector<int>  v = {1, 2};

        // unchanged constant
        const int kFrequency = 10;


        ros::Rate rate(kFrequency);

        while(nh_.ok()){

            Publish();
        }
    }

    void SampleNodelet::Publish() {

        std_msgs::StringPtr str_msg_p(new std_msgs::String);
        str_msg_p->data = "publishing";
        pub_.publish(str_msg_p);
    }

}