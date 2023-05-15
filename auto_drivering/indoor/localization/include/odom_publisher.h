//
// Created by xc on 2020/12/6.
//

#ifndef GP_LIO_ODOM_PUBLISHER_H
#define GP_LIO_ODOM_PUBLISHER_H

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace gp_lio {
    class OdometryPublisher {
    public:
        OdometryPublisher(ros::NodeHandle& nh,
                          std::string topic_name,
                          std::string base_frame_id,
                          std::string child_frame_id,
                          int buff_size);
        OdometryPublisher() = default;

        void Publish(const Eigen::Matrix4f& transform_matrix,const Eigen::Vector3f velocity, double time);
        void Publish(const Eigen::Matrix4f& transform_matrix, double time);
        void Publish(const Eigen::Matrix4f& transform_matrix);

        bool HasSubscribers();

    private:
        void PublishData(const Eigen::Matrix4f& transform_matrix, ros::Time time);
        void PublishData(const Eigen::Matrix4f& transform_matrix,const Eigen::Vector3f velocity, ros::Time time);

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        nav_msgs::Odometry odometry_;
    };
}


#endif //GP_LIO_ODOM_PUBLISHER_H
