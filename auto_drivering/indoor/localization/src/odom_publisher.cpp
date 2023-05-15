//
// Created by xc on 2020/12/6.
//

#include "odom_publisher.h"

namespace gp_lio {
    OdometryPublisher::OdometryPublisher(ros::NodeHandle& nh,
                                         std::string topic_name,
                                         std::string base_frame_id,
                                         std::string child_frame_id,
                                         int buff_size)
            :nh_(nh) {

        publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
        odometry_.header.frame_id = base_frame_id;
        odometry_.child_frame_id = child_frame_id;
    }

    void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix, double time) {
        //ros::Time ros_time((float)time);
        ros::Time ros_time((double)time);
        PublishData(transform_matrix, ros_time);
    }

    void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix,const Eigen::Vector3f velocity, double time) {
        //ros::Time ros_time((float)time);
        ros::Time ros_time((double)time);
        PublishData(transform_matrix,velocity ,ros_time);
    }

    void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix) {
        PublishData(transform_matrix, ros::Time::now());
    }

    void OdometryPublisher::PublishData(const Eigen::Matrix4f& transform_matrix, ros::Time time) {
        odometry_.header.stamp = time;

        //set the position
        odometry_.pose.pose.position.x = transform_matrix(0,3);
        odometry_.pose.pose.position.y = transform_matrix(1,3);
        odometry_.pose.pose.position.z = transform_matrix(2,3);

        Eigen::Quaternionf q;
        q = transform_matrix.block<3,3>(0,0);
        odometry_.pose.pose.orientation.x = q.x();
        odometry_.pose.pose.orientation.y = q.y();
        odometry_.pose.pose.orientation.z = q.z();
        odometry_.pose.pose.orientation.w = q.w();

        publisher_.publish(odometry_);
    }

    void OdometryPublisher::PublishData(const Eigen::Matrix4f& transform_matrix,const Eigen::Vector3f velocity,ros::Time time) {
        odometry_.header.stamp = time;

        //set the position
        odometry_.pose.pose.position.x = transform_matrix(0,3);
        odometry_.pose.pose.position.y = transform_matrix(1,3);
        odometry_.pose.pose.position.z = transform_matrix(2,3);

        Eigen::Quaternionf q;
        q = transform_matrix.block<3,3>(0,0);
        odometry_.pose.pose.orientation.x = q.x();
        odometry_.pose.pose.orientation.y = q.y();
        odometry_.pose.pose.orientation.z = q.z();
        odometry_.pose.pose.orientation.w = q.w();

        odometry_.twist.twist.linear.x = velocity.x();
        odometry_.twist.twist.linear.y = velocity.y();
        odometry_.twist.twist.linear.z = velocity.z();

        publisher_.publish(odometry_);
    }
    bool OdometryPublisher::HasSubscribers() {
        return publisher_.getNumSubscribers() != 0;
    }
}


