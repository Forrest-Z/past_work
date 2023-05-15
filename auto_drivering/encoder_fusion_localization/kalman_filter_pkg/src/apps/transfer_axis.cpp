/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-31 11:40:30
 * @LastEditors: luo
 * @LastEditTime: 2022-01-05 17:40:04
 */
#include <iostream>

//pcl
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// ros
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <queue>

std::queue<sensor_msgs::PointCloud2Ptr> tarns_lidar_buffer_;
sensor_msgs::PointCloud2 processed_lidar_msg;
ros::Subscriber sub_lidar_;
ros::Publisher pub_transfered_lidar;

void LidarCallBack(const sensor_msgs::PointCloud2Ptr lidar_msg) {
        tarns_lidar_buffer_.push(lidar_msg);
    }
void process()
{       
    int num_points=0;
    while(!tarns_lidar_buffer_.empty())
    {
        sensor_msgs::PointCloud2Ptr tmp_lidar;
        tmp_lidar = tarns_lidar_buffer_.front();
        pcl::PCLPointCloud2 lidar_pc2;
        pcl_conversions::toPCL(*tmp_lidar,lidar_pc2);
        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_data(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromPCLPointCloud2(lidar_pc2,*lidar_data);

        num_points=lidar_data->points.size();
        pcl::PointCloud<pcl::PointXYZI>::Ptr processed_lidar_pcl(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointXYZI one_point;
        for(int i=0;i<num_points;i++)
        {
            one_point.x=lidar_data->points[i].y;
            one_point.y=-lidar_data->points[i].x;
            one_point.z=lidar_data->points[i].z;

            processed_lidar_pcl->points.push_back(one_point);

        }
        pcl::toROSMsg(*processed_lidar_pcl,processed_lidar_msg);
        processed_lidar_msg.header = tmp_lidar->header;
 
        pub_transfered_lidar.publish(processed_lidar_msg);

        tarns_lidar_buffer_.pop();
    }

}
int main(int argc,char**argv){

    ros::init(argc,argv,"trasfer_axis");

    ros::NodeHandle nh_;

    sub_lidar_ = nh_.subscribe("/velodyne_points",5000,&LidarCallBack);
    pub_transfered_lidar = nh_.advertise<sensor_msgs::PointCloud2>("/transfered__points",1);

    ros::Rate rate(100);
    while (ros::ok())
    {
    process();
    ros::spinOnce();
    }
    return 0;
}
