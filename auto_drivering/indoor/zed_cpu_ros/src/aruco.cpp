#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Image.h"
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

//#include "/home/ss/catkin_indoor/src/zed_cpu_ros/include/PoseStampedArray.h"
#include "PoseStampedArray.h"
nav_msgs::Odometry odometry_;
ros::Publisher publisher_;
ros::Publisher mPubAruco;
image_transport::Publisher mPubArucoRGB;


using namespace std;
using namespace cv;

void PublishArucoPose(Eigen::Quaterniond q, Eigen::Vector3d t)
{
    odometry_.header.stamp = ros::Time::now();

    //set the position
    odometry_.pose.pose.position.x = t(0);
    odometry_.pose.pose.position.y = t(1);
    odometry_.pose.pose.position.z = t(2);

    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    odometry_.pose.pose.orientation.w = q.w();

    odometry_.header.frame_id = "/map";
    publisher_.publish(odometry_);

}

void publishArucoResult(Eigen::Matrix4d cameraPose,
                        std::vector<std::pair<int, Eigen::Matrix4d> > markerPoseArray,
                        ros::Time time, cv::Mat aruco_image){

  zed_interfaces::PoseStampedArray aruco_result_msg;
  aruco_result_msg.header.frame_id = "/map";
  aruco_result_msg.header.stamp =  time;

  aruco_result_msg.odom.header.frame_id = "/map";
  aruco_result_msg.odom.child_frame_id = "/camera";
  aruco_result_msg.odom.header.stamp = time;
  Eigen::Vector3d trans = cameraPose.block<3,1>(0,3);
  Eigen::Matrix3d rotation = cameraPose.block<3,3>(0,0);
  Eigen::Quaterniond q(rotation);
  q.normalized();
  aruco_result_msg.odom.pose.pose.position.x = trans.x();
  aruco_result_msg.odom.pose.pose.position.y = trans.y();
  aruco_result_msg.odom.pose.pose.position.z = trans.z();
  aruco_result_msg.odom.pose.pose.orientation.w = q.w();
  aruco_result_msg.odom.pose.pose.orientation.x = q.x();
  aruco_result_msg.odom.pose.pose.orientation.y = q.y();
  aruco_result_msg.odom.pose.pose.orientation.z = q.z();

  for (int i = 0; i < markerPoseArray.size(); ++i) {
    geometry_msgs::PoseStamped aruco_pose;
    aruco_pose.header.frame_id = "/map";
    aruco_pose.header.stamp = time;
    aruco_pose.header.seq = uint(markerPoseArray.at(i).first);
    Eigen::Vector3d trans = markerPoseArray.at(i).second.block<3,1>(0,3);
    Eigen::Matrix3d rotation = markerPoseArray.at(i).second.block<3,3>(0,0);
    Eigen::Quaterniond q(rotation);
    q.normalized();

    aruco_pose.pose.position.x = trans.x();
    aruco_pose.pose.position.y = trans.y();
    aruco_pose.pose.position.z = trans.z();
    aruco_pose.pose.orientation.w = q.w();
    aruco_pose.pose.orientation.x = q.x();
    aruco_pose.pose.orientation.y = q.y();
    aruco_pose.pose.orientation.z = q.z();
    aruco_result_msg.poses.push_back(aruco_pose);
  }
  mPubAruco.publish(aruco_result_msg);

//        // Display image
//        cv::imshow("Image", aruco_image);
//
//        // Handle key event
//        cv::waitKey(1);

  sensor_msgs::ImagePtr  image_msg = cv_bridge::CvImage(aruco_result_msg.header, sensor_msgs::image_encodings::RGB8, aruco_image).toImageMsg();
  mPubArucoRGB.publish(image_msg);

}


void ImageCallback(const sensor_msgs::ImageConstPtr& cam0_img){
  ros::Time  Timestamp = cam0_img->header.stamp;
  cv::Mat m_image =cv_bridge::toCvShare(cam0_img,"bgr8")->image;
  if(m_image.empty())
  {
    cout<<"m_image  is empty"<<endl;
    return;
  }
  //read para
  double markerlength=0.18;
  cv::Mat intrinsics = (Mat_<double>(3, 3) <<
                                           1400, 0.0, 960,
      0.0,1400, 540,
      0.0, 0.0, 1.0);
//std:: cout << "callback " << std::endl;
  cv::Mat distCoeffs=(cv::Mat_<double>(4, 1) <<  -0.3549, 0.1151, -0.0035, -0.0029);
  cv::Mat  imageCopy;
  m_image.copyTo(imageCopy);
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);;
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f> > corners;
  cv::aruco::detectMarkers(m_image, dictionary, corners, ids);//检测靶标
  // if at least one marker detected
  std::vector<std::pair<int, Eigen::Matrix4d> > marker_array;
  Eigen::Matrix4d pose;
  pose.setIdentity();
  if (ids.size() > 0) {
    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);//绘制检测到的靶标的框
    for(unsigned int i=0; i<ids.size(); i++)
    {
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, markerlength, intrinsics, distCoeffs, rvecs, tvecs);//求解旋转矩阵rvecs和平移矩阵tvecs
      cv::aruco::drawAxis(imageCopy,intrinsics,distCoeffs, rvecs[i], tvecs[i], 0.1);
      //3.rotaion vector to eulerAngles
      cv::Mat rmat;
      Rodrigues(rvecs[i], rmat);
      Eigen::Matrix3d rotation_matrix3d;
      cv2eigen(rmat,rotation_matrix3d);

      Eigen::Vector3d t;
      t.x() = tvecs[i][0]; t.y() = tvecs[i][1]; t.z() = tvecs[i][2];
//      Eigen::Matrix4d pose;
//      pose.setIdentity();
      pose.block<3,3>(0,0) = rotation_matrix3d;
      pose.block<3,1>(0,3) = t;

      std::pair<int, Eigen::Matrix4d> i_marker(ids.at(0), pose);
      marker_array.push_back(i_marker);

      Eigen::Vector3d eulerAngle = rotation_matrix3d.eulerAngles(0,1,2);//(0,1,2)表示分别绕XYZ轴顺序，即 顺序，逆时针为正
//      cout<<"pitch "<<eulerAngle.x()<<"yaw "<<eulerAngle.y()<<"roll"<<eulerAngle.z()<<endl;
      cout<<"aruco_id: " << ids.at(i) <<" " << "x= "<<tvecs[i][0]<<"y="<<tvecs[i][1]<<"z="<<tvecs[i][2]<<endl;
    }
  }
  publishArucoResult(pose, marker_array,Timestamp, imageCopy);
//  cv::imshow("out", imageCopy);
//  cv::waitKey();
  }
   ;




int main(int argc, char** argv){

//  google::InitGoogleLogging(argv[0]);
//  FLAGS_alsologtostderr = 1;
  ros::init(argc,argv,"aruco_node");
  ros::NodeHandle n_;
  image_transport::ImageTransport it_zed(n_);

  mPubArucoRGB = it_zed.advertise("aruco_detect_image", 1);
  mPubAruco = n_.advertise<zed_interfaces::PoseStampedArray>("aruco_detect_pose", 1);
  ros::Subscriber iamge_sub = n_.subscribe("/camera/left/image_raw", 10, ImageCallback);

  ros::spin();
  return 0;

}

