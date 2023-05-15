//
// Created by wchen on 2019/9/30.
//

//ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
//gtsam
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/dataset.h>

//std
#include <fstream>
#include <iostream>

#include <Eigen/Geometry>

using namespace gtsam;
using namespace std;

Eigen::Matrix3d rpy2dcm(const Eigen::Vector3d &rpy)//(yaw)
{
    Eigen::Matrix3d R1;
    R1(0,0) = 1.0; R1(0,1) = 0.0; R1(0,2) = 0.0;
    R1(1,0) = 0.0; R1(1,1) = cos(rpy[0]); R1(1,2) = -sin(rpy[0]);
    R1(2,0) = 0.0; R1(2,1) = -R1(1,2); R1(2,2) = R1(1,1);

    Eigen::Matrix3d R2;
    R2(0,0) = cos(rpy[1]); R2(0,1) = 0.0; R2(0,2) = sin(rpy[1]);
    R2(1,0) = 0.0; R2(1,1) = 1.0; R2(1,2) = 0.0;
    R2(2,0) = -R2(0,2); R2(2,1) = 0.0; R2(2,2) = R2(0,0);

    Eigen::Matrix3d R3;
    R3(0,0) = cos(rpy[2]); R3(0,1) = -sin(rpy[2]); R3(0,2) = 0.0;
    R3(1,0) = -R3(0,1); R3(1,1) = R3(0,0); R3(1,2) = 0.0;
    R3(2,0) = 0.0; R3(2,1) = 0.0; R3(2,2) = 1.0;

    return R3 * R2 * R1;
}

geometry_msgs::PoseStamped pose22PathPoint(Pose2& pose2){
    geometry_msgs::PoseStamped posemsg;
    posemsg.header.stamp = ros::Time::now();
    posemsg.header.frame_id = "map";
    posemsg.pose.position.x = pose2.x();
    posemsg.pose.position.y = pose2.y();
    posemsg.pose.position.z = 0.0;
    Eigen::Vector3d rpy(0.0,0.0, pose2.theta());
    Eigen::Matrix3d rot = rpy2dcm(rpy);
    Eigen::Quaterniond q(rot);
    posemsg.pose.orientation.x = q.x();
    posemsg.pose.orientation.y = q.y();
    posemsg.pose.orientation.z = q.z();
    posemsg.pose.orientation.w = q.w();
    return posemsg;
}

geometry_msgs::Point landmark2point(Point2& point2){
    geometry_msgs::Point point;
    point.x = point2.x();
    point.y = point2.y();
    point.z = 0;
    return point;
}

// load the odometry
// DR: Odometry Input (delta distance traveled and delta heading change)
//    Time (sec)  Delta Dist. Trav. (m) Delta Heading (rad)
typedef pair<double, Pose2> TimedOdometry;
list<TimedOdometry> readOdometry() {
    list<TimedOdometry> odometryList;
    string data_file = findExampleDataFile("Plaza2_DR.txt");
    ifstream is(data_file.c_str());

    while (is) {
        double t, distance_traveled, delta_heading;
        is >> t >> distance_traveled >> delta_heading;
        odometryList.push_back(
                TimedOdometry(t, Pose2(distance_traveled, 0, delta_heading)));
    }
    is.clear(); /* clears the end-of-file and error flags */
    return odometryList;
}

// load the ranges from TD
//    Time (sec)  Sender / Antenna ID Receiver Node ID  Range (m)
typedef boost::tuple<double, size_t, double> RangeTriple;
vector<RangeTriple> readTriples() {
    vector<RangeTriple> triples;
    string data_file = findExampleDataFile("Plaza2_TD.txt");
    ifstream is(data_file.c_str());

    while (is) {
        double t, sender, range;
        double receiver;
        is >> t >> sender >> receiver >> range;
        triples.push_back(RangeTriple(t, size_t(receiver), range));
    }
    is.clear(); /* clears the end-of-file and error flags */
    return triples;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "plazaPose2");
    ros::NodeHandle node;

    ros::Publisher pubPose = node.advertise<nav_msgs::Odometry>("/pose2D", 2);
    ros::Publisher pubPath = node.advertise<nav_msgs::Path>("/posePath", 1);
    ros::Publisher pubLandmark = node.advertise<visualization_msgs::Marker>("/landmark", 1);

    // load Plaza2 data
    list<TimedOdometry> odometry = readOdometry();
    //  size_t M = odometry.size();

    vector<RangeTriple> triples = readTriples();
    size_t K = triples.size();

    // parameters
    size_t minK = 20;//150; // minimum number of range measurements to process initially
    size_t incK = 10;//25; // minimum number of range measurements to process after
    bool groundTruth = false;
    bool robust = true;

    // Set Noise parameters
    Vector priorSigmas = Vector3(1,1,M_PI);
    Vector odoSigmas = Vector3(0.05, 0.01, 0.1);
    double sigmaR = 100; // range standard deviation
    const gtsam::noiseModel::Base::shared_ptr // all same type
            priorNoise = gtsam::noiseModel::Diagonal::Sigmas(priorSigmas), //prior
            odoNoise = gtsam::noiseModel::Diagonal::Sigmas(odoSigmas), // odometry
            gaussian = gtsam::noiseModel::Isotropic::Sigma(1, sigmaR), // non-robust
            tukey = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Tukey::Create(15), gaussian), //robust
            rangeNoise = robust ? tukey : gaussian;

    // Initialize iSAM
    ISAM2 isam;

    // Add prior on first pose
    Pose2 pose0 = Pose2(-34.2086489999201, 45.3007639991120,
                        M_PI - 2.02108900000000);
    NonlinearFactorGraph newFactors;
    newFactors.push_back(PriorFactor<Pose2>(0, pose0, priorNoise));
    Values initial;
    initial.insert(0, pose0);

    //  initialize points
    if (groundTruth) { // from TL file
        initial.insert(symbol('L', 1), Point2(-68.9265, 18.3778));
        initial.insert(symbol('L', 6), Point2(-37.5805, 69.2278));
        initial.insert(symbol('L', 0), Point2(-33.6205, 26.9678));
        initial.insert(symbol('L', 5), Point2(1.7095, -5.8122));
    } else { // drawn from sigma=1 Gaussian in matlab version
        initial.insert(symbol('L', 1), Point2(3.5784, 2.76944));
        initial.insert(symbol('L', 6), Point2(-1.34989, 3.03492));
        initial.insert(symbol('L', 0), Point2(0.725404, -0.0630549));
        initial.insert(symbol('L', 5), Point2(0.714743, -0.204966));
    }

    // set some loop variables
    size_t i = 1; // step counter
    size_t k = 0; // range measurement counter
    bool initialized = false;
    Pose2 lastPose = pose0;
    size_t countK = 0;

    ros::Rate rate(10);
    nav_msgs::Path path;
    path.header.frame_id = "map";
    // Loop over odometry
    gttic_(iSAM);
    for(const TimedOdometry& timedOdometry: odometry) {
        //--------------------------------- odometry loop -----------------------------------------
        double t;
        Pose2 odometry;
        boost::tie(t, odometry) = timedOdometry;

        // add odometry factor
        newFactors.push_back(BetweenFactor<Pose2>(i-1, i, odometry, odoNoise));

        // predict pose and add as initial estimate
        Pose2 predictedPose = lastPose.compose(odometry);
        lastPose = predictedPose;
        initial.insert(i, predictedPose);

        // Check if there are range factors to be added
        // and in this demo the time difference is ignored without any consideration
        while (k < K && t >= boost::get<0>(triples[k])) {
            size_t j = boost::get<1>(triples[k]);
            double range = boost::get<2>(triples[k]);
            newFactors.push_back(RangeFactor<Pose2, Point2>(i, symbol('L', j), range,rangeNoise));
            k = k + 1;
            countK = countK + 1;
        }

        // Check whether to update iSAM 2
        if ((k > minK) && (countK > incK)) {
            if (!initialized) { // Do a full optimize for first minK ranges
                gttic_(batchInitialization);
                LevenbergMarquardtOptimizer batchOptimizer(newFactors, initial);
                initial = batchOptimizer.optimize();
                gttoc_(batchInitialization);
                initialized = true;
            }
            gttic_(update);
            isam.update(newFactors, initial);
            gttoc_(update);
            gttic_(calculateEstimate);
            Values result = isam.calculateEstimate();
            gttoc_(calculateEstimate);
            lastPose = result.at<Pose2>(i);
            newFactors = NonlinearFactorGraph();
            initial = Values();
            countK = 0;

            //publish global path
            path.poses.clear();

            for (int l = 1; l <= i; ++l) {
                Pose2 ips;
                if (result.exists(l))
                     ips= result.at<Pose2>(l);
                path.poses.emplace_back(pose22PathPoint(ips));
            }
            path.header.stamp = ros::Time::now();
            pubPath.publish(path);

            //publish landmark
            visualization_msgs::Marker landmarks;
            landmarks.header.frame_id = "map";
            landmarks.type = visualization_msgs::Marker::POINTS;
            landmarks.action = visualization_msgs::Marker::ADD;
            landmarks.ns = "landmarks";
            landmarks.id=0;
            landmarks.scale.x = 1;
            landmarks.scale.y = 1;
            landmarks.color.r = 1.0f;
            landmarks.color.a = 1.0;

            Point2 p = result.at<Point2>(symbol('L', 1));
            geometry_msgs::Point  l1 = landmark2point(p);
            landmarks.points.emplace_back(l1);

            p = result.at<Point2>(symbol('L', 6));
            geometry_msgs::Point  l6 = landmark2point(p);
            landmarks.points.emplace_back(l6);

            p = result.at<Point2>(symbol('L', 0));
            geometry_msgs::Point  l0 = landmark2point(p);
            landmarks.points.emplace_back(l0);

            p = result.at<Point2>(symbol('L', 5));
            geometry_msgs::Point  l5 = landmark2point(p);
            landmarks.points.emplace_back(l5);

            pubLandmark.publish(landmarks);

        }
        i += 1;

        //publish current pose
        nav_msgs::Odometry posemsg;
        posemsg.header.frame_id = "map";
        posemsg.pose.pose.position.x = lastPose.x();
        posemsg.pose.pose.position.y = lastPose.y();
        posemsg.pose.pose.position.z = 0.0;
        Eigen::Vector3d rpy(0.0,0.0, lastPose.theta());
        Eigen::Matrix3d rot = rpy2dcm(rpy);
        Eigen::Quaterniond q(rot);
        posemsg.pose.pose.orientation.x = q.x();
        posemsg.pose.pose.orientation.y = q.y();
        posemsg.pose.pose.orientation.z = q.z();
        posemsg.pose.pose.orientation.w = q.w();

        pubPose.publish(posemsg);

        rate.sleep();
        //--------------------------------- odometry loop -----------------------------------------
    } // end for
    gttoc_(iSAM);

    // Print timings
    tictoc_print_();

    return 0;
}



