//
// Created by wchen on 2019/11/29.
//

#ifndef SRC_UTILITY_H
#define SRC_UTILITY_H

// C header
#include <cmath>
#include <numeric>
// std
#include <vector>
#include <iostream>
// ros
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "parameters.h"


namespace gp_lio
{

    typedef std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloud2ConstPtr>> MeasurementsGroupType;
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    struct CloudInfo
    {

        std::vector<int> startRingInd_;
        std::vector<int> endRingInd_;

        float startOrientation_;
        float endOrientation_;
        float orientationDiff_;

        std::vector<bool> segmentedCloudGroundFlag_;
        std::vector<uint> segmentedCloudColInd_;
        std::vector<float> segmentedCloudRange_;

        void clear()
        {
            startRingInd_.clear();
            endRingInd_.clear();
            segmentedCloudGroundFlag_.clear();
            segmentedCloudColInd_.clear();
            segmentedCloudRange_.clear();
        }
    };

    struct smoothness_t
    {
        float value;
        size_t ind;
    };

    struct by_value
    {
        bool operator()(smoothness_t const &left, smoothness_t const &right)
        {
            return left.value < right.value;
        }
    };

    class State
    {

    public:
        State() { clear(); };
        ~State(){};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double timestamp_;
        Eigen::Vector3d position_;
        Eigen::Quaterniond quaternion_;
        Eigen::Vector3d linear_velocity_;
        Eigen::Vector3d angular_velocity_; // in body-frame, being transfromed to {world} is complicated and meaningless.
        Eigen::Vector3d bias_a_;
        Eigen::Vector3d bias_g_;

        Eigen::Matrix4d getTransformation()
        {

            Eigen::Matrix4d mat4d;
            mat4d.setIdentity();

            mat4d.block<3, 3>(0, 0) = quaternion_.toRotationMatrix();
            mat4d.block<3, 1>(0, 3) = position_;

            return mat4d;
        };

        Eigen::Matrix4d getInverseTransformation(){

            Eigen::Matrix4d mat4d;
            mat4d.setIdentity();

            mat4d.block<3, 3>(0, 0) = quaternion_.toRotationMatrix().transpose();
            mat4d.block<3, 1>(0, 3) = - quaternion_.toRotationMatrix().transpose() * position_;
            return mat4d;
        }

        void resetPose(const Eigen::Matrix4d &mat)
        {
            Eigen::Matrix3d rot = mat.block(0, 0, 3, 3);
            Eigen::Quaterniond qua(rot);
            quaternion_ = qua;
            Eigen::Vector3d t = mat.block(0, 3, 3, 1);
            position_ = t;
        };

        void clear()
        {

            timestamp_ = 0.0;
            position_.setZero();
            quaternion_.setIdentity();
            linear_velocity_.setZero();
            angular_velocity_.setZero();
            bias_a_.setZero();
            bias_g_.setZero();
        };

        void swap(State &state)
        {
            timestamp_ = state.timestamp_;
            position_ = state.position_;
            quaternion_ = state.quaternion_;
            linear_velocity_ = state.linear_velocity_;
            angular_velocity_ = state.angular_velocity_;
            bias_a_ = state.bias_a_;
            bias_g_ = state.bias_g_;
        };

        Eigen::Vector3d getYPR(){

            Eigen::Matrix3d R = quaternion_.matrix();

            Eigen::Vector3d n = R.col(0);
            Eigen::Vector3d o = R.col(1);
            Eigen::Vector3d a = R.col(2);

            Eigen::Vector3d ypr(3);
            double y = atan2(n(1), n(0));
            double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
            double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
            ypr(0) = y;
            ypr(1) = p;
            ypr(2) = r;

            return ypr / M_PI * 180.0;

        }
    };

    struct ExternalParameters
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Quaterniond quaternion_; // {B}{L}R
        Eigen::Vector3d position_;      // {B}{L}p
        float dt_;

        ExternalParameters()
        {
            clear();
        }

        void clear()
        {
            quaternion_.setIdentity();
            position_.setZero();
            dt_ = 0.0;
        };
        Eigen::Matrix4d getTransform()
        {
            Eigen::Matrix4d mat4d;
            mat4d.setIdentity();
            mat4d.block<3, 3>(0, 0) = quaternion_.toRotationMatrix();
            mat4d.block<3, 1>(0, 3) = position_;
            return mat4d;
        };
    };

    struct ImuMeasurement
    {
    public:
        ImuMeasurement()
        {
            at_ = 0;
            dt_ = 0;
            ddx_ = 0;
            ddy_ = 0;
            ddz_ = 0;
            drx_ = 0;
            dry_ = 0;
            drz_ = 0;
        };

        ImuMeasurement(double at, double dt, Eigen::Vector3d acc, Eigen::Vector3d gyr)
        {
            at_ = at;
            dt_ = dt;
            ddx_ = acc.x();
            ddy_ = acc.y();
            ddz_ = acc.z();
            drx_ = gyr.x();
            dry_ = gyr.y();
            drz_ = gyr.z();
        };

        void setIMU(double at, double dt, Eigen::Vector3d acc, Eigen::Vector3d gyr)
        {
            at_ = at;
            dt_ = dt;
            ddx_ = acc.x();
            ddy_ = acc.y();
            ddz_ = acc.z();
            drx_ = gyr.x();
            dry_ = gyr.y();
            drz_ = gyr.z();
        };

        Eigen::Vector3d getAcc()
        {
            return Eigen::Vector3d(ddx_, ddy_, ddz_);
        };

        Eigen::Vector3d getGyr()
        {
            return Eigen::Vector3d(drx_, dry_, drz_);
        };

        double at_; // absolute time
        double dt_; // time shift from last imu data
        double ddx_;
        double ddy_;
        double ddz_;
        double drx_;
        double dry_;
        double drz_;
    };

// point cloud and assign each point with a timestamp
    struct CloudMeasurement
    {

        CloudMeasurement(sensor_msgs::PointCloud2ConstPtr &cloud_msg)
        {

            pcl::fromROSMsg(*cloud_msg, cloud_);
            receive_time_ = cloud_msg->header.stamp.toSec();
        };

        PointCloudT cloud_;
        double receive_time_;
        // in lego_loam, this time represents the first point timestamp.
        // But in the velodyne_driver, the timestamp of the output cloud is the packets.back.stamp.
        // Thus, we should treat the receive_time as the end time of the cloud. Thus the relative time
        // for each point should be negative.
    };

    struct PlaneFeature{

        PlaneFeature(){
            clear();
        }

        void Set(PointCloudT cloud, Eigen::Vector3f center, Eigen::Vector3f normal, float d, float radius){
            cloud_ = cloud;
            center_ = center;
            normal_ = normal;
            d_ = d;
            cp_ = normal_*d_;
            radius_ = radius;
        };

        Eigen::Vector3f GetCP(){
            return normal_*d_;
        }

        void ResetCP(Eigen::Vector3f cp, Eigen::Matrix3f covariance){
            cp_ = cp;
            d_ = cp_.norm();
            normal_ = cp_/d_;
            covariance_ = covariance;
        }

        void Transform(Eigen::Matrix4d mat){

            Eigen::Matrix4f mat_f = mat.cast<float>();

            Eigen::Vector4f plane_param = Eigen::Vector4f(normal_[0], normal_[1], normal_[2], d_);

            Eigen::Vector4f transformed_plane = mat_f.inverse().transpose() * plane_param;

            normal_ = Eigen::Vector3f(transformed_plane[0], transformed_plane[1], transformed_plane[2]);
            d_ = transformed_plane[3];
            cp_ = normal_ * d_;

            Eigen::Matrix3d rot = mat.block(0, 0, 3, 3);
            Eigen::Vector3d t = mat.block(0, 3, 3, 1);

            Eigen::Vector3d center = center_.cast<double>();
            center = rot * center + t;
            center_ = center.cast<float>();

            PointCloudT cloud_new = cloud_;
            pcl::transformPointCloud(cloud_, cloud_new, mat);
            cloud_ = cloud_new;

        }

        std::vector<Eigen::Vector2f> Get2DLine(){

            Eigen::Vector2f center_xy(center_[0], center_[1]);
            Eigen::Vector2f normal_xy(normal_[0], normal_[1]);
            Eigen::Vector2f normal_pen(normal_[1], -normal_[0]);

            Eigen::Vector2f point_0 = center_xy + radius_ * normal_pen;
            Eigen::Vector2f point_1 = center_xy - radius_ * normal_pen;
            Eigen::Vector2f temp_point;
            temp_point = point_0;
            if(point_0[0] > point_1[0]){
                point_0 = point_1;
                point_1 = temp_point;
            }

            std::vector<Eigen::Vector2f> line_points;
            line_points.resize(2);
            line_points.at(0) = point_0;
            line_points.at(1) = point_1;

            return line_points;
        }

        void clear(){
            cloud_.clear();
            center_.setZero();
            normal_.setZero();
            d_ = 0.0;
            radius_ = 0.0;
            cp_.setZero();
            covariance_.setIdentity();
        }

        PointCloudT cloud_;
        Eigen::Vector3f center_;
        Eigen::Vector3f normal_;
        float d_;
        float radius_;
        Eigen::Vector3f cp_;
        Eigen::Matrix3f covariance_;

    };

    struct CloudFeature
    {
        CloudFeature()
        {
            clear();
        }

        ~CloudFeature()
        {
        }

        double GetFeatureTimeStamp(PointT &point)
        {
            double relTime = point.intensity - int(point.intensity); // the relative time between the start point and the point
            return (cloudTimeStamp_ - (SCAN_DURATION - relTime));
        };

        void clear()
        {
            cloudTimeStamp_ = 0.0;
            surf_feature_.clear();
            edge_feature_.clear();
            less_surf_feature_.clear();
            less_edge_feature_.clear();
            ground_feature_.clear();
            outlier_feature_.clear();
            plane_cloud_.clear();
            plane_feature_.clear();
        };

        void save(std::string save_path){

            PointCloudT::Ptr cloud(new PointCloudT);
            *cloud += surf_feature_;
            *cloud += edge_feature_;
            *cloud += less_surf_feature_;
            *cloud += less_edge_feature_;
            *cloud += ground_feature_;
            *cloud += outlier_feature_;

            pcl::io::savePCDFileBinary(save_path, *cloud);
        }

        double cloudTimeStamp_; // is the timestamp of the end point

        PointCloudT surf_feature_;
        PointCloudT edge_feature_;
        PointCloudT less_surf_feature_;
        PointCloudT less_edge_feature_;
        PointCloudT ground_feature_;
        PointCloudT outlier_feature_;
        PointCloudT plane_cloud_;
        std::vector<PlaneFeature> plane_feature_;
    };

    enum FeatureType
    {
        Surf,
        Edge,
        Ground
    };

    struct FeatureCorPairs
    {

        PointT feature_; // represented in {L} not in {W}
        PointT corresponding_center_;
        Eigen::Vector3d dir_vec_;
        std::vector<PointT> corresponding_points_;
        FeatureType feature_type_;
        double distance_;

        void setFeatureType(const FeatureType &feature_type)
        {
            feature_type_ = feature_type;
        };
    };
    typedef std::vector<FeatureCorPairs> FeatureCorPairsGroup;

    struct PlaneCorPair{

        PlaneCorPair(Eigen::Vector3f plane_cp_0, Eigen::Vector3f plane_cp_1, int index_0, int index_1){
            plane_cp_0_ = plane_cp_0;
            plane_cp_1_ = plane_cp_1;
            index_0_ = index_0;
            index_1_ = index_1;
        }

        int index_0_;
        int index_1_;

        Eigen::Vector3f plane_cp_0_;
        Eigen::Vector3f plane_cp_1_;

    };
    typedef std::vector<PlaneCorPair> PlaneCorPairsGroup;

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    enum StateOrder
    {
        O_P = 0,
        O_R = 3,
        O_V = 6,
        O_BA = 9,
        O_BG = 12
    };

    // for pose graph optimization
    struct DescriptorVector{
        std::vector<float> descriptor_;
        bool calculated_;
    };

    struct LoopClosurePair{

        std::pair<std::vector<int>, std::vector<int>> traj_pair_;
        Eigen::Matrix3f rel_rotation_;
        Eigen::Vector3f rel_trans_;
        float fitness_score_;
        bool convergence_;

        LoopClosurePair(){
            fitness_score_ = FLT_MAX;
            convergence_ = false;
        }

        Eigen::Matrix4f GetTransform(){
            Eigen::Matrix4f mat4f;
            mat4f.setIdentity();
            mat4f.block<3, 3>(0, 0) = rel_rotation_;
            mat4f.block<3, 1>(0, 3) = rel_trans_;
            return mat4f;
        }

    };

    struct RegistrationResult{

        Eigen::Matrix3f rel_rotation_;
        Eigen::Vector3f rel_trans_;
        float fitness_score_;
        bool convergence_;

        RegistrationResult(){
            rel_rotation_.setIdentity();
            rel_trans_.setIdentity();
            fitness_score_ = FLT_MAX;
            convergence_ = false;
        }

        Eigen::Matrix4f GetTransformation()
        {
            Eigen::Matrix4f mat4d;
            mat4d.setIdentity();
            mat4d.block<3, 3>(0, 0) = rel_rotation_;
            mat4d.block<3, 1>(0, 3) = rel_trans_;
            return mat4d;
        };

        // from vins utility.h (refer to wiki Euler angles, ypr->ZYX order
        Eigen::Vector3f GetYPR()
        {

            Eigen::Vector3f n = rel_rotation_.col(0);
            Eigen::Vector3f o = rel_rotation_.col(1);
            Eigen::Vector3f a = rel_rotation_.col(2);

            Eigen::Vector3f ypr(3);
            float y = atan2(n(1), n(0));
            float p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
            float r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
            ypr(0) = y;
            ypr(1) = p;
            ypr(2) = r;

            return ypr;
        }
    };

    class Utility
    {

    public:
        // compute delta quaternion using the small angle changes
        template <typename Derived>
        static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &angle)
        {
            typedef typename Derived::Scalar Scalar_t;

            // using the approximate algorithm refer to vins-mono
            // due to the extremely small norm of the angle vector (AxisAngle)
            Eigen::Quaternion<Scalar_t> dq;
            Eigen::Matrix<Scalar_t, 3, 1> half_theta = angle;
            half_theta /= static_cast<Scalar_t>(2.0);
            dq.w() = static_cast<Scalar_t>(1.0);
            dq.x() = half_theta.x();
            dq.y() = half_theta.y();
            dq.z() = half_theta.z();
            dq.normalize();

            // the extract algorithm
            // http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.htm
            if (0)
            {
                Eigen::Matrix<Scalar_t, 3, 1> angle_ = angle;
                double angle_vector_x = angle_.x();
                double angle_vector_y = angle_.y();
                double angle_vector_z = angle_.z();
                double angle_scalar = angle_.norm();
                dq.x() = angle_vector_x * sin(angle_scalar / 2.0);
                dq.y() = angle_vector_y * sin(angle_scalar / 2.0);
                dq.z() = angle_vector_z * sin(angle_scalar / 2.0);
                dq.w() = cos(angle_scalar / 2.0);
                dq.normalize();
            }
            return dq;
        }

        // from vins utility.h (refer to wiki Euler angles, ypr->ZYX order
        static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
        {

            Eigen::Vector3d n = R.col(0);
            Eigen::Vector3d o = R.col(1);
            Eigen::Vector3d a = R.col(2);

            Eigen::Vector3d ypr(3);
            double y = atan2(n(1), n(0));
            double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
            double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
            ypr(0) = y;
            ypr(1) = p;
            ypr(2) = r;

            return ypr / M_PI * 180.0;
        }

        static Eigen::Vector3f R2ypr(const Eigen::Matrix3f &R)
        {

            Eigen::Vector3f n = R.col(0);
            Eigen::Vector3f o = R.col(1);
            Eigen::Vector3f a = R.col(2);

            Eigen::Vector3f ypr(3);
            float y = atan2(n(1), n(0));
            float p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
            float r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
            ypr(0) = y;
            ypr(1) = p;
            ypr(2) = r;

            return ypr / M_PI * 180.0;
        }

        template <typename Derived>
        static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
        {
            typedef typename Derived::Scalar Scalar_t;

            Scalar_t y = ypr(0) / 180.0 * M_PI;
            Scalar_t p = ypr(1) / 180.0 * M_PI;
            Scalar_t r = ypr(2) / 180.0 * M_PI;

            Eigen::Matrix<Scalar_t, 3, 3> Rz;
            Rz << cos(y), -sin(y), 0,
                    sin(y), cos(y), 0,
                    0, 0, 1;

            Eigen::Matrix<Scalar_t, 3, 3> Ry;
            Ry << cos(p), 0., sin(p),
                    0., 1., 0.,
                    -sin(p), 0., cos(p);

            Eigen::Matrix<Scalar_t, 3, 3> Rx;
            Rx << 1., 0., 0.,
                    0., cos(r), -sin(r),
                    0., sin(r), cos(r);

            return Rz * Ry * Rx;
        }

        template <typename Derived>
        static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
        {
            Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
            ans << typename Derived::Scalar(0), -q(2), q(1),
                    q(2), typename Derived::Scalar(0), -q(0),
                    -q(1), q(0), typename Derived::Scalar(0);
            return ans;
        }

        template <typename Derived>
        static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
        {
            Eigen::Quaternion<typename Derived::Scalar> qq = q;
            Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
            ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
            ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
            return ans;
        }

        template <typename Derived>
        static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
        {
            Eigen::Quaternion<typename Derived::Scalar> pp = p;
            Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
            ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
            ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
            return ans;
        }

        template <typename Derived>
        static Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(const Eigen::MatrixBase<Derived> &v3d)
        {
            Eigen::Matrix<typename Derived::Scalar, 3, 3> m;
            m << typename Derived::Scalar(0), -v3d.z(), v3d.y(),
                    v3d.z(), typename Derived::Scalar(0), -v3d.x(),
                    -v3d.y(), v3d.x(), typename Derived::Scalar(0);
            return m;
        }

        static Eigen::Matrix3d g2R(const Eigen::Vector3d &g)
        {
            Eigen::Matrix3d R0;
            Eigen::Vector3d ng1 = g.normalized();
            Eigen::Vector3d ng2{0, 0, 1.0};
            R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
            double yaw = Utility::R2ypr(R0).x() /180.0 * M_PI;
            R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
            // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
            return R0; // return a rotation matrix which can rotate a local vector to global vector
        }

        static void ComputeMeanStdev(double &stdev, std::vector<double> &vec)
        {

            double sum = std::accumulate(std::begin(vec), std::end(vec), 0.0);
            double mean = sum / double(vec.size());

            double accum = 0.0;
            std::for_each(std::begin(vec), std::end(vec), [&](const double d) {
                accum += (d - mean) * (d - mean);
            });

            stdev = sqrt(accum / double(vec.size() - 1));
        }
    };

} // namespace gp_lio

#endif //SRC_UTILITY_H
