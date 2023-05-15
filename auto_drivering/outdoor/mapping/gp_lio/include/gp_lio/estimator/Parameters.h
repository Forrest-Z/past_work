//
// Created by wchen on 2019/12/3.
//

#ifndef SRC_PARAMETERS_H
#define SRC_PARAMETERS_H

#include <ros/ros.h>

#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace gp_lio
{
    const int WINDOW_SIZE = 5;
    extern int FIXED_SIZE;

// VLP16
    const float SCAN_DURATION = 0.1;
    const int N_SCAN = 16; //16
    const int HORIZON_SCAN = 1800; //1800
    const float ANGLE_RES_X = 0.2; //0.2
    const float ANGLE_RES_Y = 2.0; //2.0
    const float ANGLE_BOTTOM = 15.0+0.1; //15.0 + 0.1;

//// livox horizon
//    const float SCAN_DURATION = 0.1;
//    const int N_SCAN = 100; //16
//    const int HORIZON_SCAN = 272; //1800
//    const float ANGLE_RES_X = 0.27; //0.2
//    const float ANGLE_RES_Y = 0.30; //2.0
//    const float ANGLE_BOTTOM = 12.5+0.1; //15.0 + 0.1;

// Ground points extraction
    extern int GROUND_SCAN_IND;
    const float SENSOR_MOUNT_ANGLE = 0.0;
    extern float GROUND_ANGLE_TH;
    extern int GROUND_FEATURE_RESOLUTION;

    const float SEGMENT_ALPHA_X = ANGLE_RES_X / 180.0 * M_PI;
    const float SEGMENT_ALPHA_Y = ANGLE_RES_Y / 180.0 * M_PI;
    const float SEGMENT_THETA = 20.0 / 180.0 * M_PI; // decrease this angle may improve accuracy //60
    const int SEGMENT_VALID_POINT_NUM = 5;
    const int SEGMENT_VALID_LINE_NUM = 3;
    extern int SEGMENT_VALID_SIZE;
    
// dynamic filter
    extern float threshold_filter_still;
    extern float threshold_filter;
    extern float threshold_filter_turning;
    extern bool is_turning;
    extern bool is_still;
    extern int is_turning_keep_on;
    // extern bool is_moving;
    // extern bool is_moving_;

// plane extraction and fitting
    extern bool USE_PLANE_FACTOR;
    extern float PLANE_FACTOR_WEIGHT;
    extern int PLANE_WINDOW_ROWS;
    extern int PLANE_WINDOW_COLS;
    extern float PLANE_WINDOW_RATE;
    extern float PLANE_CLUSTER_CENTER_DISTANCE;
    extern float PLANE_REGION_DISTANCE_THRESHOLD;
    extern float PLANE_FITTING_MSE;
    extern float PLANE_FITTING_DISTANCE_THRESHOLD;
    extern int PLANE_POINT_SIZE;
    extern float POINT_MEASUREMENT_NOISE;
    extern float PLANE_ASSOCIATED_ANGLE;
    extern float PLANE_ASSOCIATED_OVERLAPPING_RATE;

    extern float EDGE_FEATURE_TH;
    extern float SURF_FEATURE_TH;
    extern float NEAREST_FEATURE_SEARCH_SQ_DIST;

    extern bool SAVE_FEATURE_TMP_RESULT;

    const float DOWNSIZE_Th = 0.3;
    const int NEAREST_POINTS_NUM = 5;

// Odometry
    extern float MOVING_CHECK_THRE;
    extern float KEYFRAME_TIME_DIFF;
    extern float KEYFRAME_DISTANCE;
    extern float KEYFRAME_STILL_DISTANCE;
    extern float KEYFRAME_ROTATION;

// extern type config parameters needs to be set firstly
    extern bool ESTIMATE_EXTRINSIC; //0 have extract para then set constant, 1 have initial guess
    extern int CERES_NUM_ITERATIONS;
    extern float CERES_SOLVER_TIME;
    extern int CERES_SOLVER_THREADS;

    extern double ACC_NOISE_STD, GYR_NOISE_STD, ACC_W, GYR_W;
    extern Eigen::Vector3d GRAVITY;

    extern std::string GPLIO_RESULT_PATH;
    extern std::string EXTRINSIC_RESULT_PATH;
    extern std::string OUTPUT_PATH;
    extern Eigen::Quaterniond QIL;
    extern Eigen::Vector3d TIL;

    extern double EDGE_FACTOR_WEIGHT;
    extern double SURF_FACTOR_WEIGHT;
    extern bool USE_EDGE_FACTOR;
    extern bool USE_SURF_FACTOR;
    extern bool USE_GROUND_FACTOR;
    extern bool USE_AUTO_DIFF;
    extern bool USE_IMU_FACTOR;

// pose graph optimization
    // search
    extern int SEARCH_D_S;
    extern double SEARCH_V_MIN;
    extern double SEARCH_V_MAX;
    // match
    extern int MATCHING_WINDOW;
    extern double MATCHING_U_RATE;
    extern double MATCHING_THRESHOLD;
    extern int MATCHING_MIN_IDX;
    extern float ICP_FITNESS_SCORE;
    // pose graph
    extern double STD_X;
    extern double STD_Y;
    extern double STD_Z;
    extern double STD_R;
    extern double STD_P;
    extern double STD_YAW;

    // loop closure detection
    extern std::string DETECTION_METHOD;  // loop closure detection method
    extern int SEQUENCE_N; // sequence length

    extern double CLOSED_DISTANCE;

    // map resuse
    extern bool REUSE_MAP;
    extern bool FAST_RELOCALIZATION;
    extern double NET_FITNESS_SCORE;

    extern bool GLOBAL_OPT;

    enum SIZE_PAEAMETERIZATION
    {
        SIZE_POSE = 7,
        SIZE_VELOCITY_BIAS = 9,
    };

    class Parameters
    {

    public:
        Parameters();

        ~Parameters();

        void ReadParameters(ros::NodeHandle &nh);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // extrinsic parameters
        Eigen::Matrix3d RIL_;
        Eigen::Vector3d TIL_;
        Eigen::Vector3d Gravity_;
        double TD_;

        // Path
        std::string ConfigPath_;
        std::string ResultPath_;

    private:
        template <typename T>
        T readParam(ros::NodeHandle &n, std::string name);
    };
} // namespace gp_lio

#endif //SRC_PARAMETERS_H
