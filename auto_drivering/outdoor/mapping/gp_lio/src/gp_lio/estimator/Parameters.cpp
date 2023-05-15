//
// Created by wchen on 2019/12/3.
//

#include "gp_lio/estimator/Parameters.h"

namespace gp_lio
{

    bool ESTIMATE_EXTRINSIC; //0 have extract para then set constant, 1 have initial guess
    int CERES_NUM_ITERATIONS;
    float CERES_SOLVER_TIME;
    int CERES_SOLVER_THREADS;

    double ACC_NOISE_STD, GYR_NOISE_STD, ACC_W, GYR_W;
    Eigen::Vector3d GRAVITY = Eigen::Vector3d(0.0, 0.0, 9.81);

    std::string GPLIO_RESULT_PATH;
    std::string EXTRINSIC_RESULT_PATH;
    std::string OUTPUT_PATH;
    Eigen::Quaterniond QIL = Eigen::Quaterniond::Identity();
    Eigen::Vector3d TIL = Eigen::Vector3d::Zero();

    double EDGE_FACTOR_WEIGHT;
    double SURF_FACTOR_WEIGHT;
    bool USE_EDGE_FACTOR;
    bool USE_SURF_FACTOR;
    bool USE_GROUND_FACTOR;
    bool USE_IMU_FACTOR;
    bool USE_AUTO_DIFF;

    int FIXED_SIZE;

    int GROUND_SCAN_IND;
    float GROUND_ANGLE_TH;
    int GROUND_FEATURE_RESOLUTION;

// dynamic filter
    float threshold_filter_still;
    float threshold_filter;
    float threshold_filter_turning;
    bool is_turning;
    bool is_still;
    int is_turning_keep_on;
    // bool is_moving;
    // bool is_moving_;

// plane extraction and fitting
    bool USE_PLANE_FACTOR;
    float PLANE_FACTOR_WEIGHT;
    int PLANE_WINDOW_ROWS;
    int PLANE_WINDOW_COLS;
    float PLANE_WINDOW_RATE;
    float PLANE_CLUSTER_CENTER_DISTANCE;
    float PLANE_REGION_DISTANCE_THRESHOLD;
    float PLANE_FITTING_MSE;
    float PLANE_FITTING_DISTANCE_THRESHOLD;
    int PLANE_POINT_SIZE;
    float POINT_MEASUREMENT_NOISE;
    float PLANE_ASSOCIATED_ANGLE;
    float PLANE_ASSOCIATED_OVERLAPPING_RATE;

    float EDGE_FEATURE_TH;
    float SURF_FEATURE_TH;
    float NEAREST_FEATURE_SEARCH_SQ_DIST;

    int SEGMENT_VALID_SIZE;
    bool SAVE_FEATURE_TMP_RESULT;

    float MOVING_CHECK_THRE;
    float KEYFRAME_TIME_DIFF;
    float KEYFRAME_DISTANCE;
    float KEYFRAME_STILL_DISTANCE;
    float KEYFRAME_ROTATION;

    // pose graph optimization
    // search
    int SEARCH_D_S;
    double SEARCH_V_MIN;
    double SEARCH_V_MAX;
    // match
    int MATCHING_WINDOW;
    double MATCHING_U_RATE;
    double MATCHING_THRESHOLD;
    int MATCHING_MIN_IDX;
    float ICP_FITNESS_SCORE;
    // pose graph
    double STD_X;
    double STD_Y;
    double STD_Z;
    double STD_R;
    double STD_P;
    double STD_YAW;

    // loop closure detection
    std::string DETECTION_METHOD;
    int SEQUENCE_N;

    double CLOSED_DISTANCE;

    // map resuse
    bool REUSE_MAP;
    bool FAST_RELOCALIZATION;

    double NDT_FITNESS_SCORE;
    bool GLOBAL_OPT;



    Parameters::Parameters()
    {
    }

    Parameters::~Parameters()
    {
    }

    template <typename T>
    T Parameters::readParam(ros::NodeHandle &n, std::string name)
    {
        T ans;
        if (n.getParam(name, ans))
        {
            ROS_INFO_STREAM("Loaded " << name << ": " << ans);
        }
        else
        {
            ROS_ERROR_STREAM("Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }

    void Parameters::ReadParameters(ros::NodeHandle &nh)
    {

//        std::string config_file = "/home/wchen/Projects/Code/catkin_ws/src/gp_lio/config/CUHK_config_no_extrinsic.yaml";
        std::string config_file = readParam<std::string>(nh, "config_file");
        cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            std::cerr << "ERROR: Wrong path to settings" << std::endl;
        }

        CERES_SOLVER_TIME = fsSettings["max_solver_time"];
        CERES_NUM_ITERATIONS = fsSettings["max_num_iterations"];
        CERES_SOLVER_THREADS = fsSettings["ceres_solver_threads"];

        fsSettings["output_path"] >> OUTPUT_PATH;
        GPLIO_RESULT_PATH = OUTPUT_PATH + "/temp_result/gp_lio_result_no_loop.csv";
        std::cout << "result path " << GPLIO_RESULT_PATH << std::endl;

        ACC_NOISE_STD = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_NOISE_STD = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        GRAVITY.z() = fsSettings["g_norm"];
        int estimate_extrinsic = fsSettings["estimate_extrinsic"];
        ESTIMATE_EXTRINSIC = (estimate_extrinsic == 1);

        EDGE_FACTOR_WEIGHT = fsSettings["edge_factor_weight"];
        SURF_FACTOR_WEIGHT = fsSettings["surf_factor_weight"];
        int use_edge_factor = fsSettings["use_edge_factor"];
        USE_EDGE_FACTOR = (use_edge_factor == 1);
        int use_surf_factor = fsSettings["use_surf_factor"];
        USE_SURF_FACTOR = (use_surf_factor == 1);
        int use_ground_factor = fsSettings["use_ground_factor"];
        USE_GROUND_FACTOR = (use_ground_factor == 1);
        int use_imu_factor = fsSettings["use_imu_factor"];
        USE_IMU_FACTOR = (use_imu_factor == 1);

        if(!USE_SURF_FACTOR && !USE_EDGE_FACTOR && !USE_IMU_FACTOR){
            ROS_ERROR("No factors are included in ceres problem");
            exit(0);
        }

        int use_auto_diff = fsSettings["use_auto_diff"];
        USE_AUTO_DIFF = (use_auto_diff == 1);

        FIXED_SIZE = fsSettings["fixed_size"];
        threshold_filter_still = fsSettings["threshold_filter_still"];
        threshold_filter = fsSettings["threshold_filter"];
        threshold_filter_turning = fsSettings["threshold_filter_turning"];

        int save_feature_tmp_result = fsSettings["save_feature_tmp_result"];
        SAVE_FEATURE_TMP_RESULT = (save_feature_tmp_result == 1);

        SEGMENT_VALID_SIZE = fsSettings["segment_valid_size"];

        GROUND_SCAN_IND = fsSettings["ground_scan_ind"];
        GROUND_ANGLE_TH = fsSettings["ground_angle_th"];
        GROUND_FEATURE_RESOLUTION = fsSettings["ground_feature_resolution"];

        int use_plane = fsSettings["use_plane_factor"];
        USE_PLANE_FACTOR = (use_plane == 1);
        PLANE_FACTOR_WEIGHT = fsSettings["plane_factor_weight"];
        PLANE_WINDOW_ROWS = fsSettings["plane_window_rows"];
        PLANE_WINDOW_COLS = fsSettings["plane_window_cols"];
        PLANE_WINDOW_RATE = fsSettings["plane_window_rate"];
        PLANE_CLUSTER_CENTER_DISTANCE = fsSettings["plane_cluster_center_distance"];
        PLANE_REGION_DISTANCE_THRESHOLD = fsSettings["plane_region_distance_threshold"];
        PLANE_FITTING_MSE = fsSettings["plane_fitting_mse"];
        PLANE_FITTING_DISTANCE_THRESHOLD = fsSettings["plane_fitting_distance_threshold"];
        PLANE_POINT_SIZE = fsSettings["plane_point_size"];
        POINT_MEASUREMENT_NOISE = fsSettings["point_measurement_noise"];
        PLANE_ASSOCIATED_ANGLE = fsSettings["plane_associated_angle"];
        PLANE_ASSOCIATED_OVERLAPPING_RATE = fsSettings["plane_associated_overlapping_rate"];

        ROS_DEBUG_STREAM("Factor construction surf edge imu plane: " << USE_SURF_FACTOR
                                                                     << " " << USE_EDGE_FACTOR << " " << USE_IMU_FACTOR << " " << USE_PLANE_FACTOR);

        EDGE_FEATURE_TH = fsSettings["edge_feature_th"];
        SURF_FEATURE_TH = fsSettings["surf_feature_th"];
        NEAREST_FEATURE_SEARCH_SQ_DIST = fsSettings["nearest_feature_search_sq_dist"];

        KEYFRAME_TIME_DIFF = fsSettings["keyframe_time_diff"];
        KEYFRAME_DISTANCE = fsSettings["keyframe_distance"];
        KEYFRAME_STILL_DISTANCE = fsSettings["keyframe_still_distance"];
        KEYFRAME_ROTATION = fsSettings["keyframe_rotation"];
        MOVING_CHECK_THRE = fsSettings["moving_check_thre"];

        // search
        SEARCH_D_S = fsSettings["d_s"];
        SEARCH_V_MIN = fsSettings["v_min"];
        SEARCH_V_MAX = fsSettings["v_max"];
        // match
        MATCHING_WINDOW = fsSettings["matching_window"];
        MATCHING_U_RATE = fsSettings["matching_u_rate"];
        MATCHING_THRESHOLD = fsSettings["matching_threshold"];
        MATCHING_MIN_IDX = fsSettings["matching_min_idx"];
        ICP_FITNESS_SCORE = fsSettings["icp_fitness_score"];
        // pose graph
        STD_X = fsSettings["std_x"];
        STD_Y = fsSettings["std_y"];
        STD_Z = fsSettings["std_z"];
        STD_R = fsSettings["std_r"];
        STD_P = fsSettings["std_p"];
        STD_YAW = fsSettings["std_yaw"];


        fsSettings["detection_method"] >> DETECTION_METHOD;
        SEQUENCE_N = fsSettings["sequence_n"];
        CLOSED_DISTANCE = fsSettings["closed_distance"];

        int reuse_map = fsSettings["reuse_map"];
        REUSE_MAP = (reuse_map==1);
        int relocalization = fsSettings["fast_relocalization"];
        FAST_RELOCALIZATION = (relocalization == 1);
        int global_opt = fsSettings["global_opt"];
        GLOBAL_OPT = (global_opt == 1);

        if (ESTIMATE_EXTRINSIC)
        {
            ROS_WARN(" Optimize extrinsic param without initial guess!");
            EXTRINSIC_RESULT_PATH = OUTPUT_PATH + "/temp_result/extrinsic_parameter.csv";
            QIL.setIdentity();
            TIL.setZero();
        } else{
            cv::Mat cv_R, cv_T;
            fsSettings["extrinsicRotation"] >> cv_R;
            fsSettings["extrinsicTranslation"] >> cv_T;
            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            cv::cv2eigen(cv_R, eigen_R);
            cv::cv2eigen(cv_T, eigen_T);
            Eigen::Quaterniond Q(eigen_R);
            eigen_R = Q.normalized();
            QIL = eigen_R;
            TIL = eigen_T;
            ROS_INFO_STREAM("Extrinsic_R : " << std::endl
                                             << QIL.toRotationMatrix());
            ROS_INFO_STREAM("Extrinsic_T : " << std::endl
                                             << TIL.transpose());
            ROS_WARN(" fix extrinsic param ");
        }

        fsSettings.release();
    }

} // namespace gp_lio