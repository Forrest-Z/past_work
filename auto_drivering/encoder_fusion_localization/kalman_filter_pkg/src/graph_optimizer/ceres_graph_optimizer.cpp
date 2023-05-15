/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-21 16:43:06
 * @LastEditors: luo
 * @LastEditTime: 2022-01-18 11:31:36
 */

#include "kalman_filter_pkg/graph_optimizer/params/param_pravg.hpp"
#include "kalman_filter_pkg/graph_optimizer/ceres_graph_optimizer.hpp"

namespace KalmanFilter
{

CeresGraphOptimizer::CeresGraphOptimizer(const int N)
:kWindowSize(N)
{
    //config optimizer

    //a. loss function
    config_.loss_function_ptr = std::make_unique<ceres::CauchyLoss>(1.0);

    //b. solver
    config_.options.linear_solver_type = ceres::DENSE_SCHUR;
    config_.options.trust_region_strategy_type = ceres::DOGLEG;
    config_.options.num_threads = 2;
    config_.options.max_num_iterations = 10000;
    config_.options.max_solver_time_in_seconds = 0.10;

    optimized_key_frames_.clear();

    residual_blocks_.relative_pose.clear();
    // residual_blocks_.map_matchin_pose.clear();
    residual_blocks_.imu_pre_integration.clear();

}

CeresGraphOptimizer::~CeresGraphOptimizer()
{


}


//add parameter block for LIO key frame
void CeresGraphOptimizer::AddPRVAGParam(const KeyFrame &lio_key_frame, const bool fixed)
{
    //creat new key frame
    OptimizedKeyFrame optimized_key_frame;

    //a. set timestamp
    optimized_key_frame.time = lio_key_frame.time;

    //b. shall the param block be fixed
    optimized_key_frame.fixed = fixed;

    //c. set init values
    Eigen::Map<Eigen::Vector3d>     pos(optimized_key_frame.prvag + INDEX_P);
    Eigen::Map<Eigen::Vector3d> log_ori(optimized_key_frame.prvag + INDEX_R);
    Eigen::Map<Eigen::Vector3d>     vel(optimized_key_frame.prvag + INDEX_V);
    Eigen::Map<Eigen::Vector3d>     b_a(optimized_key_frame.prvag + INDEX_A);
    Eigen::Map<Eigen::Vector3d>     b_g(optimized_key_frame.prvag + INDEX_G);
    
    pos = lio_key_frame.pose.block<3, 1>(0, 3).cast<double>();

    Sophus::SO3d ori(Eigen::Quaterniond(lio_key_frame.pose.block<3, 3>(0, 0).cast<double>()));
    log_ori = ori.log();

    vel = ori * lio_key_frame.vel.v.cast<double>();

    b_a = lio_key_frame.bias.accel.cast<double>();

    b_g = lio_key_frame.bias.gyro.cast<double>();

    //d. add to data buffer
    optimized_key_frames_.push_back(optimized_key_frame);
    
}


//add residual block for relatice pose constraint from lidar frontend
void CeresGraphOptimizer::AddPRVAGRelativePoseFactor(const int param_index_i, const int param_index_j,
                const Eigen::Matrix4d& relative_pose, const Eigen::VectorXd& noise)
{
    //creat new residual block
    ResidualRelativePose residual_relative_pose;

    //a. set param block IDs
    residual_relative_pose.param_index_i = param_index_i;
    residual_relative_pose.param_index_j = param_index_j;

    //b. set measurement
    residual_relative_pose.m = Eigen::VectorXd::Zero(6);
    //b.1 position
    residual_relative_pose.m.block<3, 1>(INDEX_P, 0) = relative_pose.block<3, 1>(0, 3);
    //b.2 orientation, so3
    residual_relative_pose.m.block<3, 1>(INDEX_R, 0) = Sophus::SO3d(
                    Eigen::Quaterniond(relative_pose.block<3, 3>(0, 0).cast<double>())).log();
    //c set information matrix
    residual_relative_pose.I = GetInformationMatrix(noise);

    //d. add to data buffer
    residual_blocks_.relative_pose.push_back(residual_relative_pose);
    
}

//add residual block for prior pose constraint from map matching
// void CeresGraphOptimizer::AddPRVAGMapMatchingPoseFactor(const int param_index, const Eigen::Matrix4d& prior_pose, 
//                                     const Eigen::VectorXd& noise)
// {
//     //creat new residual block
//     ResidualMapMatchingPose residual_map_matching_pose;

//     //a. set param block ID
//     residual_map_matching_pose.param_index = param_index;

//     //b. set measurement
//     residual_map_matching_pose.m = Eigen::VectorXd::Zero(6);
//     //b.1 position
//     residual_map_matching_pose.m.block<3, 1>(INDEX_P, 0) = prior_pose.block<3, 1>(0, 3);
//     //b.2 orientation, so3
//     residual_map_matching_pose.m.block<3, 1>(INDEX_R, 0) = Sophus::SO3d(
//                 Eigen::Quaterniond(prior_pose.block<3, 3>(0, 0).cast<double>())).log();
    
//     //c. set information matrix
//     residual_map_matching_pose.I = GetInformationMatrix(noise);

//     //d, add to data buffer
//     residual_blocks_.map_matchin_pose.push_back(residual_map_matching_pose);

// }


//add residual block for IMU pre-intrgration constraint from IMU mewsurement
void CeresGraphOptimizer::AddPRVAGIMUPreIntegrationFactor(const int param_index_i, const int param_index_j,
                                        const IMUPreIntegrator::IMUPreIntrgration &imu_pre_intrgration)
{
    //creat new residual block
    ResidualIMUPreIntegration residual_imu_pre_intrgration;

    //a. set param block IDs
    residual_imu_pre_intrgration.param_index_i = param_index_i;
    residual_imu_pre_intrgration.param_index_j = param_index_j;

    //b, set measurement
    //b.1 integration interval
    residual_imu_pre_intrgration.T = imu_pre_intrgration.GetT();

    // std::cout << "-----CeresGraphOptimizer-----222222222222222-----------" << residual_imu_pre_intrgration.T << std::endl;
    //b.2 gravity constant
    residual_imu_pre_intrgration.g = imu_pre_intrgration.GetGravity();
    //b.3 measurement
    residual_imu_pre_intrgration.m = imu_pre_intrgration.GetMeasurement();
    //b.4 information
    residual_imu_pre_intrgration.I = imu_pre_intrgration.GetInformation();
    std::cout << "-----CeresGraphOptimizer-----222222222222222----IIIIIII-------" << std::endl
     << residual_imu_pre_intrgration.I << std::endl;
    //b.5 Jacobian
    residual_imu_pre_intrgration.J = imu_pre_intrgration.GetJacoBian();

    residual_blocks_.imu_pre_integration.push_back(residual_imu_pre_intrgration);
}


//c.2 residual blocks
// FactorPRVAGMapMatchingPose *CeresGraphOptimizer::GetResMapMatchingPose(
//                                 const ResidualMapMatchingPose &res_map_matching_pose)
// {
//     FactorPRVAGMapMatchingPose *factor_map_matching_pose = new FactorPRVAGMapMatchingPose();

//     factor_map_matching_pose->SetMeasurement(res_map_matching_pose.m);
//     factor_map_matching_pose->SetInformation(res_map_matching_pose.I);

//     return factor_map_matching_pose;


// }

FactorPRVAGRelativePose *CeresGraphOptimizer::GetResRelativePose(const ResidualRelativePose& res_relative_pose)
{
    FactorPRVAGRelativePose *factor_relative_pose = new FactorPRVAGRelativePose();

    factor_relative_pose->SetMeasurement(res_relative_pose.m);
    factor_relative_pose->SetInformation(res_relative_pose.I);

    return factor_relative_pose;

}

FactorPRVAGIMUPreIntrgration *CeresGraphOptimizer::GetResIMUPreIntegration(
                                    const ResidualIMUPreIntegration& res_imu_pre_integration)
{
    FactorPRVAGIMUPreIntrgration *factor_imu_pre = new FactorPRVAGIMUPreIntrgration();

    factor_imu_pre->SetT(res_imu_pre_integration.T);

    std::cout << "~~~~~CeresGraphOptimizer~~~~~TTTTTTTTTTTTTTTTTTT~~~~~~~~~~~ = " << res_imu_pre_integration.T << std::endl;
    factor_imu_pre->SetGravity(res_imu_pre_integration.g);
    factor_imu_pre->SetMeasurement(res_imu_pre_integration.m);
    factor_imu_pre->SetInformation(res_imu_pre_integration.I);
    factor_imu_pre->SetJacobian(res_imu_pre_integration.J);

    return factor_imu_pre;

}

// get num of paramter blocks
int CeresGraphOptimizer::GetNumParamBlocks()
{
    return static_cast<int>(optimized_key_frames_.size());

}

//do optimization
bool CeresGraphOptimizer::Optimize()
{
    static int optimization_count = 0;

    //get key frames count
    const int N = GetNumParamBlocks();
    std::cout << "------ optimization-------N =  " << N << std::endl;
    std::cout << "------ optimization-------kWindowSize =  " << kWindowSize << std::endl;
    
    if(kWindowSize + 1 <= N)
    {
        //creat new sliding window optimization problem
        ceres::Problem problem;

        std::cout << "------ parameter--------- " << std::endl;

        // ceres::LocalParameterization *local_parameterization = new ParamPRVAG();

        //a. add parameter blocks
        for(int i = 1; i <= kWindowSize + 1; ++i)
        {
            auto &target_key_frame = optimized_key_frames_.at(N - i);
            std::cout << "------ parameter----problem = 11 " << std::endl;

            ceres::LocalParameterization *local_parameterization = new ParamPRVAG();
            
            // std::cout << "------ parameter----problem =  22" << std::endl;

            problem.AddParameterBlock(target_key_frame.prvag, 15, local_parameterization);

            std::cout << "------ parameter----problem =  22" << std::endl;
            if(target_key_frame.fixed)
            {
                std::cout << "------ parameter----problem =  33" << std::endl;

                problem.SetParameterBlockConstant(target_key_frame.prvag);

                std::cout << "------ parameter----problem =  44" << std::endl;
            }

            std::cout << "-----------parameter--- i = -- " << i << std::endl;
        }


        std::cout << "------ marginalization----constarint----- " << std::endl;
        //add residual blocks
        //b.1 marginalization constraint
        if(
            // !residual_blocks_.map_matchin_pose.empty() &&
            !residual_blocks_.relative_pose.empty() &&
            !residual_blocks_.imu_pre_integration.empty())
        {
            auto &key_frame_m = optimized_key_frames_.at(N - kWindowSize - 1);
            auto &key_frame_r = optimized_key_frames_.at(N - kWindowSize - 0);
                std::cout << "------ marginalization----constarint---00-- " << std::endl;
            // const ceres::CostFunction *factor_map_matching_pose = GetResMapMatchingPose
            //                             (residual_blocks_.map_matchin_pose.front());
            const ceres::CostFunction *factor_relative_pose = 
                                                GetResRelativePose( residual_blocks_.relative_pose.front());
                    std::cout << "------ marginalization----constarint--11--- " << std::endl;                                                
            const ceres::CostFunction *factor_imu_pre = 
                                    GetResIMUPreIntegration( residual_blocks_.imu_pre_integration.front());
                    std::cout << "------ marginalization----constarint---22-- " << std::endl;

            FactorPRVAGMarginalization *factor_marginalization = new FactorPRVAGMarginalization();

            // factor_marginalization->SetResMapMatchingPose(
            //                     factor_map_matching_pose, std::vector<double *>{key_frame_m.prvag});
                std::cout << "------ marginalization----constarint---33-- " << std::endl;
            factor_marginalization->SetResRelativePose(
                                factor_relative_pose, std::vector<double *>{key_frame_m.prvag, key_frame_r.prvag});
                    std::cout << "------ marginalization----constarint---44-- " << std::endl;
            factor_marginalization->SetResIMUPreIntrgration(
                                factor_imu_pre, std::vector<double *>{key_frame_m.prvag, key_frame_r.prvag});
                    std::cout << "------ marginalization----constarint---55-- " << std::endl;
            factor_marginalization->Marginalize(key_frame_r.prvag);
                std::cout << "------ marginalization----constarint--66--- " << std::endl;
            //add marginalization factor into sliding window
            problem.AddResidualBlock(factor_marginalization, NULL, key_frame_r.prvag);

            std::cout << "------ marginalization----constarint--77--- " << std::endl;

            // residual_blocks_.map_matchin_pose.pop_front();
            residual_blocks_.relative_pose.pop_front();
            residual_blocks_.imu_pre_integration.pop_front();
        }

        // std::cout << "------ matching----constarint----- " << std::endl;
        //b.2 map matching pose constarint
        // if(!residual_blocks_.map_matchin_pose.empty())
        // {
        //     for(const auto &residual_map_matching_pose : residual_blocks_.map_matchin_pose)
        //     {
        //         auto &key_frame = optimized_key_frames_.at(residual_map_matching_pose.param_index);
        //         FactorPRVAGMapMatchingPose *factor_map_matching_pose = GetResMapMatchingPose(
        //             residual_map_matching_pose
        //         );
        //         std::cout << "------ matching----constarint--1111111111--- " << kWindowSize << std::endl;
        //         // add map matching factor into sliding window
        //         problem.AddResidualBlock(factor_map_matching_pose, NULL, key_frame.prvag);
        //     }
        // }


        std::cout << "------ relative_pose----constarint----- " << std::endl;
        //b.3 relative pose constraint
        if(!residual_blocks_.relative_pose.empty())
        {
            for(const auto &residual_relative_pose : residual_blocks_.relative_pose)
            {
                auto &key_frame_i = optimized_key_frames_.at(residual_relative_pose.param_index_i);
                auto &key_frame_j = optimized_key_frames_.at(residual_relative_pose.param_index_j);

                FactorPRVAGRelativePose *factor_relative_pose = GetResRelativePose(
                    residual_relative_pose
                );
                std::cout << "------ relative_pose----constarint---111111111-- " << kWindowSize << std::endl;
                //add relative pose factor into sliding window
                problem.AddResidualBlock(factor_relative_pose, NULL, key_frame_i.prvag, key_frame_j.prvag);
            }
        }


        std::cout << "------ pre-intrgration----constarint----- " << std::endl;
        //b.4 IMU pre-intrgration constraint
        if(!residual_blocks_.imu_pre_integration.empty())
        {
            for(const auto &residual_imu_pre : residual_blocks_.imu_pre_integration)
            {
                auto &key_frame_i = optimized_key_frames_.at(residual_imu_pre.param_index_i);
                auto &key_frame_j = optimized_key_frames_.at(residual_imu_pre.param_index_j);

                FactorPRVAGIMUPreIntrgration * factor_imu_pre_integration = GetResIMUPreIntegration(
                    residual_imu_pre
                );
                std::cout << "------ pre-intrgration----constarint-1111111111--- " << kWindowSize << std::endl;
                //add IMU factor into sliding window
                problem.AddResidualBlock(factor_imu_pre_integration, NULL, key_frame_i.prvag, key_frame_j.prvag);
            }
        }
        std::cout << "------ optimization----88-- " << std::endl;
        // solve
        ceres::Solver::Summary summary;

        auto start = std::chrono::steady_clock::now();

        ceres::Solve(config_.options, &problem, &summary);

        auto end = std::chrono::steady_clock::now();

        std::chrono::duration<double> time_used = end - start;
        
        ROS_ERROR("---------------optimizer------------------");
        std::cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;
        std::cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;
        std::cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;
        std::cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;

        // prompt:
        std::cout << "------ Finish Iteration " << optimization_count << " of Sliding Window Optimization -------" 
                    << std::endl
                    << "Time Used: " << time_used.count() << " seconds." << std::endl
                    << "Cost Reduced: " << summary.initial_cost - summary.final_cost << std::endl
                    << summary.BriefReport() << std::endl
                    << std::endl;
        
        return true;
    }
    
    // std::cout << "------ optimization----end-- " << std::endl;
    return false;
}

//get optimized odometry estimation
bool CeresGraphOptimizer::GetLatestOptimizedKeyFrame(KeyFrame& optimized_key_frame)
{
    const int N = GetNumParamBlocks();

    if(0 == N) return false;

    const auto &lates_optimized_key_frame = optimized_key_frames_.back();

    optimized_key_frame = KeyFrame(
        N - 1, lates_optimized_key_frame.time, lates_optimized_key_frame.prvag
    );

    return true;

}

//get optimized LIO key frame state estimation
bool CeresGraphOptimizer::GetOptimizedKeyFrames(std::deque<KeyFrame> &optimized_key_frames)
{
    optimized_key_frames.clear();

    const int N = GetNumParamBlocks();
    if(0 == N) return false;

    for(int param_id = 0; param_id < N; param_id++)
    {
        const auto &optimized_key_frame = optimized_key_frames_.at(param_id);
        optimized_key_frames.emplace_back(param_id, optimized_key_frame.time, optimized_key_frame.prvag);
    }

    return true;
    
}


Eigen::MatrixXd CeresGraphOptimizer::GetInformationMatrix(const Eigen::VectorXd& noise)
{
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(noise.rows(), noise.rows());

    for(int i = 0; i < noise.rows(); i++)
    {
        information_matrix(i, i) /= noise(i);
    }

    return information_matrix;
    
}



// bool CeresGraphOptimizer::Optimizer()
// {

    
// }

}