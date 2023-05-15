//
// Created by wchen on 2020/7/21.
//

#include "ros_test/pose_graph.h"

namespace gp_lio {

    pose_graph::pose_graph(): nh_("~"), optimized_(false){

        pub_pose_graph_ = nh_.advertise<nav_msgs::Path>("optimised_pose_graph", 1);
        pub_origin_pose_graph_ = nh_.advertise<sensor_msgs::PointCloud2>("origin_pose_graph", 1);
        LoadGraph();

    }

    pose_graph::~pose_graph() {

    }

    void pose_graph::LoadGraph() {

        std::string data_path = "/home/wchen/Projects/Code/catkin_ws/src/gp_lio/result/tmp_result/loop_1_pose_graph.txt";

        std::ifstream fin(data_path);
        std::string i_line;

        if(fin.is_open()){
            while (getline(fin, i_line)){

                std::vector<float> line_v;

                std::stringstream ss(i_line);

                std::string splited;

                while(ss >> splited){
                    float data = std::atof(splited.c_str());
                    line_v.push_back(data);
//                std::cout <<  std::fixed << std::setprecision(6) << data << " ";
                }

                if (line_v.at(0) == 0){
                    Pose i_pose;
                    i_pose.position_.x() = double(line_v.at(2));
                    i_pose.position_.y() = double(line_v.at(3));
                    i_pose.position_.z() = double(line_v.at(4));
                    i_pose.quaternion_.x() = double(line_v.at(5));
                    i_pose.quaternion_.y() = double(line_v.at(6));
                    i_pose.quaternion_.z() = double(line_v.at(7));
                    i_pose.quaternion_.w() = double(line_v.at(8));
                    graph_pose_v_.emplace_back(i_pose);
                } else {
                    loop_pair i_loop;
                    i_loop.pair_idx_.first = line_v.at(1);
                    i_loop.pair_idx_.second = line_v.at(2);
                    i_loop.rel_pose.position_.x() = double(line_v.at(3));
                    i_loop.rel_pose.position_.y() = double(line_v.at(4));
                    i_loop.rel_pose.position_.z() = double(line_v.at(5));
                    i_loop.rel_pose.quaternion_.x() = double(line_v.at(6));
                    i_loop.rel_pose.quaternion_.y() = double(line_v.at(7));
                    i_loop.rel_pose.quaternion_.z() = double(line_v.at(8));
                    i_loop.rel_pose.quaternion_.w() = double(line_v.at(9));
                    loops_v_.emplace_back(i_loop);
                }

//            std::cout << std::endl;
            }

            fin.close();

            for (auto & j : graph_pose_v_) {
                PointT position;
                position.x = j.position_.x();
                position.y = j.position_.y();
                position.z = j.position_.z();
                origin_pose_graph_.points.push_back(position);
            }

        } else{
            std::cerr << "cannot read file " << data_path  << std::endl;
            exit(0);
        }

    }

    void pose_graph::Run() {

        ros::Rate rate(10);

        while(ros::ok()){

//            Optimization();
            Optimize4DOF();
            Publish();
            rate.sleep();
            ros::spinOnce();
        }

    }

    void pose_graph::Optimization() {

        if(!optimized_) {

            // 1 --- build ceres problem
            double STD_X = 1e-4, STD_Y = 1e-4, STD_Z = 1e-4, STD_R = 1e4, STD_P = 1e4, STD_YAW = 1e-6;

            ceres::Problem problem;
            ceres::LossFunction *lossFunction;
            lossFunction = new ceres::HuberLoss(0.5);
            static size_t kf_size = graph_pose_v_.size();
            double para_pose[kf_size][SIZE_POSE];

            for (int i = 0; i < kf_size; i++) {
                ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
                problem.AddParameterBlock(para_pose[i], SIZE_POSE, local_parameterization);
            }

            problem.SetParameterBlockConstant(para_pose[0]);

            // States to Para
            for (size_t i = 0; i < kf_size; i++) {
                para_pose[i][0] = graph_pose_v_.at(i).position_.x();
                para_pose[i][1] = graph_pose_v_.at(i).position_.y();
                para_pose[i][2] = graph_pose_v_.at(i).position_.z();
                para_pose[i][3] = graph_pose_v_.at(i).quaternion_.x();
                para_pose[i][4] = graph_pose_v_.at(i).quaternion_.y();
                para_pose[i][5] = graph_pose_v_.at(i).quaternion_.z();
                para_pose[i][6] = graph_pose_v_.at(i).quaternion_.w();
            }

            // add pose graph
            Eigen::Matrix<double, 6, 6> covariance;
            covariance.setZero();
//        Eigen::DiagonalMatrix<double, 6> covariance_diag;
            covariance.diagonal() << STD_X * STD_X, STD_Y * STD_Y, STD_Z * STD_Z, STD_R * STD_R, STD_P * STD_P,
                    STD_YAW * STD_YAW;
//        covariance = covariance_diag.toDenseMatrix();
            std::cout << "near covariance" << std::endl << covariance << std::endl;
            Eigen::Matrix<double, 6, 6> sqrt_information = covariance.llt().matrixL();

            for (int i = 1; i < kf_size; ++i) {

                for (int j = 1; j < 5; j++)
                {
                    if (i - j >= 0)
                    {
                        Eigen::Vector3d pose_A_p = graph_pose_v_.at(i - j).position_;
                        Eigen::Quaterniond pose_A_q = graph_pose_v_.at(i - j).quaternion_;
                        Eigen::Vector3d pose_B_p = graph_pose_v_.at(i).position_;
                        Eigen::Quaterniond pose_B_q = graph_pose_v_.at(i).quaternion_;

                        Eigen::Quaterniond rel_q = pose_A_q.conjugate() * pose_B_q;
                        Eigen::Vector3d rel_p = pose_A_q.conjugate() * (pose_B_p - pose_A_p);
                        ceres::CostFunction *cost_function = PoseGraph3dErrorTerm::Create(rel_p, rel_q, sqrt_information);
                        problem.AddResidualBlock(cost_function, NULL, para_pose[i-j], para_pose[i]);

                    }
                }

            }

            // add loop closure factor
            if (!loops_v_.empty()) {

                for (auto &k : loops_v_) {

                    int i_a = k.pair_idx_.first;
                    int i_b = k.pair_idx_.second;

                    problem.SetParameterBlockConstant(para_pose[i_a]);

                    std::cout << "Add loop pair: " << i_a << " - " << i_b << std::endl;

                    Eigen::Quaterniond rel_q = k.rel_pose.quaternion_;
                    Eigen::Vector3d rel_p = k.rel_pose.position_;

                    covariance.setIdentity();
//                    covariance = covariance * pow(10, 2);
                    sqrt_information = covariance.llt().matrixL();

                    std::cout << "loop covariance" << std::endl << covariance << std::endl;

                    ceres::CostFunction *cost_function = PoseGraph3dErrorTerm::Create(rel_p, rel_q, sqrt_information);
                    problem.AddResidualBlock(cost_function, NULL, para_pose[i_a], para_pose[i_b]);

                    int window_size = 10;
                    for (int i = -window_size/2; i < window_size/2; ++i) {
                        if(std::max(i_a+i,i_b+i)>=graph_pose_v_.size() || std::min(i_a+i, i_b+i) < 0) continue;

                        Eigen::Matrix4d  w_ia_T  = graph_pose_v_.at(i_a).Get_T();
                        Eigen::Matrix4d  w_ib_T  = graph_pose_v_.at(i_b).Get_T();
                        Eigen::Matrix4d  w_ia_i_T  = graph_pose_v_.at(i_a+i).Get_T();
                        Eigen::Matrix4d  w_ib_i_T  = graph_pose_v_.at(i_b+i).Get_T();
                        Eigen::Matrix4d ia_i_ia_T = w_ia_i_T.inverse() * w_ia_T;
                        Eigen::Matrix4d ib_i_ib_T = w_ib_i_T.inverse() * w_ib_T;
                        Eigen::Matrix4d  ia_ib_T =  k.rel_pose.Get_T();
                        Eigen::Matrix4d ia_i_ib_i_T = ia_i_ia_T * ia_ib_T * ib_i_ib_T.inverse();
                        std::cout << "Add loop pair: " << i_a+i << " - " << i_b+i << std::endl;
                        Eigen::Matrix3d ia_i_ib_i_R = ia_i_ib_i_T.block(0, 0, 3, 3);
                        Eigen::Quaterniond q(ia_i_ib_i_R);
                        Eigen::Vector3d ia_i_ib_i_t = ia_i_ib_i_T.block(0, 3, 3, 1);

                        Eigen::Quaterniond rel_q = q;
                        Eigen::Vector3d rel_p = ia_i_ib_i_t;

                        covariance.setIdentity();
//                    covariance = covariance * pow(10, 2);
                        sqrt_information = covariance.llt().matrixL();

                        std::cout << "loop covariance" << std::endl << covariance << std::endl;

                        ceres::CostFunction *cost_function = PoseGraph3dErrorTerm::Create(rel_p, rel_q, sqrt_information);
                        problem.AddResidualBlock(cost_function, NULL, para_pose[i_a+i], para_pose[i_b+i]);

                    }
                }
            }

            // 2 --- run the solver
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//            options.linear_solver_type = ceres::DENSE_SCHUR;
            options.trust_region_strategy_type = ceres::DOGLEG;
            options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 100;

            // 3 --- get the optimized results
            TicToc t_solver;
            std::cout << "Pose graph optimization: \n begin solver" << std::endl;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            std::cout << summary.FullReport() << std::endl;
            std::cout << "Iterations : " << summary.iterations.size() << " ";
            std::cout << "ceres solver costs: " << t_solver.toc() << std::endl;

            std::cout << t_solver.toc() << " " << summary.jacobian_evaluation_time_in_seconds * 1000 << " ";

            // Para to States
            for (size_t i = 0; i < kf_size; i++) {

                graph_pose_v_.at(i).position_.x() = para_pose[i][0];
                graph_pose_v_.at(i).position_.y() = para_pose[i][1];
                graph_pose_v_.at(i).position_.z() = para_pose[i][2];
                graph_pose_v_.at(i).quaternion_.x() = para_pose[i][3];
                graph_pose_v_.at(i).quaternion_.y() = para_pose[i][4];
                graph_pose_v_.at(i).quaternion_.z() = para_pose[i][5];
                graph_pose_v_.at(i).quaternion_.w() = para_pose[i][6];
            }
            optimized_ = true;
        }

    }

    void pose_graph::Optimize4DOF() {

        if(!optimized_) {

            // 1 --- build ceres problem
            ceres::Problem problem;
            ceres::LossFunction *lossFunction;
            lossFunction = new ceres::HuberLoss(0.1);
            static size_t kf_size = graph_pose_v_.size();
//            double para_pose[kf_size][SIZE_POSE];

            double t_array[kf_size][3];
            Eigen::Quaterniond q_array[kf_size];
            double euler_array[kf_size][3];
//            double sequence_array[kf_size];

            ceres::LocalParameterization* angle_local_parameterization =
                    AngleLocalParameterization::Create();
//
            for (int i = 0; i < kf_size; i++) {

                Eigen::Quaterniond tmp_q = graph_pose_v_.at(i).quaternion_;
                Eigen::Matrix3d tmp_r = tmp_q.matrix();
                Eigen::Vector3d tmp_t = graph_pose_v_.at(i).position_;

                tmp_q = tmp_r;
                t_array[i][0] = tmp_t(0);
                t_array[i][1] = tmp_t(1);
                t_array[i][2] = tmp_t(2);
                q_array[i] = tmp_q;

                Eigen::Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
                euler_array[i][0] = euler_angle.x();
                euler_array[i][1] = euler_angle.y();
                euler_array[i][2] = euler_angle.z();

                problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);

                //add edge
//                for (int j = 1; j < 2; j++)
//                {
                int j = 1;
                if (i - j >= 0)
                {
//                        std::cout << sequence_array[i] << " " << sequence_array[i-j] << std::endl;
                    Eigen::Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                    Eigen::Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                    relative_t = q_array[i-j].inverse() * relative_t;
                    double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
                    ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                               relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, lossFunction, euler_array[i-j],
                                             t_array[i-j],
                                             euler_array[i],
                                             t_array[i]);
                }
//                }
            }

            problem.SetParameterBlockConstant(euler_array[0]);
            problem.SetParameterBlockConstant(t_array[0]);

            // add loop closure factor
            if (!loops_v_.empty()) {

                for (auto &k : loops_v_) {

                    int i_a = k.pair_idx_.first;
                    int i_b = k.pair_idx_.second;

                    std::cout << "Add loop pair: " << i_a << " - " << i_b << std::endl;

                    Eigen::Quaterniond rel_q = k.rel_pose.quaternion_;
                    Eigen::Vector3d rel_t = k.rel_pose.position_;
                    Eigen::Vector3d euler_connected = Utility::R2ypr(q_array[i_a].toRotationMatrix());
                    Eigen::Vector3d relative_t = rel_t;
                    double relative_yaw =  Utility::R2ypr(rel_q.toRotationMatrix()).x();
                    ceres::CostFunction* cost_function = FourDOFWeightError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                                                    relative_yaw, euler_connected.y(), euler_connected.z());
                    problem.AddResidualBlock(cost_function, lossFunction, euler_array[i_a],
                                             t_array[i_a],
                                             euler_array[i_b],
                                             t_array[i_b]);


                    int window_size = 10;
                    for (int i = -window_size/2; i < window_size/2; ++i) {
                        if(std::max(i_a+i,i_b+i)>=graph_pose_v_.size() || std::min(i_a+i, i_b+i) < 0) continue;

                        Eigen::Matrix4d  w_ia_T  = graph_pose_v_.at(i_a).Get_T();
                        Eigen::Matrix4d  w_ib_T  = graph_pose_v_.at(i_b).Get_T();
                        Eigen::Matrix4d  w_ia_i_T  = graph_pose_v_.at(i_a+i).Get_T();
                        Eigen::Matrix4d  w_ib_i_T  = graph_pose_v_.at(i_b+i).Get_T();
                        Eigen::Matrix4d ia_i_ia_T = w_ia_i_T.inverse() * w_ia_T;
                        Eigen::Matrix4d ib_i_ib_T = w_ib_i_T.inverse() * w_ib_T;
                        Eigen::Matrix4d  ia_ib_T =  k.rel_pose.Get_T();
                        Eigen::Matrix4d ia_i_ib_i_T = ia_i_ia_T * ia_ib_T * ib_i_ib_T.inverse();
                        std::cout << "Add loop pair: " << i_a+i << " - " << i_b+i << std::endl;
                        Eigen::Matrix3d ia_i_ib_i_R = ia_i_ib_i_T.block(0, 0, 3, 3);
                        Eigen::Quaterniond q(ia_i_ib_i_R);
                        Eigen::Vector3d ia_i_ib_i_t = ia_i_ib_i_T.block(0, 3, 3, 1);

                        Eigen::Quaterniond rel_q_i = q;
                        Eigen::Vector3d rel_t_i = ia_i_ib_i_t;
                        Eigen::Vector3d euler_connected_i = Utility::R2ypr(q_array[i_a+i].toRotationMatrix());
                        Eigen::Vector3d relative_t_i = rel_t;
                        double relative_yaw_i =  Utility::R2ypr(rel_q.toRotationMatrix()).x();
                        ceres::CostFunction* cost_function_i = FourDOFWeightError::Create(relative_t_i.x(), relative_t_i.y(), relative_t_i.z(),
                                                                                        relative_yaw_i, euler_connected_i.y(), euler_connected_i.z());
                        problem.AddResidualBlock(cost_function_i, lossFunction, euler_array[i_a+i],
                                                 t_array[i_a+i],
                                                 euler_array[i_b+i],
                                                 t_array[i_b+i]);
                    }
                }
            }

            // 2 --- run the solver
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//            options.linear_solver_type = ceres::DENSE_SCHUR;
            options.trust_region_strategy_type = ceres::DOGLEG;
            options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 50;

            // 3 --- get the optimized results
            TicToc t_solver;
            std::cout << "Pose graph optimization: \n begin solver" << std::endl;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            std::cout << summary.FullReport() << std::endl;
            std::cout << "Iterations : " << summary.iterations.size() << " ";
            std::cout << "ceres solver costs: " << t_solver.toc() << std::endl;

            std::cout << t_solver.toc() << " " << summary.jacobian_evaluation_time_in_seconds * 1000 << " ";

            // Para to States
            for (size_t i = 0; i < kf_size; i++) {
                Eigen::Quaterniond tmp_q;
                tmp_q = Utility::ypr2R(Eigen::Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
                Eigen::Vector3d tmp_t = Eigen::Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                Eigen::Matrix3d tmp_r = tmp_q.toRotationMatrix();
                graph_pose_v_.at(i).quaternion_ = tmp_q.normalized();
                graph_pose_v_.at(i).position_ = tmp_t;
            }
            optimized_ = true;
        }

    }

    void pose_graph::Publish() {

//        PointCloudT pose_graph;
//        for (auto & j : graph_pose_v_) {
//            PointT position;
//            position.x = j.position_.x();
//            position.y = j.position_.y();
//            position.z = j.position_.z();
//            pose_graph.points.push_back(position);
//        }
//        sensor_msgs::PointCloud2 pose_graph_msg;
//        pcl::toROSMsg(pose_graph, pose_graph_msg);
//        pose_graph_msg.header.frame_id = "map";
//        pub_pose_graph_.publish(pose_graph_msg);

        nav_msgs::Path traj_msg;
        traj_msg.header.frame_id = "map";
        for (auto & j : graph_pose_v_) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = j.position_.x();
            pose.pose.position.y = j.position_.y();
            pose.pose.position.z = j.position_.z();
            pose.pose.orientation.x = j.quaternion_.normalized().x();
            pose.pose.orientation.y = j.quaternion_.normalized().y();
            pose.pose.orientation.z = j.quaternion_.normalized().z();
            pose.pose.orientation.w = j.quaternion_.normalized().w();
            traj_msg.poses.push_back(pose);
        }
        pub_pose_graph_.publish(traj_msg);

        sensor_msgs::PointCloud2 origin_pose_graph_msg;
        pcl::toROSMsg(origin_pose_graph_, origin_pose_graph_msg);
        origin_pose_graph_msg.header.frame_id = "map";
        pub_origin_pose_graph_.publish(origin_pose_graph_msg);

    }

}


using namespace gp_lio;

int main(int argc, char **argv){

    ros::init(argc, argv, "pose_graph");
    pose_graph graph;
    graph.Run();

//    std::vector<double> query_traj;
//
//    query_traj.push_back(0);
//    query_traj.push_back(2);
//    query_traj.push_back(FLT_MAX);
//    query_traj.push_back(FLT_MAX);
//    query_traj.push_back(3);
//    query_traj.push_back(FLT_MAX);
//
//    for (int i = 0; i < query_traj.size(); ++i) {
//        std::cout << query_traj.at(i) << " ";
//    }
//    std::cout << std::endl;
//
//    for (auto it=query_traj.begin(); it!=query_traj.end();){
//        if(*it == INT_MAX)
//            it = query_traj.erase(it);
//        else
//            it++;
//    }
//    for (int i = 0; i < query_traj.size(); ++i) {
//        std::cout << query_traj.at(i) << " ";
//    }
//    std::cout << std::endl;

    return 0;
}
