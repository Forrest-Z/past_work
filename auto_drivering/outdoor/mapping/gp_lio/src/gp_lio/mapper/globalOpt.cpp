//
// Created by xc on 2021/2/22.
//

#include "gp_lio/mapper/globalOpt.h"

namespace gp_lio {
    GlobalOptimization::GlobalOptimization() {
        initGPS_ = false;
        newGPS_ = false;
        WGPS_T_WVIO = Eigen::Matrix4d::Identity();
        threadOpt_ = std::thread(&GlobalOptimization::optimize, this);
    }

    GlobalOptimization::~GlobalOptimization() {
        threadOpt_.detach();
    }

    void GlobalOptimization::gps2XYZ(double latitude, double longitude, double altitude, double *xyz) {
        if (!initGPS_) {
            geoConverter_.Reset(latitude, longitude, altitude);
            initGPS_ = true;
        }
        geoConverter_.Forward(latitude, longitude, altitude, xyz[0], xyz[1], xyz[2]);
    }

    void GlobalOptimization::inputOdom(double t, Eigen::Vector3d odomP, Eigen::Quaterniond odomQ) {
        mpose_.lock();
        std::array<double, 7> local_pose{odomP.x(), odomP.y(), odomP.z(), odomQ.w(), odomQ.x(), odomQ.y(), odomQ.z()};
        local_pose_v[t] = local_pose;
        Eigen::Quaterniond globalQ;
        globalQ = WGPS_T_WVIO.block<3, 3>(0, 0) * odomQ;
        Eigen::Vector3d globalP = WGPS_T_WVIO.block<3, 3>(0, 0) * odomP + WGPS_T_WVIO.block<3, 1>(0, 3);
        std::array<double, 7> global_pose{globalP.x(), globalP.y(), globalP.z(),
                                          globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
        gloabl_pose_v[t] = global_pose;
        pcl::PointXYZ tmp_pose;
        tmp_pose.x = globalP.x();
        tmp_pose.y = globalP.y();
        tmp_pose.z = globalP.z();
        global_path_.push_back(tmp_pose);
        mpose_.unlock();

    }

    void GlobalOptimization::inputGPS(double t, double latitude, double longitude, double altitude,
                                      double pos_accuracy) {
        double xyz[3];
        gps2XYZ(latitude, longitude, altitude, xyz);
        std::array<double, 4> gps_tmp{xyz[0], xyz[1], xyz[2], pos_accuracy};
        gps_pose_v[t] = gps_tmp;
        newGPS_ = true;
    }

    void GlobalOptimization::optimize() {
        while (true) {
            if (newGPS_) {
                newGPS_ = false;
                std::cout << "Global Optimization ..." << std::endl;
                TicToc global_opt_time;

                ceres::Problem problem;
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
                options.max_num_iterations = 5;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                loss_function = new ceres::HuberLoss(1.0);
                ceres::LocalParameterization *local_parameterization = new ceres::QuaternionParameterization();

                mpose_.lock();
                int length = local_pose_v.size();
                double t_array[length][3];
                double q_array[length][4];
                std::map<double, std::array<double, 7>>::iterator iter;
                iter = gloabl_pose_v.begin();
                for (int i = 0; i < length; i++, iter++) {
                    t_array[i][0] = iter->second[0];
                    t_array[i][1] = iter->second[1];
                    t_array[i][2] = iter->second[2];
                    q_array[i][0] = iter->second[3];
                    q_array[i][1] = iter->second[4];
                    q_array[i][2] = iter->second[5];
                    q_array[i][3] = iter->second[6];
                    problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                    problem.AddParameterBlock(t_array[i], 3);
                }

                std::map<double, std::array<double, 7>>::iterator iterVIO, iterVIONext;
                std::map<double, std::array<double, 4>>::iterator iterGPS;
                int i = 0;
                for (iterVIO = local_pose_v.begin(); iterVIO != local_pose_v.end(); iterVIO++, i++) {
                    //vio factor
                    iterVIONext = iterVIO;
                    iterVIONext++;
                    if (iterVIONext != local_pose_v.end()) {
                        Eigen::Matrix4d wTi = Eigen::Matrix4d::Identity();
                        Eigen::Matrix4d wTj = Eigen::Matrix4d::Identity();
                        wTi.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIO->second[3], iterVIO->second[4],
                                                                   iterVIO->second[5],
                                                                   iterVIO->second[6]).toRotationMatrix();
                        wTi.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIO->second[0], iterVIO->second[1],
                                                                iterVIO->second[2]);
                        wTj.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIONext->second[3], iterVIONext->second[4],
                                                                   iterVIONext->second[5],
                                                                   iterVIONext->second[6]).toRotationMatrix();
                        wTj.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIONext->second[0], iterVIONext->second[1],
                                                                iterVIONext->second[2]);
                        Eigen::Matrix4d iTj = wTi.inverse() * wTj;
                        Eigen::Quaterniond iQj;
                        iQj = iTj.block<3, 3>(0, 0);
                        Eigen::Vector3d iPj = iTj.block<3, 1>(0, 3);

                        ceres::CostFunction *vio_function = RelativeRTError::Create(iPj.x(), iPj.y(), iPj.z(),
                                                                                    iQj.w(), iQj.x(), iQj.y(), iQj.z(),
                                                                                    0.1, 0.01);
                        problem.AddResidualBlock(vio_function, NULL, q_array[i], t_array[i], q_array[i + 1],
                                                 t_array[i + 1]);

                        /*
                        double **para = new double *[4];
                        para[0] = q_array[i];
                        para[1] = t_array[i];
                        para[3] = q_array[i+1];
                        para[4] = t_array[i+1];

                        double *tmp_r = new double[6];
                        double **jaco = new double *[4];
                        jaco[0] = new double[6 * 4];
                        jaco[1] = new double[6 * 3];
                        jaco[2] = new double[6 * 4];
                        jaco[3] = new double[6 * 3];
                        vio_function->Evaluate(para, tmp_r, jaco);

                        std::cout << Eigen::Map<Eigen::Matrix<double, 6, 1>>(tmp_r).transpose() << std::endl
                            << std::endl;
                        std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[0]) << std::endl
                            << std::endl;
                        std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[1]) << std::endl
                            << std::endl;
                        std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[2]) << std::endl
                            << std::endl;
                        std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[3]) << std::endl
                            << std::endl;
                        */

                    }
                    //gps factor
                    double t = iterVIO->first;
                    iterGPS = gps_pose_v.find(t);
                    if (iterGPS != gps_pose_v.end()) {
                        ceres::CostFunction *gps_function = PriorTError::Create(iterGPS->second[0], iterGPS->second[1],
                                                                                iterGPS->second[2], iterGPS->second[3]);
                        //printf("inverse weight %f \n", iterGPS->second[3]);
                        problem.AddResidualBlock(gps_function, loss_function, t_array[i]);

                        /*
                        double **para = new double *[1];
                        para[0] = t_array[i];

                        double *tmp_r = new double[3];
                        double **jaco = new double *[1];
                        jaco[0] = new double[3 * 3];
                        gps_function->Evaluate(para, tmp_r, jaco);

                        std::cout << Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_r).transpose() << std::endl
                            << std::endl;
                        std::cout << Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(jaco[0]) << std::endl
                            << std::endl;
                        */
                    }

                }
                ceres::Solve(options, &problem, &summary);
                std::cout << summary.BriefReport() << "\n";

                // update global pose
                iter = gloabl_pose_v.begin();
                for (int i = 0; i < length; i++, iter++) {
                    std::array<double, 7> globalPose{t_array[i][0], t_array[i][1], t_array[i][2],
                                                     q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]};
                    iter->second = globalPose;
                    if (i == length - 1) {
                        Eigen::Matrix4d WVIO_T_body = Eigen::Matrix4d::Identity();
                        Eigen::Matrix4d WGPS_T_body = Eigen::Matrix4d::Identity();
                        double t = iter->first;
                        WVIO_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(local_pose_v[t][3], local_pose_v[t][4],
                                                                           local_pose_v[t][5],
                                                                           local_pose_v[t][6]).toRotationMatrix();
                        WVIO_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(local_pose_v[t][0], local_pose_v[t][1],
                                                                        local_pose_v[t][2]);
                        WGPS_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(globalPose[3], globalPose[4],
                                                                           globalPose[5],
                                                                           globalPose[6]).toRotationMatrix();
                        WGPS_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(globalPose[0], globalPose[1], globalPose[2]);
                        WGPS_T_WVIO = WGPS_T_body * WVIO_T_body.inverse();
                    }
                }
                updateGlobalPath();
                std::cout << "Global Optimization :    global time cost  " << global_opt_time.toc() << std::endl;
                mpose_.unlock();
            }
            std::chrono::milliseconds dura(50);
            std::this_thread::sleep_for(dura);
        }
        return;
    }

    void GlobalOptimization::updateGlobalPath() {
        global_path_.points.clear();
        std::map<double,std::array<double,7>>::iterator iter;
        for(iter=gloabl_pose_v.begin();iter != gloabl_pose_v.end();iter++){
            pcl::PointXYZ tmp;
            tmp.x = iter->second[0];
            tmp.y = iter->second[1];
            tmp.z = iter->second[2];
            global_path_.points.push_back(tmp);
        }

    }

    void GlobalOptimization::getGlobalPath(pcl::PointCloud<pcl::PointXYZ> &global_path) {
        pcl::copyPointCloud(global_path_,global_path);
    }



}

