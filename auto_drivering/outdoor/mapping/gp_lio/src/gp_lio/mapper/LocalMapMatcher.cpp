//
// Created by wchen on 2020/8/11.
//

#include "gp_lio/mapper/LocalMapMatcher.h"

namespace gp_lio{

    LocalMapMatcher::LocalMapMatcher(): showCloud_(false), test_accuracy_(false) {

        cloud_source_.reset(new PointCloudTX());
        cloud_target_.reset(new PointCloudTX());
        ground_source_.reset(new PointCloudTX());
        ground_target_.reset(new PointCloudTX());
        plane_cloud_source_.reset(new PointCloudTX());
        plane_cloud_target_.reset(new PointCloudTX());
        all_source_.reset(new PointCloudTX());
        all_target_.reset(new PointCloudTX());

        T_ground_target_.setIdentity();
        T_ground_source_.setIdentity();

        inlTh_ = 0.3;
        maxCorr_ = 1;
        tx_min_ = -20.0;
        tx_max_ = 20.0;
        epsilon_ = 0.3;

        OUTPUT_PATH_ = "/home/wchen/Projects/Code/BnB/Demo---Practical-optimal-registration-of-terrestrial-LiDAR-scan-pairs-master/data/my_data/";
        if(test_accuracy_){

            Eigen::Vector3d t({std::abs(tx_max_), 20, -10});
            Eigen::Vector3d ypr({60, -40, 80});
//    Eigen::Vector3d t({0, 0, 0});
//    Eigen::Vector3d ypr({0, 0, 0});
            Eigen::Matrix3d rot = ypr2R(ypr);
            mat_GT_.setIdentity();
            mat_GT_.block<3, 3>(0, 0) = rot;
            mat_GT_.block<3, 1>(0, 3) = t;
            std::cout << "mat_GT_: " << std::endl << mat_GT_ << std::endl;

        }


    }

    LocalMapMatcher::~LocalMapMatcher() {

    }

    RegistrationResult LocalMapMatcher::Match(PointCloudT cloud_source, PointCloudT cloud_target,
                                              PointCloudT ground_source, PointCloudT ground_target,
                                              PointCloudT plane_cloud_source, PointCloudT plane_cloud_target,
                                              std::vector<pcl::ModelCoefficients> planes_target,
                                              std::vector<pcl::ModelCoefficients> planes_source) {

        std::cout << "in local map matcher" << std::endl;
        if(!all_source_->empty()){
            all_source_->clear();  cloud_source_->clear(); ground_source_->clear(); plane_cloud_source_->clear(); planes_source_.clear();
            all_target_->clear();  cloud_target_->clear(); ground_target_->clear(); plane_cloud_target_->clear(); planes_target_.clear();
        }


        *cloud_source_ = ChangeCloud(cloud_source);
        *cloud_target_ = ChangeCloud(cloud_target);
        *ground_source_ = ChangeCloud(ground_source);
        *ground_target_ = ChangeCloud(ground_target);
        *plane_cloud_source_ = ChangeCloud(plane_cloud_source);
        *plane_cloud_target_ = ChangeCloud(plane_cloud_target);

        *all_source_ = *all_source_ + *cloud_source_;
//    *all_source_ = *all_source_ + *ground_source_;
        // *all_source_ = *all_source_ + *plane_cloud_source_;

        *all_target_ = *all_target_ + *cloud_target_;
//    *all_target_ = *all_target_ + *ground_target_;
        // *all_target_ = *all_target_ + *plane_cloud_target_;

        planes_target_.assign(planes_target.begin(), planes_target.end());
        planes_source_.assign(planes_source.begin(), planes_source.end());

        //show original point clouds
        if (showCloud_){
            pcl::visualization::PointCloudColorHandlerCustom<PointTX> colorS(cloud_source_, 255, 0, 0); //red
            pcl::visualization::PointCloudColorHandlerCustom<PointTX> colorT(cloud_target_, 255, 255, 0); // Yellow
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerVGF (new pcl::visualization::PCLVisualizer ("before matching"));
            viewerVGF->addCoordinateSystem();
            viewerVGF->setBackgroundColor (0, 0, 0);
            viewerVGF->addPointCloud<PointTX> (cloud_source_, colorS, "source cloud");
            viewerVGF->addPointCloud<PointTX> (cloud_target_, colorT, "target cloud");
            viewerVGF->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "before matching");
            viewerVGF->spinOnce();
        }

        auto start = std::chrono::steady_clock::now();
        GroundCoordinate();
        auto ground_end = std::chrono::steady_clock::now();
        std::cout<<"->>>>>>>>>>> ground runtime in ms = "<<std::chrono::duration_cast<std::chrono::milliseconds>(ground_end-start).count()<<endl;

        Eigen::Matrix4d final_transform = PlaneCoordinate();
        auto plane_end = std::chrono::steady_clock::now();
        std::cout << "final_transform: "  << std::endl << final_transform << std::endl
                  << "->>>>>>>>>>> PlaneCoordinate runtime in ms = " << std::chrono::duration_cast<std::chrono::milliseconds>(plane_end-ground_end).count()<< std::endl;

        // ICP refine
        Eigen::Matrix4f refined_final_transform = final_transform.cast<float>();
        // auto final_plane_result = ICPRegistration(*plane_cloud_target_, *plane_cloud_source_, refined_final_transform);
        // if (!final_plane_result.convergence_)
        //     return final_plane_result;
        auto final_result = ICPRegistration(*all_target_, *all_source_, refined_final_transform);
        refined_final_transform = final_result.GetTransformation();
        std::cout << "refined_final_transform: " << std::endl << refined_final_transform << std::endl;
        auto icp_end = std::chrono::steady_clock::now();
        std::cout << "->>>>>>>>>>> icp refinement runtime in ms = " << std::chrono::duration_cast<std::chrono::milliseconds>(icp_end-plane_end).count()<<endl;
        std::cout<<"->>>>>>>>>>> overall runtime in ms = "<<std::chrono::duration_cast<std::chrono::milliseconds>(icp_end-start).count()<<endl;

        // visualization
        if (showCloud_) {

            PointCloudTX::Ptr cloudSReg_1(new PointCloudTX);
            pcl::transformPointCloud(*all_source_, *cloudSReg_1, final_transform);

            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_1(
                    new pcl::visualization::PCLVisualizer("final - after maxstabbing"));
            viewer_1->setBackgroundColor(0, 0, 0);
            viewer_1->addCoordinateSystem();
            pcl::visualization::PointCloudColorHandlerCustom<PointTX> colorSReg_1(cloudSReg_1, 255, 0, 0);
            pcl::visualization::PointCloudColorHandlerCustom<PointTX> colorTReg_1(cloud_target_, 255, 255, 0);
            viewer_1->addPointCloud<PointTX>(cloudSReg_1, colorSReg_1, "source cloud");
            viewer_1->addPointCloud<PointTX>(cloud_target_, colorTReg_1, "target cloud");
            viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6,
                                                       "final - after maxstabbing");
            viewer_1->spinOnce();

        }

        if (showCloud_) {

            PointCloudTX::Ptr cloudSReg_2(new PointCloudTX);
            pcl::transformPointCloud(*all_source_, *cloudSReg_2, refined_final_transform);

            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_2(
                    new pcl::visualization::PCLVisualizer("final - after refinement by icp"));
            viewer_2->setBackgroundColor(0, 0, 0);
            viewer_2->addCoordinateSystem();
            pcl::visualization::PointCloudColorHandlerCustom<PointTX> colorSReg_2(cloudSReg_2, 255, 0, 0);
            pcl::visualization::PointCloudColorHandlerCustom<PointTX> colorTReg_2(cloud_target_, 255, 255, 0);
            viewer_2->addPointCloud<PointTX>(cloudSReg_2, colorSReg_2, "source cloud");
            viewer_2->addPointCloud<PointTX>(cloud_target_, colorTReg_2, "target cloud");
            viewer_2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6,
                                                       "final - after refinement by icp");
            while (!viewer_2->wasStopped()) {
                viewer_2->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            }
        }

        return final_result;

    }

    void LocalMapMatcher::LocalMapMatcher::Run() {

//    Eigen::Vector4d tmp_ground_source, tmp_plane_source, tmp_ground_target, tmp_plane_target;
//    tmp_ground_source << 0, 0, -1, 2;
//    tmp_plane_source << 0, 1, 0, 1;
//    tmp_ground_target << 0, 0, -1, 2;
//    tmp_plane_target << 0, -1, 0, 1;
//
//    std::vector<Eigen::Matrix4d> tmp_T_v = GetPredefinedTransformations2Plane1Line(tmp_ground_source,
//                                                                                   tmp_plane_source,
//                                                                                   tmp_ground_target,
//                                                                                   tmp_plane_target, false);
//    std::cout << tmp_T_v.at(0) << std::endl << std::endl << tmp_T_v.at(1) << std::endl << std::endl;
//
//    tmp_ground_source << 0, 0, -1, 2;
//    tmp_plane_source << 1, 0, 0, 1;
//    tmp_ground_target << 0, 0, -1, 2;
//    tmp_plane_target << -1, 0, 0, 1;
//
//    tmp_T_v = GetPredefinedTransformations2Plane1Line(tmp_ground_source,
//                                                      tmp_plane_source,
//                                                      tmp_ground_target,
//                                                      tmp_plane_target, false);
//    std::cout << tmp_T_v.at(0) << std::endl << std::endl << tmp_T_v.at(1) << std::endl;
        int loop_count = 0;
        int ref_idx = 17;
        int query_idx = 821;

        LoadData(loop_count, ref_idx, query_idx);

        //show original point clouds
        if (showCloud_){
            pcl::visualization::PointCloudColorHandlerCustom<PointTX> colorS(cloud_source_, 255, 0, 0); //red
            pcl::visualization::PointCloudColorHandlerCustom<PointTX> colorT(cloud_target_, 255, 255, 0); // Yellow
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerVGF (new pcl::visualization::PCLVisualizer ("before matching"));
            viewerVGF->addCoordinateSystem();
            viewerVGF->setBackgroundColor (0, 0, 0);
            viewerVGF->addPointCloud<PointTX> (cloud_source_, colorS, "source cloud");
            viewerVGF->addPointCloud<PointTX> (cloud_target_, colorT, "target cloud");
            viewerVGF->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "before matching");
            viewerVGF->spinOnce();
        }

        auto start = std::chrono::steady_clock::now();
        GroundCoordinate();
        auto ground_end = std::chrono::steady_clock::now();
        std::cout<<"->>>>>>>>>>> ground runtime in ms = "<<std::chrono::duration_cast<std::chrono::milliseconds>(ground_end-start).count()<<endl;

        Eigen::Matrix4d final_transform = PlaneCoordinate();
        auto plane_end = std::chrono::steady_clock::now();
        std::cout << "final_transform: "  << std::endl << final_transform << std::endl
                  << "->>>>>>>>>>> PlaneCoordinate runtime in ms = " << std::chrono::duration_cast<std::chrono::milliseconds>(plane_end-ground_end).count()<< std::endl;

        // ICP refine
        Eigen::Matrix4f refined_final_transform = final_transform.cast<float>();
        auto final_result = ICPRegistration(*all_target_, *all_source_, refined_final_transform);
        refined_final_transform = final_result.GetTransformation();
        std::cout << "refined_final_transform: " << std::endl << refined_final_transform << std::endl;
        auto icp_end = std::chrono::steady_clock::now();
        std::cout << "->>>>>>>>>>> icp refinement runtime in ms = " << std::chrono::duration_cast<std::chrono::milliseconds>(icp_end-plane_end).count()<<endl;
        std::cout<<"->>>>>>>>>>> overall runtime in ms = "<<std::chrono::duration_cast<std::chrono::milliseconds>(icp_end-start).count()<<endl;

        if(test_accuracy_){
            // accuracy evaluation
            std::cout << std::endl << std::endl << "Accuracy Evalution: Rotation(degree) translation(m)" << std::endl;

            //local icp
            Eigen::Matrix4f icp_transform = Eigen::Matrix4f::Identity();
            auto icp_transform_result = ICPRegistration(*all_target_, *all_source_, icp_transform);
            icp_transform = icp_transform_result.GetTransformation();
            std::cout << "icp_transform: " << std::endl << icp_transform << std::endl;
            std::cout << "local icp: ";
            ErrorEvaluation(mat_GT_, icp_transform.cast<double>().inverse());

            // two plane + maxstabbing
            std::cout << "two plane + maxstabbing: ";
            ErrorEvaluation(mat_GT_, final_transform.inverse());

            // two plane + maxstabbing + icp refine
            std::cout << "two plane + maxstabbing + icp refine: ";
            Eigen::Matrix4d refined_final_transform_d = refined_final_transform.cast<double>();
            ErrorEvaluation(mat_GT_, refined_final_transform_d.inverse());
        }

        // visualization
        if (showCloud_) {

            PointCloudTX::Ptr cloudSReg_1(new PointCloudTX);
            pcl::transformPointCloud(*all_source_, *cloudSReg_1, final_transform);

            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_1(
                    new pcl::visualization::PCLVisualizer("final - after maxstabbing"));
            viewer_1->setBackgroundColor(0, 0, 0);
            viewer_1->addCoordinateSystem();
            pcl::visualization::PointCloudColorHandlerCustom<PointTX> colorSReg_1(cloudSReg_1, 255, 0, 0);
            pcl::visualization::PointCloudColorHandlerCustom<PointTX> colorTReg_1(cloud_target_, 255, 255, 0);
            viewer_1->addPointCloud<PointTX>(cloudSReg_1, colorSReg_1, "source cloud");
            viewer_1->addPointCloud<PointTX>(cloud_target_, colorTReg_1, "target cloud");
            viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6,
                                                       "final - after maxstabbing");
            viewer_1->spinOnce();

        }

        if (showCloud_) {

            PointCloudTX::Ptr cloudSReg_2(new PointCloudTX);
            pcl::transformPointCloud(*all_source_, *cloudSReg_2, refined_final_transform);

            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_2(
                    new pcl::visualization::PCLVisualizer("final - after refinement by icp"));
            viewer_2->setBackgroundColor(0, 0, 0);
            viewer_2->addCoordinateSystem();
            pcl::visualization::PointCloudColorHandlerCustom<PointTX> colorSReg_2(cloudSReg_2, 255, 0, 0);
            pcl::visualization::PointCloudColorHandlerCustom<PointTX> colorTReg_2(cloud_target_, 255, 255, 0);
            viewer_2->addPointCloud<PointTX>(cloudSReg_2, colorSReg_2, "source cloud");
            viewer_2->addPointCloud<PointTX>(cloud_target_, colorTReg_2, "target cloud");
            viewer_2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6,
                                                       "final - after refinement by icp");
            while (!viewer_2->wasStopped()) {
                viewer_2->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            }
        }

//    Eigen::Matrix4f icp_transform = Eigen::Matrix4f::Identity();
//    icp_transform = ICPRegistration(*all_target_, *all_source_, icp_transform);
//    std::cout << "icp_transform: " << std::endl << icp_transform << std::endl;


    }

    void LocalMapMatcher::LoadData(int loop_count, int ref_idx, int query_idx) {

        std::string ref_save_file = OUTPUT_PATH_ + "loop_" + std::to_string(loop_count) + "_target_" + std::to_string(ref_idx) + ".pcd";
        pcl::io::loadPCDFile(ref_save_file,  *cloud_target_);

        std::string query_save_file = OUTPUT_PATH_ + "loop_" + std::to_string(loop_count) + "_source_" + std::to_string(query_idx) + ".pcd";
        pcl::io::loadPCDFile(query_save_file, *cloud_source_);

        std::string ref_ground_save_file = OUTPUT_PATH_ + "loop_" + std::to_string(loop_count) + "_target_ground_" + std::to_string(ref_idx) + ".pcd";
        pcl::io::loadPCDFile(ref_ground_save_file, *ground_target_);

        std::string query_ground_save_file = OUTPUT_PATH_ + "loop_" + std::to_string(loop_count) + "_source_ground_" + std::to_string(query_idx) + ".pcd";
        pcl::io::loadPCDFile(query_ground_save_file, *ground_source_);

        std::string ref_plane_save_file = OUTPUT_PATH_ + "loop_" + std::to_string(loop_count) + "_ref_plane_cloud_" + std::to_string(ref_idx) + ".pcd";
        pcl::io::loadPCDFile(ref_plane_save_file, *plane_cloud_target_);

        std::string query_plane_save_file = OUTPUT_PATH_ + "loop_" + std::to_string(loop_count) + "_query_plane_cloud_" + std::to_string(query_idx) + ".pcd";
        pcl::io::loadPCDFile(query_plane_save_file, *plane_cloud_source_);

        std::cout << "Cloud size (cloud ground plane): " << std::endl
                  << "-> target: " << cloud_target_->size() << " " << ground_target_->size() << " " << plane_cloud_target_->size() << std::endl
                  << "-> target: " << cloud_source_->size() << " " << ground_source_->size() << " " << plane_cloud_source_->size() << std::endl;

        planes_target_ = LoadCP(std::string(OUTPUT_PATH_ + "fused_cp_target.txt"));
        planes_source_ = LoadCP(std::string(OUTPUT_PATH_ + "fused_cp_source.txt"));

        *all_source_ = *all_source_ + *cloud_source_;
//    *all_source_ = *all_source_ + *ground_source_;
        *all_source_ = *all_source_ + *plane_cloud_source_;

        *all_target_ = *all_target_ + *cloud_target_;
//    *all_target_ = *all_target_ + *ground_target_;
        *all_target_ = *all_target_ + *plane_cloud_target_;

        // test accuracy
        if(test_accuracy_){
            all_source_->clear();  cloud_source_->clear(); ground_source_->clear(); plane_cloud_source_->clear(); planes_source_.clear();
            pcl::transformPointCloud(*all_target_, *all_source_, mat_GT_);
            pcl::transformPointCloud(*cloud_target_, *cloud_source_, mat_GT_);
            pcl::transformPointCloud(*ground_target_, *ground_source_, mat_GT_);
            pcl::transformPointCloud(*plane_cloud_target_, *plane_cloud_source_, mat_GT_);
            for (const auto & i : planes_target_) {
                planes_source_.emplace_back(TransformPlane(i, mat_GT_));
            }
        }

    }

    std::vector<pcl::ModelCoefficients> LocalMapMatcher::LoadCP(const std::string& file_path){

        std::vector<pcl::ModelCoefficients> planes;

        std::ifstream fin(file_path.c_str());
        std::string i_line;

        if(fin.is_open()){

            while (getline(fin, i_line)){

                pcl::ModelCoefficients i_plane;
                std::stringstream ss(i_line);

                std::string splited;

                while(ss >> splited){
                    float data = std::atof(splited.c_str());
                    i_plane.values.emplace_back(data);
                }
                planes.emplace_back(i_plane);
            }
            fin.close();

            // from 3D cp to 4D: (n,d)
            for (int j = 0; j < planes.size(); ++j) {
                planes.at(j) = TransformPlane(planes.at(j), Eigen::Matrix4d::Identity());
            }
            return planes;

        } else{
            std::cerr << "cannot read file " << file_path << std::endl;
            exit(0);
        }

    }

    void LocalMapMatcher::GroundCoordinate(){

        auto start = std::chrono::steady_clock::now();
        // ground plane fitting
        PointCloudTX plane_target_cloud, plane_source_cloud;
        ground_plane_target_ = PlaneRANSACFitting(*ground_target_, plane_target_cloud);
        ground_plane_source_ = PlaneRANSACFitting(*ground_source_, plane_source_cloud);

        ground_plane_target_.values.at(0) = -ground_plane_target_.values.at(0);
        ground_plane_target_.values.at(1) = -ground_plane_target_.values.at(1);
        ground_plane_target_.values.at(2) = -ground_plane_target_.values.at(2);

        ground_plane_source_.values.at(0) = -ground_plane_source_.values.at(0);
        ground_plane_source_.values.at(1) = -ground_plane_source_.values.at(1);
        ground_plane_source_.values.at(2) = -ground_plane_source_.values.at(2);

        std::cout << "plane model (ax+by+cz+d=0): " << std::endl
                  << "   target: " << ground_plane_target_.values.at(0) << " " << ground_plane_target_.values.at(1) << " "
                  << ground_plane_target_.values.at(2) << " " << ground_plane_target_.values.at(3) << std::endl
                  << "   source: " << ground_plane_source_.values.at(0) << " " << ground_plane_source_.values.at(1) << " "
                  << ground_plane_source_.values.at(2) << " " << ground_plane_source_.values.at(3) << std::endl;


    };

    PointCloudTX LocalMapMatcher::GetGroundPlane(PointCloudTX cloud_in){

        // plane fitting
        PointCloudTX cloud_out;

        for (int i = 0; i < cloud_in.points.size(); ++i) {

            float xy_distance = sqrt(cloud_in.points.at(i).x * cloud_in.points.at(i).x + cloud_in.points.at(i).y * cloud_in.points.at(i).y);
            if(xy_distance < 5.0)
                cloud_out.points.emplace_back(cloud_in.points.at(i));
        }

        std::cout << "size: " << cloud_in.size() << " -> " << cloud_out.size();

        return  cloud_out;
    };

    pcl::ModelCoefficients LocalMapMatcher::PlaneRANSACFitting(PointCloudTX cloud_in, PointCloudTX& cloud_plane){

        pcl::ModelCoefficientsPtr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndicesPtr indices(new pcl::PointIndices);

        pcl::SACSegmentation<PointTX> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setDistanceThreshold(0.01);

        seg.setInputCloud(cloud_in.makeShared());
        seg.segment(*indices, *coefficients);

        if (indices->indices.size() == 0){
            std::cout << " No ground Plane " << std::endl;
            return *coefficients;
        }

        for (int i = 0; i < indices->indices.size(); ++i) {
            cloud_plane.points.emplace_back(cloud_in.points.at(indices->indices.at(i)));
        }
//    std::cout << " -> " << cloud_plane.size() << std::endl;
        return *coefficients;
    };

    void LocalMapMatcher::ISSExt(PointCloudTX::Ptr cloud, PointCloudTX::Ptr ISS,
                                 pcl::PointIndicesPtr ISSIdx, double inlTh){

        double iss_salient_radius_ = 6 * inlTh;
        double iss_non_max_radius_ = 4 * inlTh;
        double iss_gamma_21_ (0.975);
        double iss_gamma_32_ (0.975);
        double iss_min_neighbors_ (5);
        int iss_threads_ (8); //switch to the number of threads in your cpu for acceleration

        pcl::search::KdTree<PointTX>::Ptr tree (new pcl::search::KdTree<PointTX> ());
        pcl::ISSKeypoint3D<PointTX, PointTX> iss_detector;

        iss_detector.setSearchMethod (tree);
        iss_detector.setSalientRadius (iss_salient_radius_);
        iss_detector.setNonMaxRadius (iss_non_max_radius_);
        iss_detector.setThreshold21 (iss_gamma_21_);
        iss_detector.setThreshold32 (iss_gamma_32_);
        iss_detector.setMinNeighbors (iss_min_neighbors_);
        iss_detector.setNumberOfThreads (iss_threads_);
        iss_detector.setInputCloud (cloud);
        iss_detector.compute (*ISS);
        ISSIdx->indices = iss_detector.getKeypointsIndices()->indices;
        ISSIdx->header = iss_detector.getKeypointsIndices()->header;
    };

    void LocalMapMatcher::FPFHComp(PointCloudTX::Ptr cloud, double inlTh,
                                   pcl::PointIndicesPtr ISSIdx,
                                   pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhOut){

        pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal> ());
        pcl::search::KdTree<PointTX>::Ptr tree (new pcl::search::KdTree<PointTX> ());
        pcl::NormalEstimation<PointTX, pcl::Normal> neSource;
        neSource.setInputCloud (cloud);
        neSource.setSearchMethod (tree);
        neSource.setRadiusSearch (3*inlTh);
        neSource.compute (*normal);

        //compute fpfh using normals
        pcl::FPFHEstimationOMP<PointTX, pcl::Normal, pcl::FPFHSignature33> fpfhEst;
        fpfhEst.setInputCloud(cloud);
        fpfhEst.setInputNormals(normal);
        fpfhEst.setSearchMethod(tree);
        fpfhEst.setRadiusSearch(8*inlTh);
        fpfhEst.setNumberOfThreads (4);
//    fpfhEst.setIndices(ISSIdx);
        fpfhEst.compute(*fpfhOut);
    };

    void LocalMapMatcher::CorrComp(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs,
                                   pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfht,
                                   std::vector<corrTab> &corr, int MaxNOCorrPP,
                                   std::vector<int> &corrNOS, std::vector<int> &corrNOT){

        int n = std::min(MaxNOCorrPP, (int)fpfht->size()); //maximum number of correspondences to find for each source point
        corr.clear();
        corrNOS.assign(fpfhs->size(),0);
        corrNOT.assign(fpfht->size(),0);
        // Use a KdTree to search for the nearest matches in feature space
        pcl::KdTreeFLANN<pcl::FPFHSignature33> treeS;
        treeS.setInputCloud (fpfhs);
        pcl::KdTreeFLANN<pcl::FPFHSignature33> treeT;
        treeT.setInputCloud (fpfht);
        for(size_t i=0;i<fpfhs->size();i++){
            std::vector<int> corrIdxTmp(n);
            std::vector<float> corrDisTmp(n);
            //find the best n matches in target fpfh
            treeT.nearestKSearch(*fpfhs,i,n,corrIdxTmp,corrDisTmp);

            corrTab corrTabTmp;
            corrTabTmp.idxS = i;
            corrTabTmp.idxT = corrIdxTmp[0];
            corrTabTmp.disFeature = corrDisTmp[0];
            corrTabTmp.upBnd = fpfht->size();
            corrTabTmp.lwBnd = 1;
            corr.push_back(corrTabTmp);
            corrNOS[i]++;
            corrNOT[corrIdxTmp[0]]++;


//        for(size_t j=0;j<corrIdxTmp.size();j++){
//            bool removeFlag = true;
//            int searchIdx = corrIdxTmp[j];
//            std::vector<int> corrIdxTmpT(n);
//            std::vector<float> corrDisTmpT(n);
//            treeS.nearestKSearch(*fpfht,searchIdx,n,corrIdxTmpT,corrDisTmpT);
//            for(size_t k=0;k<n;k++){
//                if(corrIdxTmpT.data()[k]==i){
//                    removeFlag = false;
//                    break;
//                }
//            }
//            if(removeFlag == false){
//                corrTab corrTabTmp;
//                corrTabTmp.idxS = i;
//                corrTabTmp.idxT = corrIdxTmp[j];
//                corrTabTmp.disFeature = corrDisTmp[j];
//                corrTabTmp.upBnd = fpfht->size();
//                corrTabTmp.lwBnd = 1;
//                corr.push_back(corrTabTmp);
//                corrNOS[i]++;
//                corrNOT[corrIdxTmp[j]]++;
//            }
//        }
        }
    };

    void LocalMapMatcher::PreprocessingCloud(PointCloudTX::Ptr cloud, Eigen::Matrix4d& T_mCent, PointCloudTX::Ptr iss,
                                             pcl::PointIndicesPtr issIdx, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh){


//    cout<<"performing voxel grid sampling with grid size = "<<inlTh_<<endl;
//    VGF(cloud, cloud, inlTh_);

        cout << "extracting ISS keypoints..." << inlTh_ << endl;
        ISSExt(cloud, iss, issIdx, inlTh_);
        cout << "size of iss = " << iss->size() << endl;
//    std::cout << iss->points.at(0) << std::endl;
        //translating the center of both point clouds to the origin
//    Eigen::Vector3d cent(0, 0, 0);
//    double r;
//    CentAndRComp(iss, cent, r);
//    Vector3 mCent(-cent.x, -cent.y, -cent.z);
//    transPC(iss, mCent);
        T_mCent.setIdentity();
//    T_mCent.block<3, 1>(0, 3) = Eigen::Vector3d({mCent.x, mCent.y, mCent.z});

//    std::cout << iss->points.at(0) << std::endl;
        FPFHComp(cloud, inlTh_, issIdx, fpfh);
    };

    Eigen::Matrix4d LocalMapMatcher::PlaneCoordinate(){

        auto start = std::chrono::steady_clock::now();
        // compute point correspondence
        PointCloudTX::Ptr iss_source (new PointCloudTX);
        PointCloudTX::Ptr iss_target (new PointCloudTX);
        pcl::PointIndicesPtr issIdx_target(new pcl::PointIndices);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_target (new pcl::PointCloud<pcl::FPFHSignature33> ());
        pcl::PointIndicesPtr issIdx_source(new pcl::PointIndices);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_source (new pcl::PointCloud<pcl::FPFHSignature33> ());
        Eigen::Matrix4d T_Center_target, T_Center_source;

//    PreprocessingCloud(all_target_, T_Center_target, iss_target, issIdx_target, fpfh_target);
//    PreprocessingCloud(all_source_, T_Center_source, iss_source, issIdx_source, fpfh_source);
        FPFHComp(all_target_, inlTh_, issIdx_target, fpfh_target);
        FPFHComp(all_source_, inlTh_, issIdx_source, fpfh_source);
        std::cout << "fpfh " << fpfh_target->size() << " - " << fpfh_source->size() << std::endl;
        //match features
        std::vector<corrTab> corr;
        std::vector<int> corrNOS,corrNOT;
        std::cout<<"matching correspodences..."<<endl;
        CorrComp(fpfh_source, fpfh_target, corr, maxCorr_, corrNOS, corrNOT);
        std::cout<<"NO. corr = "<< corr.size()<<endl;

        auto end = std::chrono::steady_clock::now();
        std::cout<<"->>>>>>>>>>> point cloud preprocessing runtime in ms = "<<std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count()<<endl;

        std::vector<MaxStabbingResult> stabbing_result_v;
        MaxStabbingResult best_result;

        // retrival all the plane correspondence
        for (int i = 0; i < planes_source_.size(); i++) {
            auto i_plane_source = planes_source_.at(i);
            int inlier_count = 0;

            for (int j = 0; j < planes_target_.size(); j++) {

                auto j_plane_target = planes_target_.at(j);

                Eigen::Vector4d tmp_ground_source, tmp_plane_source, tmp_ground_target, tmp_plane_target;
                tmp_ground_source = PlaneModel2Eigen(ground_plane_source_);
                tmp_plane_source = PlaneModel2Eigen(i_plane_source);
                tmp_ground_target = PlaneModel2Eigen(ground_plane_target_);
                tmp_plane_target = PlaneModel2Eigen(j_plane_target);
                std::vector<Eigen::Matrix4d> tmp_T_v = GetPredefinedTransformations2Plane1Line(tmp_ground_source,
                                                                                               tmp_plane_source,
                                                                                               tmp_ground_target,
                                                                                               tmp_plane_target, false);

                auto result = PointMatching(*all_source_, *all_target_, tmp_T_v, corr);
                if (result.inlier_count != -1){
                    result.source_plane_idx = i;
                    result.target_plane_idx = j;
                    stabbing_result_v.push_back(result);
                    if (result.inlier_count > best_result.inlier_count)
                        best_result = result;
                }
//            Eigen::Vector4d transformed_tmp_ground_source = tmp_T_v.at(0).inverse().transpose() * tmp_ground_source;
//            Eigen::Vector4d transformed_tmp_plane_source = tmp_T_v.at(0).inverse().transpose() * tmp_plane_source;
//            Eigen::Vector4d transformed_tmp_ground_target = tmp_T_v.at(1).inverse().transpose() * tmp_ground_target;
//            Eigen::Vector4d transformed_tmp_plane_target = tmp_T_v.at(1).inverse().transpose() * tmp_plane_target;

//            std::cout << "tmp_ground_source: " <<  tmp_ground_source.transpose() << " -> " << transformed_tmp_ground_source.transpose() << std::endl;
//            std::cout << "tmp_plane_source: " <<  tmp_plane_source.transpose() << " -> " << transformed_tmp_plane_source.transpose() << std::endl;
//            std::cout << "tmp_ground_target: " <<  tmp_ground_target.transpose() << " -> " << transformed_tmp_ground_target.transpose() << std::endl;
//            std::cout << "tmp_plane_target: " <<  tmp_plane_target.transpose() << " -> " << transformed_tmp_plane_target.transpose() << std::endl << std::endl;

                // singularity on 180
//            tmp_plane_source = -tmp_plane_source; tmp_plane_source(3) = std::abs(tmp_plane_source(3));
//            tmp_T_v = GetPredefinedTransformations2Plane1Line(tmp_ground_source,
//                                                              tmp_plane_source,
//                                                              tmp_ground_target,
//                                                              tmp_plane_target, false);
//
//            result = PointMatching(*iss_source, *iss_target, tmp_T_v, corr);
//            if (result.inlier_count != -1){
//                result.source_plane_idx = i;
//                result.target_plane_idx = j;
//                stabbing_result_v.push_back(result);
//                if (result.inlier_count > best_result.inlier_count)
//                    best_result = result;
//            }


            }
        }

//        for (int k = 0; k < stabbing_result_v.size(); ++k) {
//            stabbing_result_v.at(k).plot();
//        }

        Eigen::Matrix4d T_cloud_source = best_result.T_source;
        Eigen::Matrix4d T_cloud_target = best_result.T_target;
        double tx = best_result.tx;
        Eigen::Matrix4d T_tx;
        T_tx << 1, 0, 0, tx,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

        // construct final T
//        std::cout << "T_source: " << std::endl << T_cloud_source << std::endl;
//        std::cout << "T_target: " << std::endl << T_cloud_target << std::endl;
        std::cout << "tx: " << tx << std::endl;
        std::cout << "inlier count: " << best_result.inlier_count << std::endl;

        Eigen::Matrix4d final_T = (T_cloud_target).inverse() * T_tx * (T_cloud_source);

        // check for the 180 singularity
//    auto plane_s = TransformPlane(planes_source_.at(6), Eigen::Matrix4d::Identity());
//    auto plane_t = planes_target_.at(8);
//    std::cout << std::endl << "corr plane: source " << (plane_s)
//              << " -> target " << plane_t << std::endl;
//    std::cout << std::endl << "corr plane: source " << PlaneModel2Eigen(plane_s).transpose()
//              << " -> target " << PlaneModel2Eigen(plane_t).transpose() << std::endl;

//    final_T = T_180 * final_T;
//    plane_s = TransformPlane(planes_source_.at(best_result.source_plane_idx), final_T);
//    plane_t = planes_target_.at(best_result.target_plane_idx);
//
//    std::cout << std::endl << "corr plane: source " << PlaneModel2Eigen(plane_s).transpose()
//              << " -> target " << PlaneModel2Eigen(plane_t).transpose() << std::endl;

//    std::cout << "final_T: " << std::endl << final_T << std::endl;

        if (showCloud_){
            // show without T_tx
            PointCloudTX all_source_tran, all_target_tran;
            Eigen::Matrix4d T_view; T_view.setIdentity();
//    T_view << 1,0,0,0,
//            0,1,0,0,
//            0,0,1,30,
//            0,0,0,1;
            Eigen::Matrix4d T_view_source = T_view * T_cloud_source;
            Eigen::Matrix4d T_view_target = T_view * T_cloud_target;
            pcl::transformPointCloud(*all_source_, all_source_tran, T_view_source);
            pcl::transformPointCloud(*all_target_, all_target_tran, T_view_target);

//    std::string save_path = OUTPUT_PATH_ + "after_ground_all_target.pcd";
//    pcl::io::savePCDFileBinary(save_path.c_str(), all_target_tran);
//    save_path = OUTPUT_PATH_ + "after_ground_all_source.pcd";
//    pcl::io::savePCDFileBinary(save_path.c_str(), all_source_tran);

            //show point clouds after VGF
            pcl::visualization::PointCloudColorHandlerCustom<PointTX> colorS(all_source_, 255, 0, 0); //red
            pcl::visualization::PointCloudColorHandlerCustom<PointTX> colorT(all_target_, 255, 255, 0); // Yellow
            pcl::visualization::PointCloudColorHandlerCustom<PointTX> colorS_tran(all_source_tran.makeShared(), 255, 69, 0); // orangeRed
            pcl::visualization::PointCloudColorHandlerCustom<PointTX> colorT_tran(all_target_tran.makeShared(), 255, 215, 0); // gold
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerVGF (new pcl::visualization::PCLVisualizer ("After predefined transformation"));
            viewerVGF->addCoordinateSystem();
            viewerVGF->setBackgroundColor (0, 0, 0);
//    viewerVGF->addPointCloud<PointTX> (all_source_, colorS, "source cloud");
//    viewerVGF->addPointCloud<PointTX> (all_target_, colorT, "target cloud");
            viewerVGF->addPointCloud<PointTX> (all_source_tran.makeShared(), colorS_tran, "transformed source cloud");
            viewerVGF->addPointCloud<PointTX> (all_target_tran.makeShared(), colorT_tran,"transformed target cloud");

            viewerVGF->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "After predefined transformation");
            viewerVGF->spinOnce();

        }

        return  final_T;
    };

    MaxStabbingResult LocalMapMatcher::PointMatching(const PointCloudTX& iss_source,
                                                     const PointCloudTX& iss_target,
                                                     std::vector<Eigen::Matrix4d> tmp_T_v,
                                                     std::vector<corrTab> corr){

        PointCloudTX iss_source_tran, iss_target_tran;
        Eigen::Matrix4d T_cloud_target = tmp_T_v.at(1);
        Eigen::Matrix4d T_cloud_source = tmp_T_v.at(0);
        pcl::transformPointCloud(iss_source, iss_source_tran, T_cloud_source);
        pcl::transformPointCloud(iss_target, iss_target_tran, T_cloud_target);

        std::vector<line_interval> intervals;
        std::vector<int> interval_idxs;
        for (int i = 0; i < corr.size(); i++) {
            auto i_corr = corr.at(i);
            // 3d line - ball intersection problem ->? 2D line-circle in YZ-plane
//                std::cout << iss_source_tran.points.at(i_corr.idxS) << " " << iss_target_tran.points.at(i_corr.idxT) << std::endl;
            auto interval = LineBallIntersection(iss_source_tran.points.at(i_corr.idxS), iss_target_tran.points.at(i_corr.idxT), epsilon_);
            if(!interval.empty) {
                interval_idxs.emplace_back(i);
                interval.cor_idx = i;
                intervals.emplace_back(interval);
            }
        }

        if(!intervals.empty()){

            double out_t = 0;
            int inlier_count = MaxStabbing(intervals, out_t);
            MaxStabbingResult result;
            result.inlier_corr_idxs = interval_idxs;
            result.inlier_count = inlier_count;
            result.tx = out_t;
            result.T_source = T_cloud_source;
            result.T_target = T_cloud_target;

            return result;
        } else
            return {};
    };

    line_interval LocalMapMatcher::LineBallIntersection(PointTX p_source, PointTX p_target, double eps){

        Eigen::Vector3d p_source_v(p_source.x, p_source.y, p_source.z);
        Eigen::Vector3d left_p_source(p_source.x + tx_min_, p_source.y, p_source.z);
        Eigen::Vector3d right_p_source(p_source.x + tx_max_, p_source.y, p_source.z);

        Eigen::Vector3d p_target_v(p_target.x, p_target.y, p_target.z);

        // point - line distance
        double d = (p_target_v - left_p_source).cross(p_target_v - right_p_source).norm() / (right_p_source - left_p_source).norm();

//    std::cout << "d " << d << std::endl;
        if (d > eps){
            return {};
        }

        // line-sphere intersection
        Eigen::Vector3d line_vector = (right_p_source - left_p_source).normalized();
        double delta = pow(line_vector.dot((left_p_source - p_target_v)), 2) - (pow((left_p_source - p_target_v).norm(), 2) - pow(eps, 2));
//    std::cout << "delta " << delta << std::endl;

        if(delta < 0.0)
            return {};
        else if (delta == 0.0){
            double line_distance = - (line_vector.dot(left_p_source - p_target_v));
            Eigen::Vector3d intersection_point = left_p_source + line_distance * line_vector - p_source_v;
            return {intersection_point.x(), intersection_point.x()};
        } else{

            double left_line_distance = - (line_vector.dot(left_p_source - p_target_v)) - sqrt(delta);
            double right_line_distance = - (line_vector.dot(left_p_source - p_target_v)) + sqrt(delta);
            Eigen::Vector3d left_intersection_point = left_p_source + left_line_distance * line_vector - p_source_v;
            Eigen::Vector3d right_intersection_point = left_p_source + right_line_distance * line_vector - p_source_v;
//        std::cout << left_intersection_point.transpose() << " " << right_intersection_point.transpose() << std::endl;
            return {std::max(left_intersection_point.x(), tx_min_), std::min(right_intersection_point.x(), tx_max_)};

        }

    };

    inline bool compareIntervalEnd(const intervalEnd &intA, const intervalEnd &intB)
    {
        return (intA.location<intB.location/*||(intA.location==intB.location&&intA.isStart)*/);
    }

    int LocalMapMatcher::MaxStabbing(std::vector<line_interval>& intervals, double& out_t){

        auto origin_intervals = intervals;
        // assemble the endpoint array
        std::vector<intervalEnd> interval_array;
        for (int i = 0; i < intervals.size(); ++i) {
            intervalEnd left_endpoint, right_endpoint;
            left_endpoint.corrIdx = intervals.at(i).cor_idx;
            left_endpoint.isStart = true;
            left_endpoint.location = intervals.at(i).t_min;

            right_endpoint.corrIdx = intervals.at(i).cor_idx;
            right_endpoint.isStart = false;
            right_endpoint.location = intervals.at(i).t_max;

            interval_array.emplace_back(left_endpoint);
            interval_array.emplace_back(right_endpoint);
        }
        std::sort(interval_array.begin(),interval_array.end(), compareIntervalEnd);

        int inlier_count = 0;

        int currUpbnd = 0;
        int outUpbnd = 0;
        out_t = 0;
        //1. sort interval_array
        std::sort(interval_array.begin(),interval_array.end(), compareIntervalEnd);
        double currLoc = tx_min_;
        int NOEnd = 0;
        for(size_t i=0;i<interval_array.size();i++){
            //is a starting point
            if(interval_array[i].isStart){
                currUpbnd++;
                if(currUpbnd>outUpbnd){
                    outUpbnd = currUpbnd;
                    out_t = interval_array[i].location;
                }
            }
            else{
                NOEnd++;
            }
            if(interval_array[i].location>currLoc){
                currUpbnd-=NOEnd;
                NOEnd = 0;
                currLoc = interval_array[i].location;
            }
        }
        currUpbnd-=NOEnd;

        intervals.clear();
        for (int j = 0; j < origin_intervals.size(); ++j) {

            if (origin_intervals.at(j).t_min <= out_t && out_t <= origin_intervals.at(j).t_max)
                intervals.emplace_back(origin_intervals.at(j));
        }

        return outUpbnd;

    };

    Eigen::Matrix4d LocalMapMatcher::GroundPlane2T(pcl::ModelCoefficients plane_model){

        Eigen::Vector3d normal ({plane_model.values.at(0), plane_model.values.at(1), plane_model.values.at(2)});
        double d(plane_model.values.at(3));

        Eigen::Vector3d x_axis({1,0,0});
        Eigen::Vector3d r3 = normal.normalized();
        Eigen::Vector3d r1 = (x_axis.cross(r3)).normalized();
        Eigen::Vector3d r2 = (r1.cross(r3)).normalized();

        Eigen::Matrix3d R; Eigen::Vector3d t;
        R << r1(0), r2(0), r3(0),
                r1(1), r2(1), r3(1),
                r1(2), r2(2), r3(2);
        t = d * R * (normal.normalized());

        Eigen::Matrix4d T; T.setIdentity();
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = t;

        return T;
    };

    Eigen::Matrix4d LocalMapMatcher::TwoPlane2T(pcl::ModelCoefficients ground_plane,
                                                pcl::ModelCoefficients choosed_plane){

        Eigen::Vector3d ground_norm(ground_plane.values[0], ground_plane.values[1], ground_plane.values[2]);
        Eigen::Vector3d plane_norm(choosed_plane.values[0], choosed_plane.values[1], choosed_plane.values[2]);
        double d_ground = ground_plane.values[3];
        double d_plane = choosed_plane.values[3];

        Eigen::Vector3d line_dir = ground_norm.cross(plane_norm);
        Eigen::Vector3d line_moment = d_ground * plane_norm - d_plane * ground_norm;

        Eigen::Vector3d closed_point = line_moment.cross(line_dir);

        std::cout << "ground plane: " << ground_norm.transpose() << " " << d_ground
                  << "; choosed plane: " << plane_norm.transpose() << " " << d_plane
                  << "; line_dir: " << line_dir.transpose()
                  << "; line_moment: " << line_moment.transpose()
                  << "; closed_point: " << closed_point.transpose() << std::endl;

        Eigen::Matrix3d rot;
        rot << line_dir[0], -line_dir[1], 0,
                line_dir[1], line_dir[0], 0,
                0, 0, 1;

        Eigen::Vector3d t = rot * closed_point;

        Eigen::Matrix4d T; T.setIdentity();
        T.block<3, 3>(0, 0) = rot;
        T.block<3, 1>(0, 3) = t;

        return T;

    };

    std::vector<Eigen::Matrix4d> LocalMapMatcher::GetPredefinedTransformations2Plane1Line(Eigen::Vector4d Pi1, Eigen::Vector4d Pi2,
                                                                                          Eigen::Vector4d Nu1, Eigen::Vector4d Nu2,
                                                                                          bool verbose)
    {

        std::vector<Eigen::Matrix4d> outTrans;
        Eigen::Matrix4d auxTrans;
        Eigen::Matrix3d auxRot;
        Eigen::Vector3d auxT;

        double pi11, pi12, pi13, pi14;
        double pi21, pi22, pi23, pi24;
        double nu11, nu12, nu13, nu14;
        double nu21, nu22, nu23, nu24;

        pi11 = Pi1(0);
        pi12 = Pi1(1);
        pi13 = Pi1(2);
        pi14 = Pi1(3);
        pi21 = Pi2(0);
        pi22 = Pi2(1);
        pi23 = Pi2(2);
        pi24 = Pi2(3);
        nu11 = Nu1(0);
        nu12 = Nu1(1);
        nu13 = Nu1(2);
        nu14 = Nu1(3);
        nu21 = Nu2(0);
        nu22 = Nu2(1);
        nu23 = Nu2(2);
        nu24 = Nu2(3);

        // Compute U Matrix
        auxRot << 1.0 / sqrt((pi12 * pi12 + pi13 * pi13) * pow((pi12 * pi12) * pi21 + (pi13 * pi13) * pi21 - pi11 * pi12 * pi22 - pi11 * pi13 * pi23, 2.0) + pow(pi12 * pi23 - pi13 * pi22, 2.0) * (pi12 * pi12 + pi13 * pi13) * (pi11 * pi11 + pi12 * pi12 + pi13 * pi13)) * (pi12 * pi23 - pi13 * pi22) * (pi12 * pi12 + pi13 * pi13),
                -1.0 / sqrt((pi12 * pi12 + pi13 * pi13) * pow((pi12 * pi12) * pi21 + (pi13 * pi13) * pi21 - pi11 * pi12 * pi22 - pi11 * pi13 * pi23, 2.0) + pow(pi12 * pi23 - pi13 * pi22, 2.0) * (pi12 * pi12 + pi13 * pi13) * (pi11 * pi11 + pi12 * pi12 + pi13 * pi13)) * (pi11 * pi23 - pi13 * pi21) * (pi12 * pi12 + pi13 * pi13),
                1.0 / sqrt((pi12 * pi12 + pi13 * pi13) * pow((pi12 * pi12) * pi21 + (pi13 * pi13) * pi21 - pi11 * pi12 * pi22 - pi11 * pi13 * pi23, 2.0) + pow(pi12 * pi23 - pi13 * pi22, 2.0) * (pi12 * pi12 + pi13 * pi13) * (pi11 * pi11 + pi12 * pi12 + pi13 * pi13)) * (pi11 * pi22 - pi12 * pi21) * (pi12 * pi12 + pi13 * pi13),
                -1.0 / sqrt((pi12 * pi12 + pi13 * pi13) * pow((pi12 * pi12) * pi21 + (pi13 * pi13) * pi21 - pi11 * pi12 * pi22 - pi11 * pi13 * pi23, 2.0) + pow(pi12 * pi23 - pi13 * pi22, 2.0) * (pi12 * pi12 + pi13 * pi13) * (pi11 * pi11 + pi12 * pi12 + pi13 * pi13)) * (pi12 * pi12 + pi13 * pi13) * 1.0 / sqrt(pi11 * pi11 + pi12 * pi12 + pi13 * pi13) * ((pi12 * pi12) * pi21 + (pi13 * pi13) * pi21 - pi11 * pi12 * pi22 - pi11 * pi13 * pi23),
                -1.0 / sqrt((pi12 * pi12 + pi13 * pi13) * pow((pi12 * pi12) * pi21 + (pi13 * pi13) * pi21 - pi11 * pi12 * pi22 - pi11 * pi13 * pi23, 2.0) + pow(pi12 * pi23 - pi13 * pi22, 2.0) * (pi12 * pi12 + pi13 * pi13) * (pi11 * pi11 + pi12 * pi12 + pi13 * pi13)) * (pi12 * pi12 + pi13 * pi13) * 1.0 / sqrt(pi11 * pi11 + pi12 * pi12 + pi13 * pi13) * ((pi11 * pi11) * pi22 + (pi13 * pi13) * pi22 - pi11 * pi12 * pi21 - pi12 * pi13 * pi23),
                -1.0 / sqrt((pi12 * pi12 + pi13 * pi13) * pow((pi12 * pi12) * pi21 + (pi13 * pi13) * pi21 - pi11 * pi12 * pi22 - pi11 * pi13 * pi23, 2.0) + pow(pi12 * pi23 - pi13 * pi22, 2.0) * (pi12 * pi12 + pi13 * pi13) * (pi11 * pi11 + pi12 * pi12 + pi13 * pi13)) * (pi12 * pi12 + pi13 * pi13) * 1.0 / sqrt(pi11 * pi11 + pi12 * pi12 + pi13 * pi13) * ((pi11 * pi11) * pi23 + (pi12 * pi12) * pi23 - pi11 * pi13 * pi21 - pi12 * pi13 * pi22),
                pi11 * 1.0 / sqrt(pi11 * pi11 + pi12 * pi12 + pi13 * pi13),
                pi12 * 1.0 / sqrt(pi11 * pi11 + pi12 * pi12 + pi13 * pi13),
                pi13 * 1.0 / sqrt(pi11 * pi11 + pi12 * pi12 + pi13 * pi13);

        // Compute u vector
        auxT << 0.0,
                -1.0 / sqrt(pi11 * pi11 + pi12 * pi12 + pi13 * pi13) * ((pi11 * pi11) * pi24 + (pi12 * pi12) * pi24 + (pi13 * pi13) * pi24 - pi11 * pi14 * pi21 - pi12 * pi14 * pi22 - pi13 * pi14 * pi23) * 1.0 / sqrt((pi11 * pi11) * (pi22 * pi22) + (pi12 * pi12) * (pi21 * pi21) + (pi11 * pi11) * (pi23 * pi23) + (pi13 * pi13) * (pi21 * pi21) + (pi12 * pi12) * (pi23 * pi23) + (pi13 * pi13) * (pi22 * pi22) - pi11 * pi12 * pi21 * pi22 * 2.0 - pi11 * pi13 * pi21 * pi23 * 2.0 - pi12 * pi13 * pi22 * pi23 * 2.0),
                pi14 * 1.0 / sqrt(pi11 * pi11 + pi12 * pi12 + pi13 * pi13);

        auxTrans.setIdentity();
        auxTrans.topLeftCorner(3, 3) = auxRot;
        auxTrans.topRightCorner(3, 1) = auxT;

        if (verbose)
            cout << "[VERBOSE] Matrix [U,u;0,0,0,1]" << endl
                 << auxTrans << endl;
        if (verbose)
            cout << "[VERBOSE] inv(T)*T = eye(3)" << endl
                 << auxTrans.inverse() * auxTrans << endl;

        // add U,u to the output vector
        outTrans.push_back(auxTrans);

        // Compute V Matrix
        auxRot << 1.0 / sqrt((nu12 * nu12 + nu13 * nu13) * pow((nu12 * nu12) * nu21 + (nu13 * nu13) * nu21 - nu11 * nu12 * nu22 - nu11 * nu13 * nu23, 2.0) + pow(nu12 * nu23 - nu13 * nu22, 2.0) * (nu12 * nu12 + nu13 * nu13) * (nu11 * nu11 + nu12 * nu12 + nu13 * nu13)) * (nu12 * nu23 - nu13 * nu22) * (nu12 * nu12 + nu13 * nu13),
                -1.0 / sqrt((nu12 * nu12 + nu13 * nu13) * pow((nu12 * nu12) * nu21 + (nu13 * nu13) * nu21 - nu11 * nu12 * nu22 - nu11 * nu13 * nu23, 2.0) + pow(nu12 * nu23 - nu13 * nu22, 2.0) * (nu12 * nu12 + nu13 * nu13) * (nu11 * nu11 + nu12 * nu12 + nu13 * nu13)) * (nu11 * nu23 - nu13 * nu21) * (nu12 * nu12 + nu13 * nu13),
                1.0 / sqrt((nu12 * nu12 + nu13 * nu13) * pow((nu12 * nu12) * nu21 + (nu13 * nu13) * nu21 - nu11 * nu12 * nu22 - nu11 * nu13 * nu23, 2.0) + pow(nu12 * nu23 - nu13 * nu22, 2.0) * (nu12 * nu12 + nu13 * nu13) * (nu11 * nu11 + nu12 * nu12 + nu13 * nu13)) * (nu11 * nu22 - nu12 * nu21) * (nu12 * nu12 + nu13 * nu13),
                -1.0 / sqrt((nu12 * nu12 + nu13 * nu13) * pow((nu12 * nu12) * nu21 + (nu13 * nu13) * nu21 - nu11 * nu12 * nu22 - nu11 * nu13 * nu23, 2.0) + pow(nu12 * nu23 - nu13 * nu22, 2.0) * (nu12 * nu12 + nu13 * nu13) * (nu11 * nu11 + nu12 * nu12 + nu13 * nu13)) * (nu12 * nu12 + nu13 * nu13) * 1.0 / sqrt(nu11 * nu11 + nu12 * nu12 + nu13 * nu13) * ((nu12 * nu12) * nu21 + (nu13 * nu13) * nu21 - nu11 * nu12 * nu22 - nu11 * nu13 * nu23),
                -1.0 / sqrt((nu12 * nu12 + nu13 * nu13) * pow((nu12 * nu12) * nu21 + (nu13 * nu13) * nu21 - nu11 * nu12 * nu22 - nu11 * nu13 * nu23, 2.0) + pow(nu12 * nu23 - nu13 * nu22, 2.0) * (nu12 * nu12 + nu13 * nu13) * (nu11 * nu11 + nu12 * nu12 + nu13 * nu13)) * (nu12 * nu12 + nu13 * nu13) * 1.0 / sqrt(nu11 * nu11 + nu12 * nu12 + nu13 * nu13) * ((nu11 * nu11) * nu22 + (nu13 * nu13) * nu22 - nu11 * nu12 * nu21 - nu12 * nu13 * nu23),
                -1.0 / sqrt((nu12 * nu12 + nu13 * nu13) * pow((nu12 * nu12) * nu21 + (nu13 * nu13) * nu21 - nu11 * nu12 * nu22 - nu11 * nu13 * nu23, 2.0) + pow(nu12 * nu23 - nu13 * nu22, 2.0) * (nu12 * nu12 + nu13 * nu13) * (nu11 * nu11 + nu12 * nu12 + nu13 * nu13)) * (nu12 * nu12 + nu13 * nu13) * 1.0 / sqrt(nu11 * nu11 + nu12 * nu12 + nu13 * nu13) * ((nu11 * nu11) * nu23 + (nu12 * nu12) * nu23 - nu11 * nu13 * nu21 - nu12 * nu13 * nu22),
                nu11 * 1.0 / sqrt(nu11 * nu11 + nu12 * nu12 + nu13 * nu13),
                nu12 * 1.0 / sqrt(nu11 * nu11 + nu12 * nu12 + nu13 * nu13),
                nu13 * 1.0 / sqrt(nu11 * nu11 + nu12 * nu12 + nu13 * nu13);

        // Compute v vector
        auxT << 0.0,
                -1.0 / sqrt(nu11 * nu11 + nu12 * nu12 + nu13 * nu13) * ((nu11 * nu11) * nu24 + (nu12 * nu12) * nu24 + (nu13 * nu13) * nu24 - nu11 * nu14 * nu21 - nu12 * nu14 * nu22 - nu13 * nu14 * nu23) * 1.0 / sqrt((nu11 * nu11) * (nu22 * nu22) + (nu12 * nu12) * (nu21 * nu21) + (nu11 * nu11) * (nu23 * nu23) + (nu13 * nu13) * (nu21 * nu21) + (nu12 * nu12) * (nu23 * nu23) + (nu13 * nu13) * (nu22 * nu22) - nu11 * nu12 * nu21 * nu22 * 2.0 - nu11 * nu13 * nu21 * nu23 * 2.0 - nu12 * nu13 * nu22 * nu23 * 2.0),
                nu14 * 1.0 / sqrt(nu11 * nu11 + nu12 * nu12 + nu13 * nu13);

        auxTrans.setIdentity();
        auxTrans.topLeftCorner(3, 3) = auxRot;
        auxTrans.topRightCorner(3, 1) = auxT;

        if (verbose)
            cout << "[VERBOSE] Matrix [V,v;0,0,0,1]" << endl
                 << auxTrans << endl;
        if (verbose)
            cout << "[VERBOSE] inv(T)*T = eye(3)" << endl
                 << auxTrans.inverse() * auxTrans << endl;

        // add V,v to the output vector
        outTrans.push_back(auxTrans);

        return outTrans;
    };


    pcl::PointCloud<pcl::PointXYZ>  LocalMapMatcher::ChangeCloud(pcl::PointCloud<pcl::PointXYZI> cloud) {

        pcl::PointCloud<pcl::PointXYZ> new_cloud;
        for (int i = 0; i < cloud.size(); ++i) {

            pcl::PointXYZ point;
            point.x = cloud.at(i).x;
            point.y = cloud.at(i).y;
            point.z = cloud.at(i).z;

            new_cloud.points.emplace_back(point);
        }
        return new_cloud;
    }

    pcl::ModelCoefficients LocalMapMatcher::TransformPlane(pcl::ModelCoefficients plane, Eigen::Matrix4d T){

        // pcl plane model (ax+by+cz+d=0);
        // my plane model (ax+by+cz-d=0) -> cp = n*d

        Eigen::Vector3d normal_vector;
        double d;

        if (plane.values.size() == 4){
            normal_vector = Eigen::Vector3d(plane.values.at(0), plane.values.at(1), plane.values.at(2)).normalized();
            d = plane.values.at(3);
        } else{
            normal_vector = Eigen::Vector3d(plane.values.at(0), plane.values.at(1), plane.values.at(2)).normalized();
            d = Eigen::Vector3d(plane.values.at(0), plane.values.at(1), plane.values.at(2)).norm();
        }

        Eigen::Vector4d plane_param = Eigen::Vector4d(normal_vector[0], normal_vector[1], normal_vector[2], d);

        Eigen::Vector4d transformed_plane = T.inverse().transpose() * plane_param;

        pcl::ModelCoefficients plane_tran;
        plane_tran.values.clear();
        plane_tran.values.emplace_back(transformed_plane[0]);
        plane_tran.values.emplace_back(transformed_plane[1]);
        plane_tran.values.emplace_back(transformed_plane[2]);
        plane_tran.values.emplace_back(transformed_plane[3]);
        return plane_tran;
    };

    Eigen::Vector4d LocalMapMatcher::PlaneModel2Eigen(pcl::ModelCoefficients plane){

        // pcl plane model (ax+by+cz+d=0);
        // my plane model (ax+by+cz-d=0) -> cp = n*d

        Eigen::Vector3d normal_vector;
        double d;

        if (plane.values.size() == 4){
            normal_vector = Eigen::Vector3d(plane.values.at(0), plane.values.at(1), plane.values.at(2)).normalized();
            d = plane.values.at(3);
        } else{
            normal_vector = Eigen::Vector3d(plane.values.at(0), plane.values.at(1), plane.values.at(2)).normalized();
            d = Eigen::Vector3d(plane.values.at(0), plane.values.at(1), plane.values.at(2)).norm();
        }

        Eigen::Vector4d plane_param = Eigen::Vector4d(normal_vector[0], normal_vector[1], normal_vector[2], d);

        return plane_param;

    };

    Eigen::Vector3d LocalMapMatcher::R2ypr(const Eigen::Matrix3d &R)
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

    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 3> LocalMapMatcher::ypr2R(const Eigen::MatrixBase<Derived> &ypr)
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

    RegistrationResult LocalMapMatcher::ICPRegistration(PointCloudTX ref_cloud, PointCloudTX query_cloud, Eigen::Matrix4f init_guess) {


        pcl::IterativeClosestPoint<PointTX, PointTX> icp;
        icp.setMaxCorrespondenceDistance(1);
        icp.setMaximumIterations(20);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
//        icp.setRANSACIterations(0);
        // Align clouds
        icp.setInputSource(query_cloud.makeShared());
        icp.setInputTarget(ref_cloud.makeShared());
        PointCloudTX::Ptr unused_result(new PointCloudTX());
        icp.align(*unused_result, init_guess);

        std::cout << "icp Converged: " << icp.hasConverged()
                  << " fitnessScore: " << icp.getFitnessScore()
                  << " No. iteration: " << icp.nr_iterations_
                  << " ConvergenceState: " << icp.getConvergeCriteria()->getConvergenceState()<< std::endl;

        RegistrationResult new_result;
        new_result.fitness_score_ = icp.getFitnessScore();
        new_result.convergence_ = icp.hasConverged();
        Eigen::Matrix4f rel_T  = icp.getFinalTransformation();
        new_result.rel_rotation_  = rel_T.block(0, 0, 3, 3);
        new_result.rel_trans_ = rel_T.block(0, 3, 3, 1);

        return new_result;
    }

    void LocalMapMatcher::ErrorEvaluation(Eigen::Matrix4d GT, Eigen::Matrix4d Mes){

        Eigen::Matrix4d diff_SE3 = GT * Mes.inverse();

//    std::cout << "diff mat: " << std::endl << diff_SE3 << std::endl;

        Eigen::Matrix3d R_diff = diff_SE3.block(0, 0, 3, 3);
        Eigen::Vector3d t_diff = diff_SE3.block(0, 3, 3, 1);

        std::cout << "error: " << R2ypr(R_diff).transpose()  << " " << t_diff.transpose() << std::endl;

    }



}