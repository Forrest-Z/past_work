//
// Created by wchen on 2020/7/20.
//

#include "ros_test/LocalMapMatching.h"

namespace gp_lio{

    LocalMapMatching::LocalMapMatching(): loop_count_(4), ref_idx_(21), query_idx_(368) {

        downsize_filter_XYZI_.setLeafSize(0.3, 0.3, 0.3);

        LoadData();

        align_mat_.setIdentity();

    }

    LocalMapMatching::~LocalMapMatching() {

    }

    void LocalMapMatching::LoadData() {

        std::string OUTPUT_PATH = "/home/wchen/Projects/Code/catkin_ws/src/gp_lio/result/tmp_result/";

        pcl::PointCloud<pcl::PointXYZ> ref_cloud;
        pcl::PointCloud<pcl::PointXYZ> query_cloud;

        std::string ref_save_file = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) +"_target_"+ std::to_string(ref_idx_) + ".pcd";
        pcl::io::loadPCDFile(ref_save_file,  ref_cloud);

        std::string query_save_file = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) + "_source_"+ std::to_string(query_idx_) + ".pcd";
        pcl::io::loadPCDFile(query_save_file, query_cloud);

        std::string ref_ground_save_file = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) +"_target_ground_" + std::to_string(ref_idx_) + ".pcd";
        pcl::io::loadPCDFile(ref_ground_save_file,  ref_ground_cloud_);
        ref_cloud_ += ref_ground_cloud_;

        std::string query_ground_save_file = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) + "_source_ground_"+ std::to_string(query_idx_) + ".pcd";
        pcl::io::loadPCDFile(query_ground_save_file, query_ground_cloud_);
        query_cloud_ += query_ground_cloud_;

        std::string ref_plane_save_file = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) + "_ref_plane_cloud_" + std::to_string(ref_idx_) + ".pcd";
        pcl::io::loadPCDFile(ref_plane_save_file, ref_plane_cloud_);
        ref_cloud_ += ref_plane_cloud_;

        std::string query_plane_save_file = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) + "_query_plane_cloud_" + std::to_string(query_idx_) + ".pcd";
        pcl::io::loadPCDFile(query_plane_save_file, query_plane_cloud_);
        query_cloud_ += query_plane_cloud_;

        ref_cloud_ = ChangeCloud(ref_cloud);
        query_cloud_ = ChangeCloud(query_cloud);

        std::string plane_2d_line_path = OUTPUT_PATH + "loop_" + std::to_string(loop_count_) + "_plane_2d_line_correspondence.txt";
        std::ifstream fin(plane_2d_line_path.c_str());
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

                Plane2D plane_line;
                plane_line.line_left_ << line_v.at(1), line_v.at(2);
                plane_line.line_right_ << line_v.at(3), line_v.at(4);
                plane_line.cp_ << line_v.at(5), line_v.at(6);

                if (line_v.at(0) == 0){
                    ref_planes_.emplace_back(plane_line);
                } else {
                    query_planes_.emplace_back(plane_line);
                }

//            std::cout << std::endl;
            }
            fin.close();

        } else{
            std::cerr << "cannot read file " << plane_2d_line_path << std::endl;
            exit(0);
        }

    }

    void LocalMapMatching::Run() {

        ShowClouds(ref_cloud_, query_cloud_, "before registration");

        GroundRegistration();

        PlaneRegistration();

        AllCloudRegistration();



        std::cout << "complete registration " << std::endl;
    }

    void LocalMapMatching::GroundRegistration() {
        std::cout << std::endl << std::endl << "ground registration" << std::endl;
        // Roll Pitch Z using ground points
//        ShowClouds(ref_ground_cloud_, query_ground_cloud_, "ground");
        RegistrationResult ground_result = ICPRegistration(query_ground_cloud_, ref_ground_cloud_);

        Eigen::Matrix4f init_guess; init_guess.setIdentity();
        if(ground_result.convergence_){

            auto ypr = ground_result.GetYPR();
            std::cout << "ypr : " << ypr.transpose() * 180.0 / M_PI << std::endl;
            std::cout << "position : " << ground_result.rel_trans_.transpose() << std::endl;
            PointCloudT transformed_cloud;

            Eigen::Vector3f xyz = ground_result.rel_trans_;
            Eigen::Matrix3f rot = Utility::ypr2R(Eigen::Vector3f{0, ypr[1], ypr[2]});
            Eigen::Vector3f position(0,0,xyz[2]);

            init_guess.block<3, 3>(0, 0) = rot;
            init_guess.block<3, 1>(0, 3) = position;
            pcl::transformPointCloud(query_ground_cloud_, transformed_cloud, init_guess);
//            ShowClouds(ref_ground_cloud_, transformed_cloud, "ground_refined");

            align_mat_ = init_guess;

        }else{
            std::cout << "ground registration failed!!!" << std::endl;
        }
    }

    void LocalMapMatching::PlaneRegistration() {

        std::cout <<  std::endl <<  std::endl << "Plane registration " << std::endl;

        Eigen::Matrix4f init_guess = align_mat_;
        // Yaw using plane points
        ShowClouds(ref_plane_cloud_, query_plane_cloud_, "plane");
        RegistrationResult plane_result = ICPRegistration(query_plane_cloud_, ref_plane_cloud_, init_guess);
        if(plane_result.convergence_){

            auto plane_ypr = plane_result.GetYPR();
            std::cout << "plane_ypr : " << plane_ypr.transpose()  * 180.0 / M_PI << std::endl;
            std::cout << "position : " << plane_result.rel_trans_.transpose() << std::endl;

            Eigen::Matrix3f ground_rot = align_mat_.block(0, 0, 3, 3);
            Eigen::Vector3f ground_ypr = Utility::R2ypr(ground_rot) / 180.0 * M_PI;
            Eigen::Vector3f ground_position = align_mat_.block(0, 3, 3, 1);

            Eigen::Matrix4f refined_guess; refined_guess.setIdentity();
            Eigen::Matrix3f rot = Utility::ypr2R(Eigen::Vector3f{plane_ypr[0], ground_ypr[1], ground_ypr[2]});
            Eigen::Vector3f position(0, 0, ground_position[2]);

            refined_guess.block<3, 3>(0, 0) = rot;
            refined_guess.block<3, 1>(0, 3) = position;

            PointCloudT transformed_cloud;
            pcl::transformPointCloud(query_plane_cloud_, transformed_cloud, refined_guess);
            ShowClouds(ref_plane_cloud_, transformed_cloud, "plane_refined");

            align_mat_ = refined_guess;

        }else{
            std::cout << "plane registration failed!!!" << std::endl;
        }

    }

    void LocalMapMatching::AllCloudRegistration() {

        std::cout <<  std::endl <<  std::endl << "cloud registration "  << std::endl;

        Eigen::Matrix4f init_guess = align_mat_;

        PointCloudT transformed_query_cloud;
        pcl::transformPointCloud(query_cloud_, transformed_query_cloud, init_guess);
        ShowClouds(ref_cloud_, transformed_query_cloud, "all");
        RegistrationResult cloud_result = ICPRegistration(query_cloud_, ref_cloud_, init_guess);
        if(cloud_result.convergence_){
            auto ypr = cloud_result.GetYPR();
            std::cout << "ypr : " << ypr.transpose() * 180.0 / M_PI << std::endl;
            std::cout << "position : " << cloud_result.rel_trans_.transpose() << std::endl;
            PointCloudT transformed_cloud;
            pcl::transformPointCloud(query_cloud_, transformed_cloud, cloud_result.GetTransformation());
            ShowClouds(ref_cloud_, transformed_cloud, "cloud_refined");

            Eigen::Matrix4f final_guess; final_guess.setIdentity();
            final_guess.block<3, 3>(0, 0) = cloud_result.rel_rotation_;
            final_guess.block<3, 1>(0, 3) = cloud_result.rel_trans_;
            align_mat_ = final_guess;

            Eigen::Quaternionf qua(cloud_result.rel_rotation_);
            std::cout << "final result " << std::endl
                      << cloud_result.rel_trans_.x() << " "
                      << cloud_result.rel_trans_.y() << " "
                      << cloud_result.rel_trans_.z() << " "
                      << qua.x() << " "
                      << qua.y() << " "
                      << qua.z() << " "
                      << qua.w() << std::endl;
        }else{
            std::cout << "All cloud registration failed!!!" << std::endl;
        }
    }

    PointCloudT LocalMapMatching::ChangeCloud(pcl::PointCloud<pcl::PointXYZ> cloud) {

        PointCloudT new_cloud;
        for (int i = 0; i < cloud.size(); ++i) {

            PointT point;
            point.x = cloud.at(i).x;
            point.y = cloud.at(i).y;
            point.z = cloud.at(i).z;

            new_cloud.points.emplace_back(point);
        }
        return new_cloud;

    }

    void LocalMapMatching::ShowClouds(gp_lio::PointCloudT ref_cloud, gp_lio::PointCloudT query_cloud,
                                      std::string type) {

        cloud_viewer_.reset(new pcl::visualization::PCLVisualizer("show_"+type+"_cloud"));
        cloud_viewer_->setBackgroundColor(255,255,255);
        cloud_viewer_->addCoordinateSystem(1.0);
        cloud_viewer_->initCameraParameters();

        pcl::visualization::PointCloudColorHandlerCustom<PointT> ref_color(255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> query_color(0, 0, 255);

        cloud_viewer_->addPointCloud<PointT>(ref_cloud.makeShared(), ref_color, "ref_"+type);
        cloud_viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "ref_"+type);

        cloud_viewer_->addPointCloud<PointT>(query_cloud.makeShared(), query_color, "query_"+type);
        cloud_viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "query_"+type);

        while (!cloud_viewer_->wasStopped()){

            cloud_viewer_->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100));
        }

    }

    RegistrationResult LocalMapMatching::ICPRegistration(pcl::PointCloud<pcl::PointXYZI> source_cloud,
                                                         pcl::PointCloud<pcl::PointXYZI> target_cloud,
                                                         Eigen::Matrix4f init_guess) {
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setMaxCorrespondenceDistance(10);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
//        icp.setEuclideanFitnessEpsilon(1);
        icp.setRANSACIterations(0);
        // Align clouds
        icp.setInputSource(source_cloud.makeShared());
        icp.setInputTarget(target_cloud.makeShared());
        pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());
        icp.align(*unused_result, init_guess);

        std::cout << source_cloud.size() << " " << target_cloud.size() <<  std::endl;
        std::cout << icp.getCorrespondenceRejectors().size() << std::endl;
        std::cout << icp.getIndices()->size() << std::endl;

        std::cout << "icp Converged: " << icp.hasConverged() << " fitnessScore: " << icp.getFitnessScore() << std::endl;

        RegistrationResult new_result;
        new_result.fitness_score_ = icp.getFitnessScore();
        new_result.convergence_ = icp.hasConverged();
        Eigen::Matrix4f rel_T  = icp.getFinalTransformation();
        new_result.rel_rotation_  = rel_T.block(0, 0, 3, 3);
        new_result.rel_trans_ = rel_T.block(0, 3, 3, 1);

        return new_result;
    }

}

using namespace gp_lio;

int main(int argc, char **argv){

    ros::init(argc, argv, "LocalMapMatching");
    LocalMapMatching localMapMatching;

    localMapMatching.Run();

    return 0;
}