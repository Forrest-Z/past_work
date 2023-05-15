//
// Created by xc on 2020/12/4.
//

#include "gp_lio/localization/lgi_based_localization.h"

namespace gp_lio{

    Localization::Localization():
    ndt_ptr_(new pcl::NormalDistributionsTransform<pcl::PointXYZI,pcl::PointXYZI>),
    local_map_(new pcl::PointCloud<pcl::PointXYZI>),
    global_map_(new pcl::PointCloud<pcl::PointXYZI>),
    current_scan_(new pcl::PointCloud<pcl::PointXYZI>){
        InitWithConfig();
        InitGlobalMap();
        Eigen::Vector3d map_init(0.0,0.0,0.0);
        ResetLocalMap(map_init);
        has_inited_ = false;

    };
    Localization::~Localization() {};


    bool Localization::InitWithConfig() {

        map_path_ = MAP_PATH;
        SetNdtParam(RES,STEP_SIZE,TRANS_EPS,MAX_ITER);
        global_map_filter_.setLeafSize(GLOBAL_MAP_FILTER,GLOBAL_MAP_FILTER,GLOBAL_MAP_FILTER);
        local_map_filter_.setLeafSize(LOCAL_MAP_FILTER,LOCAL_MAP_FILTER,LOCAL_MAP_FILTER);
        current_frame_filter_.setLeafSize(FRAME_FILTER,FRAME_FILTER,FRAME_FILTER);
        local_map_size_ = LOCAL_MAP_SIZE;
    }

    bool Localization::ResetLocalMap(const Eigen::Vector3d &frame_pose) {
        map_manager_.GetLocalMap(local_map_,all_local_map_length_,frame_pose,local_map_size_);
        std::cout << "local map size: " << local_map_->size();
        local_map_filter_.setInputCloud(local_map_);
        local_map_filter_.filter(*local_map_);
        std::cout << "  ----->  " << local_map_->size() <<std::endl;
        ndt_ptr_->setInputTarget(local_map_);
        has_new_local_map_ = true;
    }

    bool Localization::GetGlobalMap(pcl::PointCloud<pcl::PointXYZI>::Ptr &global_map) {;
        global_map_filter_.setInputCloud(global_map_);
        global_map_filter_.filter(*global_map_);
        global_map = global_map_;
    }

    bool Localization::GetLocalMap(pcl::PointCloud<pcl::PointXYZI>::Ptr &local_map) {
        local_map = local_map_;
    }

    bool Localization::GetCurrentScan(pcl::PointCloud<pcl::PointXYZI>::Ptr &current_scan) {
        current_scan = current_scan_;

    }

    bool Localization::HasInited() {
        return has_inited_;

    }

    bool Localization::HasNewGlobalMap() {
        return has_new_gloabl_map_;
    }

    bool Localization::HasNewLocalMap() {
        return has_new_local_map_;
    }


    bool Localization::SetInitPose(const Eigen::Matrix4f& init_pose) {
        init_pose_ = init_pose;
        Eigen::Vector3d tmp_pose = init_pose.topRightCorner(3,1).cast<double>();
        ResetLocalMap(tmp_pose);
        return true;
    }

    bool Localization::InitGlobalMap() {
        map_manager_.LoadMap(global_map_,map_path_);
        has_new_gloabl_map_ = true;
        std::cout << "global map size: " << global_map_->size() <<std::endl;
        return true;
    }

    bool Localization::Update(pcl::PointCloud<pcl::PointXYZI>::Ptr &current_scan, Eigen::Matrix4f &cloud_pose) {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*current_scan,*current_scan,indices);
        current_frame_filter_.setInputCloud(current_scan);
        current_frame_filter_.filter(*current_scan_);

        static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
        static Eigen::Matrix4f last_pose = init_pose_;
        static Eigen::Matrix4f predict_pose = init_pose_;

        if (!has_inited_) {
            //last_pose = current_gauss_pose_;  //TODO::
            predict_pose = current_gauss_pose_;
        }
       //match
        pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        ScanMatch(current_scan_,predict_pose,result_cloud_ptr,cloud_pose);
        pcl::transformPointCloud(*current_scan_,*current_scan_,cloud_pose);

        step_pose = last_pose.inverse()*cloud_pose;
        predict_pose = cloud_pose * step_pose;
        last_pose = cloud_pose;
        forward_length_ += step_pose.topRightCorner(3,1).norm(); // 累计路程
        // whether or not to update local map
        if((forward_length_/all_local_map_length_) > 0.3)
            ResetLocalMap(cloud_pose.topRightCorner(3,1).cast<double>());
        return true;

    }

    void Localization::SetNdtParam(float res, float step_size, float trans_eps, int max_iter) {
        ndt_ptr_->setResolution(res);
        ndt_ptr_->setStepSize(step_size);
        ndt_ptr_->setTransformationEpsilon(trans_eps);
        ndt_ptr_->setMaximumIterations(max_iter);
    }

    bool Localization::ScanMatch(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
            const Eigen::Matrix4f& predict_pose,
            pcl::PointCloud<pcl::PointXYZI>::Ptr& result_cloud,
            Eigen::Matrix4f& result_pose) {
        ndt_ptr_->setInputSource(source);
        ndt_ptr_->align(*result_cloud, predict_pose);
        result_pose = ndt_ptr_->getFinalTransformation();
    }

    void Localization::GaussPoseByGnss(Eigen::Vector3d& gnss_pose) {
        Eigen::Vector3d p;
        Eigen::Quaterniond q;
        map_manager_.GetInitPose(gnss_pose,p,q);
        current_gauss_pose_.block<3,3>(0,0) = q.toRotationMatrix().cast<float>() ;
        current_gauss_pose_.topRightCorner<3,1>() = p.cast<float>();
        std::cout << std::fixed << std::setprecision(1);
        std::cout << "gnss " << gnss_pose.x() <<" " << gnss_pose.y() <<" " << gnss_pose.z() <<" ";
        std::cout <<"-------> " << p.x() << " "  << p.y() << " " << p.z() << " "<<std::endl;
        static int gnss_cnt = 0;
        if(gnss_cnt==0)
            SetInitPose(current_gauss_pose_);
        if(gnss_cnt > 3){
            has_inited_ = true;
            std::cout << "Localization Iinited" <<std::endl;
        }
        gnss_cnt++;
    }


}
