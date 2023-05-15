//
// Created by wchen on 2019/12/3.
//

#include "gp_lio/feature/LidarFeature.h"

namespace gp_lio
{

    LidarFeature::LidarFeature()
    {
    }

    CloudFeature LidarFeature::PutinCloud(CloudMeasurement &cloud_measurement, PointCloudT &cloud_last, Eigen::Matrix4d &transform_matrix)
    {
        Init();

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(cloud_measurement.cloud_, cloud_, indices);
        last_cloud_ = cloud_;
        received_time_ = cloud_measurement.receive_time_;

        dynamic_filter(cloud_last, cloud_, transform_matrix,threshold_filter_still, threshold_filter, threshold_filter_turning,  1); 

        RangeImageProcess();

        FeatureExtractionProcess();

        FreeMemory();

        return extracted_feature_;
    }

    void LidarFeature::dynamic_filter(PointCloudT &point_cloud1, PointCloudT &point_cloud2, Eigen::Matrix4d &transform_matrix, float threshold1, float threshold2, float threshold3, int k) 
    {

        float threshold_filter_adapted;
        threshold_filter_adapted = threshold2;
        // std::cout << "still " <<  std::endl;

        // if(is_moving)
        // {
        //     std::cout << "moving " <<  std::endl;
        //     threshold_filter_adapted = 10*threshold;

        // }
        if(is_still)
        {
            threshold_filter_adapted = threshold1;
            std::cout << "still " <<  std::endl;
        }

        if(is_turning)
        {
            threshold_filter_adapted = threshold3;
            std::cout << "turning " <<  std::endl;
            is_turning_keep_on = 3;

        }
        else if(is_turning_keep_on)
        {
            is_turning_keep_on--;
            threshold_filter_adapted = threshold3;
            std::cout << "turning keep " <<  std::endl;
        }
        
        
        std::cout << "threshold_filter_adapted " << threshold_filter_adapted <<  std::endl;

        PointCloudT point_cloud1_;
    
        pcl::transformPointCloud(point_cloud1, point_cloud1_, transform_matrix);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr range_cloud1_(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(point_cloud1_, *range_cloud1_);
        int cloudSize1 = point_cloud1_.points.size();

        // std::cout << "range_cloud1_[0] " << range_cloud1_->points[0]  << std::endl;
        // std::cout << "range_cloud1_[1] " << range_cloud1_->points[1]  << std::endl;

        pcl::PointXYZ thisPoint;
        float theta, beta;

        for (size_t i = 0; i < cloudSize1; ++i)
        {


            beta = atan2(range_cloud1_->points[i].z, sqrt(range_cloud1_->points[i].x * range_cloud1_->points[i].x + range_cloud1_->points[i].y * range_cloud1_->points[i].y)) * 180 / M_PI;
            theta = atan2(range_cloud1_->points[i].y, range_cloud1_->points[i].x) * 180 / M_PI;


            thisPoint.x = 0;
            thisPoint.y = theta;
            thisPoint.z = beta;

            range_cloud1_->points[i] = thisPoint;

        }
        // return *rangeCloud__;

        // *range_cloud1_ = transform_PointCloud2Sphere(*range_cloud1_);

        TicToc time1;

        pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
        kdTree.setInputCloud(range_cloud1_);

        // std::cout << "range_cloud1_[0] " << range_cloud1_->points[0]  << std::endl;
        // std::cout << "range_cloud1_[1] " << range_cloud1_->points[1]  << std::endl;

        // ROS_DEBUG("KD tree build cost %f ms", time1.toc());

        pcl::PointCloud<pcl::PointXYZ>::Ptr range_cloud2_(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(point_cloud2, *range_cloud2_);
        int cloudSize2 = range_cloud2_->points.size();

        // *range_cloud2_ = transform_PointCloud2Sphere(*range_cloud2_);
        for (size_t i = 0; i < cloudSize2; ++i)
        {


            beta = atan2(range_cloud2_->points[i].z, sqrt(range_cloud2_->points[i].x * range_cloud2_->points[i].x + range_cloud2_->points[i].y * range_cloud2_->points[i].y)) * 180 / M_PI;
            theta = atan2(range_cloud2_->points[i].y, range_cloud2_->points[i].x) * 180 / M_PI;


            thisPoint.x = 0;
            thisPoint.y = theta;
            thisPoint.z = beta;

            range_cloud2_->points[i] = thisPoint;

        }

        // int cloudSize1 = point_cloud1_.points.size();
        // int cloudSize2 = range_cloud2_->points.size();

        std::vector<int> neighbor_index;
        std::vector<float> neighbor_square_distance;
        // pcl::PointIndices::Ptr inliers(new pcl::PointIndices()); 
        // pcl::ExtractIndices<pcl::PointXYZ> extract;
        float diff_relative, R1, R2;
        // long unsigned int index_i=0;

        TicToc time2;

        for (int i=0;i<cloudSize2;++i)
        {

            if(kdTree.nearestKSearch(range_cloud2_->points[i], k, neighbor_index, neighbor_square_distance) > 0)
            {

                R1 = sqrt(point_cloud2.points[i].x * point_cloud2.points[i].x + point_cloud2.points[i].y * point_cloud2.points[i].y + point_cloud2.points[i].z * point_cloud2.points[i].z);
                R2 = sqrt(point_cloud1_.points[neighbor_index[0]].x * point_cloud1_.points[neighbor_index[0]].x + point_cloud1_.points[neighbor_index[0]].y * point_cloud1_.points[neighbor_index[0]].y + point_cloud1_.points[neighbor_index[0]].z * point_cloud1_.points[neighbor_index[0]].z);
                float diff_relative = (R2-R1)/(R1+R2);

                // std::cout << "diff_relative " << diff_relative << std::endl;


                if (fabs(diff_relative) > threshold_filter_adapted) 
                {
                    // inliers->indices.push_back(i);
                    // std::cout << "dynamic points " <<  std::endl;
                    point_cloud2.points[i].x = 0;
                    point_cloud2.points[i].y = 0;
                    point_cloud2.points[i].z = 0;
                    point_cloud2.points[i].intensity = 0;

                }
            }

        }

        ROS_DEBUG("KD nearestKSearch cost %f ms", time2.toc());

        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
        // extract.setInputCloud(range_cloud2_);
        // extract.setIndices(inliers); 
        // extract.setNegative(true); 
        // extract.filter(*cloud_XYZ);
        pcl::copyPointCloud(point_cloud2, cloud_);

        point_cloud1_.clear();
        range_cloud1_->clear();
        range_cloud2_->clear();


    }
    LidarFeature::~LidarFeature()
    {
        extracted_feature_.clear();
    }

    void LidarFeature::Init()
    {

        rangeMat_ = cv::Mat(N_SCAN, HORIZON_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat_ = cv::Mat(N_SCAN, HORIZON_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat_ = cv::Mat(N_SCAN, HORIZON_SCAN, CV_32S, cv::Scalar::all(0));
        featureTypeMat_ = cv::Mat(N_SCAN, HORIZON_SCAN, CV_32S, cv::Scalar::all(0));

        fullCloud_.reset(new PointCloudT());
        fullInfoCloud_.reset(new PointCloudT());
        groundCloud_.reset(new PointCloudT());
        outlierCloud_.reset(new PointCloudT());
        segmentedCloud_.reset(new PointCloudT());

        fullCloud_->points.resize(N_SCAN * HORIZON_SCAN);
        fullInfoCloud_->points.resize(N_SCAN * HORIZON_SCAN);
        PointT nanPoint;
        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;
        std::fill(fullCloud_->points.begin(), fullCloud_->points.end(), nanPoint);
        std::fill(fullInfoCloud_->points.begin(), fullInfoCloud_->points.end(), nanPoint);

        segCloudInfo_.startRingInd_.assign(N_SCAN, 0);
        segCloudInfo_.endRingInd_.assign(N_SCAN, 0);
        segCloudInfo_.segmentedCloudGroundFlag_.assign(N_SCAN * HORIZON_SCAN, false);
        segCloudInfo_.segmentedCloudColInd_.assign(N_SCAN * HORIZON_SCAN, false);
        segCloudInfo_.segmentedCloudRange_.assign(N_SCAN * HORIZON_SCAN, false);

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1;
        neighbor.second = 0;
        neighborIterator_.push_back(neighbor);
        neighbor.first = 0;
        neighbor.second = 1;
        neighborIterator_.push_back(neighbor);
        neighbor.first = 0;
        neighbor.second = -1;
        neighborIterator_.push_back(neighbor);
        neighbor.first = 1;
        neighbor.second = 0;
        neighborIterator_.push_back(neighbor);
        allPushedIndX_ = new uint16_t[N_SCAN * HORIZON_SCAN];
        allPushedIndY_ = new uint16_t[N_SCAN * HORIZON_SCAN];
        queueIndX_ = new uint16_t[N_SCAN * HORIZON_SCAN];
        queueIndY_ = new uint16_t[N_SCAN * HORIZON_SCAN];

        labelCount_ = 1;

        cloudSmoothness_.resize(N_SCAN * HORIZON_SCAN);
        cloudCurvature_.assign(N_SCAN * HORIZON_SCAN, 0.0);
        cloudNeighborPicked_.assign(N_SCAN * HORIZON_SCAN, 0);
        cloudLabel_.assign(N_SCAN * HORIZON_SCAN, 0);

        cornerPointsSharp_.reset(new PointCloudT());
        cornerPointsLessSharp_.reset(new PointCloudT());
        surfPointsFlat_.reset(new PointCloudT());
        surfPointsLessFlat_.reset(new PointCloudT());
        surfPointsLessFlatScan_.reset(new PointCloudT());
        surfPointsLessFlatScanDS_.reset(new PointCloudT());
        planeCloud_.reset(new PointCloudT());

        downSizeFilter_.setLeafSize(0.2, 0.2, 0.2);
    }

    void LidarFeature::RangeImageProcess()
    {

        FindStartEndAngle();

        ProjectPointCloud();

        GroundPointsExtraction();

        CloudSegmentation();
    }

    void LidarFeature::FindStartEndAngle()
    {

        segCloudInfo_.startOrientation_ = (-81.7/2)*M_PI/180;
        segCloudInfo_.endOrientation_ = (81.7/2)*M_PI/180;
        segCloudInfo_.orientationDiff_ = segCloudInfo_.endOrientation_ - segCloudInfo_.startOrientation_;

    }

    void LidarFeature::ProjectPointCloud()
    {
        // range image projection
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize;
        PointT thisPoint;

        cloudSize = cloud_.points.size();
        int size_useful = 0;
        for (size_t i = 0; i < cloudSize; ++i)
        {

            thisPoint.x = cloud_.points[i].x;
            thisPoint.y = cloud_.points[i].y;
            thisPoint.z = cloud_.points[i].z;
            if(thisPoint.x == 0 && thisPoint.y == 0 && thisPoint.z == 0)
                continue;
//            std::cout << thisPoint.x << " " << thisPoint.y << " " << thisPoint.z << " ";

            // find the row and column index in the image for this point
            verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            rowIdn = (verticalAngle + ANGLE_BOTTOM) / ANGLE_RES_Y;

//            std::cout << verticalAngle << " " <<  rowIdn << " ";

            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            horizonAngle = atan2(thisPoint.y, thisPoint.x) * 180 / M_PI;

            columnIdn = round(horizonAngle / ANGLE_RES_X) + HORIZON_SCAN / 2;
            if (columnIdn < 0 || columnIdn >= HORIZON_SCAN)
                continue;

            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);

//            std::cout << horizonAngle << " " <<  columnIdn << " " << range << std::endl;

            if(range >= rangeMat_.at<float>(rowIdn,columnIdn)){
                continue;
            }

            rangeMat_.at<float>(rowIdn, columnIdn) = range;
            if (range < 500)
                size_useful++;

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0; // store rowIdn for feature association and (columnIdn / 10000.0) just for indicating different column

            index = columnIdn + rowIdn * HORIZON_SCAN;
            fullCloud_->points[index] = thisPoint;
            fullInfoCloud_->points[index] = thisPoint;
            fullInfoCloud_->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
        }

        ROS_DEBUG_STREAM("useful range point: " << size_useful << "; full size: " << cloudSize);
    }

    void LidarFeature::GroundPointsExtraction()
    {

        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // groundMat
        // -1, no valid info to check if ground of not
        //  0, initial value, after validation, means not ground
        //  1, ground
        for (size_t j = 0; j < HORIZON_SCAN; ++j)
        {
            for (size_t i = 0; i < GROUND_SCAN_IND; ++i)
            {

                lowerInd = j + (i)*HORIZON_SCAN;
                upperInd = j + (i + 1) * HORIZON_SCAN;

                if (fullCloud_->points[lowerInd].intensity == -1 ||
                    fullCloud_->points[upperInd].intensity == -1)
                {
                    // no info to check, invalid points
                    groundMat_.at<int8_t>(i, j) = -1;
                    continue;
                }

                diffX = fullCloud_->points[upperInd].x - fullCloud_->points[lowerInd].x;
                diffY = fullCloud_->points[upperInd].y - fullCloud_->points[lowerInd].y;
                diffZ = fullCloud_->points[upperInd].z - fullCloud_->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;

                if (abs(angle - SENSOR_MOUNT_ANGLE) <= GROUND_ANGLE_TH)
                {
                    groundMat_.at<int8_t>(i, j) = 1;
                    groundMat_.at<int8_t>(i + 1, j) = 1;
                }
            }
        }

        // refine ground mat by roughness sequence
        for (int i = 0; i <= GROUND_SCAN_IND; ++i) {

            std::vector<float> roughness(HORIZON_SCAN, 1);
            int S = 5;
            for (int j = S; j < HORIZON_SCAN-S; ++j) {

                if(rangeMat_.at<float>(i, j) == FLT_MAX)
                    continue;

                float r_j = groundMat_.at<float>(i, j);

                float sum_d = 0;
                int count = 0;

                for (int k = j-S; k < j+S; ++k) {
                    if(rangeMat_.at<float>(i, k) >= 100)
                        continue;
                    count++;
                    sum_d += rangeMat_.at<float>(i, k) - r_j;
                }

                roughness.at(j) = std::abs(sum_d)/(2.0*count*r_j);
            }
            S = 1;
            for (int l = S; l < HORIZON_SCAN-S; ++l) {
                float sum_c = 0, count = 0;
                for (int j = l-S; j < l+S; ++j) {
                    if(rangeMat_.at<float>(i, j) == FLT_MAX)
                        continue;
                    count++;
                    sum_c += roughness.at(j);
                }

                if(sum_c/float(count) > 0.01|| rangeMat_.at<float>(i, l) >= 20)
                    for (int j = l-S; j < l+S; ++j) {
                        groundMat_.at<int8_t>(i, j) = 0;
                    }
            }
        }

        // extract ground cloud (groundMat == 1)
        // mark entry that doesn't need to label (ground and invalid point) for segmentation
        // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
        for (size_t i = 0; i < N_SCAN; ++i)
        {
            for (size_t j = 0; j < HORIZON_SCAN; ++j)
            {

                if (groundMat_.at<int8_t>(i, j) == 1 || rangeMat_.at<float>(i, j) == FLT_MAX)
                {
                    // does not need to be labelled in the segmentation step
                    // for ground points or invalid points.
                    labelMat_.at<int>(i, j) = -1;
                }
            }
        }

        for (size_t i = 0; i <= GROUND_SCAN_IND; ++i)
        {
            for (size_t j = 0; j < HORIZON_SCAN; ++j)
            {
                if (groundMat_.at<int8_t>(i, j) == 1)
                {
                    if (j % GROUND_FEATURE_RESOLUTION != 0 && j > GROUND_FEATURE_RESOLUTION && j < HORIZON_SCAN - GROUND_FEATURE_RESOLUTION) //remove all the ground cloud
                        continue;
                    groundCloud_->push_back(fullCloud_->points[j + i * HORIZON_SCAN]);
                }
            }
        }
    }

    void LidarFeature::CloudSegmentation()
    {

        // segmentation process
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < HORIZON_SCAN; ++j)
                if (labelMat_.at<int>(i, j) == 0)
                    labelComponents(i, j);

        int sizeOfSegCloud = 0;

//        ROS_DEBUG("Segmentation has %d clusters", ++labelCount_);
        // extract segmented cloud for lidar odometry
        for (size_t i = 0; i < N_SCAN; ++i)
        {

            segCloudInfo_.startRingInd_[i] = sizeOfSegCloud - 1 + 5;
            // ROS_DEBUG("************ 1343 ***********");
            for (size_t j = 0; j < HORIZON_SCAN; ++j)
            {

                if (rangeMat_.at<float>(i, j) > 500)
                    continue;

                if (labelMat_.at<int>(i, j) > 0 )
                {
                    // outliers that will not be used for optimization (always continue)
                    if (labelMat_.at<int>(i, j) == 999999)
                    {
                        if (i > GROUND_SCAN_IND && j % 5 == 0)
                        {
                            outlierCloud_->push_back(fullCloud_->points[j + i * HORIZON_SCAN]);
                            continue;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    // majority of ground points are skipped for decrease the feature cloud size (choose 1 from 50)
                    if (groundMat_.at<int8_t>(i, j) == 1)
                    {
//                        if (j % GROUND_FEATURE_RESOLUTION != 0 && j > GROUND_FEATURE_RESOLUTION && j < HORIZON_SCAN - GROUND_FEATURE_RESOLUTION)
                        continue;
                    }
                    // ROS_DEBUG("************ 1344 ***********");
                    // mark ground points so they will not be considered as edge features later
                    segCloudInfo_.segmentedCloudGroundFlag_[sizeOfSegCloud] = (groundMat_.at<int8_t>(i, j) == 1);
                    // mark the points' column index for marking occlusion later
                    segCloudInfo_.segmentedCloudColInd_[sizeOfSegCloud] = j;
                    // save range inf
                    segCloudInfo_.segmentedCloudRange_[sizeOfSegCloud] = rangeMat_.at<float>(i, j);
                    // save seg cloud
                    segmentedCloud_->push_back(fullCloud_->points[j + i * HORIZON_SCAN]);
                    // save seg point idx in mat
                    int idx = i * 10000 + j;
                    segmentedCloudIdx_.push_back(idx);
                    // ROS_DEBUG("************ 1345 ***********");
                    // size of seg cloud
                    ++sizeOfSegCloud;
                }
            }

            segCloudInfo_.endRingInd_[i] = sizeOfSegCloud-1-5;
        }
    }

    void LidarFeature::labelComponents(int row, int col)
    {
        // use std::queue std::vector std::deque will slow the program down greatly
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY;
        bool lineCountFlag[N_SCAN] = {false};

        queueIndX_[0] = row;
        queueIndY_[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX_[0] = row;
        allPushedIndY_[0] = col;
        int allPushedIndSize = 1;

        while (queueSize > 0)
        {
            // Pop point
            fromIndX = queueIndX_[queueStartInd];
            fromIndY = queueIndY_[queueStartInd];
            --queueSize;
            ++queueStartInd;
            // Mark popped point
            labelMat_.at<int>(fromIndX, fromIndY) = labelCount_;
            // Loop through all the neighboring grids of popped grid
            for (auto iter = neighborIterator_.begin(); iter != neighborIterator_.end(); ++iter)
            {
                // new index
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;
                // index should be within the boundary
                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;
                // at range image margin (left or right side)
                if (thisIndY < 0)
                    thisIndY = HORIZON_SCAN - 1;
                if (thisIndY >= HORIZON_SCAN)
                    thisIndY = 0;
                // prevent infinite loop (caused by put already examined point back)
                if (labelMat_.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                if(rangeMat_.at<float>(fromIndX, fromIndY) > 500 || rangeMat_.at<float>(thisIndX, thisIndY) > 500)
                    continue;

                d1 = std::max(rangeMat_.at<float>(fromIndX, fromIndY),
                              rangeMat_.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat_.at<float>(fromIndX, fromIndY),
                              rangeMat_.at<float>(thisIndX, thisIndY));

                if ((*iter).first == 0)
                    alpha = SEGMENT_ALPHA_X;
                else
                    alpha = SEGMENT_ALPHA_X;

                angle = atan2(d2 * sin(alpha), (d1 - d2 * cos(alpha)));

                if (angle > SEGMENT_THETA)
                {

                    queueIndX_[queueEndInd] = thisIndX;
                    queueIndY_[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat_.at<int>(thisIndX, thisIndY) = labelCount_;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX_[allPushedIndSize] = thisIndX;
                    allPushedIndY_[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }

        // check if this segment is valid
        bool feasibleSegment = false;
        if (allPushedIndSize >= SEGMENT_VALID_SIZE)
            feasibleSegment = true;
        else if (allPushedIndSize >= SEGMENT_VALID_POINT_NUM)
        {
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= SEGMENT_VALID_LINE_NUM)
                feasibleSegment = true;
        }
        // segment is valid, mark these points
        if (feasibleSegment == true)
        {
            ++labelCount_;
        }
        else
        { // segment is invalid, mark these points
            for (size_t i = 0; i < allPushedIndSize; ++i)
            {
                labelMat_.at<int>(allPushedIndX_[i], allPushedIndY_[i]) = 999999;
            }
        }
    }

    void LidarFeature::FeatureExtractionProcess()
    {

//        AssignTimestampToPoints();

        CalculateSmoothness();

//        MarkOccludedPoints();

        ExtractFeatures();

        ExtractPlane();

        ConstructCloudFeature();
    }

    void LidarFeature::AssignTimestampToPoints()
    {

        bool halfPassed = false;
        int cloudSize = segmentedCloud_->points.size();

        PointT point;

        for (int i = 0; i < cloudSize; ++i)
        {

            point = segmentedCloud_->points[i];

            float ori = -atan2(point.y, point.x);

            if (!halfPassed)
            {
                if (ori < segCloudInfo_.startOrientation_ - M_PI / 2)
                    ori += 2 * M_PI;
                else if (ori > segCloudInfo_.startOrientation_ + M_PI * 3 / 2)
                    ori -= 2 * M_PI;

                if (ori - segCloudInfo_.startOrientation_ > M_PI)
                    halfPassed = true;
            }
            else
            {
                ori += 2 * M_PI;

                if (ori < segCloudInfo_.endOrientation_ - M_PI * 3 / 2)
                    ori += 2 * M_PI;
                else if (ori > segCloudInfo_.endOrientation_ + M_PI / 2)
                    ori -= 2 * M_PI;
            }

            // ROS_DEBUG_STREAM(segCloudInfo_.startOrientation_ *180/M_PI << " " << ori*180/M_PI << " " <<  segCloudInfo_.endOrientation_*180/M_PI);
            float oriRatio = (ori - segCloudInfo_.startOrientation_) / segCloudInfo_.orientationDiff_;
            point.intensity = int(segmentedCloud_->points[i].intensity) + SCAN_DURATION * oriRatio;

            segmentedCloud_->points[i] = point;
        }
    }

    void LidarFeature::CalculateSmoothness()
    {

        int cloudSize = segmentedCloud_->points.size();

        constexpr float kDistanceFaraway = 25;

        for (int i = 5; i < cloudSize - 5; i++) {
            float dis = segCloudInfo_.segmentedCloudRange_[i];
            int kNumCurvSize = 5;
            if (dis > kDistanceFaraway) {
                kNumCurvSize = 2;
            }
            float diffX = 0, diffY = 0, diffZ = 0;
            for (int j = 1; j <= kNumCurvSize; ++j) {
                diffX += segmentedCloud_->points[i - j].x + segmentedCloud_->points[i + j].x;
                diffY += segmentedCloud_->points[i - j].y + segmentedCloud_->points[i + j].y;
                diffZ += segmentedCloud_->points[i - j].z + segmentedCloud_->points[i + j].z;
            }
            diffX -= 2 * kNumCurvSize * segmentedCloud_->points[i].x;
            diffY -= 2 * kNumCurvSize * segmentedCloud_->points[i].y;
            diffZ -= 2 * kNumCurvSize * segmentedCloud_->points[i].z;

            float tmp2 = diffX * diffX + diffY * diffY + diffZ * diffZ;
            float tmp = sqrt(tmp2);

            /// use normalized curvature
            cloudCurvature_[i] = tmp / (2 * kNumCurvSize * dis + 1e-3);

//            std::cout << cloudCurvature_[i] << " ";

            cloudNeighborPicked_[i] = 0;
            cloudLabel_[i] = 0;

            cloudSmoothness_[i].value = cloudCurvature_[i];
            cloudSmoothness_[i].ind = i;

//            constexpr float kMaxFeatureDis = 1e4;
//            if (fabs(dis) > kMaxFeatureDis || fabs(dis) < 1e-4 || !std::isfinite(dis)) {
//                cloudLabel_[i] = 99;
//                cloudNeighborPicked_[i] = 1;
//            }

        }
//        std::cout << std::endl;
    }

    void LidarFeature::MarkOccludedPoints()
    {

        int cloudSize = segmentedCloud_->points.size();

        for (int i = 5; i < cloudSize - 6; i++) {
            float diffX = segmentedCloud_->points[i + 1].x - segmentedCloud_->points[i].x;
            float diffY = segmentedCloud_->points[i + 1].y - segmentedCloud_->points[i].y;
            float diffZ = segmentedCloud_->points[i + 1].z - segmentedCloud_->points[i].z;
            float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

            float diffX2 = segmentedCloud_->points[i].x - segmentedCloud_->points[i - 1].x;
            float diffY2 = segmentedCloud_->points[i].y - segmentedCloud_->points[i - 1].y;
            float diffZ2 = segmentedCloud_->points[i].z - segmentedCloud_->points[i - 1].z;
            float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;
            float dis = segmentedCloud_->points[i].x * segmentedCloud_->points[i].x +
                        segmentedCloud_->points[i].y * segmentedCloud_->points[i].y +
                        segmentedCloud_->points[i].z * segmentedCloud_->points[i].z;

            if (diff > 0.00015 * dis && diff2 > 0.00015 * dis) {
                cloudNeighborPicked_[i] = 1;
            }
        }

    }

    void LidarFeature::ExtractFeatures()
    {

        int kNumCurvSize = 5;
        int kNumRegion = 6;
        for (int i = 0; i < N_SCAN; i++)
        {

            surfPointsLessFlatScan_->clear();
//            if ( segCloudInfo_.endRingInd_[i] -  segCloudInfo_.startRingInd_[i] < kNumCurvSize) continue;
//            std::cout << "1 ";
            for (int j = 0; j < kNumRegion; j++) {
                int sp = (segCloudInfo_.startRingInd_[i] * (6 - j) + segCloudInfo_.endRingInd_[i] * j) / 6;
                int ep = (segCloudInfo_.startRingInd_[i] * (5 - j) + segCloudInfo_.endRingInd_[i] * (j + 1)) / 6 - 1;

//                std::cout << sp << "-" << ep << "// ";

                if (sp >= ep)
                    continue;

                std::sort(cloudSmoothness_.begin() + sp, cloudSmoothness_.begin() + ep, by_value());

                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSmoothness_[k].ind;
                    if (cloudNeighborPicked_[ind] == 0 &&
                        cloudCurvature_[ind] > EDGE_FEATURE_TH)
                    {

                        int idx = segmentedCloudIdx_.at(ind);
                        int row = idx / 10000;
                        int col = idx % 10000;

                        largestPickedNum++;
                        if (largestPickedNum <= 2)
                        {
                            cloudLabel_[ind] = 2;
                            cornerPointsSharp_->push_back(segmentedCloud_->points[ind]);
                            cornerPointsLessSharp_->push_back(segmentedCloud_->points[ind]);
                            featureTypeMat_.at<int>(row, col) = -1;
                        }
                        else if (largestPickedNum <= 20)
                        {
                            cloudLabel_[ind] = 1;
                            cornerPointsLessSharp_->push_back(segmentedCloud_->points[ind]);
                            featureTypeMat_.at<int>(row, col) = -1;
                        }
                        else
                        {
                            break;
                        }

                        cloudNeighborPicked_[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            float diffX = segmentedCloud_->points[ind + l].x -
                                          segmentedCloud_->points[ind + l - 1].x;
                            float diffY = segmentedCloud_->points[ind + l].y -
                                          segmentedCloud_->points[ind + l - 1].y;
                            float diffZ = segmentedCloud_->points[ind + l].z -
                                          segmentedCloud_->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.01) {
                                break;
                            }
                            cloudNeighborPicked_[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = segmentedCloud_->points[ind + l].x -
                                          segmentedCloud_->points[ind + l - 1].x;
                            float diffY = segmentedCloud_->points[ind + l].y -
                                          segmentedCloud_->points[ind + l - 1].y;
                            float diffZ = segmentedCloud_->points[ind + l].z -
                                          segmentedCloud_->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.01) {
                                break;
                            }
                            cloudNeighborPicked_[ind + l] = 1;
                        }
                    }
                }

                // find surf features
                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSmoothness_[k].ind;
                    if (cloudNeighborPicked_[ind] == 0 &&
                        cloudCurvature_[ind] <= SURF_FEATURE_TH)
                    {

                        int idx = segmentedCloudIdx_.at(ind);
                        int row = idx / 10000;
                        int col = idx % 10000;

                        cloudLabel_[ind] = -1;
                        surfPointsFlat_->push_back(segmentedCloud_->points[ind]);
                        featureTypeMat_.at<int>(row, col) = 1;

                        smallestPickedNum++;
                        if (smallestPickedNum >= 4)
                        {
                            break;
                        }

                        cloudNeighborPicked_[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            float diffX = segmentedCloud_->points[ind + l].x -
                                          segmentedCloud_->points[ind + l - 1].x;
                            float diffY = segmentedCloud_->points[ind + l].y -
                                          segmentedCloud_->points[ind + l - 1].y;
                            float diffZ = segmentedCloud_->points[ind + l].z -
                                          segmentedCloud_->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.02) {
                                break;
                            }
                            cloudNeighborPicked_[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = segmentedCloud_->points[ind + l].x -
                                          segmentedCloud_->points[ind + l - 1].x;
                            float diffY = segmentedCloud_->points[ind + l].y -
                                          segmentedCloud_->points[ind + l - 1].y;
                            float diffZ = segmentedCloud_->points[ind + l].z -
                                          segmentedCloud_->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.02) {
                                break;
                            }
                            cloudNeighborPicked_[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel_[k] <= 0 && cloudCurvature_[k] < SURF_FEATURE_TH)
                    {
                        surfPointsLessFlatScan_->push_back(segmentedCloud_->points[k]);
                        int idx = segmentedCloudIdx_.at(k);
                        int row = idx / 10000;
                        int col = idx % 10000;
                        featureTypeMat_.at<int>(row, col) = 1;
                    }
                }
            }
//            std::cout << "2 ";
            surfPointsLessFlatScanDS_->clear();
            downSizeFilter_.setInputCloud(surfPointsLessFlatScan_);
            downSizeFilter_.filter(*surfPointsLessFlatScanDS_);

            *surfPointsLessFlat_ += *surfPointsLessFlatScanDS_;

        }
//        std::cout << "3 ";
        int cloudSize = segmentedCloud_->points.size();
        for (int l = 0; l < cloudSize; ++l) {

            if (cloudNeighborPicked_[l] == 0) {

                int idx = segmentedCloudIdx_.at(l);
                int row = idx / 10000;
                int col = idx % 10000;

                if (cloudCurvature_[l] <= SURF_FEATURE_TH) {

                    cloudLabel_[l] = -1;
                    surfPointsLessFlat_->push_back(segmentedCloud_->points[l]);
                    featureTypeMat_.at<int>(row, col) = 1;
                    cloudNeighborPicked_[l] = 1;

                } else if (cloudCurvature_[l] > 10 * EDGE_FEATURE_TH){
                    cloudLabel_[l] = 1;
                    cornerPointsLessSharp_->push_back(segmentedCloud_->points[l]);
                    featureTypeMat_.at<int>(row, col) = -1;
                    cloudNeighborPicked_[l] = 1;
                }

            }
        }
//        std::cout << "4 "<< std::endl;
    }

    void LidarFeature::ExtractPlane() {

        // step 1: using convolution-like operation to find the more concentrated part of the surf features;
        // step 2: extract the plane segments from the high value;
        // step 3: clustering the overlapping plane segments using the window distance;
        // step 4: check the range distribution and remove the clusters from the same scan;
        // step 5: plane fitting, treat the eigen vector corresponding to the smallest eigen value as normal vector;
        // step 6: treat eigen solves as initial guess, estabilish the closest point based optimization problem
        //          and compute the related covariance matrix;

        TicToc time;

        int window_rows = PLANE_WINDOW_ROWS;
        int window_cols = PLANE_WINDOW_COLS;
        int threshold = std::ceil(PLANE_WINDOW_ROWS * PLANE_WINDOW_COLS * PLANE_WINDOW_RATE);

        // do 2d filter in the windows to find the seed plane
        cv::Mat filter_Mat = cv::Mat(N_SCAN, HORIZON_SCAN, CV_32S, cv::Scalar::all(0));
        {
            for (int i = 0; i < N_SCAN; i = i + 5) {
                for (int j = 0; j < HORIZON_SCAN; j = j + 10) {

                    int row_begin = (i - window_rows / 2 <= 0) ? 0 : (i - window_rows / 2);
                    int row_end = (i + window_rows / 2 >= (N_SCAN - 1)) ? (N_SCAN - 1) : (i + window_rows / 2);
                    int col_begin = (j - window_cols / 2 <= 0) ? 0 : (j - window_cols / 2);
                    int col_end = (j + window_cols / 2 >= (HORIZON_SCAN - 1)) ? (HORIZON_SCAN - 1) : (j + window_cols / 2);

                    int sum = 0;
                    int empty_size = 0;
                    for (int k = row_begin; k < row_end; ++k) {
                        for (int l = col_begin; l < col_end; ++l) {
                            if(featureTypeMat_.at<int>(k,l) == 1)
                                sum += featureTypeMat_.at<int>(k,l);
                            if(rangeMat_.at<float>(k,l) > 500)
                                empty_size++;
                        }
                    }
                    if(sum >= (threshold-empty_size))
                        filter_Mat.at<int>(i,j) = sum;
                }
            }

        }

        if (SAVE_FEATURE_TMP_RESULT) {

            std::string filter_Mat_save_path = OUTPUT_PATH + "filter_Mat.txt";
            std::ofstream fout1(filter_Mat_save_path.c_str());
            for (int i = 0; i < N_SCAN; ++i) {
                for (int j = 0; j < HORIZON_SCAN; ++j) {
                    fout1 << filter_Mat.at<int>(i, j) << "\t";
                }
                fout1 << std::endl;
            }
            fout1.close();
        }

        // find plane segment
        std::vector<std::pair<int, int>> candidate_idx;
        std::vector<float> range_mean;
        {
            for (int i = 0; i < filter_Mat.rows; ++i) {
                for (int j = 0; j < filter_Mat.cols; ++j) {

                    if (filter_Mat.at<int>(i,j) >= 0){
                        candidate_idx.emplace_back(i,j);

                        int i_begin = (i - window_rows/2 <= 0) ? 0 : (i - window_rows/2);
                        int i_end = (i + window_rows/2 >= (N_SCAN-1)) ? (N_SCAN-1) : (i + window_rows/2);
                        int j_begin = (i - window_cols/2 <= 0) ? 0 : (i - window_cols/2);
                        int j_end = (i + window_cols/2 >= (HORIZON_SCAN-1)) ? (HORIZON_SCAN-1) : (i + window_cols/2);

                        float sum = 0.0;
                        float count = 0.0;
                        for (int k = i_begin; k < i_end; ++k) {
                            for (int l = j_begin; l < j_end; ++l) {
                                if(rangeMat_.at<float>(k,l) >= 500)
                                    continue;
                                count++;
                                sum += rangeMat_.at<float>(k,l);
                            }
                        }
                        range_mean.emplace_back(sum/count);
                    }
                }
            }
        }

        // clustering the plane segment, especially for the overlapping segmentation
        std::vector<std::pair<int, int>> candidate_idx_copy = candidate_idx;
        std::vector<int> judged_pair;
        judged_pair.resize(candidate_idx_copy.size(), 0);
        int cluster_idx = 1;
        {
            for (int i = 0; i < candidate_idx_copy.size(); ++i) {

                if(judged_pair.at(i) != 0) continue;

                std::pair<int, int> choosed_idx = candidate_idx_copy.at(i);
                judged_pair.at(i) = cluster_idx;

                for (int j = 0; j < candidate_idx_copy.size(); ++j) {
                    if(i==j) continue;
                    if (abs(candidate_idx_copy.at(j).second - choosed_idx.second) < 2 * window_cols){// &&
                        //abs(range_mean.at(j) - range_mean.at(i)) < PLANE_CLUSTER_DISTANCE){
                        judged_pair.at(j) = cluster_idx;
                    }
                }
                cluster_idx++;
            }
        }

//        ROS_DEBUG_STREAM("candidate_idx_copy: " << candidate_idx_copy.size() << "; cluster size: " << cluster_idx-1);

        // group points in every cluster
//        ROS_DEBUG("group points in every cluster");
        cv::Mat judge_Mat = cv::Mat(N_SCAN, HORIZON_SCAN, CV_8S, cv::Scalar::all(0));
        std::vector<std::vector<Eigen::Vector2i>> clusters_plane_points_idx;
        clusters_plane_points_idx.resize(cluster_idx - 1);
        {
            for (int i = 1; i < cluster_idx-1; ++i) {
                PointCloudT cloud;

                for (int j = 0; j < judged_pair.size(); ++j) {
                    if (judged_pair.at(j) != i) continue;

                    int index_row = candidate_idx_copy.at(j).first;
                    int index_col = candidate_idx_copy.at(j).second;

                    int i_begin = ((index_row - window_rows/2)<= 0) ? 0 : (index_row - window_rows/2);
                    int i_end = ((index_row + window_rows/2) >= (N_SCAN-1)) ? (N_SCAN-1) : (index_row + window_rows/2);
                    int j_begin = ((index_col - window_cols/2) <= 0) ? 0 : (index_col - window_cols/2);
                    int j_end = ((index_col + window_cols/2) >= (HORIZON_SCAN-1)) ? (HORIZON_SCAN-1) : (index_col + window_cols/2);


                    for (int k = i_begin; k < i_end; ++k) {
                        for (int l = j_begin; l < j_end; ++l) {
                            if(judge_Mat.at<int8_t>(k,l) == 1 || featureTypeMat_.at<int>(k,l) != 1) continue;

                            PointT point = fullCloud_->points.at(l+k*HORIZON_SCAN);
                            cloud.points.emplace_back(point);
                            judge_Mat.at<int8_t>(k,l) = 1;
                            Eigen::Vector2i found_index = Eigen::Vector2i(k, l);
                            clusters_plane_points_idx.at(i - 1).push_back(found_index);

                        }
                    }

                }
//                if(SAVE_FEATURE_TMP_RESULT){
//                    if(cloud.size()>=PLANE_POINT_SIZE){
//                        std::string cloud_save_file = OUTPUT_PATH + "cluster_points_"+std::to_string(i)+".pcd";
//                        pcl::io::savePCDFileBinary(cloud_save_file, cloud);
//                    }
//                }
                cloud.clear();
            }
        }

//        ROS_DEBUG("check the range distribution");
        // check the range distribution // not accurate
        std::vector<std::vector<Eigen::Vector2i>> re_clustered_cloud;
        re_clustered_cloud.reserve(clusters_plane_points_idx.size()*2);
        {
            for (auto & i : clusters_plane_points_idx) {

                if (i.size() < PLANE_POINT_SIZE) continue;

                std::vector<float> range_vector;
                range_vector.resize(i.size());
                for (int j = 0; j < i.size(); ++j) {
                    range_vector.at(j) = rangeMat_.at<float>(i.at(j)[0], i.at(j)[1]);
                }

                std::vector<bool> judged;
                judged.assign(range_vector.size(), false);

                int judged_count = range_vector.size();

                while (judged_count > 0) {

                    auto ite = std::find(judged.begin(), judged.end(), false);
                    if (ite == judged.end()) break;
                    auto index = std::distance(std::begin(judged), ite);
                    float seed_range = range_vector.at(index);
                    judged.at(index) = true;
                    judged_count--;

                    float mean = seed_range;
                    int count = 1;

                    std::vector<Eigen::Vector2i> pair;
                    pair.push_back(i.at(index));

                    for (int j = 0; j < range_vector.size(); ++j) {
                        if (judged.at(j)) continue;

                        if(std::abs(seed_range - range_vector.at(j)) <= PLANE_FITTING_DISTANCE_THRESHOLD){
                            mean = (mean * count + range_vector.at(j))/float(count + 1);
                            pair.push_back(i.at(j));
                            judged.at(j) = true;
                            judged_count--;
                            count++;
                        }
                    }

                    if(pair.size() >= PLANE_POINT_SIZE)
                        re_clustered_cloud.push_back(pair);

                }
            }

//            if(SAVE_FEATURE_TMP_RESULT){
//                PointCloudT cloud;
//                for (int i = 0; i < re_clustered_cloud.size(); ++i) {
//
//                    for (int j = 0; j < re_clustered_cloud.at(i).size(); ++j) {
//                        Eigen::Vector2i index = re_clustered_cloud.at(i).at(j);
//                        cloud.push_back(fullCloud_->points.at(index[1] + index[0] * HORIZON_SCAN));
//                    }
//                }
//                if (!cloud.empty()){
//                    std::string cloud_save_file = OUTPUT_PATH + "re_cluster_points"+".pcd";
//                    pcl::io::savePCDFileBinary(cloud_save_file, cloud);
//                }
//            }
        }

        // remove clusters whose points lie on the same scan.
        std::vector<std::vector<Eigen::Vector2i>> clustered_cloud;
        clustered_cloud.reserve(re_clustered_cloud.size());
        {
            for (auto & i : re_clustered_cloud) {
                int scan_id = i.at(0)[0];
                int j;
                for (j = 1; j < i.size(); ++j) {
                    if (abs(i.at(j)[0] - scan_id) <= 1) continue;
                    else break;
                }
                if(j != i.size())
                    clustered_cloud.emplace_back(i);
            }

//            if(SAVE_FEATURE_TMP_RESULT){
//
//                PointCloudT cloud;
//                for (int i = 0; i < clustered_cloud.size(); ++i) {
//
//                    for (int j = 0; j < clustered_cloud.at(i).size(); ++j) {
//                        Eigen::Vector2i index = clustered_cloud.at(i).at(j);
//                        cloud.push_back(fullCloud_->points.at(index[1] + index[0] * HORIZON_SCAN));
//                    }
//                }
//                if (!cloud.empty()){
//                    std::string cloud_save_file = OUTPUT_PATH + "final_clustered_cloud"+".pcd";
//                    pcl::io::savePCDFileBinary(cloud_save_file, cloud);
//                }
//            }
        }

        // set feature type of plane cloud as zero, and reconstruct the less edge feature cloud ---- not good
//        if(USE_SURF_FACTOR){
//
//            for (int i = 0; i < clustered_cloud.size(); ++i) {
//                for (int j = 0; j < clustered_cloud.at(i).size(); ++j) {
//                    Eigen::Vector2i index = clustered_cloud.at(i).at(j);
//                    featureTypeMat_.at<int>(index[0],index[1]) = 2;
//                }
//            }
//
//            surfPointsLessFlatScan_ -> points.clear();
//            for (int k = 0; k < N_SCAN; ++k) {
//                for (int l = 0; l < HORIZON_SCAN; ++l) {
//                    if(featureTypeMat_.at<int>(k, l) == 1)
//                        surfPointsLessFlatScan_->points.push_back(fullCloud_->points.at(l + k * HORIZON_SCAN));
//                }
//            }
//
//            surfPointsLessFlatScanDS_->clear();
//            downSizeFilter_.setInputCloud(surfPointsLessFlatScan_);
//            downSizeFilter_.filter(*surfPointsLessFlatScanDS_);
//
//            *surfPointsLessFlat_ += *surfPointsLessFlatScanDS_;
//
//        }


        // plane fitting using the eigen vector method, and use the CP-based optimization method to compute the 3x3 covariance
        std::vector<PlaneFeature> plane_feature_v;
        {
            for (int i = 0; i < clustered_cloud.size(); ++i) {

                float radius = 0;

                PointCloudT cloud;
                float sum_x = 0, sum_y = 0, sum_z = 0;
                for (int j = 0; j < clustered_cloud.at(i).size(); ++j) {
                    Eigen::Vector2i index = clustered_cloud.at(i).at(j);
                    PointT point = fullCloud_->points.at(index[1] + index[0] * HORIZON_SCAN);
                    cloud.push_back(point);
                    sum_x += point.x;
                    sum_y += point.y;
                    sum_z += point.z;
                }
                float center_x = sum_x/float(clustered_cloud.at(i).size());
                float center_y = sum_y/float(clustered_cloud.at(i).size());
                float center_z = sum_z/float(clustered_cloud.at(i).size());

                float C_00 = 0;
                float C_01 = 0;
                float C_02 = 0;
                float C_11 = 0;
                float C_12 = 0;
                float C_22 = 0;
                for (auto & point : cloud.points) {
                    C_00 += (point.x - center_x) * (point.x - center_x);
                    C_01 += (point.x - center_x) * (point.y - center_y);
                    C_02 += (point.x - center_x) * (point.z - center_z);
                    C_11 += (point.y - center_y) * (point.y - center_y);
                    C_12 += (point.y - center_y) * (point.z - center_z);
                    C_22 += (point.z - center_z) * (point.z - center_z);
                }

                float C_10 = C_01;
                float C_20 = C_02;
                float C_21 = C_12;

                Eigen::Matrix3f C;
                C <<    C_00, C_01, C_02,
                        C_10, C_11, C_12,
                        C_20, C_21, C_22;

                auto svd = C.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
                Eigen::Vector3f  plane_normal = svd.matrixU().rightCols<1>();
                float d = std::abs(plane_normal.dot(Eigen::Vector3f(center_x, center_y, center_z))); // plane model: n.dot(center) = d

                // check MSE
                float mse = 0;
                for (int k = 0; k < cloud.size(); ++k) {
                    PointT point  = cloud.at(k);
                    mse += std::pow(plane_normal.dot(Eigen::Vector3f(point.x, point.y, point.z)) - d, 2);

                    float p_c_d = Eigen::Vector3f(point.x - center_x, point.y - center_y, point.z - center_z).norm();
                    if(p_c_d >= radius)
                        radius = p_c_d;
                }
                mse = mse/float(cloud.size());
//                ROS_DEBUG_STREAM("plane: " << plane_normal.transpose() << " " << d << " " << mse);
                if(mse <= PLANE_FITTING_MSE){
                    PlaneFeature plane;
                    plane.Set(cloud, Eigen::Vector3f(center_x, center_y, center_z), plane_normal, d, radius);

//                    // use the CP plane model to refine the plane model and compute 3x3 covariance matrix
//                    CPPlaneRefine(plane);

//                    ROS_DEBUG_STREAM("refined plane: " << plane.normal_.transpose() << " " << plane.d_ << "" << std::endl << plane.covariance_);
                    plane_feature_v.push_back(plane);

                    *planeCloud_ += cloud;
                }
            }

//            planeFeature_ = plane_feature_v;
        }

        // TODO: how to fuse the plane segments?
        // Fuse the plane segments and region
        // similar with find plane correspondences without odometry error: use center-vector along norm vector to evaluate the distance
        std::vector<std::vector<PlaneFeature>> clustered_planes;
        std::vector<int> judged;
        judged.resize(plane_feature_v.size(), 0);
        {
            for (int m = 0; m < plane_feature_v.size(); ++m) {
                if (judged.at(m)==1) continue;
                std::vector<PlaneFeature> i_clustered_planes;
                i_clustered_planes.emplace_back(plane_feature_v.at(m));
                judged.at(m) = 1;
                for (int i = m+1; i < plane_feature_v.size(); ++i) {
                    if (judged.at(i)==1) continue;
                    auto center_line = plane_feature_v.at(m).center_ - plane_feature_v.at(i).center_;
                    float d1 = std::abs(center_line.dot(plane_feature_v.at(m).normal_));
                    float d2 = std::abs(center_line.dot(plane_feature_v.at(i).normal_));
//                    std::cout << d1 << ", " << d2 << " // " ;
                    if (d1 < 20*PLANE_FITTING_MSE && d2 < 20*PLANE_FITTING_MSE){
                        i_clustered_planes.emplace_back(plane_feature_v.at(i));
                        judged.at(i) = 1;
                    }
                }
//                std::cout << std::endl;

                clustered_planes.emplace_back(i_clustered_planes);
            }
//            std::cout << "cluster size " << clustered_planes.size() << std::endl;

            for (int n = 0; n < clustered_planes.size(); ++n) {

                auto plane_n = clustered_planes.at(n).at(0);

                // update center and cloud
                auto center_n = plane_n.center_;
                for (int i = 1; i < clustered_planes.at(n).size(); ++i) {
                    center_n = plane_n.cloud_.size()*center_n + clustered_planes.at(n).at(i).center_*clustered_planes.at(n).at(i).cloud_.size();
                    center_n = center_n/float(plane_n.cloud_.size()+clustered_planes.at(n).at(i).cloud_.size());
                    plane_n.cloud_ += clustered_planes.at(n).at(i).cloud_;
                }
                // update radius
                float r = 0;
                for (int j = 0; j < plane_n.cloud_.size(); ++j) {
                    Eigen::Vector3f point = Eigen::Vector3f(plane_n.cloud_.at(j).x, plane_n.cloud_.at(j).y, plane_n.cloud_.at(j).z);
                    float d = (point-center_n).norm();
                    if (d > r)
                        r = d;
                }

                plane_n.center_ = center_n;
                plane_n.radius_ = r;
                CPPlaneRefine(plane_n);
                planeFeature_.emplace_back(plane_n);
            }

        }

        ROS_DEBUG("extract plane all cost %f ms, find planes %d->%d", time.toc(), plane_feature_v.size(), planeFeature_.size());
    }

    void LidarFeature::CPPlaneRefine(gp_lio::PlaneFeature &planeFeature) {

        double cp_param[3];

        ceres::Problem problem;
        ceres::LossFunction *lossFunction;
        lossFunction = new ceres::HuberLoss(0.5);

        problem.AddParameterBlock(cp_param, 3);

        Eigen::Vector3f cp = planeFeature.GetCP();
        cp_param[0] = double (cp.x());
        cp_param[1] = double (cp.y());
        cp_param[2] = double (cp.z());

        for (int i = 0; i < planeFeature.cloud_.size(); ++i) {

            Eigen::Vector3f point = Eigen::Vector3f(planeFeature.cloud_.points.at(i).x,
                                                    planeFeature.cloud_.points.at(i).y,
                                                    planeFeature.cloud_.points.at(i).z);

            auto *planeFittingCostFunction = new PlaneFittingCostFunction(point, POINT_MEASUREMENT_NOISE);

            problem.AddResidualBlock(planeFittingCostFunction, NULL, cp_param);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
//        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 10;

        TicToc t_solver;
//        ROS_DEBUG("begin solver");
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

//        ROS_DEBUG_STREAM(summary.BriefReport() << std::endl );
//        ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
//        ROS_DEBUG("ceres solver costs: %f ms", t_solver.toc());

        Eigen::Vector3f cp_new = Eigen::Vector3f(cp_param[0], cp_param[1], cp_param[2]);

        Eigen::Matrix3f R;
        R.setIdentity();
        R = R * (POINT_MEASUREMENT_NOISE * POINT_MEASUREMENT_NOISE);

        Eigen::Matrix3f covariance;
        covariance.setZero();

        for (auto & j : planeFeature.cloud_) {
            Eigen::Vector3f point = Eigen::Vector3f(j.x, j.y, j.z);
            Eigen::Vector3f jaco;

            float cp_new_norm = cp_new.norm();

            Eigen::Vector3f new_normal = cp_new.normalized();

            jaco = (point.transpose()/cp_new_norm - (point.transpose() * cp_new) * (cp_new.transpose()/std::pow(cp_new_norm, 3)) - cp_new.transpose()/cp_new_norm).transpose();
            float w = new_normal.transpose() * R * new_normal;

            w = 1.0;

            covariance += jaco * w * jaco.transpose();

        }

        covariance = covariance.inverse();
        planeFeature.ResetCP(cp_new, covariance);

//        ROS_DEBUG_STREAM("refined plane: " << planeFeature.normal_.transpose() << " " << planeFeature.d_ << "" << std::endl << planeFeature.covariance_);

    }

    void LidarFeature::ConstructCloudFeature()
    {
        extracted_feature_.clear();
        extracted_feature_.cloudTimeStamp_ = received_time_;
        extracted_feature_.edge_feature_ = *cornerPointsSharp_;
        extracted_feature_.less_edge_feature_ = *cornerPointsLessSharp_;
        extracted_feature_.surf_feature_ = *surfPointsFlat_;
        extracted_feature_.less_surf_feature_ = *surfPointsLessFlat_;
        extracted_feature_.ground_feature_ = *groundCloud_;
        extracted_feature_.outlier_feature_ = *outlierCloud_;
        extracted_feature_.plane_cloud_ = *planeCloud_;
        extracted_feature_.plane_feature_ = planeFeature_;

        if (SAVE_FEATURE_TMP_RESULT){

            std::string range_img_save_path = OUTPUT_PATH + "range_mat.txt";
            std::ofstream fout(range_img_save_path.c_str());
            for (int i = 0; i <= GROUND_SCAN_IND; ++i) {
                for (int j = 0; j <  HORIZON_SCAN; ++j) {
//                    if (groundMat_.at<int8_t>(i, j) == 1)
                    fout << rangeMat_.at<float>(i, j) << "\t";
//                    else
//                        fout << FLT_MAX << "\t";
                }
                fout << std::endl;
            }
            fout.close();

            std::string feature_type_save_path = OUTPUT_PATH + "featureTypeMat_.txt";
            std::ofstream fout1(feature_type_save_path.c_str());
            for (int i = 0; i < N_SCAN; ++i) {
                for (int j = 0; j <  HORIZON_SCAN; ++j) {
                    fout1 << featureTypeMat_.at<int>(i, j) << "\t";
                }
                fout1 << std::endl;
            }
            fout1.close();

            if(!fullCloud_->empty()){
                std::string ground_save_file = OUTPUT_PATH + "points_full.pcd";
                pcl::io::savePCDFileBinary(ground_save_file, *fullCloud_);
            }

            if(!groundCloud_->empty()){
                std::string ground_save_file = OUTPUT_PATH + "points_ground.pcd";
                pcl::io::savePCDFileBinary(ground_save_file, *groundCloud_);
            }

            if(!planeCloud_->empty()){
                std::string ground_save_file = OUTPUT_PATH + "points_plane.pcd";
                pcl::io::savePCDFileBinary(ground_save_file, *planeCloud_);
            }

            for (int k = 0; k < planeFeature_.size(); ++k) {
                if(!planeFeature_.at(k).cloud_.empty()){
                    std::string ground_save_file = OUTPUT_PATH + "points_plane_" + std::to_string(k)+".pcd";
                    pcl::io::savePCDFileBinary(ground_save_file, planeFeature_.at(k).cloud_);
                }
            }

            if(!cornerPointsLessSharp_->empty()){
                std::string less_surf_save_file = OUTPUT_PATH + "points_edge.pcd";
                pcl::io::savePCDFileBinary(less_surf_save_file, *cornerPointsLessSharp_);
            }

            if(!segmentedCloud_->empty()){
                std::string seg_save_file = OUTPUT_PATH + "seg_points.pcd";
                pcl::io::savePCDFileBinary(seg_save_file, *segmentedCloud_);
            }

            if(!surfPointsLessFlat_->empty()){
                std::string less_surf_save_file = OUTPUT_PATH + "points_surf.pcd";
                pcl::io::savePCDFileBinary(less_surf_save_file, *surfPointsLessFlat_);
            }

        }

    }

    void LidarFeature::FreeMemory()
    {

        cloud_.clear();

        fullCloud_->clear();
        fullInfoCloud_->clear();
        groundCloud_->clear();
        outlierCloud_->clear();
        segmentedCloud_->clear();
        segmentedCloudIdx_.clear();
        segmentedCloudIdx_.shrink_to_fit();

        segCloudInfo_.clear();

        neighborIterator_.clear();
        neighborIterator_.shrink_to_fit();

        rangeMat_.release();
        groundMat_.release();
        labelMat_.release();
        featureTypeMat_.release();

        if (! planeFeature_.empty()){
            planeFeature_.clear();
            planeFeature_.shrink_to_fit();
        }

        neighborIterator_.clear();
        segCloudInfo_.clear();

//        cloudSmoothness_.clear();
//        cloudSmoothness_.shrink_to_fit();
//        ROS_DEBUG("6.0");
//        cloudCurvature_.clear();
//        cloudCurvature_.shrink_to_fit();
//        ROS_DEBUG("6.1");
//        cloudNeighborPicked_.clear();
//        cloudNeighborPicked_.shrink_to_fit();
//        ROS_DEBUG("6.2");
//        cloudLabel_.clear();
//        cloudLabel_.shrink_to_fit();

        cornerPointsSharp_->clear();
        cornerPointsLessSharp_->clear();
        surfPointsFlat_->clear();
        surfPointsLessFlat_->clear();
        surfPointsLessFlatScan_->clear();
        surfPointsLessFlatScanDS_->clear();
        if(!planeCloud_->empty())
            planeCloud_->clear();

        delete[] allPushedIndX_;
        delete[] allPushedIndY_;
        delete[] queueIndX_;
        delete[] queueIndY_;
    }

} // namespace gp_lio