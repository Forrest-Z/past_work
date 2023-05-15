//
// Created by wchen on 2020/7/10.
//

#include "gp_lio/feature/LidarHandcraftFeature.h"

namespace gp_lio{

    LidarHandcraftFeature::LidarHandcraftFeature(pcl::PointCloud<pcl::PointXYZ> &feature_cloud) {

        point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        *point_cloud_ = feature_cloud;

    }

    LidarHandcraftFeature::~LidarHandcraftFeature() {

    }

    void LidarHandcraftFeature::generate_feature(pcl::PointCloud<struct PointxyzWithHandcraftFeature>::Ptr & features, int k_start, int k_end, int k_step) {

        int pointN = point_cloud_->size();
        std::vector<std::vector<std::vector<int>>> indices;
        std::vector<std::vector<Eigen::Matrix<double ,3,3>>> covs33;
        std::vector<std::vector<double>> entropys(pointN);
        int k_size = ((k_end-k_start)/k_step)+1;
        for(int i=0;i<entropys.size();++i){
            entropys[i].resize(k_size);
        }
        std::vector<std::vector<Eigen::Vector3d>> eigen3ds;
        std::vector<std::vector<Eigen::Vector2d>> eigen2ds;
        std::vector<std::vector<Eigen::Matrix3d>>vectors;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
        kdTree.setInputCloud(point_cloud_);
        double cost1 =0,cost2=0;
        for (int j=0;j<k_size;++j) {// 计算所有点云在不同k值时的　特征值，特征向量，协方差，k近邻，特征熵
            std::vector<std::vector<int>> indices_vec;
            std::vector<Eigen::Matrix<double ,3,3>> cov_vec;
            std::vector<Eigen::Matrix3d> eigen_vector_vec;
            std::vector<Eigen::Vector3d> eigen3d_value_vec;
            std::vector<Eigen::Vector2d> eigen2d_value_vec;
            std::vector<int> neighbor_index;//领域索引
            std::vector<float> neighbor_square_distance;//领域距离大小
            for (int i=0;i<pointN;++i){
                if(kdTree.nearestKSearch(point_cloud_->points[i], k_start + j * k_step, neighbor_index, neighbor_square_distance) > 0)//寻找领域为k_start+j*k_step的值
                {
                    Eigen::Vector4d centroid;
                    pcl::compute3DCentroid(*point_cloud_, neighbor_index, centroid);  // 计算质心
                    Eigen::Matrix3d convariance_matrix33;  // 3x3 协方差矩阵
                    pcl::computeCovarianceMatrix(*point_cloud_, neighbor_index, centroid, convariance_matrix33);//计算协方差
                    Eigen::Matrix3d eigenVectors;
                    Eigen::Vector3d eigen3dValues;
                    Eigen::Vector2d eigen2dValues;//
                    pcl::eigen33(convariance_matrix33, eigenVectors, eigen3dValues);
                    Eigen::Vector3d::Index maxRow, maxCol, minRow, minCol; //对特征值．特征向量由大到小排序
                    eigen3dValues.maxCoeff(&maxRow, &maxCol);
                    eigen3dValues.minCoeff(&minRow, &minCol);
                    // l1 > l2 > l3
                    const double l1 = eigen3dValues[maxRow];
                    const double l2 = eigen3dValues[3 - maxRow - minRow];
                    const double l3 = eigen3dValues[minRow];
                    Eigen::Vector3d v1 = eigenVectors.col(maxRow);
                    Eigen::Vector3d v2 = eigenVectors.col(3 - maxRow - minRow);
                    Eigen::Vector3d v3 = eigenVectors.col(minRow);
                    eigen3dValues  << l1,l2,l3;
                    eigenVectors.col(0) = v1;
                    eigenVectors.col(1) = v2;
                    eigenVectors.col(2) = v3;
                    double entropy = CalculateEntropy(eigen3dValues); //特征熵

                    Eigen::Matrix<double ,2,2> convariance_matrix22 = convariance_matrix33.block<2,2>(0,0);// 2x2 水平投影协方差矩阵
                    Eigen::EigenSolver<Eigen::Matrix<double ,2,2>> es(convariance_matrix22);
                    Eigen::Matrix2d eigen22 = es.pseudoEigenvalueMatrix();
                    Eigen::Matrix2d::Index maxR, maxC, minR, minC;
                    eigen22.maxCoeff(&maxR,&maxC);
                    eigen22.minCoeff(&minR,&minC);
                    eigen2dValues(0) = eigen22(maxR,maxC);
                    eigen2dValues(1) = eigen22(minR,minC);

                    indices_vec.push_back(neighbor_index);
                    cov_vec.push_back(convariance_matrix33);
                    entropys[i][j] = entropy;
                    eigen_vector_vec.push_back(eigenVectors);
                    eigen3d_value_vec.push_back(eigen3dValues);
                    eigen2d_value_vec.push_back(eigen2dValues);
                }
            }
            indices.push_back(indices_vec);
            covs33.push_back(cov_vec);
            eigen3ds.push_back(eigen3d_value_vec);
            eigen2ds.push_back(eigen2d_value_vec);
            vectors.push_back(eigen_vector_vec);
        }
        // 求每个点的熵最小值对应的k
        std::vector<int> kmin_indx_vec;
        for(auto e :entropys){
            auto min_iter = std::min_element(e.begin(),e.end());
            int min_indx = std::distance(e.begin(),min_iter);
            kmin_indx_vec.push_back(min_indx);
        }
        features->resize(4096);//输出的handcraft特征向量　[4096x13]

        clock_t t4 = std::clock();
        for(int j=0;j<pointN;++j){//为每个点计算特征
            std::vector<int> neighborhood = indices[kmin_indx_vec[j]][j];
            Eigen::Vector3d eigen3d = eigen3ds[kmin_indx_vec[j]][j];
            Eigen::Vector2d eigen2d = eigen2ds[kmin_indx_vec[j]][j];
            Eigen::Matrix3d vector = vectors[kmin_indx_vec[j]][j];
            PointxyzWithHandcraftFeature feature_temp = CalculateHandcraftFeatures(j,neighborhood,eigen3d,eigen2d,vector);
            features->at(j) = feature_temp;
        }

    }

    double LidarHandcraftFeature::CalculateEntropy(Eigen::Vector3d &eigen) {

        double L_ = (eigen[0] - eigen[1]) / eigen[0];
        double P_ = (eigen[1] - eigen[2]) / eigen[0];
        double S_ = eigen[2] / eigen[0];
        double Entropy = -L_*std::log(L_)-P_*std::log(P_)-S_*std::log(S_);
        return Entropy;

    }

    PointxyzWithHandcraftFeature LidarHandcraftFeature::CalculateHandcraftFeatures(int i, std::vector<int> &nbrs_index,
                                                                                   Eigen::Vector3d &eigen3d,
                                                                                   Eigen::Vector2d &eigen2d,
                                                                                   Eigen::Matrix3d &vector) {

        PointxyzWithHandcraftFeature feature;
        // point
        feature.x = float(point_cloud_->at(i).x);
        feature.y = float(point_cloud_->at(i).y);
        feature.z = float(point_cloud_->at(i).z);
        // 3D
        float C = eigen3d[2]/eigen3d.sum();
        float O = std::pow(eigen3d.prod(),1.0/3)/eigen3d.sum();
        float L = (eigen3d[0]-eigen3d[1])/eigen3d[0];
        float E = -(eigen3d/eigen3d.sum()).dot((eigen3d/eigen3d.sum()).array().log().matrix());
        float D = 3.0 * nbrs_index.size()/(4.0 * M_PI * eigen3d.prod());
        //2D
        float S_2 = eigen2d.sum();
        float L_2 = eigen2d[1]-eigen2d[0];
        Eigen::VectorXd nbr_zd(nbrs_index.size());
        for(int j=0;j<nbrs_index.size();++j){
            nbr_zd(j) = (point_cloud_->at(nbrs_index[j]).z);
        }
        nbr_zd = nbr_zd.array() - nbr_zd.minCoeff();
        float dZ = nbr_zd.maxCoeff();
        float vZ = std::sqrt(nbr_zd.array().pow(1.0/2).sum()/(nbr_zd.size()-1));//z的均方差
        float V = vector(2,2);
        feature.c = C;
        feature.o = O;
        feature.l = L;
        feature.e = E;
        feature.d = D;
        feature.s2 = S_2;
        feature.l2 = L_2;
        feature.dz = dZ;
        feature.vz = vZ;
        feature.v = V;
        return feature;

    }

}