//
// Created by wchen on 2020/4/4.
// Copy from VINS-MONO
//

#ifndef GP_LIO_MARGINALIZATIONFACTOR_H
#define GP_LIO_MARGINALIZATIONFACTOR_H

#include <ros/ros.h>
#include <ros/console.h>

#include <ceres/ceres.h>
#include <unordered_map>
#include "gp_lio/utility/tic_toc.h"

const int NUM_THREADS = 4;

struct ResidualBlockInfo
{
    ResidualBlockInfo(ceres::CostFunction *cost_function, ceres::LossFunction *loss_function, std::vector<double *> parameter_blocks, std::vector<int> drop_set)
        : cost_function(cost_function), loss_function(loss_function), parameter_blocks(parameter_blocks), drop_set(drop_set) {}

    void Evaluate();

    ceres::CostFunction *cost_function;
    ceres::LossFunction *loss_function;
    std::vector<double *> parameter_blocks;
    std::vector<int> drop_set;

    double **raw_jacobians;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
    Eigen::VectorXd residuals;

    int localSize(int size)
    {
        return size == 7 ? 6 : size;
    }
};

struct ThreadsStruct
{
    std::vector<ResidualBlockInfo *> sub_factors;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    std::unordered_map<long, int> parameter_block_size; //global size
    std::unordered_map<long, int> parameter_block_idx;  //local size
};

class MarginalizationInfo
{

public:
    ~MarginalizationInfo();
    int localSize(int size) const;
    int globalSize(int size) const;
    void addResidualBlockInfo(ResidualBlockInfo *residual_block_info);
    void preMarginalize();
    void marginalize();
    std::vector<double *> getParameterBlocks(std::unordered_map<long, double *> &addr_shift);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<ResidualBlockInfo *> factors;
    int m, n;
    std::unordered_map<long, int> parameter_block_size; //global size
    int sum_block_size;
    std::unordered_map<long, int> parameter_block_idx; //local size
    std::unordered_map<long, double *> parameter_block_data;

    std::vector<int> keep_block_size; //global size
    std::vector<int> keep_block_idx;  //local size
    std::vector<double *> keep_block_data;

    Eigen::MatrixXd linearized_jacobians;
    Eigen::VectorXd linearized_residuals;
    const double eps = 1e-8;
};

class MarginalizationFactor : public ceres::CostFunction
{

public:
    MarginalizationFactor(MarginalizationInfo *marginalization_info);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    MarginalizationInfo *marginalization_info_;
};

#endif //GP_LIO_MARGINALIZATIONFACTOR_H
