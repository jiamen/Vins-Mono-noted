#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <pthread.h>
#include <ceres/ceres.h>
#include <unordered_map>

#include "../utility/utility.h"
#include "../utility/tic_toc.h"

const int NUM_THREADS = 4;

// 先验约束项 残差因子
struct ResidualBlockInfo
{
    // 构造函数需要，cost function（约束），loss function：残差的计算方式，相关联的参数块，待边缘化的参数块的索引
    // 核函数_loss_function；   优化变量_parameter_blocks；    待marg变量的序号_drop_set；
    ResidualBlockInfo(ceres::CostFunction *_cost_function, ceres::LossFunction *_loss_function, std::vector<double *> _parameter_blocks, std::vector<int> _drop_set)
        : cost_function(_cost_function), loss_function(_loss_function), parameter_blocks(_parameter_blocks), drop_set(_drop_set) {}

    void Evaluate();

    ceres::CostFunction* cost_function;     // 残差函数
    ceres::LossFunction* loss_function;     // ρ(s), s 为残差的平方，处理离群点，鲁棒 核函数

    std::vector<double *> parameter_blocks;
    std::vector<int> drop_set;

    double **raw_jacobians;         // Jacobian
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;

    Eigen::VectorXd residuals;      // 误差项

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
    std::unordered_map<long, int> parameter_block_size;  // global size  优化变量维度     eg:旋转用四元数表示， 7
    std::unordered_map<long, int> parameter_block_idx;   // local size   变量自由度      自由度是6
};


// 用来进行边缘化操作的类
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

    std::vector<ResidualBlockInfo *> factors;

    int m, n;
    // m为需要marg掉的变量的总维度
    // n为需要保留的变量的总维度


    std::unordered_map<long, int> parameter_block_size; //global size   // 前面的long是数据地址->global size  每个变量的维度
    int sum_block_size;
    std::unordered_map<long, int> parameter_block_idx; //local size // 地址->参数排列的顺序idx  每个变量在H矩阵中的索引
    std::unordered_map<long, double *> parameter_block_data;    // 地址->参数块实际内容的地址

    std::vector<int> keep_block_size; //global size
    std::vector<int> keep_block_idx;  //local size
    std::vector<double *> keep_block_data;

    Eigen::MatrixXd linearized_jacobians;
    Eigen::VectorXd linearized_residuals;
    const double eps = 1e-8;

};

// 由于边缘化的costfuntion不是固定大小的，因此只能继承最基本的类
class MarginalizationFactor : public ceres::CostFunction
{
  public:
    MarginalizationFactor(MarginalizationInfo* _marginalization_info);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    MarginalizationInfo* marginalization_info;
};
