#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"


// 相当于定义了g2o中的顶点
class PoseLocalParameterization : public ceres::LocalParameterization
{
    // 定义顶点更新函数
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    // 计算雅可比
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;

    // 优化变量维度
    virtual int GlobalSize() const { return 7; };
    // 实际自由度
    virtual int LocalSize() const { return 6; };
};
