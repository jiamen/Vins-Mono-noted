#include "utility.h"


// 这个过程在《VIO第7讲》  视觉SFM与预积分对齐过程中，对齐流程中的第一步就是求得R_wc0，也就是这里的R0
Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d& g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();        // g_c0
    Eigen::Vector3d ng2{0, 0, 1.0};     // g_w
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    double yaw = Utility::R2ypr(R0).x();         // Θ角
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}
