//
// Created by zlc on 2021/5/20.
//

#include "parameters.h"


std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;

int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;               // 频率
double F_THRESHOLD;
int SHOW_TRACK;         // 展示跟踪
int STEREO_TRACK;
int EQUALIZE;
int ROW;                // 图像 行高
int COL;                // 图像 列宽
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;


template <typename T>
T readParam(ros::NodeHandle& n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_INFO_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

// 读取参数配置，通过roslaunch文件的参数服务器获得
void readParameters(ros::NodeHandle& n)
{
    std::string config_file;

    // 首先获得配置文件的路径，具体查看 vins_estimator/launch/euroc.launch  node<feature_tracker>标签
    config_file = readParam<std::string>(n, "config_file");

    // 使用opencv的yaml文件接口来读取文件
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings." << std::endl;
    }

    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");        // Vins-Mono-noted文件夹

    fsSettings["image_topic"] >> IMAGE_TOPIC;       // "/cam0/image_raw"
    fsSettings["imu_topic"] >> IMU_TOPIC;           // "/imu0"

    MAX_CNT = fsSettings["max_cnt"];                // 最大迭代次数
    MIN_DIST = fsSettings["min_dist"];              // 两个特征点之间的最小距离
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];

    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];        // ransac threshold (pixel)，如果存在误匹配，需要用RANSAC方法进行连续帧之间的位姿估计（单目相机的话进一步用内点三角化恢复地图点）

    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];              // 是否做直方图均衡化处理
    FISHEYE  = fsSettings["fisheye"];               // if using fisheye, turn on it. A circle mask will be loaded to remove edge noisy points
    if (FISHEYE == 1)
            FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;           // 窗口大小为20帧
    STEREO_TRACK = false;       // 双目跟踪
    FOCAL_LENGTH = 460;         // 焦距
    PUB_THIS_FRAME = false;     // 是否发布此帧，初始设置为false

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();
}

