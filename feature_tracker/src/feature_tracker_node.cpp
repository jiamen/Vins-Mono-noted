//
// Created by zlc on 2021/5/20.
//

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>

#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

ros::Publisher pub_img,pub_match;
ros::Publisher pub_restart;

FeatureTracker trackerData[NUM_OF_CAM];

double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;


// 图片的回调函数
void img_callback(const sensor_msgs::ImageConstPtr& img_msg)
{
    if (first_image_flag)       // 对第一帧图像的基本操作
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
        last_image_time  = img_msg->header.stamp.toSec();
        return;
    }
    // detect unstable camera stream
    // 检查时间戳是否正常，这里认为超过一秒或者错乱就异常
    // 图像时间差太多，光流追踪就会失败，这里没有描述子匹配，因此对时间戳要求就高
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
    {
        // 一些常规的reset操作
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;

        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);              // 告诉其他模块要重启了
        return;
    }
    last_image_time = img_msg->header.stamp.toSec();    // 更新上一帧图像时间

    // frequency control
    // 控制一下发给后端的频率
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)     // 保证发给后端的跑不超过这个频率
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        // 这段时间的频率和预设频率十分接近，就认为这段时间很棒，重启一下，避免 delta t 太大
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    // 即使不发布也是正常做光流追踪的！ 光流对图像的变化要求尽可能小

    cv_bridge::CvImageConstPtr ptr;

    // 把ros message 转成 cv::Mat
    if (img_msg->encoding=="8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width  = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;
    TicToc t_r;
    for (int i=0; i<NUM_OF_CAM; i ++)
    {
        ROS_DEBUG("processing camera %d", i);
        if (i!=1 )
        {

        }
    }

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker");      // ros节点初始化
    ros::NodeHandle n("~");                             // 声明一个句柄，～代表这个节点的命名空间
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);    // 设置ros log级别
    readParameters(n);              // 读取配置文件

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);    // 获得每个相机的内参

    if (FISHEYE)
    {
        for (int i = 0; i<NUM_OF_CAM; i ++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    // 这个向roscore注册订阅这个topic，收到一次message就执行一次回调函数
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

    // 注册一些publisher
    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000); // 实际发出去的是 /feature_tracker/feature
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    ros::spin();    // spin代表这个节点开始循环查询topic是否接收
    return 0;
}
