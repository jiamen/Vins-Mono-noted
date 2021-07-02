//
// Created by zlc on 2021/5/20.
//

#ifndef _FEATURE_TRACKER_FEATURE_TRACKER_H_
#define _FEATURE_TRACKER_FEATURE_TRACKER_H_

#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>       // Declares the functions backtrace, backtrace_symbols, backtrace_symbols_fd.
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"  // 相机工厂模式
#include "camodocal/camera_models/CataCamera.h"     // MEI相机模型：（卡特鱼眼相机）：将像素坐标投影到单位圆内，这里涉及了鱼眼相机模型
#include "camodocal/camera_models/PinholeCamera.h"  // 针孔相机模型： 将像素坐标直接转换到归一化平面（z=1）并采用逆畸变模型(k1,k2,p1,p2)去畸变等。

#include "parameters.h"     // 读取参数
#include "tic_toc.h"        // 计时


using namespace std;
using namespace camodocal;
using namespace Eigen;


bool inBorder(const cv::Point2f& pt);

void reduceVector(vector<cv::Point2f>& v, vector<uchar> status);
void reduceVector(vector<int>& v, vector<uchar> status);


class FeatureTracker
{
public:
    FeatureTracker();

    void readImage(const cv::Mat &_img, double _cur_time);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string& calib_file);

    void showUndistortion(const string& name);

    void rejectWithF();

    void undistortedPoints();

    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img;                // cur_img 上一帧图像; forw_img 当前帧图像;

    vector<cv::Point2f> n_pts;      // 存放补充提取的点，比如150点，跟踪之后当前帧还有100点，那么再提取50个点进行补充
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;    // cur_pts上一帧的点坐标，forw_pts当前帧的点坐标
    vector<cv::Point2f> prev_un_pts, cur_un_pts;        // cur_un_pts：最新帧的相机系 归一化坐标
    vector<cv::Point2f> pts_velocity;

    vector<int> ids;                                    // 每个点的id号
    vector<int> track_cnt;                              // 每个点被跟踪的次数

    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;

    camodocal::CameraPtr m_camera;

    double cur_time;            // 当前帧时间
    double prev_time;           // 上一帧时间

    static int n_id;
};


#endif // _FEATURE_TRACKER_FEATURE_TRACKER_H_
