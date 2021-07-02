//
// Created by zlc on 2021/5/20.
//

#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

// 判断一个点是否在边界内
bool inBorder(const cv::Point2f& pt)
{
    // 边界大小
    const int BORDER_SIZE = 1;

    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);

    return BORDER_SIZE<=img_x && img_x<COL-BORDER_SIZE && BORDER_SIZE<=img_y && img_y<ROW-BORDER_SIZE;
}

// 根据状态位，进行“瘦身” 788025
void reduceVector(vector<cv::Point2f>& v, vector<uchar> status)
{
    int j = 0;
    for (int i=0; i<int(v.size()); i ++)
    {
        if (status[i])
            v[j ++] = v[i];
    }
    v.resize(j);
}

void reduceVector(vector<int>& v, vector<uchar> status)
{
    int j = 0;
    for (int i=0; i<int(v.size()); i ++)
    {
        if (status[i])
            v[j ++] = v[i];
    }
    v.resize(j);
}

FeatureTracker::FeatureTracker()
{

}

// 给现有的特征点设置mask，目的为了特征点的均匀化
void FeatureTracker::setMask()
{
    // 如果是鱼眼相机的话
    if (FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time.
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    // 把跟踪次数、跟踪点坐标、跟踪点序号进行绑定，输入cnt_pts_id中
    for (unsigned int i=0; i<forw_pts.size(); i ++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    // 利用光流特征，追踪次数多的稳定性好，排前面
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>>& a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    // 把上述三项信息全部清空
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    // 重新遍历特征点，在每个点周围画圈，防止特征点过于稠密
    for (auto& it :cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)     // 注意255是白，0是黑
        {
            // 把挑选剩下的特征点重新放进容器
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            // Opencv函数，把周围一个园内全部置0，这个区域不允许别的特征点存在，避免特征点过于集中，0是黑
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}


// 把新的点加入容器，id给-1作为区分
void FeatureTracker::addPoints()
{
    for (auto& p : n_pts)
    {
        forw_pts.push_back(p);      // 存放到当前帧稳定跟踪点 的 后面
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

/**
 * @brief
 *
 * @param[in] _img 输入图像
 * @param[in] _cur_time 图像的时间戳
 * 1、图像均衡化预处理
 * 2、光流追踪
 * 3、提取新的特征点（如果发布）
 * 4、所有特征点去畸变，计算速度
 */
void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    if (EQUALIZE)
    {
        // 图像太暗或者太亮，提特征点比较难，所以均衡化一下
        // ! opencv 函数看一下
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8,8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs:%f ms", t_c.toc());      // 直方图均衡化花费多长时间
    }
    else
        img = _img;

    // 这里forw表示当前帧，cur表示上一帧
    if (forw_img.empty())       // 第一次输入图像，prev_img这个没用
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    // 上一帧有特征点，就可以进行光流追踪了，根据cur_pts计算当前帧特征点forw_pts
    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;

        // 调用opencv函数进行光流追踪
        // Step 1： 通过opencv光流追踪给的状态剔除outlier，追踪成功后，每个成功点的status会置1
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        for (int i=0; i<int(forw_pts.size()); i ++)
        {
            // step 2：通过图像边界剔除outlier
            if (status[i] && !inBorder(forw_pts[i]))    // 追踪状态好检查在不在图像范围
                status[i] = 0;
        }
        reduceVector(prev_pts, status);     // 没用到
        reduceVector(cur_pts, status);      // 根据状态位，对上一帧的特征点进行瘦身
        reduceVector(forw_pts, status);     // 根据状态位，对当前帧的特征点进行瘦身
        reduceVector(ids, status);          // 特征点的id
        reduceVector(cur_un_pts, status);   // 去畸变后的坐标
        reduceVector(track_cnt, status);    // 被追踪次数
        ROS_DEBUG("temporal optical flow costs: %f ms", t_o.toc());
    }

    // 被追踪到的是上一帧就存在的，因此追踪数 + 1
    for (auto& n : track_cnt)
        n ++;

    if (PUB_THIS_FRAME)
    {
        // step 3：通过对极约束来剔除outlier
        rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs: %fms", t_m.toc());

        // 比如每个待发布的帧都要保证150个特征点，经过跟踪之后的稳定点是100个，那么还要再提取50个点
        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;

            // 只有发布才可以提取更多特征点，同时避免提的点进mask
            // 会不会这些点集中？会，不过没关系，他们下一次作为老将就得接受均匀化的洗礼
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        // 把新提取的点，加入到
        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;   // 以上三个量无用
    cur_img = forw_img;         // 实际上是上一帧的图像
    cur_pts = forw_pts;         // 上一帧的特征点
    undistortedPoints();
    prev_time = cur_time;
}



// 当前帧所有点统一去畸变，同时计算特征点速度，用来后续时间戳标定
void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    // cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i=0; i<cur_pts.size(); i ++)
    {
        // 有的之前去过畸变了，这里连同新人重新做一次
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);             // 根据相机模型把 平面像素点 投影到 三维空间中
        cur_un_pts.push_back(cv::Point2f( b.x()/b.z(), b.y()/b.z()) );

        // id->坐标的map
        cur_un_pts_map.insert(make_pair( ids[i], cv::Point2f(b.x()/b.z(), b.y()/b.z()) ) );
        // printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }

    // calculate points velocity，计算特征点速度
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i=0; i<cur_un_pts.size(); i ++)
        {
            if (ids[i] != -1)       // 不是新添加的特征点
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);

                // 找到同一个特征点
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    // 得到在归一化平面的速度
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else        // 新添加的特征点
            {
                pts_velocity.push_back(cv::Point2f(0,0));
            }
        }
    }
    else
    {
        // 第一帧的情况
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }

    prev_un_pts_map = cur_un_pts_map;
}


