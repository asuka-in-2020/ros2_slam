#pragma once

#include <opencv2/opencv.hpp> // 用于图像处理，类似于cv_bridge
#include <cv_bridge/cv_bridge.h> // 用于图像处理，类似于cv_bridge
// #include <Eigen/Dense>        // 用于数值计算，类似于numpy
// #include <Eigen/Core>         // 用于数值计算，类似于numpy
#include <chrono>             // 用于时间处理
#include "SLAMutils.hpp"



struct Frame {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
};
class MonoSLAM
{
private:
    cv::Matx33f K_;
    cv::Matx<float, 5, 1> D_;
    cv::Mat preImg;
    Frame prevFrame;
    Frame currFrame;

    cv::Ptr<cv::ORB> orb_ = cv::ORB::create();

    cv::Mat cam_pose_;
    cv::Mat rotation_;
    // Eigen::Matrix3f rotation_ = Eigen::Matrix3f::Identity();

    std::vector<cv::Matx<float, 3, 1>> trajectory_;
    std::vector<cv::Point2f>  map_points_; 
    std::vector<uint16_t> map_ref_frames_;
    std::vector<cv::Vec3b> map_colors_; //turple of BGR

    Timer timer_;
public:
    MonoSLAM(const cv::Matx33f &K, const cv::Matx<float, 5, 1> &D);
    ~MonoSLAM() = default;

    void setK(const cv::Matx33f &K);

    void setD(const cv::Matx<float, 5, 1> &D);

    void motionCal(const cv::Mat& img);
    Frame extractFeatures(const cv::Mat& image, const cv::Matx33f &K, const cv::Matx<float, 5, 1> &D);

    friend class RGBD_subscriber;
};