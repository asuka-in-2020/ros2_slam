#pragma once
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <opencv2/opencv.hpp>
#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"

struct CameraParam {
    float fx, fy, cx, cy;
    float d0, d1, d2, d3, d4;
};
struct xyzPoint
{
    std::vector<cv::Point3f> point;
    std::vector<cv::Point3f> color;
};
class Timer {
public:
    Timer() {
        start = std::chrono::high_resolution_clock::now();
    }

    inline void reset() {
        start = std::chrono::high_resolution_clock::now();
    }

    inline void getElapsedTime() {
        auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> elapsed = end - start;
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "cvtColor time: " << duration.count() << "ms" << std::endl;
        // return duration.count();
    }

    private:
        std::chrono::time_point<std::chrono::high_resolution_clock> start;
}; 

inline void printRedText(const std::string &text) {
    std::cout << "\033[1;31m" << text << "\033[0m" << std::endl;
}

xyzPoint& rgbd2xyz_from_points_list(
    const std::vector<cv::Point2f> &map_points,
    const cv::Mat &img,
    const cv::Mat &depth,
    const CameraParam &cam_intrinsics,
    int step = 1
);

std::shared_ptr<sensor_msgs::msg::PointCloud2> create_pointcloud(const std::vector<cv::Point3f> &points, int num = 1000);