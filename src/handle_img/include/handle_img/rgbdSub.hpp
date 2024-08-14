#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <opencv2/opencv.hpp> // 用于图像处理，类似于cv_bridge
#include <cv_bridge/cv_bridge.h> // 用于图像处理，类似于cv_bridge
#include <chrono>             // 用于时间处理
#include "SLAMutils.hpp"
#include "odometry.hpp"
class RGBD_subscriber : public rclcpp::Node {

public:
    RGBD_subscriber(const std::string &name = "rgbd_subscriber");

    void process_messages();


private:
    const CameraParam cam_intrinsics_ {591.1,590.1,331.0,234.0,-0.0410,0.3286,0.0087,0.0051,-0.5643};
    const cv::Matx33f K_ = cv::Matx33f(cam_intrinsics_.fx, 0, cam_intrinsics_.cx,
                                        0, cam_intrinsics_.fy, cam_intrinsics_.cy,
                                        0, 0, 1);
    const cv::Matx<float, 5, 1> D_ = cv::Matx<float, 5, 1>(cam_intrinsics_.d0, cam_intrinsics_.d1, cam_intrinsics_.d2, cam_intrinsics_.d3, cam_intrinsics_.d4);
    //subscributers to get msg
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;


    //publishers to send msg
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    cv::Mat rgb_img, depth_img;
    cv_bridge::CvImagePtr cv_ptr;

    MonoSLAM mono_slam;

    nav_msgs::msg::Path path_;


    


};
