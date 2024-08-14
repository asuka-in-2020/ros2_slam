#include "TUM_Datasets.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.h>

class RGBD_publisher : public rclcpp::Node {
public:
    RGBD_publisher(const std::string &node_name, TUM_RGBD_dataset &datasetObject)
    : Node{node_name}, datasetObject_{datasetObject}
    {
        RCLCPP_INFO(this->get_logger(), "RGBD_publisher has been started.");
        img_num_ = datasetObject_.img_list_.size();

        rgb_publisher = this->create_publisher<sensor_msgs::msg::Image>("rgb_pub", 10);
        depth_publisher = this->create_publisher<sensor_msgs::msg::Image>("depth_pub", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RGBD_publisher::timer_callback, this));
    }
    ~RGBD_publisher(){
        RCLCPP_INFO(this->get_logger(), "RGBD_publisher has been stopped.");
    }



private:
    TUM_RGBD_dataset &datasetObject_;
    int img_num_;
    int index_ {};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher;
    rclcpp::TimerBase::SharedPtr timer_;


    void timer_callback()
    {
        if (index_ < img_num_) {
            cv::Mat rgb_img = cv::imread(datasetObject_.img_dir_+datasetObject_.img_list_[index_].rgb_path, cv::IMREAD_COLOR);
            cv::Mat depth_img = cv::imread(datasetObject_.img_dir_+datasetObject_.img_list_[index_].depth_path, cv::IMREAD_UNCHANGED);

            sensor_msgs::msg::Image::SharedPtr rgb_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb_img).toImageMsg();
            sensor_msgs::msg::Image::SharedPtr depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth_img).toImageMsg();

            rgb_publisher->publish(*rgb_msg);
            depth_publisher->publish(*depth_msg);

            RCLCPP_INFO(this->get_logger(), "Publishing image %d", index_);
            index_++;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    TUM_RGBD_dataset datasetObject_ {"rgbd_dataset_freiburg1_xyz"};
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<RGBD_publisher>("RGBD_publisher", datasetObject_);
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}