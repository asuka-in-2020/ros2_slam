#include "rgbdSub.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RGBD_subscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}