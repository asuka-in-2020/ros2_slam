#include "SLAMutils.hpp"
#include "geometry_msgs/msg/point.hpp"


xyzPoint& rgbd2xyz_from_points_list(const std::vector<cv::Point2f> &map_points, const cv::Mat &img, const cv::Mat &depth, const CameraParam &cam_intrinsics, int step)
{
    int factor = 5000;
    static xyzPoint xyz;
    int num = map_points.size();
    step = num > 500 ? num / 500 : step;
    // std::vector<cv::Point3f> points;
    // std::vector<cv::Point3f> colors;
    for (int i = 0; i < num; i += step)
    {
        cv::Point2f pt = map_points[i];
        float d = depth.at<float>(pt.y, pt.x);
        if (d == 0)
            continue;
        cv::Point3f p;
        p.z = d /factor *1000;
        p.x = (pt.x - cam_intrinsics.cx) * p.z / cam_intrinsics.fx;
        p.y = (pt.y - cam_intrinsics.cy) * p.z / cam_intrinsics.fy;
        xyz.point.push_back(p);
        cv::Point3f color;
        color.x = img.at<cv::Vec3b>(pt)[0];
        color.y = img.at<cv::Vec3b>(pt)[1];
        color.z = img.at<cv::Vec3b>(pt)[2];
        xyz.color.push_back(color);
    }
    return xyz;
}

std::shared_ptr<sensor_msgs::msg::PointCloud2> create_pointcloud(const std::vector<cv::Point3f> &points, int num){
    int step {};
    std::vector<cv::Point3f> selected_points;
    {
        auto _ = points.size();
        step = _ > 1000 ? _ / num : 1;
        // 根据step选择点
        for (size_t i = 0; i < _; i += step) {
            selected_points.push_back(points[i]);
        }
    }

    auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pc2_msg->header.stamp = rclcpp::Clock().now();
    pc2_msg->header.frame_id = "map";
    pc2_msg->height = 1;
    pc2_msg->width = selected_points.size();

    // 定义点云字段
    pc2_msg->fields.resize(3);
    pc2_msg->fields[0].offset = 0;
    pc2_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc2_msg->fields[0].count = 1;
    pc2_msg->fields[1].offset = 4;
    pc2_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc2_msg->fields[1].count = 1;
    pc2_msg->fields[2].offset = 8;
    pc2_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc2_msg->fields[2].count = 1;

    // 填充字段信息
    pc2_msg->is_bigendian = false;
    pc2_msg->point_step = 12; // 3个float32，每个4字节
    pc2_msg->row_step = pc2_msg->width * pc2_msg->point_step;

    // 计算数据大小并分配内存
    size_t data_size = pc2_msg->row_step * pc2_msg->height;
    std::vector<uint8_t> data(data_size, 0);

    // 填充点云数据
    for (size_t i = 0; i < selected_points.size(); ++i) {
        float* ptr = reinterpret_cast<float*>(&data[i * pc2_msg->point_step]);
        ptr[0] = selected_points[i].x;
        ptr[1] = selected_points[i].y;
        ptr[2] = selected_points[i].z;
    }

    pc2_msg->data = data;

    return pc2_msg;

}


