#include "rgbdSub.hpp"

RGBD_subscriber::RGBD_subscriber(const std::string &name) : Node(name) , mono_slam(K_, D_){
    RCLCPP_INFO(this->get_logger(), "rgbd_subscriber node has been created");

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/rgb_pub", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            rgb_img = cv_ptr->image;

            RCLCPP_INFO(this->get_logger(), "rgb_img size: %d, %d", rgb_img.rows, rgb_img.cols);

            process_messages();
            // cv::imshow("rgb", img);
            // cv::waitKey(1);
        });

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/depth_pub", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
            } catch (cv_bridge::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            depth_img = cv_ptr->image;
            process_messages();
        });
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);
    map_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_points", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);


}

void RGBD_subscriber::process_messages(){
    if (!(rgb_img.empty() || depth_img.empty())) {

        mono_slam.motionCal(rgb_img);
        cv::Matx<double, 3, 1> pose = mono_slam.cam_pose_;
        RCLCPP_INFO(this->get_logger(), "pose: %f, %f, %f", pose(0), pose(1), pose(2));

        //recover 3D points from depth image
        xyzPoint& points = rgbd2xyz_from_points_list(mono_slam.map_points_, rgb_img, depth_img, cam_intrinsics_, 1);
        std::vector<cv::Point3f> pointMoved;
        for (auto &point : points.point) {
            cv::Point3f p;
            p.x = point.x + pose(0);
            p.y = point.y + pose(1);
            p.z = point.z + pose(2);
            pointMoved.push_back(p);
        }
        auto pc2_msg = create_pointcloud(pointMoved);
        // point_cloud_pub_->publish(*pc2_msg);
        auto map_points_msg = create_pointcloud(points.color);

        // trajectory display
        // nav_msgs::msg::Path path;
        path_.header.frame_id = "map";
        path_.header.stamp = rclcpp::Clock().now();
        for (auto &point : mono_slam.trajectory_) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.pose.position.x = point(0);
            pose_stamped.pose.position.y = point(1);
            pose_stamped.pose.position.z = point(2);
            pose_stamped.pose.orientation.x = 0;
            pose_stamped.pose.orientation.y = 0;
            pose_stamped.pose.orientation.z = 0;
            pose_stamped.pose.orientation.w = 1;
            path_.poses.push_back(pose_stamped);
        }

        path_pub_->publish(path_);

        point_cloud_pub_->publish(*pc2_msg);
        map_points_pub_->publish(*map_points_msg);
        
        rgb_img = cv::Mat();
        depth_img = cv::Mat();

    }

}