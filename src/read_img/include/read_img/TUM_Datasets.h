#ifndef TUM_RGBD_DATASET_H
#define TUM_RGBD_DATASET_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

class TUM_RGBD_dataset {
private:
    std::string data_name_;
    const std::string dataset_path_ = "/home/asuka2020/";
    std::string img_dir_;
    std::vector<std::string> rgb_list_, depth_list_;

    struct Camera {
        float fx, fy, cx, cy;
        float d0, d1, d2, d3, d4;
    };

    const Camera cam_intrinsics_ {591.1,590.1,331.0,234.0,-0.0410,0.3286,0.0087,0.0051,-0.5643};
    const Eigen::Matrix3f K = (Eigen::Matrix3f() << cam_intrinsics_.fx, 0, cam_intrinsics_.cx,
                                                    0, cam_intrinsics_.fy, cam_intrinsics_.cy,
                                                    0, 0, 1).finished();
    const Eigen::Matrix<float, 5, 1> D = (Eigen::Matrix<float, 5, 1>() << cam_intrinsics_.d0, cam_intrinsics_.d1, cam_intrinsics_.d2, cam_intrinsics_.d3, cam_intrinsics_.d4).finished();


    struct rgbd_img {
        std::string rgb_path;
        std::string depth_path;
    };

    std::vector<rgbd_img> img_list_;

public:
    TUM_RGBD_dataset(const std::string& data_name = "rgbd_dataset_freiburg1_xyz");

    std::vector<std::string> split(const std::string& str, char delimiter);
    bool readImgTxt(const std::string& txt_path, std::vector<std::string>& img_list);

friend class RGBD_publisher;
};

#endif // TUM_RGBD_DATASET_H