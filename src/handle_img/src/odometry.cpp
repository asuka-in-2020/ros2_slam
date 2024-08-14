#include "odometry.hpp"

MonoSLAM::MonoSLAM(const cv::Matx33f &K, const cv::Matx<float, 5, 1> &D)
    : K_(K), D_(D) // 使用成员初始化列表
{
    cam_pose_ = cv::Mat(3, 1, CV_64F, cv::Scalar(0));
    rotation_ = cv::Mat::eye(3, 3, CV_64F);


}

void MonoSLAM::setK(const cv::Matx33f &K)
{
    K_ = K;
}

void MonoSLAM::setD(const cv::Matx<float, 5, 1> &D)
{
    D_ = D;
}


Frame MonoSLAM::extractFeatures(const cv::Mat& image, const cv::Matx33f &K, const cv::Matx<float, 5, 1> &D) {
    Frame frame;
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat undistorted;
    cv::undistort(gray, undistorted, K, D);

    orb_->detectAndCompute(image, cv::noArray(), frame.keypoints, frame.descriptors);
    return frame;
}

void MonoSLAM::motionCal(const cv::Mat& img){

    currFrame = extractFeatures(img, K_, D_);

    if (!preImg.empty())
    {
        // Feature matching
        timer_.reset();
        cv::BFMatcher matcher(cv::NORM_HAMMING, true); // 使用汉明距离，并且启用crossCheck

        std::vector<cv::DMatch> matches; // 存储匹配结果的向量

        // 使用match函数进行特征匹配
        matcher.match(prevFrame.descriptors, currFrame.descriptors, matches);

        timer_.getElapsedTime();

        // cam motion comuputation
        timer_.reset();
        // 假设matches是一个已经定义好的std::vector<cv::DMatch>类型的向量
        // 假设self.prev_keypoints和keypoints是已经定义好的std::vector<cv::KeyPoint>类型的向量
        // 假设self.k是相机内参，这里假设为cv::Mat类型

        // 将matches中的匹配点转换为float32类型的点集
        std::vector<cv::Point2f> ref_pts, curr_pts;
        for (const cv::DMatch& m : matches) {
            ref_pts.push_back(prevFrame.keypoints[m.queryIdx].pt);
            curr_pts.push_back(currFrame.keypoints[m.trainIdx].pt);
        }

        // 将点集转换为OpenCV的Mat类型，用于findEssentialMat和recoverPose函数
        cv::Mat ref_pts_mat, curr_pts_mat;
        ref_pts_mat = cv::Mat(ref_pts).reshape(2);
        curr_pts_mat = cv::Mat(curr_pts).reshape(2);

        // 使用RANSAC方法找到本质矩阵
        double threshold = 1.0; // 这里设置阈值
        double prob = 0.999; // 置信度
        cv::Mat E, mask;
        E= cv::findEssentialMat(curr_pts_mat, ref_pts_mat, K_, cv::RANSAC, prob, threshold, mask);

        // 恢复相机姿态
        cv::Mat R;
        cv::Mat T(3, 1, CV_64F);
        cv::recoverPose(E, curr_pts_mat, ref_pts_mat, K_, R, T,
                        mask);
        timer_.getElapsedTime();

        // update cam pose
        cv::Mat rotated_T(3, 1, CV_64F);
        
        cv::gemm(rotation_, T, 1, cv::Matx<double, 3, 1>::zeros(), 1, rotated_T);
        cam_pose_ += rotated_T;
        cv::Mat _= R * rotation_;

        trajectory_.push_back(cam_pose_);

        //update map point
        for (size_t i = 0; i < matches.size(); ++i) {
            if (mask.at<bool>(0, i) == true) {
                int map_idx = matches[i].trainIdx; // 注意：trainIdx 是 int 类型
                // // 检查 map_idx 是否已经存在于 self.map_points 中
                if (std::find(map_points_.begin(), map_points_.end(), currFrame.keypoints[map_idx].pt) == map_points_.end()) {
                    map_points_.push_back(currFrame.keypoints[map_idx].pt);
                    map_ref_frames_.push_back(trajectory_.size() - 1);
                    // 从图像中提取颜色
                    cv::Vec3b color = img.at<cv::Vec3b>(static_cast<int>(currFrame.keypoints[map_idx].pt.y), static_cast<int>(currFrame.keypoints[map_idx].pt.x));
                    // 将颜色转换为 std::vector 或其他适当的容器类型
                    map_colors_.push_back(color);
                }
            }
        }
    }
    prevFrame = currFrame;
    preImg = img;
}