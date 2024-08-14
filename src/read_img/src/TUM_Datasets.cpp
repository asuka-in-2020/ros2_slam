#include "TUM_Datasets.h"

TUM_RGBD_dataset::TUM_RGBD_dataset(const std::string& data_name): data_name_{data_name} {
    img_dir_ = dataset_path_ + data_name_ + "/";
    std::ifstream file(img_dir_ + "rgb.txt"); // 创建ifstream对象并传入文件路径

    if (readImgTxt(img_dir_ + "rgb.txt", rgb_list_) && readImgTxt(img_dir_ + "depth.txt", depth_list_)) {
        std::cout << "TUM_RGBD_dataset has read" << std::endl;
    }
    else {
        std::cerr << "can not open file" << std::endl;
        return;
    }
    
    auto it1 = rgb_list_.begin();
    auto it2 = depth_list_.begin();

    while (it1 != rgb_list_.end() && it2 != rgb_list_.end()) {
        img_list_.push_back({split(*it1, ' ')[1], split(*it2, ' ')[1]});
        ++it1;
        ++it2;
    }

}

std::vector<std::string> TUM_RGBD_dataset::split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);

    while (getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }

    return tokens;
}

bool TUM_RGBD_dataset::readImgTxt(const std::string& txt_path, std::vector<std::string>& img_list) {
    std::ifstream file(txt_path); // 创建ifstream对象并传入文件路径

    if (!file.is_open()) {
        std::cerr << "can not open file :" << txt_path << std::endl;
        return false;
    }
    std::string line;
    while (std::getline(file, line)) { // 逐行读取文件
        if (line.substr(0, 1) == "#") continue;
        img_list.push_back(line);
    }
    return true;
}