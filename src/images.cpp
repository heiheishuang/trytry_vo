//
// Created by heihei on 2021/12/2.
//

#include "trytry_vo/images.h"
#include "trytry_vo/frame.h"

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>

const std::vector<std::string> &Images::getFilenameImageDepth() const {
    return filename_image_depth;
}

const std::vector<std::string> &Images::getFilenameImageL() const {
    return filename_image_l;
}

const std::vector<std::string> &Images::getFilenameImageR() const {
    return filename_image_r;
}

void Images::loadStereoImages(std::string path) {

}

void Images::loadMonoImages(std::string path) {

}

void Images::loadDepthImages(const std::string& path) {

    std::ifstream file_in(path + "associations.txt");
    while (!file_in.eof())
    {
        std::string s;
        getline(file_in, s);
        if (!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            std::string name_rgb, name_depth;
            ss >> t;
            this->timestamps.push_back(t);
            ss >> name_rgb;
            this->filename_image_mono.push_back(name_rgb);
            ss >> t;
            ss >> name_depth;
            this->filename_image_depth.push_back(name_depth);
        }
    }
    file_in.close();
}

const std::vector<std::string> &Images::getFilenameImageMono() const {
    return filename_image_mono;
}

const std::vector<double> &Images::getTimestamps() const {
    return timestamps;
}

void Images::getNextDepthRGBImage(cv::Mat &depth_image, cv::Mat &rgb_image) {
    cv::Mat depth;
    cv::Mat rgb;
    depth = cv::imread(path + this->filename_image_depth[current_id], CV_LOAD_IMAGE_UNCHANGED);
    rgb = cv::imread(path + this->filename_image_mono[current_id][current_id], CV_LOAD_IMAGE_UNCHANGED);

    depth_image = depth;
    rgb_image = rgb;

}

Frame Images::getNextFrame() {

    Frame new_frame(current_id);

    cv::Mat depth;
    cv::Mat rgb;
    depth = cv::imread(path + this->filename_image_depth[current_id], CV_LOAD_IMAGE_UNCHANGED);
    rgb = cv::imread(path + this->filename_image_mono[current_id][current_id], CV_LOAD_IMAGE_UNCHANGED);

    new_frame.setColorImage(rgb);
    new_frame.setDepthImage(depth);
    new_frame.setTimeStamp(this->timestamps[current_id]);

    current_id++;

    return new_frame;
}

Images::Images() = default;

Images::~Images() = default;
