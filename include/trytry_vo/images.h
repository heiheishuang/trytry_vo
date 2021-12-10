//
// Created by heihei on 2021/12/2.
//

#ifndef TRYTRY_VO_IMAGES_H
#define TRYTRY_VO_IMAGES_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include "trytry_vo/frame.h"


class Images {

public:
    Images();

    ~Images();

    void loadMonoImages(std::string path);

    void loadStereoImages(std::string path);

    void loadDepthImages(const std::string& path);

    void getNextDepthRGBImage(cv::Mat &depth_image, cv::Mat &rgb_image);

    bool getNextFrame(Frame &new_frame);

    // Generate

    const std::vector<std::string> &getFilenameImageDepth() const;

    const std::vector<std::string> &getFilenameImageL() const;

    const std::vector<std::string> &getFilenameImageR() const;

    const std::vector<std::string> &getFilenameImageMono() const;

    const std::vector<double> &getTimestamps() const;

private:

    std::vector<std::string> filename_image_mono{};
    std::vector<std::string> filename_image_depth{};
    std::vector<std::string> filename_image_l{};
    std::vector<std::string> filename_image_r{};
    std::vector<double> timestamps{};

    std::string path;

    int current_id{0};
};


#endif //TRYTRY_VO_IMAGES_H
