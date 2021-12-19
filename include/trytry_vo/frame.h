//
// Created by heihei on 2021/12/1.
//

#ifndef TRYTRY_VO_FRAME_H
#define TRYTRY_VO_FRAME_H

#include <vector>

#include <opencv2/core.hpp>
#include <memory>
#include "trytry_vo/camera.h"
#include "trytry_vo/config.h"

class Frame {

public:
    Frame();

    explicit Frame(int id);

    bool computeORB();

    std::shared_ptr<std::vector<cv::DMatch>> matchFrames(std::shared_ptr<Frame> &f, double eta);

    double getDepthValue(int x, int y);

    Eigen::Vector3d getWorldPoint(int x, int y);

    Eigen::Vector3d getWorldPoint(int index);

    cv::KeyPoint getKeyPoint(int index);

    // Generate
    void setColorImage(const cv::Mat &colorImage);

    void setDepthImage(const cv::Mat &depthImage);

    const cv::Mat &getColorImage() const;

    const cv::Mat &getDepthImage() const;

    const cv::Mat &getLeftImage() const;

    const cv::Mat &getRightImage() const;

    const Eigen::Vector3d &getPose() const;

    void setPose(const Eigen::Vector3d &pose);

    void setLeftImage(const cv::Mat &leftImage);

    void setRightImage(const cv::Mat &rightImage);

    void setTimeStamp(double timeStamp);

    void setId(int id);

    int getId() const;

    void setCamera(const Camera &camera);

private:

    int id{0};

    Camera camera;

    bool is_key_frame{};

    std::vector<cv::KeyPoint> key_points;
    cv::Mat descriptor;
    std::vector<Eigen::Vector3d> world_points;

    cv::Mat color_image;
    cv::Mat depth_image;
    cv::Mat left_image;
    cv::Mat right_image;
    double time_stamp{};

    Eigen::Vector3d pose;

    double min_depth{};
    double max_depth{};
    double depth_factor{};
    double key_frame_num{};

};


#endif //TRYTRY_VO_FRAME_H
