//
// Created by heihei on 2021/12/3.
//

#include <iostream>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include "trytry_vo/frontend.h"

const std::vector<Eigen::Matrix4d> &Frontend::getTf() const {
    return tfs;
}

const std::vector<std::shared_ptr<Frame>> &Frontend::getFrames() const {
    return frames;
}

const std::shared_ptr<Frame> &Frontend::getLastFramePtr() const {
    return last_frame_ptr;
}

const std::shared_ptr<Frame> &Frontend::getCurrentFramePtr() const {
    return current_frame_ptr;
}

Status Frontend::getStatus() const {
    return status;
}

Frontend::~Frontend() = default;

bool Frontend::addFrame(const Frame& frame) {

    current_frame_ptr = std::make_shared<Frame>(frame);

    switch (status) {
        case Status::INIT:
            Init();
            break;
        case Status::TRACKING_GOOD:
        case Status::TRACKING_BAD:
            Track();
            break;
        case Status::LOST:
            Reset();
            break;

    }

    last_frame_ptr = current_frame_ptr;


    return true;
}

bool Frontend::Init() {

    last_frame_ptr = current_frame_ptr;
    this->tfs.emplace_back(Eigen::Matrix4f::Identity());


    this->status = Status::TRACKING_GOOD;

    std::cout << "Init Successfully!!!" << std::endl;
    return true;
}

bool Frontend::Track() {

    int interiors = this->estimateCurrentPose();

    if (interiors < this->interiors_threshold_lost) {
        this->status = Status::LOST;
    } else if (interiors < this->interiors_threshold_bad) {
        this->status = Status::TRACKING_BAD;
    } else {
        this->status = Status::TRACKING_GOOD;
    }

    return true;
}

bool Frontend::Reset() {

    std::cout << "Reset is not implemented!" << std::endl;

    return true;
}

Frontend::Frontend() {
    this->status = Status::INIT;

    this->interiors_threshold_bad = Config::getInstance()->getData<int>("interiors_threshold_bad");
    this->interiors_threshold_lost = Config::getInstance()->getData<int>("interiors_threshold_lost");

}

int Frontend::estimateCurrentPose() {
    
    this->current_frame_ptr->computeORB();

    //TODO 这里还有一个参数需要放到配置文件中
    auto matches = current_frame_ptr->matchFrames(last_frame_ptr, 0.4);
    
    // 我先写一个完整的流程
    // 然后再去考虑匹配不好的步骤
    
    std::vector<Eigen::Vector3d> point_current;
    std::vector<Eigen::Vector3d> point_last;
    
    for (auto & match : *matches) {
        point_current.push_back(current_frame_ptr->getWorldPoint(match.queryIdx));
        point_last.push_back(last_frame_ptr->getWorldPoint(match.queryIdx));
    }
    
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    
    std::vector<int> inlines;
    this->estimateRT(point_last, point_current, R, t);
    last_frame_ptr = current_frame_ptr;
    Eigen::Matrix4f calculate_tf = Eigen::Matrix4f::Identity();
    if (t.norm() < 0.15)
    {
        calculate_tf.block<3, 3>(0, 0) = R;
        calculate_tf.block<3, 1>(0, 3) = t;
    }
    this->tfs.emplace_back(calculate_tf);

    return inlines.size();
}

void Frontend::setCamera(const Camera &config) {
    //TODO
//    Frontend::camera = config;
    this->camera = config;
}

bool Frontend::estimateRT(std::vector<Eigen::Vector3d> last, std::vector<Eigen::Vector3d> current,
                          Eigen::Matrix3d &R, Eigen::Vector3d &t) {
    return false;
}
