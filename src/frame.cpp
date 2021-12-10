//
// Created by heihei on 2021/12/1.
//

#include <opencv2/features2d.hpp>
#include "trytry_vo/frame.h"


const cv::Mat &Frame::getLeftImage() const {
    return left_image;
}

const cv::Mat &Frame::getRightImage() const {
    return right_image;
}

const Eigen::Vector3d &Frame::getPose() const {
    return pose;
}

void Frame::setPose(const Eigen::Vector3d &pose) {
    Frame::pose = pose;
}

void Frame::setLeftImage(const cv::Mat &leftImage) {
    left_image = leftImage;
}

void Frame::setRightImage(const cv::Mat &rightImage) {
    right_image = rightImage;
}

void Frame::setTimeStamp(double timeStamp) {
    time_stamp = timeStamp;
}

bool Frame::computeORB() {
    //默认的参数，暂时不管，其实可以从config读入
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    detector->detectAndCompute(color_image, cv::Mat(), this->key_points, this->descriptor);
    std::vector<cv::KeyPoint> tKeyPts;
    cv::Mat tDesptors;
    for (size_t i = 0; i < key_points.size(); i++)
    {
        double tempDepth = getDepthValue(key_points[i].pt.x, key_points[i].pt.y);
        if (tempDepth < min_depth || tempDepth > max_depth)
            continue;
        Eigen::Vector3d worldPt = camera.pix2World(
                Eigen::Vector2d(key_points[i].pt.x, key_points[i].pt.y), tempDepth);

        //把点筛选一遍
        tKeyPts.push_back(key_points[i]);
        tDesptors.push_back(descriptor.row(i));
        world_points.emplace_back(worldPt);
    }
    key_points = tKeyPts;
    descriptor = tDesptors;
}

Frame::Frame(int id) {
    this->id = id;
    this->min_depth = Config::getData<double>("min_depth");
    this->max_depth = Config::getData<double>("max_depth");
    this->depth_factor = Config::getData<double>("depth_factor");
    this->key_frame_num = Config::getData<int>("key_frame_num");
}

void Frame::setId(int id) {
    this->id = id;
}

std::shared_ptr<std::vector<cv::DMatch>> Frame::matchFrames(std::shared_ptr<Frame> &f, double eta) {
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::makePtr<cv::FlannBasedMatcher>(
            cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
    std::shared_ptr<std::vector<cv::DMatch>> matches = std::make_shared<std::vector<cv::DMatch>>();

    matcher->match(descriptor, f->descriptor, *matches);

    std::shared_ptr<std::vector<cv::DMatch>> good_matches = std::make_shared<std::vector<cv::DMatch>>();
    double max_distance = 0;
    for (unsigned int i = 0; i < matches->size(); ++i) {
        max_distance = std::max(max_distance, (double) (*matches)[i].distance);
    }
    for (unsigned int i = 0; i < matches->size(); ++i) {
        if ((*matches)[i].distance < max_distance * eta)
            good_matches->push_back((*matches)[i]);
    }

    return good_matches;
}

void Frame::setCamera(const Camera &camera) {
    this->camera = camera;
}

double Frame::getDepthValue(int x, int y) {
    double value = this->depth_image.at<double>(y, x);
    double depth = value / this->depth_factor;

    if (depth < this->min_depth or depth > max_depth) {
        return -1;
    }

    return depth;
}

Eigen::Vector3d Frame::getWorldPoint(int x, int y) {
    double depth = getDepthValue(x, y);

    if (depth < 0) {
        return Eigen::Vector3d::Identity();
    } else {
        return this->camera.pix2World(Eigen::Vector2d(x, y), depth);
    }
}

Eigen::Vector3d Frame::getWorldPoint(int index) {
    return this->world_points[index];
}

const cv::Mat &Frame::getColorImage() const {
    return color_image;
}

const cv::Mat &Frame::getDepthImage() const {
    return depth_image;
}

void Frame::setColorImage(const cv::Mat &colorImage) {
    color_image = colorImage;
}

void Frame::setDepthImage(const cv::Mat &depthImage) {
    depth_image = depthImage;
}

Frame::Frame() {
    this->min_depth = Config::getData<double>("min_depth");
    this->max_depth = Config::getData<double>("max_depth");
    this->depth_factor = Config::getData<double>("depth_factor");
    this->key_frame_num = Config::getData<int>("key_frame_num");
}