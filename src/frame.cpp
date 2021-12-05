//
// Created by heihei on 2021/12/1.
//

#include <opencv2/features2d.hpp>
#include "trytry_vo/frame.h"

const cv::Mat &Frame::getColorImage() const {
    return color_image;
}

const cv::Mat &Frame::getDepthImage() const {
    return depth_image;
}

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

void Frame::setColorImage(const cv::Mat &colorImage) {
    color_image = colorImage;
}

void Frame::setDepthImage(const cv::Mat &depthImage) {
    depth_image = depthImage;
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
    std::vector<cv::KeyPoint> temp_key_points;
    cv::Mat temp_descriptor;

    detector->detectAndCompute(color_image, cv::Mat(), temp_key_points, temp_descriptor);

    for (int i = 0; i < temp_key_points.size(); i++) {
        double tempDepth = this->getDepthValue((int) temp_key_points[i].pt.x, (int) temp_key_points[i].pt.y);
        if (tempDepth < min_depth || tempDepth > max_depth) {
            continue;
        } else {
            Eigen::Vector3f world_point = this->camera.pix2World(
                    Eigen::Vector2f(temp_key_points[i].pt.x, temp_key_points[i].pt.y), tempDepth);

            this->key_points.push_back(temp_key_points[i]);
            this->descriptor.push_back(temp_descriptor.row(i));
            this->world_points.emplace_back(world_point);
        }
    }
}

Frame::Frame(int id) {
    this->id = id;

    this->min_depth = Config::getInstance()->getData<float>("min_depth");
    this->max_depth = Config::getInstance()->getData<float>("max_depth");
    this->depth_factor = Config::getInstance()->getData<float>("depth_factor");
    this->key_frame_num = Config::getInstance()->getData<int>("key_frame_num");
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
