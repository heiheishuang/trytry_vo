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
    std::vector<cv::KeyPoint> temp_key_points;
    cv::Mat temp_descriptor;
    detector->detectAndCompute(this->color_image, cv::Mat(), temp_key_points, temp_descriptor);
    for (size_t i = 0; i < temp_key_points.size(); i++) {
        double temp_depth = getDepthValue((int) temp_key_points.at(i).pt.x, (int) temp_key_points.at(i).pt.y);
        if (temp_depth < this->min_depth || temp_depth > this->max_depth) {
            continue;
        }
        Eigen::Vector3d world_point = this->camera.pix2World(
                Eigen::Vector2d(temp_key_points.at(i).pt.x, temp_key_points.at(i).pt.y), temp_depth);

        this->key_points.push_back(temp_key_points.at(i));
        this->descriptor.push_back(temp_descriptor.row((int) i));
        this->world_points.push_back(world_point);
    }
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

    matcher->match(this->descriptor, f->descriptor, *matches);

    std::shared_ptr<std::vector<cv::DMatch>> good_matches = std::make_shared<std::vector<cv::DMatch>>();
    double max_distance = 0;
    for (int i = 0; i < matches->size(); ++i) {
        max_distance = std::max(max_distance, (double) (*matches).at(i).distance);
    }
    for (int i = 0; i < matches->size(); ++i) {
        if ((*matches).at(i).distance < max_distance * eta)
            good_matches->push_back((*matches).at(i));
    }

    return good_matches;
}

void Frame::setCamera(const Camera &camera) {
    this->camera = camera;
}

double Frame::getDepthValue(int x, int y) {
    short value = this->depth_image.at<short>(y, x);
    double depth = value / this->depth_factor;

    if (depth < this->min_depth or depth > this->max_depth) {
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

cv::KeyPoint Frame::getKeyPoint(int index) {
    return this->key_points[index];
}

int Frame::getId() const {
    return this->id;
}
