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

bool Frontend::addFrame(const Frame &frame) {

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
    this->tfs.emplace_back(Eigen::Matrix4d::Identity());
    current_frame_ptr->computeORB();


    this->status = Status::TRACKING_GOOD;

    std::cout << "Frontend Init Successfully!!!" << std::endl;
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

    this->interiors_threshold_bad = Config::getData<int>("interiors_threshold_bad");
    this->interiors_threshold_lost = Config::getData<int>("interiors_threshold_lost");

}

int Frontend::estimateCurrentPose() {

    this->current_frame_ptr->computeORB();

    //TODO 这里还有一个参数需要放到配置文件中
    auto matches = current_frame_ptr->matchFrames(last_frame_ptr, 0.4);

    // 我先写一个完整的流程
    // 然后再去考虑匹配不好的步骤

    std::vector<Eigen::Vector3d> point_current;
    std::vector<Eigen::Vector3d> point_last;

    for (auto &match : *matches) {
        point_current.push_back(current_frame_ptr->getWorldPoint(match.queryIdx));
        point_last.push_back(last_frame_ptr->getWorldPoint(match.queryIdx));
    }

    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    estimateRigid3D(point_last, point_current, R, t);

    std::vector<int> inlines;
    this->estimateRT(point_last, point_current, R, t);
    last_frame_ptr = current_frame_ptr;
    Eigen::Matrix4d calculate_tf = Eigen::Matrix4d::Identity();
    if (t.norm() < 0.15) {
        calculate_tf.block<3, 3>(0, 0) = R;
        calculate_tf.block<3, 1>(0, 3) = t;
    }
    this->tfs.emplace_back(calculate_tf);

    return matches->size();
}

void Frontend::setCamera(const Camera &config) {
    //TODO
//    Frontend::camera = config;
    this->camera = config;
}

bool Frontend::estimateRT(std::vector<Eigen::Vector3d> last,
                          std::vector<Eigen::Vector3d> current,
                          Eigen::Matrix3d &R,
                          Eigen::Vector3d &t) {
    srand(time(nullptr));
    std::vector<int> inlines;

    if (last.size() < 10)
        return false;
    int max_iters = 1000;
    double error = 0.01;
    for (int i = 0; i < max_iters; i++) {
        int i1 = rand() % last.size();
        int i2 = rand() % last.size();
        int i3 = rand() % last.size();
        int i4 = rand() % last.size();
        int i5 = rand() % last.size();
        int i6 = rand() % last.size();
        int i7 = rand() % last.size();
        int i8 = rand() % last.size();
        int i9 = rand() % last.size();
        int i10 = rand() % last.size();

        std::vector<Eigen::Vector3d> part1, part2;
        part1.emplace_back(last[i1]);
        part1.emplace_back(last[i2]);
        part1.emplace_back(last[i3]);
        part1.emplace_back(last[i4]);
        part1.emplace_back(last[i5]);
        part1.emplace_back(last[i6]);
        part1.emplace_back(last[i7]);
        part1.emplace_back(last[i8]);
        part1.emplace_back(last[i9]);
        part1.emplace_back(last[i10]);

        part2.emplace_back(current[i1]);
        part2.emplace_back(current[i2]);
        part2.emplace_back(current[i3]);
        part2.emplace_back(current[i4]);
        part2.emplace_back(current[i5]);
        part2.emplace_back(current[i6]);
        part2.emplace_back(current[i7]);
        part2.emplace_back(current[i8]);
        part2.emplace_back(current[i9]);
        part2.emplace_back(current[i10]);

        estimateRigid3D(part1, part2, R, t);
        if (estimateReprojection(part1, part2, R, t, {}) <= error) {
            inlines.resize(last.size(), 0);
            part1.clear();
            part2.clear();
            for (int j = 0; j < last.size(); j++)
                if ((R * last[j] + t - current[j]).norm() <= error) {
                    inlines[j] = 1;
                    part1.emplace_back(last[j]);
                    part2.emplace_back(current[j]);
                }
            if (part1.size() < 10)
                continue;
            estimateRigid3D(part1, part2, R, t);
            if (estimateReprojection(last, current, R, t, inlines) <= error)
                return true;
        }
    }
    R = Eigen::Matrix3d::Identity();
    t = Eigen::Vector3d::Zero();
    return false;
}

double Frontend::estimateReprojection(std::vector<Eigen::Vector3d> &last,
                                      std::vector<Eigen::Vector3d> &current,
                                      Eigen::Matrix3d &R,
                                      Eigen::Vector3d &t,
                                      std::vector<int> inlines) {
    if (inlines.empty()) {
        double re = 0;
        for (int i = 0; i < last.size(); i++)
            re += (R * last[i] + t - current[i]).norm();
        re /= last.size();
        return re;
    } else {
        double re = 0;
        int j = 0;
        for (int i = 0; i < last.size(); i++)
            if (inlines[i]) {
                re += (R * last[i] + t - current[i]).norm();
                j++;
            }
        re /= j;
        return re;
    }
}

void Frontend::estimateRigid3D(std::vector<Eigen::Vector3d> &last,
                               std::vector<Eigen::Vector3d> &current,
                               Eigen::Matrix3d &R, Eigen::Vector3d &t) {
    std::vector<Eigen::Vector3d> q1, q2;
    Eigen::Vector3d c1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d c2 = Eigen::Vector3d::Zero();
    size_t n = last.size();
    for (size_t i = 0; i < n; i++) {
        c1 += last[i];
        c2 += current[i];
    }
    c1 /= (int) n;
    c2 /= (int) n;

    for (size_t i = 0; i < n; i++) {
        q1.emplace_back(last[i] - c1);
        q2.emplace_back(current[i] - c2);
    }

    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < n; i++)
        W += q1[i] * q2[i].transpose();

    Eigen::JacobiSVD<Eigen::MatrixXd>
            svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    R = U * V.transpose();
    if (R.determinant() < 0.5) {
        // R = Eigen::Matrix3d::Identity();
        // t = Eigen::Vector3d::Zero();
        R = -R;
        //return;
    }
    t = c1 - R * c2;
}

