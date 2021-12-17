//
// Created by heihei on 2021/12/3.
//

#include <iostream>

#include "trytry_vo/frontend.h"
#include "trytry_vo/g2o_types.h"

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

    current_frame_ptr->computeORB();

    last_frame_ptr = current_frame_ptr;
    this->tfs.emplace_back(Eigen::Matrix4d::Identity());

    this->status = Status::TRACKING_GOOD;

    std::cout << "Frontend Init Successfully!!!" << std::endl;
    return true;
}

bool Frontend::Track() {

    this->current_frame_ptr->computeORB();

    int interiors = this->estimateCurrentPose();

    if (interiors < this->interiors_threshold_lost) {
        this->status = Status::LOST;
        std::cout << " LOST ";
    } else if (interiors < this->interiors_threshold_bad) {
        this->status = Status::TRACKING_BAD;
        std::cout << " TRACKING_BAD ";
    } else {
        this->status = Status::TRACKING_GOOD;
        std::cout << " TRACKING_GOOD ";
    }

    this->toViewer();

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
    this->match_size_threshold = Config::getData<int>("match_size_threshold");
    this->reprojection_error_threshold = Config::getData<double>("reprojection_error_threshold");
    this->ransac_max_iterators = Config::getData<int>("ransac_max_iterators");
    this->match_distance_eta = Config::getData<double>("match_distance_eta");
}

int Frontend::estimateCurrentPose() {

    auto matches = current_frame_ptr->matchFrames(last_frame_ptr, match_distance_eta);

    int inlines;
    if (matches->size() < this->match_size_threshold) {
        std::cout << "matches size " << matches->size() << " ";
        inlines = estimateNotEnoughMatches(matches);
    } else {
        inlines = estimateEnoughMatches(matches);
        if (inlines == 0) {
            inlines = estimateNotEnoughMatches(matches);
        }
        std::cout << "inline " << inlines << " ";
    }
    return inlines;
}

void Frontend::setCamera(const Camera &config) {
    this->camera = config;
}

int Frontend::estimateRT_RANSAC(std::vector<Eigen::Vector3d> last,
                                std::vector<Eigen::Vector3d> current,
                                Eigen::Matrix3d &R,
                                Eigen::Vector3d &t) const {
    srand(time(nullptr));
    std::vector<int> inlines;

    for (int i = 0; i < ransac_max_iterators; i++) {

        std::vector<Eigen::Vector3d> part1, part2;

        for (int k = 0; k < 10; k++) {
            int index = rand() % last.size();
            part1.emplace_back(last.at(index));
            part2.emplace_back(current.at(index));
        }

        inlines.clear();

        estimateRigid3D(part1, part2, R, t);
        if (estimateReprojection(part1, part2, R, t) <= reprojection_error_threshold) {
            part1.clear();
            part2.clear();
            for (int j = 0; j < last.size(); j++) {
                if ((R * last[j] + t - current[j]).norm() <= reprojection_error_threshold) {
                    inlines.push_back(j);
                    part1.emplace_back(last[j]);
                    part2.emplace_back(current[j]);
                }
            }
            if (part1.size() < 10) {
                continue;
            }
            estimateRigid3D(part1, part2, R, t);
            if (estimateReprojection(part1, part2, R, t) <= reprojection_error_threshold) {
                return (int) inlines.size();
            }
        }
    }
    R = Eigen::Matrix3d::Identity();
    t = Eigen::Vector3d::Zero();
    return (int) inlines.size();
}

double Frontend::estimateReprojection(std::vector<Eigen::Vector3d> &last,
                                      std::vector<Eigen::Vector3d> &current,
                                      Eigen::Matrix3d &R,
                                      Eigen::Vector3d &t) {
    double reprojection_error = 0;
    for (int i = 0; i < last.size(); i++) {
        reprojection_error += (R * last[i] + t - current[i]).norm();
    }
    reprojection_error /= (int) last.size();
    return reprojection_error;
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
        R = -R;
    }
    t = c1 - R * c2;
}

int Frontend::estimateNotEnoughMatches(std::shared_ptr<std::vector<cv::DMatch>> &matches) {

    std::vector<Eigen::Vector3d> point_current;
    std::vector<Eigen::Vector3d> point_last;

    for (auto &match : *matches) {
        point_current.push_back(current_frame_ptr->getWorldPoint(match.queryIdx));
        point_last.push_back(last_frame_ptr->getWorldPoint(match.trainIdx));
    }


    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();

    Eigen::Matrix4d calculate_tf = Eigen::Matrix4d::Identity();
    calculate_tf.block<3, 3>(0, 0) = R;
    calculate_tf.block<3, 1>(0, 3) = t;
    this->tfs.emplace_back(calculate_tf);

    int inlines = 0;
    for (int j = 0; j < point_last.size(); j++)
        if ((R * point_last[j] + t - point_current[j]).norm() <= reprojection_error_threshold) {
            inlines++;
        }

    this->last_frame_ptr = this->current_frame_ptr;
    return inlines;
}

int Frontend::estimateEnoughMatches(std::shared_ptr<std::vector<cv::DMatch>> &matches) {

    std::vector<Eigen::Vector3d> point_current;
    std::vector<Eigen::Vector3d> point_last;

    for (auto &match : *matches) {
        point_current.push_back(current_frame_ptr->getWorldPoint(match.queryIdx));
        point_last.push_back(last_frame_ptr->getWorldPoint(match.trainIdx));
    }

    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    int inlines = this->estimateRT_RANSAC(point_last, point_current, R, t);
//    int inlines = this->estimateRT_PNP(matches, R, t);

    if (inlines != 0) {
        Eigen::Matrix4d calculate_tf = Eigen::Matrix4d::Identity();
        calculate_tf.block<3, 3>(0, 0) = R;
        calculate_tf.block<3, 1>(0, 3) = t;
        this->tfs.emplace_back(calculate_tf);
    }

    last_frame_ptr = current_frame_ptr;

    return inlines;
}

void Frontend::toViewer() {

    Viewer::getInstance()->addRelativeTF(this->tfs.back());

    Viewer::getInstance()->publishPath();
}

int Frontend::estimateRT_PNP(std::shared_ptr<std::vector<cv::DMatch>> &matches,
                             Eigen::Matrix3d &R,
                             Eigen::Vector3d &t) {

    std::vector<cv::Point3d> point_last;
    std::vector<cv::Point2d> point_current;
    for (auto &match : *matches) {
        Eigen::Vector3d last = last_frame_ptr->getWorldPoint(match.trainIdx);
        point_last.emplace_back(last.x(), last.y(), last.z());
        cv::KeyPoint current = current_frame_ptr->getKeyPoint(match.queryIdx);
        point_current.emplace_back(current.pt.x, current.pt.y);
    }

    cv::Mat R_vec, t_vec, inlines;
    cv::solvePnPRansac(point_last, point_current,
                       this->camera.getMatK(), cv::Mat(),
                       R_vec, t_vec,
                       false, 1000, 0.5, 0.99,
                       inlines);
    cv::Rodrigues(R_vec, R_vec);

    cv::cv2eigen(R_vec, R);
    cv::cv2eigen(t_vec, t);

    return inlines.rows;
}

int Frontend::estimateRT_BA(std::vector<Eigen::Vector3d> last, std::vector<Eigen::Vector3d> current, Eigen::Matrix3d &R,
                            Eigen::Vector3d &t) const {
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(
                    g2o::make_unique<LinearSolverType>()
            )
    );

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    auto *vertex_pose = new VertexPose();
    vertex_pose->setId(0);
    Eigen::Matrix3d current_R = this->tfs.back().block<3, 3>(0, 0).matrix();
    Eigen::Vector3d current_t = this->tfs.back().block<3, 1>(0, 3).matrix();
    Sophus::SE3d current_pose(R, t);
    vertex_pose->setEstimate(current_pose);
    optimizer.addVertex(vertex_pose);

    Eigen::Matrix3d K = camera.getCameraK();

    int index = 1;
    for (int i = 0; i < current.size(); i++) {
        auto *edge = new EdgeProjectionPoseOnly(last.at(i));
        edge->setId(index);
        edge->setVertex(0, vertex_pose);
        edge->setMeasurement(current.at(i));
        edge->setInformation(Eigen::Matrix3d::Identity());
        optimizer.addEdge(edge);
        index++;
    }

    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);


    Sophus::SE3d pose;
    pose = vertex_pose->estimate();

    R = pose.rotationMatrix();
    t = pose.translation();

    int inlines;
    for (int j = 0; j < last.size(); j++) {
        if ((R * last[j] + t - current[j]).norm() <= reprojection_error_threshold) {
            inlines++;
        }
    }

    return 0;
}
