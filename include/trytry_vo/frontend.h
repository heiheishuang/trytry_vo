//
// Created by heihei on 2021/12/3.
//

#ifndef TRYTRY_VO_FRONTEND_H
#define TRYTRY_VO_FRONTEND_H

#include "trytry_vo/config.h"
#include "trytry_vo/images.h"
#include "trytry_vo/camera.h"
#include "trytry_vo/frame.h"
#include "trytry_vo/viewer.h"

#include <vector>
#include <Eigen/Dense>
#include <opencv2/calib3d/calib3d.hpp>
#include <sophus/so3.hpp>

enum class Status {
    INIT,
    TRACKING_GOOD,
    TRACKING_BAD,
    LOST
};

class Frontend {
public:
    Frontend();

    ~Frontend();

    bool addFrame(const Frame &frame);

    bool Init();

    bool Track();

    bool Reset();

    int estimateCurrentPose();

    //Viewer
    void toViewer();

    // Return RT RANSAC with SVD
    void estimateRT_RANSAC(std::vector<Eigen::Vector3d> last, std::vector<Eigen::Vector3d> current,
                           Eigen::Matrix3d &R, Eigen::Vector3d &t) const;

    // Return RT BA
    void estimateRT_BA(std::vector<Eigen::Vector3d> last, std::vector<Eigen::Vector3d> current,
                       Eigen::Matrix3d &R, Eigen::Vector3d &t) const;

    // Return RT opencv PnP
    void estimateRT_PNP(std::shared_ptr<std::vector<cv::DMatch>> &matches,
                        Eigen::Matrix3d &R, Eigen::Vector3d &t) const;

    // Return RT BA
    static void estimateRigid3D(std::vector<Eigen::Vector3d> &last, std::vector<Eigen::Vector3d> &current,
                                Eigen::Matrix3d &R, Eigen::Vector3d &t);

    static double estimateReprojection(std::vector<Eigen::Vector3d> &last, std::vector<Eigen::Vector3d> &current,
                                       Eigen::Matrix3d &R, Eigen::Vector3d &t);

    const std::vector<Eigen::Matrix4d> &getTf() const;

    const std::vector<std::shared_ptr<Frame>> &getFrames() const;

    const std::shared_ptr<Frame> &getLastFramePtr() const;

    const std::shared_ptr<Frame> &getCurrentFramePtr() const;

    void setCamera(const Camera &config);

    Status getStatus() const;

private:
    Camera camera;

    std::vector<Eigen::Matrix4d> tfs{};
    std::vector<std::shared_ptr<Frame>> frames{};

    std::shared_ptr<Frame> last_frame_ptr{};
    std::shared_ptr<Frame> current_frame_ptr{};

    Status status{};

    int interiors_threshold_bad{};
    int interiors_threshold_lost{};
    int match_size_threshold{};

    int estimateEnoughMatches(std::shared_ptr<std::vector<cv::DMatch>> &matches);

    int estimateNotEnoughMatches(std::shared_ptr<std::vector<cv::DMatch>> &matches);

    int computeInlines(std::vector<Eigen::Vector3d> &last, std::vector<Eigen::Vector3d> &current,
                       Eigen::Matrix3d &R, Eigen::Vector3d &t) const;

    int error_count{};

    double reprojection_error_threshold{};
    int ransac_max_iterators;

    double match_distance_eta;
};


#endif //TRYTRY_VO_FRONTEND_H
