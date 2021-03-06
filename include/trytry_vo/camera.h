//
// Created by heihei on 2021/12/1.
//

#ifndef TRYTRY_VO_CAMERA_H
#define TRYTRY_VO_CAMERA_H

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

class Camera {

public:

    Camera();

    Camera(const Camera &camera);

    explicit Camera(double fx, double fy, double cx, double cy);

    Eigen::Vector3d pix2World(Eigen::Vector2d pix_pos, double depth);

    Eigen::Vector2d world2Pix(const Eigen::Vector3d &world_pos);

    const Eigen::Matrix3d &getCameraK() const;

    cv::Mat getMatK() const;
private:

    Eigen::Matrix3d K{};

};


#endif //TRYTRY_VO_CAMERA_H
