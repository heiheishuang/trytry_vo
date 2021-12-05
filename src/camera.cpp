//
// Created by heihei on 2021/12/1.
//

#include "trytry_vo/camera.h"

#include <Eigen/Dense>

Camera::Camera(double fx, double fy, double cx, double cy) {

    this->K << fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0., 1.0;
}

Eigen::Vector2d Camera::world2Pix(const Eigen::Vector3d& world_pos) {
    Eigen::Vector3d temp;

    temp = K * world_pos;
    temp = temp / temp[2];
    Eigen::Vector2d pos(temp[0], temp[1]);

    return pos;
}

Eigen::Vector3d Camera::pix2World(Eigen::Vector2d pix_pos, double depth) {
    Eigen::Vector3d pos(pix_pos[0], pix_pos[1], 1);

    pos = pos * depth;
    pos = this->K.inverse() * pos;

    return pos;
}

Camera::Camera(const Camera &camera) {
    this->K = camera.K;
}

Camera::Camera() = default;
