//
// Created by heihei on 2021/12/17.
//

#ifndef TRYTRY_VO_G2O_TYPES_H
#define TRYTRY_VO_G2O_TYPES_H


#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include "g2o/types/slam3d/se3quat.h"
#include "g2o/core/base_vertex.h"

#include <sophus/se3.hpp>

/// 位姿顶点
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override {
        _estimate = Sophus::SE3d();
    }

    /// left multiplication on SE3
    virtual void oplusImpl(const double *update) override {
        Eigen::Matrix<double, 6, 1> update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }

    virtual bool read(std::istream &in) override {}

    virtual bool write(std::ostream &out) const override {}
};

class VertexSE3Expmap : public g2o::BaseVertex<6, g2o::SE3Quat> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setToOriginImpl() override { _estimate = g2o::SE3Quat(); }

    void oplusImpl(const double *update_) override {

        Eigen::Map<const g2o::Vector6> update(update_);
        _estimate = g2o::SE3Quat::exp(update) * _estimate;
    }

    bool read(std::istream &in) override { return true; }

    bool write(std::ostream &out) const override { return true; }

};

/// 路标顶点
class VertexXYZ : public g2o::BaseVertex<3, Eigen::Matrix<double, 3, 1>> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override { _estimate = Eigen::Matrix<double, 3, 1>::Zero(); }

    virtual void oplusImpl(const double *update) override {
        _estimate[0] += update[0];
        _estimate[1] += update[1];
        _estimate[2] += update[2];
    }

    virtual bool read(std::istream &in) override { return true; }

    virtual bool write(std::ostream &out) const override { return true; }
};

/// 仅估计位姿的一元边
class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjectionPoseOnly(const Eigen::Vector3d &point) : _point(point) {}

    virtual void computeError() override {
        const VertexPose *pose = static_cast<VertexPose *>(_vertices[0]);

        // measurement is p, point is p'
        _error = _measurement - pose->estimate() * _point;
    }

    virtual void linearizeOplus() {
        auto *pose = static_cast<VertexPose *>(_vertices[0]);
        Sophus::SE3d T = pose->estimate();
        Eigen::Vector3d xyz_trans = T * _point;

        _jacobianOplusXi.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
        _jacobianOplusXi.block<3, 3>(0, 3) = Sophus::SO3d::hat(xyz_trans);
    }

    virtual bool read(std::istream &in) override { return true; }

    virtual bool write(std::ostream &out) const override { return true; }

private:
    Eigen::Vector3d _point;
};

#endif //TRYTRY_VO_G2O_TYPES_H
