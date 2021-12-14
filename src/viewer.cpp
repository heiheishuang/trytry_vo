//
// Created by heihei on 2021/12/13.
//

#include "trytry_vo/viewer.h"


Viewer::Viewer() = default;

void Viewer::publishPath() {
    nav_msgs::Path path;

    for (auto &tf : this->tfs) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = tf.translation().x();
        pose.pose.position.y = tf.translation().y();
        pose.pose.position.z = tf.translation().z();

        Eigen::Quaterniond quaternion;
        quaternion = tf.rotationMatrix();
        pose.pose.orientation.x = quaternion.x();
        pose.pose.orientation.y = quaternion.y();
        pose.pose.orientation.z = quaternion.z();
        pose.pose.orientation.w = quaternion.w();

        path.poses.push_back(pose);
    }

    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    this->pub_path.publish(path);

}

void Viewer::addTF(Eigen::Matrix4d &tf) {
    Eigen::Matrix3d R = tf.block<3, 3>(0, 0).matrix();
    Eigen::Vector3d t = tf.block<3, 1>(0, 3).matrix();
    Sophus::SE3d tSE3(R, t);
    if (this->tfs.empty())
        this->tfs.push_back(tSE3);
    else {
        Sophus::SE3d new_SE3 = this->tfs.back() * tSE3;
        this->tfs.push_back(new_SE3);
    }
}

Viewer *Viewer::getInstance() {
    if (instance == nullptr) {
        boost::unique_lock<boost::mutex> uq_lock_instance(mutex_instance);
        if (instance == nullptr) {
            instance = new Viewer();
        }
    }
    return instance;
}

Viewer::~Viewer() {
    delete Viewer::instance;
}

Viewer *Viewer::instance = nullptr;

void Viewer::setTopicName(const std::string &path_topic_name) {
    this->node_handle = ros::NodeHandle();
    this->pub_path = this->node_handle.advertise<nav_msgs::Path>(path_topic_name, 1000);
    this->pub_pose = this->node_handle.advertise<geometry_msgs::PoseStamped>("try_pose", 1000);

}

boost::mutex Viewer::mutex_instance;

Viewer::Viewer(const Viewer &) {

}

Viewer &Viewer::operator=(const Viewer &) {

}

void Viewer::setInitPose(const Eigen::Quaterniond &rotation,
                         const Eigen::Vector3d &translation) {


    Eigen::Matrix3d R = rotation.normalized().toRotationMatrix();
    Eigen::Vector3d t = translation;
    Sophus::SE3d initSE3(R, t);

    this->tfs.push_back(initSE3);

}
