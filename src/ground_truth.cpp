//
// Created by heihei on 2021/12/15.
//
#include "trytry_vo/viewer.h"
#include "trytry_vo/config.h"
#include <algorithm>
#include <fstream>
#include <string>

void LoadImage(std::vector<double> &vTimestamps,
               std::vector<Eigen::Matrix4d> &tf, std::string &path) {
    std::ifstream file_in;
    file_in.open(path + "groundtruth.txt");
    while (!file_in.eof()) {
        std::string s;
        getline(file_in, s);
        if (!s.empty()) {
            if (s.at(0) == '#') {
                continue;
            }
            std::stringstream ss;
            ss << s;
            double t;
            float tx, ty, tz, qx, qy, qz, qw;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
            Eigen::Quaterniond q = Eigen::Quaterniond(qw, qx, qy, qz);
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            Eigen::Matrix3d R = q.normalized().toRotationMatrix();
            T.block<3, 3>(0, 0) = R;
            T.block<3, 1>(0, 3) = Eigen::Vector3d(tx, ty, tz);
            tf.emplace_back(T);
        }
    }
    file_in.close();
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "viewer");

    Config::getInstance()->setFileName("/home/heihei/slam_ws/src/trytry_vo/TUM1.yaml");
    Viewer::getInstance()->setTopicName("ground_truth");

    std::string path = Config::getData<std::string>("data_set");

    std::vector<double> time_stamps;
    std::vector<Eigen::Matrix4d> tfs;
    LoadImage(time_stamps, tfs, path);

    for (size_t i = 0; i < time_stamps.size(); i++) {
        Viewer::getInstance()->addAbsoluteTF(tfs[i]);
    }

    while (true) {
        Viewer::getInstance()->publishPath();
    }

    return 0;
}
