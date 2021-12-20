//
// Created by heihei on 2021/12/1.
//
#include <ros/ros.h>
#include "trytry_vo/vo.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "trytry_vo");

    Config::getInstance()->setFileName("/home/heihei/slam_ws/src/trytry_vo/TUM1.yaml");
    Viewer::getInstance()->setTopicName("PNP5");

    // Init Pose
    Eigen::Quaterniond quaternion = Eigen::Quaterniond(-0.3909, 0.8851, 0.2362, -0.0898);
    Eigen::Vector3d vector = Eigen::Vector3d(1.3112, 0.8507, 1.5186);
    Viewer::getInstance()->setInitPose(quaternion, vector, 1305031453.3595);

    Vo vo;

    assert(vo.Init() == true);

    std::cout << "Vo Init successfully!!!" << std::endl;

    vo.Run();

    std::cout << "Vo Stop! " << std::endl;
    std::cout << "heihei successfully!!! " << std::endl;

    Viewer::getInstance()->writeFile("PNP5.txt");

    return 0;

}
