//
// Created by heihei on 2021/12/1.
//
#include <ros/ros.h>
#include "trytry_vo/vo.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "trytry_vo");

    Config::getInstance()->setFileName("/home/heihei/slam_ws/src/trytry_vo/TUM1.yaml");

    std::string topic_name = Config::getData<std::string>("topic_name");
    Viewer::getInstance()->setTopicName(topic_name);

    // Init Pose
    Eigen::Quaterniond quaternion = Eigen::Quaterniond(Config::getData<double>("rotation_quaternion_w"),
                                                       Config::getData<double>("rotation_quaternion_x"),
                                                       Config::getData<double>("rotation_quaternion_y"),
                                                       Config::getData<double>("rotation_quaternion_z"));
    Eigen::Vector3d vector = Eigen::Vector3d(Config::getData<double>("translation_x"),
                                             Config::getData<double>("translation_y"),
                                             Config::getData<double>("translation_z"));
    Viewer::getInstance()->setInitPose(quaternion, vector, Config::getData<double>("timestamp"));

    Vo vo;

    assert(vo.Init() == true);

    std::cout << "Vo Init successfully!!!" << std::endl;

    vo.Run();

    std::cout << "Vo Stop! " << std::endl;
    std::cout << "heihei successfully!!! " << std::endl;


    // write odometer data
    std::string write_file = Config::getData<std::string>("write_file");
    if (write_file == "T") {
        std::string file_name = Config::getData<std::string>("file_name");
        Viewer::getInstance()->writeFile(file_name);
    }

    return 0;

}
