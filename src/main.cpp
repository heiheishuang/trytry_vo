//
// Created by heihei on 2021/12/1.
//
#include <ros/ros.h>
#include "trytry_vo/vo.h"

int main(int argc, char *argv[]) {
//    ros::init(argc, argv, "try_vo");

    Vo vo;

    assert(vo.Init() == true);

    std::cout << "Vo Init successfully!!!" << std::endl;

    vo.Run();

    std::cout << "Vo Stop! " << std::endl;
    std::cout << "heihei successfully!!! " << std::endl;

    return 0;

}
