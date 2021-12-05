//
// Created by heihei on 2021/12/1.
//

#include <iostream>
#include "trytry_vo/vo.h"
#include "trytry_vo/config.h"

Vo::~Vo() = default;

bool Vo::Init() {

    Config::setFileName("/home/heihei/slam_ws/src/my_slam/TUM1.yaml");

    std::string path = "/home/heihei/Downloads/rgbd_dataset_freiburg1_desk/";

    this->images_ptr->loadDepthImages(path);

    auto fx = Config::getInstance()->getData<double>("Camera.fx");
    auto fy = Config::getInstance()->getData<double>("Camera.fy");
    auto cx = Config::getInstance()->getData<double>("Camera.cx");
    auto cy = Config::getInstance()->getData<double>("Camera.cy");
    this->camera = Camera(fx, fy, cx, cy);

    frontend.setCamera(this->camera);

    return true;
}

bool Vo::Run() {

    while (true) {
        std::cout << "Vo is running!!!" << std::endl;

        if (!this->Step()) {
            break;
        }
    }

    std::cout << "Vo stop running!" << std::endl;
    return true;
}

bool Vo::Step() {
    Frame frame = this->images_ptr->getNextFrame();
    // TODO 我需要找一个更好的位置来set camera
    // 首先是camera是否需要变成一个单例模式
    // 同时我也认为camera类可以拥有更多的属性，然后每一个frame都有只有一个camera
    frame.setCamera(camera);

    auto t1 = boost::chrono::steady_clock::now();

    if (frontend.addFrame(frame)) {
        auto t2 = boost::chrono::steady_clock::now();
        auto time_used = boost::chrono::duration_cast<boost::chrono::duration<double>>(t2 - t1);
        std::cout << "VO cost time: " << time_used.count() << " seconds.";
        return true;
    } else {
        return false;
    }
}

Status Vo::getFrontStatus() const {
    return frontend.getStatus();
}

Vo::Vo() = default;
