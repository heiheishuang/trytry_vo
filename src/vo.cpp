//
// Created by heihei on 2021/12/1.
//

#include <iostream>
#include "trytry_vo/vo.h"
#include "trytry_vo/config.h"

Vo::~Vo() = default;

bool Vo::Init() {

    Config::getInstance()->setFileName("/home/heihei/slam_ws/src/trytry_vo/TUM1.yaml");

    std::string path = "/home/heihei/Downloads/rgbd_dataset_freiburg1_desk/";

    this->images.loadDepthImages(path);

    auto fx = Config::getData<double>("camera_fx");
    auto fy = Config::getData<double>("camera_fy");
    auto cx = Config::getData<double>("camera_cx");
    auto cy = Config::getData<double>("camera_cy");
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
    Frame frame;

   if (!this->images.getNextFrame(frame)) {
        return false;
    }

    // TODO 我需要找一个更好的位置来set camera
    // 首先是camera是否需要变成一个单例模式
    // 同时我也认为camera类可以拥有更多的属性，然后每一个frame都有只有一个camera
    frame.setCamera(camera);

    auto t1 = boost::chrono::steady_clock::now();

    if (frontend.addFrame(frame)) {
        auto t2 = boost::chrono::steady_clock::now();
        auto time_used = boost::chrono::duration_cast<boost::chrono::duration<double>>(t2 - t1);
        std::cout << "VO cost time: " << time_used.count() << " seconds." << std::endl;
        return true;
    } else {
        return false;
    }
}

Status Vo::getFrontStatus() const {
    return frontend.getStatus();
}

Vo::Vo() = default;
