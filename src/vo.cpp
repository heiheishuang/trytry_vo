//
// Created by heihei on 2021/12/1.
//

#include <iostream>
#include "trytry_vo/vo.h"
#include "trytry_vo/config.h"

Vo::~Vo() = default;

bool Vo::Init() {

    auto path = Config::getData<std::string>("data_set");

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

    std::cout << "Vo is running!!!" << std::endl;

    while (true) {
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
