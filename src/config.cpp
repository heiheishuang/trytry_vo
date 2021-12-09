//
// Created by heihei on 2021/12/2.
//

#include "../include/trytry_vo/config.h"

#include <utility>

Config::~Config() {
    delete Config::instance;
}

Config::Config() {
//    this->file_storage = cv::FileStorage(Config::file_name, cv::FileStorage::READ);
}
Config* Config::instance = nullptr;
boost::mutex Config::mutex_instance;

Config* Config::getInstance() {

    if (instance == nullptr) {
        boost::unique_lock<boost::mutex> uq_lock_instance(mutex_instance);
        if (instance == nullptr) {
            instance = new Config();
        }
    }
    return instance;
}

Config::Config(const Config &) {

}

Config &Config::operator=(const Config &) {

}

void Config::setFileName(const std::string& path) {

    file_name = path;

    this->file_storage = cv::FileStorage(Config::file_name, cv::FileStorage::READ);

}




