//
// Created by heihei on 2021/12/2.
//

#include "../include/trytry_vo/config.h"

#include <utility>

Config::~Config() {
    delete Config::instance;
}

Config::Config() {
    this->file_storage = cv::FileStorage(this->file_name, cv::FileStorage::READ);
}

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

}

template<typename T>
T Config::getData(const char *name) const {
    return static_cast<T>(file_storage[name]);
}




