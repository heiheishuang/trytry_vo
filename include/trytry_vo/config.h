//
// Created by heihei on 2021/12/2.
//

#ifndef TRYTRY_VO_CONFIG_H
#define TRYTRY_VO_CONFIG_H

#include <opencv2/core.hpp>
#include <boost/thread/mutex.hpp>

class Config {
public:
    static Config *getInstance();

    void setFileName(const std::string &path);

    template<typename T>
    static T getData(const char *name) {
        return T(Config::getInstance()->file_storage[name]);
    }

    virtual ~Config();

private:
    Config();

    Config(const Config &);

    Config &operator=(const Config &);

    static Config *instance;
    static boost::mutex mutex_instance;

    cv::FileStorage file_storage;
    std::string file_name;


};


#endif //TRYTRY_VO_CONFIG_H
