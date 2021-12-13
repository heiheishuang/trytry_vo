//
// Created by heihei on 2021/12/13.
//

#ifndef TRYTRY_VO_VIEWER_H
#define TRYTRY_VO_VIEWER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <vector>
#include <boost/thread/mutex.hpp>

class Viewer {
public:
    static Viewer *getInstance();

    void setTopicName(const std::string &path_topic_name);

    virtual ~Viewer();

    void publishPath();

    void addTF(Eigen::Matrix4d &tf);

private:
    Viewer();

    Viewer(const Viewer &);

    Viewer &operator=(const Viewer &);

    static Viewer *instance;
    static boost::mutex mutex_instance;


    ros::NodeHandle node_handle;

    std::vector<Sophus::SE3d> tfs;

    ros::Publisher pub_path;
    ros::Publisher pub_map;

};


#endif //TRYTRY_VO_VIEWER_H
