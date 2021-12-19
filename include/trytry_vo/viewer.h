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

#include <fstream>

class Viewer {
public:
    virtual ~Viewer();

    static Viewer *getInstance();

    void setTopicName(const std::string &path_topic_name);

    void setInitPose(const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation);

    void publishPath();

    void addRelativeTF(Eigen::Matrix4d &tf);

    void addAbsoluteTF(Eigen::Matrix4d &tf);

    void writeFile(const std::string &file_name);

private:
    // Constructor
    Viewer();

    Viewer(const Viewer &);

    Viewer &operator=(const Viewer &);

    // Singleton Pattern
    static Viewer *instance;
    static boost::mutex mutex_instance;

    // ROS
    ros::NodeHandle node_handle;
    ros::Publisher pub_path;
    ros::Publisher pub_map;
    ros::Publisher pub_pose;

    std::vector<Sophus::SE3d> tfs;
};


#endif //TRYTRY_VO_VIEWER_H
