#ifndef JOINTPROVIDERROS_HPP
#define JOINTPROVIDERROS_HPP

#include <map>
#include <string>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <dart/model/host_only_model.h>

class JointProviderROS {
public:
    JointProviderROS();

    bool setJointNames(const dart::HostOnlyModel &model);

    std::map<std::string, float>& getJoints() {
        return joints;
    }

    bool subscribe_joints(const std::string joint_topic);

private:
    void setJoints(const sensor_msgs::JointStateConstPtr &msg_jnt);

    std::map<std::string, float> joints;
    std::mutex mutex;

    ros::NodeHandle n;
    ros::Subscriber sub;
};

#endif // JOINTPROVIDERROS_HPP
