#include <dart_ros/JointProviderROS.hpp>

JointProviderROS::JointProviderROS() {

}

bool JointProviderROS::setJointNames(const dart::HostOnlyModel &model) {
    mutex.lock();
    joints.clear();
    for(uint j(0); j<model.getNumJoints(); j++) {
        joints[model.getJointName(j)] = 0;
    }
    mutex.unlock();
    return true;
}

bool JointProviderROS::subscribe_joints(const std::string joint_topic) {
    if(joints.empty()) {
        throw std::runtime_error("no joints specified, you should add joint names directly or by model using the .setJointNames() method");
        return false;
    }
    else {
        sub = n.subscribe(joint_topic, 1, &JointProviderROS::setJoints, this);
        return true;
    }
}

void JointProviderROS::setJoints(const sensor_msgs::JointStateConstPtr &msg_jnt) {
    mutex.lock();
    for(uint j(0); j<msg_jnt->name.size(); j++) {
        if(joints.count(msg_jnt->name[j])==1) {
            joints.at(msg_jnt->name[j]) = msg_jnt->position[j];
        }
    }
    mutex.unlock();
}
