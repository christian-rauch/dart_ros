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

dart::SE3 JointProviderROS::getTransform(const std::string frame1, const std::string frame2) {
    tf::StampedTransform transform;
    do {
        try{
            listener.lookupTransform(frame2, frame1, ros::Time(), transform);
        }
        catch(tf::TransformException){
          ros::Duration(0.01).sleep();
        }
    } while(transform.frame_id_.empty());
    const tf::Matrix3x3 R = transform.getBasis();
    const tf::Vector3 t = transform.getOrigin();

    dart::SE3 T;
    T.r0 = make_float4(R[0].x(), R[0].y(), R[0].z(), t.x());
    T.r1 = make_float4(R[1].x(), R[1].y(), R[1].z(), t.y());
    T.r2 = make_float4(R[2].x(), R[2].y(), R[2].z(), t.z());

    return T;
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
