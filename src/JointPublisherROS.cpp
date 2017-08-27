#include <dart_ros/JointPublisherROS.hpp>
#include <sensor_msgs/JointState.h>

namespace dart {

JointPublisherROS::JointPublisherROS(const std::string topic) : n("~") {
    pub = n.advertise<sensor_msgs::JointState>(topic, 1);
}

void JointPublisherROS::publish(const dart::Pose &pose, const double time) const {
    sensor_msgs::JointState js;

    js.header.stamp = ros::Time(time);
    for(int j(0); j<pose.getReducedArticulatedDimensions(); j++) {
        js.name.push_back(pose.getReducedName(j));
        js.position.push_back(double(pose.getReducedArticulation()[j]));
    }

    pub.publish(js);
}

}
