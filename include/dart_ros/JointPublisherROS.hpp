#ifndef JOINTPUBLISHERROS_HPP
#define JOINTPUBLISHERROS_HPP

#include <ros/ros.h>
#include <dart/pose/pose.h>
#include <std_msgs/Header.h>

namespace dart {

class JointPublisherROS {
public:
    JointPublisherROS(const std::string topic);

    void publish(const dart::Pose &pose, const std_msgs::Header header = std_msgs::Header()) const;

    void publish(const dart::Pose &pose, const double time) const;

private:
    ros::NodeHandle n;
    ros::Publisher pub;
};

} // namespace dart

#endif // JOINTPUBLISHERROS_HPP
