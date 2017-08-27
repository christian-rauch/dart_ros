#ifndef JOINTPUBLISHERROS_HPP
#define JOINTPUBLISHERROS_HPP

#include <ros/ros.h>
#include <dart/pose/pose.h>

namespace dart {

class JointPublisherROS {
public:
    JointPublisherROS(const std::string topic);

    void publish(const dart::Pose &pose, const double time = 0) const;

private:
    ros::NodeHandle n;
    ros::Publisher pub;
};

} // namespace dart

#endif // JOINTPUBLISHERROS_HPP
