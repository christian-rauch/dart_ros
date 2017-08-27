#ifndef FRAMEPOSEPUBLISHER_HPP
#define FRAMEPOSEPUBLISHER_HPP

#include <dart/model/model.h>
#include <dart/model/mirrored_model.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>

namespace dart {

class FramePosePublisher {
public:
    FramePosePublisher(const dart::Model &model_reported,
                       const dart::MirroredModel &model_estimated,
                       const std::string &topic);

    void publishFrame(const std::string &name, const std::string &camera_frame, const uint64_t time = 0);

private:
    static
    Eigen::Isometry3f convertPose(const dart::SE3 &dart_pose);

    static
    geometry_msgs::Pose convertPose(const Eigen::Isometry3d &eigen_pose);

    const dart::Model &model_reported;
    const dart::MirroredModel &model_estimated;

    ros::NodeHandle n;
    const ros::Publisher pub_pose_rep;
    const ros::Publisher pub_pose_est;
    const ros::Publisher pub_pose_diff;
    const ros::Publisher pub_pose_err;
};

} // namespace dart

#endif // FRAMEPOSEPUBLISHER_HPP
