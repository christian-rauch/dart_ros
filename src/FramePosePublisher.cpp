#include <dart_ros/FramePosePublisher.hpp>
#include <image_classification_msgs/PoseErrorMagnitude.h>

namespace dart {

typedef Eigen::Transform<float, 3, Eigen::Isometry, Eigen::RowMajor> Isometry3frm;

FramePosePublisher::FramePosePublisher(
        const dart::Model &model_reported,
        const dart::MirroredModel &model_estimated,
        const std::string &topic) :
    model_reported(model_reported),
    model_estimated(model_estimated),
    n("~"),
    pub_pose_rep(n.advertise<geometry_msgs::PoseStamped>(topic+"/rep", 1)),
    pub_pose_est(n.advertise<geometry_msgs::PoseStamped>(topic+"/est", 1)),
    pub_pose_diff(n.advertise<geometry_msgs::PoseStamped>(topic+"/diff", 1)),
    pub_pose_err(n.advertise<image_classification_msgs::PoseErrorMagnitude>(topic+"/err", 1))

{

}

void FramePosePublisher::publishFrame(const std::string &name, const std::string &camera_frame, const uint64_t time) {
    // this is the transformation from camera to frame
    // e.g. it provides the pose of requested frame in the camera frame
    const Eigen::Isometry3f pose_est = convertPose(model_estimated.getTransformFrameToCamera(int(model_estimated.getFrameIdByName(name))));
    const Eigen::Isometry3f pose_rep = convertPose(model_reported.getTransformFrameToCamera(int(model_reported.getFrameIdByName(name))));

    std_msgs::Header header;
    header.frame_id = camera_frame;
    header.stamp.fromNSec(time);

    // publish reported pose
    geometry_msgs::PoseStamped pose_est_rep;
    pose_est_rep.header = header;
    pose_est_rep.pose = convertPose(pose_rep.cast<double>());
    pub_pose_rep.publish(pose_est_rep);

    // publish estimated pose
    geometry_msgs::PoseStamped pose_est_msg;
    pose_est_msg.header = header;
    pose_est_msg.pose = convertPose(pose_est.cast<double>());
    pub_pose_est.publish(pose_est_msg);

    // publish estimation error, e.g. the transformation that needs to be applied
    // to the estimated pose to obtain the reported pose
    const Eigen::Isometry3f pose_diff = pose_est.inverse() * pose_rep;
    geometry_msgs::PoseStamped pose_diff_msg;
    pose_diff_msg.header = header;
    pose_diff_msg.pose = convertPose(pose_diff.cast<double>());
    pub_pose_diff.publish(pose_diff_msg);

    image_classification_msgs::PoseErrorMagnitude err;
    err.header = header;
    err.position = pose_diff.translation().norm();
    err.orientation = Eigen::AngleAxisf(pose_diff.rotation()).angle();
    pub_pose_err.publish(err);
}

Eigen::Isometry3f FramePosePublisher::convertPose(const dart::SE3 &dart_pose) {
    Isometry3frm pose = Isometry3frm::Identity();
    std::memcpy(pose.matrix().row(0).data(), &dart_pose.r0, sizeof(float4));
    std::memcpy(pose.matrix().row(1).data(), &dart_pose.r1, sizeof(float4));
    std::memcpy(pose.matrix().row(2).data(), &dart_pose.r2, sizeof(float4));
    return pose;
}

geometry_msgs::Pose FramePosePublisher::convertPose(const Eigen::Isometry3d &eigen_pose) {
    geometry_msgs::Pose pose;

    pose.position.x = eigen_pose.translation()[0];
    pose.position.y = eigen_pose.translation()[1];
    pose.position.z = eigen_pose.translation()[2];

    const Eigen::Quaterniond q(eigen_pose.rotation());
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();

    return pose;
}

} // namespace dart
