#include <dart_ros/TrackControl.hpp>
#include <Eigen/Geometry>
#include <std_msgs/UInt64.h>

namespace dart {

TrackControl::TrackControl(std::atomic_bool &do_track, std::atomic_bool &do_reset) :
    n("~/tracker"), track(do_track), reset_pose(do_reset)
{
    srv_reset = n.advertiseService("reset", &TrackControl::trackerReset, this);
    srv_start = n.advertiseService("start", &TrackControl::trackerStart, this);
    srv_stop = n.advertiseService("stop", &TrackControl::trackerStop, this);
    srv_pose_perturbation = n.advertiseService("perturbation", &TrackControl::setPerturbation, this);
    srv_iter = n.advertiseService("get_iteration", &TrackControl::getIterations, this);

    // set identity pose
    perturbation.r0 = make_float4(1, 0, 0, 0);
    perturbation.r1 = make_float4(0, 1, 0, 0);
    perturbation.r2 = make_float4(0, 0, 1, 0);
}

bool TrackControl::trackerReset(std_srvs::Trigger::Request &/*req*/,
                                std_srvs::Trigger::Response &res)
{
    reset_pose = true;
    res.success = true;
    return true;
}

bool TrackControl::trackerStart(std_srvs::Trigger::Request &/*req*/,
                                std_srvs::Trigger::Response &res)
{
    iterN = 0;
    track = true;
    res.success = true;
    return true;
}

bool TrackControl::trackerStop(std_srvs::Trigger::Request &/*req*/,
                               std_srvs::Trigger::Response &res)
{
    track = false;
    res.success = true;
    return true;
}

bool TrackControl::setPerturbation(image_classification_msgs::SetPose::Request &req,
                                   image_classification_msgs::SetPose::Response &/*res*/)
{
    // to Eigen types
    const Eigen::Vector3d t(req.pose.position.x,
                            req.pose.position.y,
                            req.pose.position.z);
    const Eigen::Matrix3d R = Eigen::Quaterniond(
                req.pose.orientation.w,
                req.pose.orientation.x,
                req.pose.orientation.y,
                req.pose.orientation.z).toRotationMatrix();

    mutex_pose.lock();
    perturbation.r0 = make_float4(R(0,0), R(0,1), R(0,2), t.x());
    perturbation.r1 = make_float4(R(1,0), R(1,1), R(1,2), t.y());
    perturbation.r2 = make_float4(R(2,0), R(2,1), R(2,2), t.z());
    mutex_pose.unlock();

    return true;
}

bool TrackControl::getIterations(image_classification_msgs::GetUInt64::Request &/*req*/,
                                 image_classification_msgs::GetUInt64::Response &res)
{
    res.uint64.data = iterN;
    return true;
}

} // namespace dart
