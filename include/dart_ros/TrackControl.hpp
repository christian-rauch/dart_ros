#ifndef TRACKCONTROL_HPP
#define TRACKCONTROL_HPP

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <robot_localization/SetPose.h>
#include <dart_msgs/SetPose.h>
#include <dart_msgs/GetUInt64.h>

#include <dart/pose/pose.h>

#include <atomic>
#include <mutex>

namespace dart {

class TrackControl {
public:
    TrackControl(std::atomic_bool &do_track, std::atomic_bool &do_reset, std::shared_ptr<dart::SE3> &init_pose);

    void setIterations(const uint N) { iterN = N; }

    void incrementIterations() { iterN++; }

    dart::SE3 getPerturbation() {
        std::lock_guard<std::mutex> lck(mutex_pose);
        return perturbation;
    }

private:
    ros::NodeHandle n;

    bool trackerReset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool trackerStart(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool trackerStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool setPerturbation(dart_msgs::SetPose::Request &req, dart_msgs::SetPose::Response &res);

    bool getIterations(dart_msgs::GetUInt64::Request &req, dart_msgs::GetUInt64::Response &res);

    bool setPose(robot_localization::SetPose::Request &req, robot_localization::SetPose::Response &res);

    ros::ServiceServer srv_reset;
    ros::ServiceServer srv_start;
    ros::ServiceServer srv_stop;
    ros::ServiceServer srv_iter;
    ros::ServiceServer srv_pose_perturbation;
    ros::ServiceServer srv_set_pose;

    std::atomic_bool &track;
    std::atomic_bool &reset_pose;
    std::shared_ptr<SE3> &init_pose;

    dart::SE3 perturbation;

    uint iterN;

    std::mutex mutex_pose;
};

} // namespace dart

#endif // TRACKCONTROL_HPP
