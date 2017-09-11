#ifndef TRACKCONTROL_HPP
#define TRACKCONTROL_HPP

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <image_classification_msgs/SetPose.h>
#include <image_classification_msgs/GetUInt64.h>

#include <dart/pose/pose.h>

#include <atomic>
#include <mutex>

namespace dart {

class TrackControl {
public:
    TrackControl();

    bool getTrackState() { return track; }

    bool getInitState() { return reset_pose.exchange(false); }

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

    bool setPerturbation(image_classification_msgs::SetPose::Request &req, image_classification_msgs::SetPose::Response &res);

    bool getIterations(image_classification_msgs::GetUInt64::Request &req, image_classification_msgs::GetUInt64::Response &res);

    ros::ServiceServer srv_reset;
    ros::ServiceServer srv_start;
    ros::ServiceServer srv_stop;
    ros::ServiceServer srv_iter;
    ros::ServiceServer srv_pose_perturbation;

    std::atomic<bool> track;
    std::atomic<bool> reset_pose;

    dart::SE3 perturbation;

    uint iterN;

    std::mutex mutex_pose;
};

} // namespace dart

#endif // TRACKCONTROL_HPP
