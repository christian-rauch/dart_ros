#ifndef SYNC_HPP
#define SYNC_HPP

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <image_transport/subscriber_filter.h>

class Sync {
public:
    typedef std::function<void(const sensor_msgs::ImageConstPtr&, const sensor_msgs::ImageConstPtr&)> CbImg;
    typedef std::function<void(const sensor_msgs::JointStateConstPtr&)> CbJoints;

    Sync();

    void addSyncImgJoints(image_transport::SubscriberFilter& sub_colour,
                          image_transport::SubscriberFilter& sub_depth,
                          const CbImg& cb_img,
                          message_filters::Subscriber<sensor_msgs::JointState>& sub_joints,
                          const CbJoints& cb_joints);

    void addCbImg(const CbImg& cb) { cbs_img.push_back(cb); }

    void addCbJoints(const CbJoints& cb) { cbs_joints.push_back(cb); }

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::JointState> ApproximateTimePolicy;

    void syncCb(const sensor_msgs::ImageConstPtr& img_colour,
                const sensor_msgs::ImageConstPtr& img_depth,
                const sensor_msgs::JointStateConstPtr& joints);

    message_filters::Synchronizer<ApproximateTimePolicy> sync;

    std::vector<CbImg> cbs_img;
    std::vector<CbJoints> cbs_joints;
};

#endif // SYNC_HPP
