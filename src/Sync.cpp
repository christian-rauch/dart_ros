#include <dart_ros/Sync.hpp>

Sync::Sync() : sync(ApproximateTimePolicy(25)) {
    sync.registerCallback(&Sync::syncCb, this);
}

void Sync::addSyncImgJoints(image_transport::SubscriberFilter& sub_colour,
                            image_transport::SubscriberFilter& sub_depth,
                            const CbImg& cb_img,
                            message_filters::Subscriber<sensor_msgs::JointState>& sub_joints,
                            const CbJoints& cb_joints)
{
    sync.connectInput(sub_colour, sub_depth, sub_joints);

    cbs_img.push_back(cb_img);
    cbs_joints.push_back(cb_joints);
}

void Sync::syncCb(const sensor_msgs::ImageConstPtr& img_colour,
                  const sensor_msgs::ImageConstPtr& img_depth,
                  const sensor_msgs::JointStateConstPtr& joints)
{
    // image callbacks
    for(const CbImg& cb : cbs_img) {
        if(cb) { cb(img_colour, img_depth); }
    }

    // joint callbacks
    for(const CbJoints& cb : cbs_joints) {
        if(cb) { cb(joints); }
    }
}
