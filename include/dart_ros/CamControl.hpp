#ifndef CAMCONTROL_HPP
#define CAMCONTROL_HPP

#include <pangolin/display/opengl_render_state.h>
#include <ros/ros.h>
#include <image_classification_msgs/SetPose.h>
#include <image_classification_msgs/SetCamLookAt.h>


class CamControl {
public:
    CamControl(pangolin::OpenGlRenderState *cam);

    bool setPose(image_classification_msgs::SetCamLookAt::Request  &req,
                 image_classification_msgs::SetCamLookAt::Response &res);

private:
    pangolin::OpenGlRenderState *cam;
    ros::NodeHandle n;
    ros::ServiceServer srv_cam_pose;
};

#endif // CAMCONTROL_HPP
