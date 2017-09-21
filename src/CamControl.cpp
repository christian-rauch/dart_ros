#include <dart_ros/CamControl.hpp>

CamControl::CamControl(pangolin::OpenGlRenderState *cam) :
    cam(cam),
    n("~"),
    srv_cam_pose(n.advertiseService("set_cam_pose", &CamControl::setPose, this))
{

}

bool CamControl::setPose(dart_msgs::SetCamLookAt::Request  &req,
                         dart_msgs::SetCamLookAt::Response &/*res*/)
{
    cam->SetModelViewMatrix(pangolin::ModelViewLookAt(
        req.position.x, req.position.y, req.position.z,
        req.look_at.x, req.look_at.y, req.look_at.z,
        pangolin::AxisY));
    return true;
}
