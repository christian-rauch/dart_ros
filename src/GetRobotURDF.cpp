#include <dart_ros/GetRobotURDF.hpp>
#include <ros/param.h>

std::string GetRobotURDF(const std::string ns, const std::string param_name) {
    std::string urdf;
    if(!ros::param::get(param_name, urdf)) {
        throw std::runtime_error("no parameter "+ns+param_name);
    }
    else {
        return urdf;
    }
}
