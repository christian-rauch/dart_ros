#ifndef GETROBOTURDF_HPP
#define GETROBOTURDF_HPP

#include <string>
#include <ros/ros.h>

std::string GetRobotURDF(const std::string ns = "/", const std::string param_name = "robot_description");

#endif // GETROBOTURDF_HPP
