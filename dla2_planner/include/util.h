#pragma once


#include <geometry_msgs/Point.h>
#include <Eigen/Core>

namespace cd {
geometry_msgs::Point createPoint(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}
geometry_msgs::Point eigToGeo(Eigen::Vector3d v) {
  geometry_msgs::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}
}
