#ifndef SET_BOUNDARIES_H 
#define SET_BOUNDARIES_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace std;

visualization_msgs::Marker 
set_boundaries(std::string frame_id, string ns, int id, int shape, int action,
           vector<geometry_msgs::Point> extent, float scale, float r, float g, float b, float a);

#endif