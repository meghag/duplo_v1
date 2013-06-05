#ifndef FIND_EXTENTS_H 
#define FIND_EXTENTS_H

#include <ros/ros.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl_ros/transforms.h>

std::vector<geometry_msgs::Point> find_extents(pcl::PointCloud<pcl::PointXYZRGB> pcd);

#endif