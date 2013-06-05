#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include <pcl/filters/passthrough.h>

using namespace std;
using namespace pcl;

int pass_through(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_filtered)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudx(new pcl::PointCloud<pcl::PointXYZRGB>);	//Filtered cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudy(new pcl::PointCloud<pcl::PointXYZRGB>);	//Filtered cloud

  ROS_INFO("Inside pass through.");
	for (size_t i = 0; i < 10; ++i)
		ROS_INFO("\t%f\t%f\t%f",pcd_orig->points[i].x, pcd_orig->points[i].y, pcd_orig->points[i].z);
	
  /****************** Filter out the non-table points ******************/
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (pcd_orig);
  pass.setFilterFieldName ("x");
//  pass.setFilterLimits (0.2, 1.0);
  pass.setFilterLimits (-0.3, 0.3);
//  pass.setFilterLimitsNegative (true);
  pass.filter (*cloudx);
  std::cerr << "Filtered x cloud size: " << cloudx->points.size () << std::endl;

  pass.setInputCloud (cloudx);
  pass.setFilterFieldName ("y");
//  pass.setFilterLimits (-0.2, 0.7);
  pass.setFilterLimits (-0.4, 0.4);
//  pass.setFilterLimitsNegative (true);
  pass.filter (*cloudy);
  std::cerr << "Filtered y cloud size: " << cloudy->points.size () << std::endl;

  pass.setInputCloud (cloudy);
  pass.setFilterFieldName ("z");
//  pass.setFilterLimits (0.7, 0.9);
  pass.setFilterLimits (0.7, 0.95);
//  pass.setFilterLimitsNegative (true);
  pass.filter (*pcd_filtered);
  std::cerr << "Filtered cloud size: " << pcd_filtered->points.size () << std::endl;

  return (0);
}