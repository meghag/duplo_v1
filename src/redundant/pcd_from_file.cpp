#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "duplo_ros/Process_PCD.h"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>

#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
//#include <vector>

//#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;

int read_pcd(char* filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd);

int main (int argc, char** argv)
{
  ros::init(argc, argv, "pcd_from_file");
  if (argc != 2) {
	ROS_INFO("Incorrect number of arguments. \nUsage: pcd_from_file filename.pcd");
	return (1);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (read_pcd(argv[1], cloud) == -1)
  {
    std::cerr << "Could not read file " << argv[1] << std::endl;
    return false;
  }  

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud,msg);
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<duplo_ros::Process_PCD>("process_pcd");
  duplo_ros::Process_PCD srv;
  srv.request.pcd_in = msg;

  if (client.call(srv))
  {
	ROS_INFO("Processed the point cloud.");
  } else {
	ROS_ERROR("Failed to call service process_pcd.");
	return 1;
  }
 
  return (0);
}
