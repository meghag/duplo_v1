#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "duplo_v1/Process_PCD.h"
#include "duplo_v1/Grasp_Duplo.h"
#include "duplo_v1/Get_New_PCD.h"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
//#include <pcl/filters/radius_outlier_removal.h>

#include <tf/transform_listener.h>

sensor_msgs::PointCloud2 the_chosen_one;
bool processed_pointcloud;

int planar_seg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr orig_cloud, 
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud, 
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr o_cloud,
               char* fname1, char* fname2);

int pass_through(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_filtered);

int cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr o_cloud);

bool process_ros(duplo_v0::Process_PCD::Request &req,
                 duplo_v0::Process_PCD::Response &res)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudz(new pcl::PointCloud<pcl::PointXYZRGB>);	//Filtered cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZRGB>); //Inliers after removing outliers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr planar_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster1(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::fromROSMsg(req.pcd_in,*cloud);
	
  /****************** Filter out the non-table points ******************/
  if (pass_through(cloud,cloudz) != 0)
  {
      std::cerr << "Pass-through filter error" << std::endl;
      return false;
  }
 
/* ********* Segmentation of the cloud into table and object clouds **********/
  planar_seg(cloudz,planar_cloud,object_cloud,"table_try.pcd","object_try.pcd");
	 

  /********************** Cluster objects based on Euclidean distance **********/
  //vector<double> volumes = cluster(object_cloud);
  int num_clusters_found = cluster(object_cloud);
  if (num_clusters_found == 0)
		the_chosen_one.data.clear();
  res.n_clusters = num_clusters_found;
 
  processed_pointcloud = true;
  return true;
}  

int
  main (int argc, char** argv)
{
  ros::init(argc, argv, "process_pcd_server");
  ros::NodeHandle n;

  processed_pointcloud = false;
  int nocluster_count = 0;

  ros::ServiceServer service = n.advertiseService("process_pcd", process_ros);
  ROS_INFO("Ready to process input point cloud data.");

  ros::ServiceClient client = n.serviceClient<duplo_v0::Grasp_Duplo>("grasp_duplo");
  duplo_v0::Grasp_Duplo srv;

  ros::ServiceClient client_newpcd = 
		n.serviceClient<duplo_v0::Get_New_PCD>("get_new_pcd");
  duplo_v0::Get_New_PCD srv_newpcd;

  while (ros::ok() && processed_pointcloud == false && nocluster_count < 3) {
	ros::spinOnce();
	if (processed_pointcloud == true) {
	  if (the_chosen_one.data.size() != 0) {
		  nocluster_count = 0;
		  srv.request.pick_cluster = the_chosen_one;

		  if (client.call(srv)) {
			ROS_INFO("Called the grasping pipeline.");
		  } else {
			ROS_ERROR("Failed to call service grasp duplo.");
			return 1;
		  }
	  } else {
		  ROS_INFO("No cluster found. Retrying with new point cloud.");
		  nocluster_count += 1;
		  ros::Duration(2).sleep();
		  // Ask for new PCD
		  srv_newpcd.request.question = true;

		  if (client_newpcd.call(srv_newpcd)) {
			ROS_INFO("Requesting for new point cloud.");
		  } else {
			ROS_ERROR("Failed to call service get new pcd.");
			return 1;
	  	  }
	  }
	}
    processed_pointcloud = false;
  }

  if (nocluster_count == 3) {
	  ROS_INFO("Cleared the table");
	  ros::shutdown();
  }
	  
  return (0);
}
