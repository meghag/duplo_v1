/*
 *  pcd_from_kinect.h
 *  
 *
 *  Created by Megha Gupta on 09/20/12.
 *  Copyright 2011 USC. All rights reserved.
 *
 */

#ifndef PROCESS_PCD_H
#define PROCESS_PCD_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <algorithm>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <robothead.h>

#include "duplo_v1/Process_PCD.h"
//#include "duplo_v1/Grasp_Duplo.h"
#include "duplo_v1/Manipulate_Duplo.h"
//#include "duplo_v1/Spread_Duplo.h"
//#include "duplo_v1/Tumble_Duplo.h"
#include "duplo_v1/Get_New_PCD.h"

#include "pass_through_gen.h"
#include "planar_seg.h"
#include "cluster.h"
#include "find_extents.h"
#include "set_boundaries.h"
#include "set_marker.h"
#include "move_arm.h"
#include "incontact.h"

using namespace std;
using namespace pcl;

std::map<std::string, ros::Publisher*> cloud_publishers;

namespace sort_duplos {
	class DataProcessor
	{
	public:
		DataProcessor (ros::NodeHandle & n);
		~DataProcessor (void);
		//friend class DuploGrasper;
		
	private:
		//Functions
		void pubCloud(const std::string &topic_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::string frame_id);
		bool processDataCallback(duplo_v1::Process_PCD::Request &req, duplo_v1::Process_PCD::Response &res);
		vector<sensor_msgs::PointCloud2> find_regions(PointCloud<PointXYZRGB>::Ptr input);
		int find_state(pcl::PointCloud<PointXYZRGB>::Ptr cloud, int region_id);
		int get_object_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_orig, 
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr o_cloud);
		bool mysort(pair<int,int> state1, pair<int,int> state2);
		void process();
		void assign_manipulation_primitive();
		bool pick_n_place(int region_id);
		bool spread(int region_id);
		bool tumble(int region_id);
			
		//Variables
		ros::NodeHandle n_;
		ros::ServiceServer process_service_;
		ros::Publisher pub_active_region_;
		ros::Publisher marker_pub_;
		ros::Publisher regions_pub_;
		//ros::ServiceClient client_grasp_;
		ros::ServiceClient client_manipulate_;
		//ros::ServiceClient client_tumble_;
		ros::ServiceClient client_newpcd_;

		RobotHead head_;

		int nocluster_count_;
		bool processed_pointcloud_;
		bool new_cloud_wanted_;
		geometry_msgs::Point reset_posn;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr planar_cloud_;
		std::vector<geometry_msgs::Point> table_extent_;
		vector<sensor_msgs::PointCloud2> regions_;
		//vector<std::pair<int,int> > state_;
		vector<std::pair<PointCloud<PointXYZRGB>::Ptr, int> > state_;
		sensor_msgs::PointCloud2 region_publish[4];
		vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > region_ptr;
	};
}

#endif
