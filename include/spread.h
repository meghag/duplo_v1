/*
 *  spread.h
 *  
 *  Created by Megha Gupta on 12/25/12.
 *  Copyright 2011 USC. All rights reserved.
 *
 */

#ifndef SPREAD_H
#define SPREAD_H

#include <ros/ros.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

#include "duplo_v1/Spread_Duplo.h"
#include "duplo_v1/Process_PCD.h"

//#include <arm_navigation_msgs/CollisionObject.h>
//#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include "robothead.h"
#include "gripper.h"
#include "move_arm.h"
#include "incontact.h"
#include "cluster.h"
#include "find_extents.h"
#include "set_marker.h"

using namespace std;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

namespace sort_duplos {
	class Spreader
	{
		public:
			Spreader (ros::NodeHandle & n);
			~Spreader (void);
		
		private:
			//Functions
			bool spreadCallback(duplo_v1::Spread_Duplo::Request &req, duplo_v1::Spread_Duplo::Response &res);
			std::vector<geometry_msgs::Point> find_circular_path(pcl::PointCloud<pcl::PointXYZRGB> pcd);
			
			//Variables
			ros::NodeHandle n_;
			ros::ServiceServer spread_service_;
			ros::Publisher marker_pub;
			ros::Publisher coll_obj_pub;
			ros::Publisher spread_cloud_pub;
			ros::ServiceClient collision_processing_srv;
			RobotHead head;
			Gripper gripper;
	
			bool spread_service_called;
			uint32_t shape[4];
			vector<geometry_msgs::Point> path_;
			geometry_msgs::PointStamped reset_posn;
	};
}

#endif
