/*
 *  tumble.h
 *  
 *  Created by Megha Gupta on 12/25/12.
 *  Copyright 2011 USC. All rights reserved.
 *
 */

#ifndef TUMBLE_H
#define TUMBLE_H

#include <ros/ros.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>

#include "duplo_v1/Tumble_Duplo.h"
#include "duplo_v1/Process_PCD.h"
//#include <arm_navigation_msgs/CollisionObject.h>
//#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include "robothead.h"
#include "gripper.h"
#include "move_arm.h"
//#include "incontact.h"
//#include "cluster.h"
//#include "find_extents.h"
#include "set_marker.h"

using namespace std;
using namespace pcl;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

const std::string COLLISION_PROCESSING_SERVICE_NAME = 
"/tabletop_collision_map_processing/tabletop_collision_map_processing";

namespace sort_duplos {
	class Tumbler
	{
		public:
			Tumbler (ros::NodeHandle & n);
			~Tumbler (void);
		
		private:
			//Functions
			bool tumbleCallback(duplo_v1::Tumble_Duplo::Request &req, duplo_v1::Tumble_Duplo::Response &res);
			std::vector<geometry_msgs::PointStamped> find_waypoints(pcl::PointCloud<pcl::PointXYZRGB> pcd);
			
			//Variables
			ros::NodeHandle n_;
			ros::ServiceServer tumble_service_;
			ros::Publisher marker_pub;
			ros::Publisher coll_obj_pub;
			ros::Publisher tumble_cloud_pub;
			ros::ServiceClient client_collision_processing_;
			RobotHead head;
			Gripper gripper;
	
			bool tumble_service_called;
			uint32_t shape[4];
			std::vector<geometry_msgs::PointStamped> waypoints_;
			geometry_msgs::PointStamped reset_posn;
	};
}

#endif
