/*
 *  pick_n_place.h
 *  
 *
 *  Created by Megha Gupta on 12/24/12.
 *  Copyright 2011 USC. All rights reserved.
 *
 */

#ifndef PICK_N_PLACE_H
#define PICK_N_PLACE_H

#include <string.h>

#include <ros/ros.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <sensor_msgs/point_cloud_conversion.h>

#include <actionlib/client/simple_action_client.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>
#include <tabletop_object_detector/TabletopObjectRecognition.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
//#include <pr2_controllers_msgs/PointHeadAction.h>
//#include <arm_navigation_msgs/MoveArmAction.h>
//#include <arm_navigation_msgs/utils.h>

#include "robot_head.h"
#include "gripper.h"
#include "move_arm.h"

#include "duplo_v1/Grasp_Duplo.h"
#include "duplo_v1/Get_New_PCD.h"

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

//set service and action names
const std::string OBJECT_DETECTION_SERVICE_NAME = "/object_detection";
const std::string COLLISION_PROCESSING_SERVICE_NAME = "/tabletop_collision_map_processing/tabletop_collision_map_processing";
const std::string PICKUP_ACTION_NAME = "/object_manipulator/object_manipulator_pickup";
/*
geometry_msgs::Point RESET_POSN;
geometry_msgs::Point RED;
geometry_msgs::Point BLUE;
geometry_msgs::Point GREEN;

RESET_POSN.x = 0.6;	RESET_POSN.y = -0.5; RESET_POSN.z = 0;
RED.x = 0.6;  RED.y = -0.5; RED.z = -0.1;  //Red bin
BLUE.x = 0.6;  BLUE.y = -0.6; BLUE.z = -0.1;	//Blue bin
GREEN.x = 0.6; GREEN.y = -0.7; GREEN.z = -0.1;  //Green bin
*/

using namespace std;
using namespace pcl;

namespace sort_duplos {
	class DuploGrasper
	{
	public:
		DuploGrasper (ros::NodeHandle & n);
		~DuploGrasper (void);
		
	private:
		//Functions
		//void kinectDataCallback(const::sensor_msgs::PointCloud2::ConstPtr &);
		bool graspDuploCallback(duplo_v1::Grasp_Duplo::Request &req, duplo_v1::Grasp_Duplo::Response &res);
		bool TabletopDetection (void);
		bool CollisionMapProcessing (void);
		bool PickupObject (void);
		void PlaceObject(void);
		//int move_arm(geometry_msgs::Point go_to);
		float findLongestDim(pcl::PointCloud<pcl::PointXYZRGB> pcd);
		
		//Variables
		ros::NodeHandle n_;
		//ros::Subscriber kinectDataSub_;
		ros::Publisher targetObjectPub_;
		ros::ServiceServer graspDuploServer_;
		ros::ServiceClient getNewDataClient_;
		ros::ServiceClient objectDetectionClient_;
		ros::ServiceClient collisionProcessingClient_;
		//actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> pickupClient_(PICKUP_ACTION_NAME, true);
		//tf::TransformListener listener_;
		
		vector<sensor_msgs::PointCloud2> targetObject_;
		tabletop_object_detector::TabletopDetection detectionCall_;
		tabletop_collision_map_processing::TabletopCollisionMapProcessing processingCall_;
		object_manipulation_msgs::PickupGoal pickupGoal_;
		RobotHead head_;
		Gripper gripper_;

		bool pickupSuccess_;
		geometry_msgs::Point RESET_POSN;
		geometry_msgs::Point RED;
		geometry_msgs::Point BLUE;
		geometry_msgs::Point GREEN;
		
	};
}

#endif
