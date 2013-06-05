/*
 *  manipulator_try.h
 *  
 *  Created by Megha Gupta on 01/17/13.
 *  Copyright 2013 USC. All rights reserved.
 *
 */

#ifndef MANIPULATOR_TRY_H
#define MANIPULATOR_TRY_H

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
#include <geometry_msgs/Vector3.h>

#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

#include <arm_navigation_msgs/GetCollisionObjects.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <object_manipulation_msgs/FindClusterBoundingBox2.h>
#include <object_manipulation_msgs/ClusterBoundingBox.h>

#include "duplo_v1/Manipulate_Duplo.h"
#include "duplo_v1/Process_PCD.h"
#include "duplo_v1/ExecuteCartesianIKTrajectory.h"

//#include <arm_navigation_msgs/CollisionObject.h>
//#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include "robothead.h"
#include "gripper.h"
//#include "move_arm.h"
#include "incontact.h"
#include "cluster.h"
#include "find_extents.h"
#include "set_marker.h"

using namespace std;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
static const std::string GET_COLLISION_OBJECTS_NAME = "/environment_server/get_collision_objects";

namespace sort_duplos {
	class Manipulator
	{
		public:
			Manipulator (ros::NodeHandle & n, int option);
			~Manipulator (void);
		
		private:
			//Functions
			geometry_msgs::Point find_centroid(pcl::PointCloud<PointXYZRGB> pcd);
			void minmax3d(tf::Vector3 &min, tf::Vector3 &max, pcl::PointCloud<PointXYZRGB>::Ptr &cloud);
			bool callback(duplo_v1::Manipulate_Duplo::Request &req, duplo_v1::Manipulate_Duplo::Response &res);
			bool pick_n_place();
			bool spread();
			bool tumble();

			std::vector<geometry_msgs::Point> find_spread_path(pcl::PointCloud<pcl::PointXYZRGB> pcd);
			std::vector<geometry_msgs::PointStamped> find_waypoints(pcl::PointCloud<pcl::PointXYZRGB> pcd);
			geometry_msgs::Point findPlaceLocation();
			float findLongestDim(pcl::PointCloud<pcl::PointXYZRGB> pcd);

			void resetCollisionModels();

			void getClusterBoundingBox(const sensor_msgs::PointCloud2 &cluster,
					geometry_msgs::PoseStamped &pose_stamped,
					geometry_msgs::Vector3 &dimensions);

			void processCollisionGeometryForBoundingBox(const object_manipulation_msgs::ClusterBoundingBox &box,
					std::string &collision_name);

			void collision_op(std::string object1, std::string object2, int operation, double penetration,
					arm_navigation_msgs::CollisionOperation& cop);

			void add_collision_object(arm_navigation_msgs::CollisionObject& cob, int operation, int primitive);

			//int move_arm_pp(geometry_msgs::Point go_to, int cluster_id, int cluster_op);
			int move_arm(geometry_msgs::Point go_to, int cluster_id, int cluster_op, int plan_id, int action);
			//int move_arm_tumble(geometry_msgs::Point go_to, int cluster_id, int cluster_op, int plan_id);

			void breakpoint();

			//Variables
			ros::NodeHandle n_;
			ros::ServiceServer manipulate_service_;
			ros::Publisher marker_pub_;
			ros::Publisher coll_obj_pub_;
			ros::Publisher target_pub_;
			ros::Publisher spread_cloud_pub_;
			ros::Publisher tumble_cloud_pub_;
			ros::Publisher bbx_pub_;
			ros::Publisher bbx_pose_pub_;
			ros::ServiceClient collision_processing_srv;
			ros::ServiceClient cartesian_client_;
			
			ros::ServiceClient get_planning_scene_client_;
			ros::ServiceClient get_collision_objects_client_;
			ros::ServiceClient bbx_client_;
			//ros::Publisher collision_map_pub_;
			ros::Publisher arm_destpose_pub;
			ros::Publisher collision_object_pub_;

			RobotHead head_;
			Gripper gripper_;
	
			bool manipulate_service_called_;
			uint32_t shape[4];
			vector<geometry_msgs::Point> spread_path_;
			vector<geometry_msgs::PointStamped> tumble_waypoints_;
			geometry_msgs::PointStamped reset_posn_;
			vector<sensor_msgs::PointCloud2> vec_target_cloud2_;
			sensor_msgs::PointCloud2 target_cloud2_;
			std::vector<geometry_msgs::Point> table_extent_;

			object_manipulation_msgs::ClusterBoundingBox bbx_;
			std::string collision_name_;

			geometry_msgs::Point LEFT_RESET;
			geometry_msgs::Point RIGHT_RESET;
			geometry_msgs::Point RESET_POSN;
			geometry_msgs::Point RED;
			geometry_msgs::Point BLUE;
			geometry_msgs::Point GREEN;
			geometry_msgs::Point active_reset_;
			std::string active_arm_;
			int active_arm_sym_;
			int option_;
			int count_pp;	//Num of pick and place actions
			int count_spread;
			int count_tumble;
			//geometry_msgs::Quaternion default_orientation;
	};
}

#endif
