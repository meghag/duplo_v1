#include <ros/ros.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/transform_listener.h>

#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>

using namespace std;

bool set_pickup_goal(sensor_msgs::PointCloud2 target,string support_surface_name,
                    object_manipulation_msgs::PickupGoal &pickup_goal)
{
		object_manipulation_msgs::GraspableObject object;
		sensor_msgs::PointCloud goal_pcd;
		sensor_msgs::PointCloud2 pick_cluster;

		if (sensor_msgs::convertPointCloud2ToPointCloud(target,goal_pcd) == true)
		{
			goal_pcd.header.frame_id = "base_link";
			goal_pcd.header.stamp = ros::Time::now();

			object.cluster = goal_pcd;
			object.reference_frame_id = "base_link";
			pickup_goal.target = object;
			ROS_INFO("PP: Set the goal target as a graspable object\n");

		} else {
			ROS_ERROR("PP: Conversion from pointcloud2 to pointcloud failed.\n");
			return false;
		}

		//pass the collision name of the table, also returned by the collision 
		//map processor
		pickup_goal.collision_support_surface_name = support_surface_name;
		pickup_goal.allow_gripper_support_collision = true;
		
		//pick up the object with the right arm
		pickup_goal.arm_name = "right_arm";
		//specify the desired distance between pre-grasp and final grasp
		pickup_goal.desired_approach_distance = 0.15;
		pickup_goal.min_approach_distance = 0.1;
		//we will be lifting the object along the "vertical" direction
		//which is along the z axis in the base_link frame
		geometry_msgs::Vector3Stamped direction;
		direction.header.stamp = ros::Time::now();
		direction.header.frame_id = "base_link";
		direction.vector.x = 0;
		direction.vector.y = 0;
		direction.vector.z = 1;
		pickup_goal.lift.direction = direction;
		//request a vertical lift of 10cm after grasping the object
		pickup_goal.lift.desired_distance = 0.15;
		pickup_goal.lift.min_distance = 0.1;
		//do not use tactile-based grasping or tactile-based lift
		pickup_goal.use_reactive_lift = false;
		pickup_goal.use_reactive_execution = false;

		return true;
	}
	