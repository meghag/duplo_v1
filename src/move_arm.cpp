#include "move_arm.h"

int move_arm(geometry_msgs::Point go_to){
	
	actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);
	move_arm.waitForServer();
	ROS_INFO("DUPLO: Connected to server");
	arm_navigation_msgs::MoveArmGoal goalA;

	goalA.motion_plan_request.group_name = "right_arm";
	goalA.motion_plan_request.num_planning_attempts = 1;
	goalA.motion_plan_request.planner_id = std::string("");
	goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
	
	arm_navigation_msgs::SimplePoseConstraint desired_pose;
	desired_pose.header.frame_id = "base_link";
//	desired_pose.header.frame_id = "torso_lift_link";
	desired_pose.header.stamp = ros::Time::now();
	desired_pose.link_name = "r_wrist_roll_link";
	desired_pose.pose.position = go_to;
	//desired_pose.pose.position.z = go_to.z - 0.25;

	desired_pose.pose.orientation.x = 0;
	desired_pose.pose.orientation.y = 0;
	desired_pose.pose.orientation.z = 0;
	desired_pose.pose.orientation.w = 1;

	desired_pose.absolute_position_tolerance.x = 0.02;
	desired_pose.absolute_position_tolerance.y = 0.02;
	desired_pose.absolute_position_tolerance.z = 0.02;

	desired_pose.absolute_roll_tolerance = 0.04;
	desired_pose.absolute_pitch_tolerance = 0.04;
	desired_pose.absolute_yaw_tolerance = 0.04;

	arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

	bool finished_within_time = false;
	move_arm.sendGoal(goalA);
	finished_within_time = move_arm.waitForResult(ros::Duration(10.0));
	if (!finished_within_time)
	{
		move_arm.cancelGoal();
		ROS_INFO("DUPLO: Timed out achieving goal A");
		return -1;
	}
	else
	{
		actionlib::SimpleClientGoalState state = move_arm.getState();
		bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
		if(success) {
			ROS_INFO("DUPLO: Action finished: %s",state.toString().c_str());
			return 0;
		}
		else {
			ROS_INFO("DUPLO: Action failed: %s",state.toString().c_str());
			return -1;
		}
	}
	return 0;
}

int move_arm_spread(geometry_msgs::Point go_to, int plan_id){

	actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);
	move_arm.waitForServer();
	ROS_INFO("DUPLO: Connected to server");
	arm_navigation_msgs::MoveArmGoal goalA;

	goalA.motion_plan_request.group_name = "right_arm";
	goalA.motion_plan_request.num_planning_attempts = 1;
	goalA.motion_plan_request.planner_id = std::string("");
	goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

	arm_navigation_msgs::SimplePoseConstraint desired_pose;
	desired_pose.header.frame_id = "base_link";
	desired_pose.header.stamp = ros::Time::now();
	desired_pose.link_name = "r_wrist_roll_link";
	desired_pose.pose.position = go_to;
	if (plan_id == 1)
		//desired_pose.pose.position.z = go_to.z + 0.002;
		desired_pose.pose.position.z = go_to.z + 0.19;
	else
		desired_pose.pose.position.x = go_to.x - 0.08;
	/*
	 desired_pose.pose.position.x =  0.6; //0.75;
	 desired_pose.pose.position.y = -0.5;//-0.188;
	 desired_pose.pose.position.z = 0;
	 */
	if (plan_id == 2) {
		desired_pose.pose.orientation.x = 0;
		desired_pose.pose.orientation.y = 0;
		desired_pose.pose.orientation.z = 0;
		desired_pose.pose.orientation.w = 1;
	} else {
		desired_pose.pose.orientation.x = -0.74;
		desired_pose.pose.orientation.y = -0.04;
		desired_pose.pose.orientation.z = 0.67;
		desired_pose.pose.orientation.w = -0.04;
	}

	desired_pose.absolute_position_tolerance.x = 0.005;
	desired_pose.absolute_position_tolerance.y = 0.005;
	desired_pose.absolute_position_tolerance.z = 0.005;  //0.02

	desired_pose.absolute_roll_tolerance = 2.0; //0.04;
	desired_pose.absolute_pitch_tolerance = 2.0; //0.04;
	desired_pose.absolute_yaw_tolerance = 2.0; //0.04;

	arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

	bool finished_within_time = false;
	move_arm.sendGoal(goalA);
	finished_within_time = move_arm.waitForResult(ros::Duration(10.0));
	if (!finished_within_time)
	{
		move_arm.cancelGoal();
		ROS_INFO("DUPLO: Timed out achieving goal A");
		return -1;
	}
	else
	{
		actionlib::SimpleClientGoalState state = move_arm.getState();
		bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
		if(success) {
			ROS_INFO("DUPLO: Action finished: %s",state.toString().c_str());
			return 0;
		}
		else {
			ROS_INFO("DUPLO: Action failed: %s",state.toString().c_str());
			return -1;
		}
	}
	return 0;
}

int move_arm_tumble(geometry_msgs::Point go_to, int plan_id){
	
	actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);
	move_arm.waitForServer();
	ROS_INFO("DUPLO: Connected to server");
	arm_navigation_msgs::MoveArmGoal goalA;

	goalA.motion_plan_request.group_name = "right_arm";
	goalA.motion_plan_request.num_planning_attempts = 3;
	goalA.motion_plan_request.planner_id = std::string("");
	goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
	
	arm_navigation_msgs::SimplePoseConstraint desired_pose;
	desired_pose.header.frame_id = "base_link";
	//desired_pose.header.stamp = ros::Time::now();
	desired_pose.link_name = "r_wrist_roll_link";
	desired_pose.pose.position = go_to;
	if (plan_id < 3)
		desired_pose.pose.position.z = go_to.z+0.155;
	else
		desired_pose.pose.position.x = go_to.x - 0.08;
/*
  	desired_pose.pose.position.x =  0.6; //0.75;
	desired_pose.pose.position.y = -0.5;//-0.188;
	desired_pose.pose.position.z = 0;
*/
	if (plan_id >= 3) {
		desired_pose.pose.orientation.x = 0.0;
		desired_pose.pose.orientation.y = 0.0;
		desired_pose.pose.orientation.z = 0.0;
		desired_pose.pose.orientation.w = 1.0;
	} else {
		desired_pose.pose.orientation.x = -0.74;
		desired_pose.pose.orientation.y = -0.04; //-0.04;
		desired_pose.pose.orientation.z = 0.67;
		desired_pose.pose.orientation.w = -0.04; //-0.04;
	}
	
	desired_pose.absolute_position_tolerance.x = 0.01;
	desired_pose.absolute_position_tolerance.y = 0.01;
	desired_pose.absolute_position_tolerance.z = 0.01;

	desired_pose.absolute_roll_tolerance = 4.0;
	desired_pose.absolute_pitch_tolerance = 4.0;
	desired_pose.absolute_yaw_tolerance = 4.0;

	arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

	bool finished_within_time = false;
	
	move_arm.sendGoal(goalA);
	finished_within_time = move_arm.waitForResult(ros::Duration(10.0));
	if (!finished_within_time)
	{
		move_arm.cancelGoal();
		ROS_INFO("DUPLO: Timed out achieving goal A");
		return -1;
	}
	else
	{
		actionlib::SimpleClientGoalState state = move_arm.getState();
		bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
		if(success) {
			ROS_INFO("DUPLO: Action finished: %s",state.toString().c_str());
			return 0;
		}
		else {
			ROS_INFO("DUPLO: Action failed: %s",state.toString().c_str());
			return -1;
		}
	}

	return 0;
}

void addCollisionobject()
{
	//add the cylinder into the collision space
	/*	arm_navigation_msgs::CollisionObject cylinder_object;
	 cylinder_object.id = "pile";
	 cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
	 cylinder_object.header.frame_id = "base_link";
	 cylinder_object.header.stamp = ros::Time::now();
	 arm_navigation_msgs::Shape object;
	 object.type = arm_navigation_msgs::Shape::CYLINDER;
	 object.dimensions.resize(2);
	 object.dimensions[0] = .25;
	 object.dimensions[1] = .25;
	 geometry_msgs::Pose pose;
	 //	pose.position = waypoints[1].point;
	 pose.position.x = 0.8;
	 pose.position.y = 0;
	 pose.position.z = 0.8;

	 pose.orientation.x = 0;
	 pose.orientation.y = 0;
	 pose.orientation.z = 0;
	 pose.orientation.w = 1;
	 cylinder_object.shapes.push_back(object);
	 cylinder_object.poses.push_back(pose);

	 coll_obj_pub.publish(cylinder_object);

	 //call collision map processing
	 ROS_INFO("DUPLO: PP: Calling collision map processing");
	 tabletop_collision_map_processing::TabletopCollisionMapProcessing 
		 processing_call;
	 //pass the result of the tabletop detection 
	 //processing_call.request.detection_result = detection_call.response.detection;
	 //ask for the exising map and collision models to be reset
	 processing_call.request.reset_static_map = true;
	 //processing_call.request.reset_collision_models = true;
	 //processing_call.request.reset_attached_models = true;
	 //ask for a new static collision map to be taken with the laser
	 //after the new models are added to the environment
	 processing_call.request.take_static_collision_map = true;
	 //ask for the results to be returned in base link frame
	 processing_call.request.desired_frame = "base_link";
	 if (!collision_processing_srv.call(processing_call))
	 {
		 ROS_ERROR("PP: Collision map processing service failed");
		 return false;
	}

	ros::Duration(2.0).sleep();
	*/	
}
