#include "manipulator.h"

namespace sort_duplos
{
	Manipulator::Manipulator(ros::NodeHandle & n): n_(n)
	{
		manipulate_service_called_ = false;
		manipulate_service_ = n_.advertiseService("manipulate_duplo", &Manipulator::callback, this);
		marker_pub_ = n_.advertise<visualization_msgs::Marker>("manipulator_marker", 10);
		spread_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("to_spread",1);
		tumble_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("to_tumble",1);
	
		shape[0] = visualization_msgs::Marker::CUBE;
		shape[1] = visualization_msgs::Marker::SPHERE;
		shape[2] = visualization_msgs::Marker::CYLINDER;
		shape[3] = visualization_msgs::Marker::CUBE;

		reset_posn_.point.x = 0.68;	reset_posn_.point.y = -0.5; reset_posn_.point.z = 1;
		reset_posn_.header.frame_id = "base_link";

		LEFT_RESET.x = 0.3;
		LEFT_RESET.y = 0.7; //0.5
		LEFT_RESET.z = 1.1; //1.0
		RIGHT_RESET.x = 0.3;
		RIGHT_RESET.y = -0.7;  //-0.5
		RIGHT_RESET.z = 1.1;   //1.0

		active_arm_ = "left_arm";
		active_arm_sym_ = 0;
		active_reset_ = LEFT_RESET;

		target_object_pub_ = n_.advertise<sensor_msgs::PointCloud2>("target_object",1);
		collision_object_pub_ = n_.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);

		ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
		get_planning_scene_client_ = n_.serviceClient<arm_navigation_msgs::GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);
		get_collision_objects_client_ = n_.serviceClient<arm_navigation_msgs::GetCollisionObjects>(GET_COLLISION_OBJECTS_NAME);
		bbx_client_ = n_.serviceClient<object_manipulation_msgs::FindClusterBoundingBox2>("/find_cluster_bounding_box2");
	}

	Manipulator::~Manipulator() {}

	bool Manipulator::callback(duplo_v1::Manipulate_Duplo::Request &req,
			duplo_v1::Manipulate_Duplo::Response &res)
	{
		target_cloud2_ = req.target_cloud;
		target_cloud2_.header.frame_id = "base_link";
		target_cloud2_.header.stamp = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZRGB> pcd;
		fromROSMsg(target_cloud2_, pcd);

		table_extent_ = req.table_extent;

		// 1. Remove all collision objects
		ROS_INFO("Removing all collision objects.");
		resetCollisionModels();

		// 2. Get bounding box of spread cloud
		ROS_INFO("Finding bounding box of spread cloud");
		getClusterBoundingBox(target_cloud2_, bbx_.pose_stamped, bbx_.dimensions);

		// 3. Add this bounding box as a collision object
		ROS_INFO("Adding a collision object for spread cloud");
		processCollisionGeometryForBoundingBox(bbx_, collision_name_);

		active_arm_ = "right_arm";
		active_arm_sym_ = 1;
		active_reset_ = RIGHT_RESET;

		res.done = true;
		if (req.action == 1) {//Spread
			if (!spread())
				res.done = false;
		} else if (req.action == 2) {//Tumble
			if (!tumble())
				res.done = false;
		}
		return true;
	}

	bool Manipulator::spread()
	{
		spread_cloud_pub_.publish(target_cloud2_);
		ROS_INFO("DUPLO: Finding central cluster");
		pcl::PointCloud<pcl::PointXYZRGB> pcd;
		fromROSMsg(target_cloud2_,pcd);
		spread_path_ = find_circular_path(pcd);

		// Publish the central markers
		for (size_t i = 0; i < spread_path_.size(); i++)
			marker_pub_.publish(set_marker("base_link","central",i,shape[1],spread_path_[i],0.02,0.0f,1.0f,0.0f,1.0));

		ROS_INFO("Calling move arm");

		//Spread
		gripper_.close();
		int move_count = 0;
		for (int plan_id = 1; plan_id <= 1; plan_id++) {
			if (move_arm_spread(spread_path_[0], 100, 0, plan_id) != -1)
			{
				move_count++;
				for (size_t i = 1; i < spread_path_.size(); i++) {
					if (move_arm_spread(spread_path_[i], 1, 0, plan_id) != -1)
						move_count++;
				}
			}
			/*		if (move_count > 3)
								break;
								else if (plan_id == 1)
								move_count = 0;
			 */
		}

		//move_arm_spread(reset_posn_.point,1,0,2);
		move_arm_spread(reset_posn_.point,1,1,2);

		ROS_INFO("Move count = %d", move_count);
		breakpoint();

		manipulate_service_called_ = true;
		if (move_count > 3)
			return true;
		else 
			return false;
	}

	bool Manipulator::tumble()
	{
		tumble_cloud_pub_.publish(target_cloud2_);
		pcl::PointCloud<pcl::PointXYZRGB> pcd;
		fromROSMsg(target_cloud2_,pcd);
		ROS_INFO("DUPLO: Finding waypoints");
		tumble_waypoints_ = find_waypoints(pcd);

		reset_posn_.header.stamp = tumble_waypoints_[0].header.stamp;
		//waypoints.push_back(reset_posn_);

		ROS_INFO("Calling move arm");
		gripper_.close();
		int move_count = 0;
		int plan_id;
		//	move_arm_tumble(waypoints[3].point);
		for (plan_id = 1; plan_id <= 2 ; plan_id++) {
			for (size_t i = 0; i < 3; i++) {
				if (move_arm_tumble(tumble_waypoints_[3*plan_id-3+i].point, 1, 0, plan_id) != -1)
					move_count++;
			}
			if (move_count >= 2)
				break;
			else if (plan_id != 2)
				move_count = 0;
		}
		move_arm_tumble(tumble_waypoints_[12].point, 1, 0, plan_id);

		/*	if (move_arm_tumble(waypoints[0].point) != -1) {
						if (move_arm_tumble(waypoints[1].point) != -1) {
							if (move_arm_tumble(waypoints[2].point) != -1) {
								move_count = 3;
								move_arm_tumble(waypoints[3].point);
				} else {
					move_count = 2;
				}
				}
				//res.done = true;
				} else {
					if (move_arm_tumble(waypoints[2].point) != -1) {
						if (move_arm_tumble(waypoints[1].point) != -1) {
							move_count = 2;
							move_arm_tumble(waypoints[3].point);
				}
				}
				}
		 */
		move_arm_tumble(reset_posn_.point, 1, 0, plan_id);
		move_arm_tumble(reset_posn_.point, 1, 1, plan_id);
		//	head.lookAt("base_link", 0.2, 0.0, 1.0);
		manipulate_service_called_ = true;
		if (move_count < 2)
			return false;
		else
			return true;
	}

	std::vector<geometry_msgs::Point> Manipulator::find_circular_path(pcl::PointCloud<pcl::PointXYZRGB> pcd)
	{ 
		sensor_msgs::PointCloud2 pcd2;
		toROSMsg(pcd, pcd2);
		ROS_INFO("DUPLO: Inside Find path");
		ROS_INFO("DUPLO: Frame id: %s",pcd2.header.frame_id.c_str());
		std::vector<geometry_msgs::Point> cloud_extent = find_extents(pcd);
		PointCloud<PointXYZRGB>::Ptr pcd_ptr(new PointCloud<PointXYZRGB>);
		*pcd_ptr = pcd;
		vector<sensor_msgs::PointCloud2> clusters = cluster(pcd_ptr,0);
		vector<int> count;
		for (size_t i = 0; i < clusters.size(); i++)
			count.push_back(0);

		for (size_t i = 0; i < clusters.size()-1; i++) {
			for (size_t j = i+1; j < clusters.size(); j++) {
				//ROS_INFO("DUPLO: Inside in contact for loop region %d",region_id);
				pcl::PointCloud<PointXYZRGB> temp1, temp2;
				fromROSMsg(clusters[i], temp1);
				fromROSMsg(clusters[j], temp2);

				if (incontact(temp1, temp2)) {
					//The two clusters are closer than 0.5 cm
					count[i] = count[i]+1;
					count[j] = count[j]+1;
				}
			}
		}

		int max_count = *max_element(count.begin(), count.end());	
		vector<sensor_msgs::PointCloud2>::iterator it2 = clusters.begin();
		sensor_msgs::PointCloud2 central_duplo2;
		PointCloud<PointXYZRGB> central;

		for (vector<int>::iterator pos = count.begin(); pos != count.end(); pos++) {
			if (*pos == max_count){
				central_duplo2 = *it2;
			}
			it2++;
		}	

		fromROSMsg(central_duplo2,central);

		vector<geometry_msgs::Point> path;
		geometry_msgs::Point center, temp;

		std::vector<geometry_msgs::Point> central_extent = find_extents(central);
		center.x = central_extent[0].x + (central_extent[1].x - central_extent[0].x)/2-0.01;
		center.y = central_extent[2].y + (central_extent[3].y - central_extent[2].y)/2;
		center.z = central_extent[4].z + (central_extent[5].z - central_extent[4].z)/2 + 0.018;
		temp = center;
		temp.z += 0.1;
		path.push_back(temp);
		temp.z -= 0.1;
		path.push_back(temp);

		if (abs(cloud_extent[1].x - center.x + 0.01) > abs(cloud_extent[0].x - center.x + 0.01)) {
			if (cloud_extent[0].x - 0.1 > 0.3) {
				temp.x -= 0.1;
			}
		}
		else if (cloud_extent[1].x + 0.1 < 0.85)
			temp.x += 0.1;
		path.push_back(temp);
		//temp.x = center.x;

		if (abs(cloud_extent[3].y - center.y) > abs(cloud_extent[2].y - center.y)) {
			if (cloud_extent[2].y - 0.1 > -0.35) {
				temp.y -= 0.1;
				path.push_back(temp);
			}
		}
		else if (cloud_extent[3].y + 0.1 < 0.3) {
			temp.y += 0.1;
			path.push_back(temp);
		}

		temp.z += 0.1;
		path.push_back(temp);

		return path;
	}

	std::vector<geometry_msgs::PointStamped> Manipulator::find_waypoints(pcl::PointCloud<pcl::PointXYZRGB> pcd)
	{
		sensor_msgs::PointCloud2 pcd2;
		toROSMsg(pcd, pcd2);

		ROS_INFO("DUPLO: Inside Find waypoints");
		ROS_INFO("DUPLO: Frame id: %s",pcd2.header.frame_id.c_str());
		vector<double> vecx, vecy, vecz;
		//float minx, miny, minz, maxx, maxy, maxz;
		for (vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> >::iterator it1 = pcd.points.begin(); it1 != pcd.points.end(); ++it1) {
			//cout << "cluster " << j << ": x = " << it1->x << " y = " << it1->y << " z = " << it1->z << endl;
			vecx.push_back(it1->x);
			vecy.push_back(it1->y);
			vecz.push_back(it1->z);
		}

		//vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> >::iterator it_minx, it_miny, it_minz,
		//it_maxx, it_maxy, it_maxz;
		vector<double>::iterator it_minx, it_miny, it_minz, it_maxx, it_maxy, it_maxz;
		it_minx = min_element(vecx.begin(), vecx.end());
		it_miny = min_element(vecy.begin(), vecy.end());
		it_minz = min_element(vecz.begin(), vecz.end());
		it_maxx = max_element(vecx.begin(), vecx.end());
		it_maxy = max_element(vecy.begin(), vecy.end());
		it_maxz = max_element(vecz.begin(), vecz.end());

		//float dim1, dim2, dim3;
		pcl::PointXYZRGB pt_minx, pt_miny, pt_minz, pt_maxx, pt_maxy, pt_maxz;
		vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> >::iterator it2 = pcd.points.begin();

		for (vector<double>::iterator pos = vecx.begin(); pos != vecx.end(); ++pos) {
			if (pos == it_minx){
				pt_minx = *it2;
			}
			if (pos == it_maxx){
				pt_maxx = *it2;
			}
			++it2;
		}

		it2 = pcd.points.begin();
		for (vector<double>::iterator pos = vecy.begin(); pos != vecy.end(); ++pos) {
			if (pos == it_miny){
				pt_miny = *it2;
			}
			if (pos == it_maxy){
				pt_maxy = *it2;
			}
			++it2;
		}

		it2 = pcd.points.begin();
		for (vector<double>::iterator pos = vecz.begin(); pos != vecz.end(); ++pos) {
			if (pos == it_minz){
				pt_minz = *it2;
			}
			if (pos == it_maxz){
				pt_maxz = *it2;
			}
			++it2;
		}

		geometry_msgs::Point xmin,xmax,ymin,ymax,zmin,zmax;
		xmin.x = pt_minx.x;
		xmin.y = pt_minx.y;
		xmin.z = pt_minx.z;
		xmax.x = pt_maxx.x;
		xmax.y = pt_maxx.y;
		xmax.z = pt_maxx.z;
		ymin.x = pt_miny.x;
		ymin.y = pt_miny.y;
		ymin.z = pt_miny.z;
		ymax.x = pt_maxy.x;
		ymax.y = pt_maxy.y;
		ymax.z = pt_maxy.z;
		zmin.x = pt_minz.x;
		zmin.y = pt_minz.y;
		zmin.z = pt_minz.z;
		zmax.x = pt_maxz.x;
		zmax.y = pt_maxz.y;
		zmax.z = pt_maxz.z;

		marker_pub_.publish(set_marker("base_link","extents",0,shape[1],xmin,0.02,0.0f,0.0f,1.0f,1.0));
		marker_pub_.publish(set_marker("base_link","extents",1,shape[1],xmax,0.02,0.0f,0.0f,1.0f,1.0));
		marker_pub_.publish(set_marker("base_link","extents",2,shape[1],ymin,0.02,0.0f,0.0f,1.0f,1.0));
		marker_pub_.publish(set_marker("base_link","extents",3,shape[1],ymax,0.02,0.0f,0.0f,1.0f,1.0));
		marker_pub_.publish(set_marker("base_link","extents",4,shape[1],zmin,0.02,0.0f,0.0f,1.0f,1.0));
		marker_pub_.publish(set_marker("base_link","extents",5,shape[1],zmax,0.02,0.0f,0.0f,1.0f,1.0));

		geometry_msgs::PointStamped wp[13];

		wp[1].point.x = pt_maxz.x-0.01;
		wp[1].point.y = pt_maxz.y;
		wp[1].point.z = pt_maxz.z;//+0.01;
		for (int i = 0; i < 12; i++) {
			wp[i] = wp[1];
		}

		wp[0].point.x += 0.07;
		wp[2].point.x -= 0.07;
		wp[3].point.y += 0.07;
		wp[5].point.y -= 0.07;
		wp[6].point.y += 0.07;
		wp[8].point.y -= 0.07;
		wp[9].point.x += 0.07;
		wp[11].point.x -= 0.07;
		wp[12].point.z += 0.1;

		std::vector<geometry_msgs::PointStamped> waypoints;
		for (int i = 0; i < 13; i++) {
			wp[i].header.stamp = ros::Time();
			wp[i].header.frame_id = "base_link";
			waypoints.push_back(wp[i]);
		}

		/*	if (abs(pt_maxz.y - pt_maxy.y) - abs(pt_maxz.y - pt_miny.y) <= 0.02) {
				wp[0].point.y = wp[1].point.y - 0.05;
				wp[2].point.y = wp[1].point.y + 0.05;
		} else {
			wp[0].point.y = wp[1].point.y + 0.05;
			wp[2].point.y = wp[1].point.y - 0.05;
		}

		wp[0].point.z = wp[1].point.z;
		wp[2].point.x = wp[1].point.x;
		wp[2].point.y = wp[1].point.y + 0.1;
		wp[2].point.z = wp[1].point.z;

		wp[0].point.x = pt_miny.x;
		wp[0].point.y = pt_miny.y;
		wp[0].point.z = pt_maxz.z;
		//	wp[0].point.z = pt_minz.z + (pt_maxz.z - pt_minz.z)/2;

		//	wp[1].point.z = pt_minz.z + (pt_maxz.z - pt_minz.z)/2;
		//	wp[1].z = pt_minz.rgb;
		wp[2].point.x = pt_maxy.x;
		wp[2].point.y = pt_maxy.y;
		wp[2].point.z = pt_maxz.z;
		//	wp[2].point.z = pt_minz.z + (pt_maxz.z - pt_minz.z)/2;
		 */

		/*	geometry_msgs::Point minx, miny, maxx, maxy, minz, maxz;
			 minx = wp[0].point.
				 marker_pub.publish(set_marker("openni_rgb_optical_frame","keypoints",0,shape[0],point,
				 0.02,0.0f,0.0f,1.0f,0.5));
		 */
		//	ROS_INFO("DUPLO: Waypoint: %f, %f, %f", wp[1].point.x, wp[1].point.y, wp[1].point.z);

		for (int i = 0; i < 3; i++) {
			// Publish the waypoints markers
			marker_pub_.publish(set_marker("base_link","waypoints",i,shape[i],wp[i].point,
					0.02,0.0f,1.0f,0.0f,1.0));
		}

		ROS_INFO("DUPLO: Exiting Finding waypoints");
		return waypoints;
	}

	void Manipulator::resetCollisionModels() {
		arm_navigation_msgs::CollisionObject reset_object;
		reset_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
		reset_object.header.frame_id = "base_link";
		reset_object.header.stamp = ros::Time::now();
		reset_object.id = "all";
		collision_object_pub_.publish(reset_object);
		//collision_object_current_id_ = 0;
	}

	void Manipulator::getClusterBoundingBox(const sensor_msgs::PointCloud2 &cluster,
			geometry_msgs::PoseStamped &pose_stamped,
			geometry_msgs::Vector3 &dimensions) {
		object_manipulation_msgs::FindClusterBoundingBox2 srv;
		srv.request.cluster = cluster;
		if (!bbx_client_.call(srv.request, srv.response)) {
			ROS_ERROR("Failed to call cluster bounding box client");
			//throw CollisionMapException("Failed to call cluster bounding box client");
		}
		pose_stamped = srv.response.pose;
		dimensions = srv.response.box_dims;
		if (dimensions.x == 0.0 && dimensions.y == 0.0 && dimensions.z == 0.0) {
			ROS_ERROR("Cluster bounding box client returned an error (0.0 bounding box)");
			//throw CollisionMapException("Bounding box computation failed");
		}
	}

	void Manipulator::processCollisionGeometryForBoundingBox(const object_manipulation_msgs::ClusterBoundingBox &box,
			std::string &collision_name) {
		ROS_INFO("Adding bounding box with dimensions %f %f %f to collision map",
				box.dimensions.x, box.dimensions.y, box.dimensions.z);

		arm_navigation_msgs::CollisionObject collision_object;
		collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

		//collision_name = getNextObjectName();
		collision_name = "object_0";
		collision_object.id = collision_name;
		//target_collision_name_ = collision_name;

		collision_object.header.frame_id = box.pose_stamped.header.frame_id;
		collision_object.header.stamp = ros::Time::now();

		arm_navigation_msgs::Shape shape;
		shape.type = arm_navigation_msgs::Shape::BOX;
		shape.dimensions.resize(3);
		shape.dimensions[0] = box.dimensions.x;
		shape.dimensions[1] = box.dimensions.y;
		shape.dimensions[2] = box.dimensions.z;
		collision_object.shapes.push_back(shape);
		collision_object.poses.push_back(box.pose_stamped.pose);

		collision_object_pub_.publish(collision_object);
	}

	void Manipulator::collision_op(std::string object1, std::string object2,
			int operation, double penetration,
			arm_navigation_msgs::CollisionOperation& cop) {
		cop.object1 = object1;
		cop.object2 = object2;
		cop.operation = operation;    //0 = Disable, 1 = Enable
		cop.penetration_distance = penetration;
	}

	//int Manipulator::move_arm_spread(geometry_msgs::Point go_to, int plan_id){
	int Manipulator::move_arm_spread(geometry_msgs::Point go_to, int cluster_id, int cluster_op, int plan_id)
	{
		actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);
		move_arm.waitForServer();
		ROS_INFO("DUPLO: Connected to server");

		ROS_INFO("Creating move arm goal");
		arm_navigation_msgs::MoveArmGoal goalA;
		goalA.motion_plan_request.group_name = "right_arm";
		goalA.motion_plan_request.num_planning_attempts = 1;
		goalA.motion_plan_request.planner_id = std::string("");
		goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
		goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);


		if (cluster_id != 100) {
			arm_navigation_msgs::CollisionObject co;
			co.id = collision_name_;
			if (cluster_op == 0)
				co.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD; //0: add, 1: remove, 2: detach and add; 3: attach and remove
			else
				co.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE; //0: add, 1: remove, 2: detach and add; 3: attach and remove
			co.header.frame_id = "base_link";
			co.header.stamp = ros::Time::now();
			arm_navigation_msgs::Shape object;
			object.type = arm_navigation_msgs::Shape::BOX;  //0: sphere; 1: box; 2: cylinder; 3: mesh
			//arm_navigation_msgs::Shape::CYLINDER;
			object.dimensions.resize(3);
			object.dimensions[0] = bbx_.dimensions.x + 0.1; // need large padding in x direction if push is in x direction bcoz after push the hand will be in collision
			object.dimensions[1] = bbx_.dimensions.y + 0.1;
			object.dimensions[2] = bbx_.dimensions.z + 0.1;
			co.shapes.push_back(object);

			co.poses.push_back(bbx_.pose_stamped.pose);
			goalA.planning_scene_diff.collision_objects.push_back(co);

			if (cluster_op == 0) {
				arm_navigation_msgs::CollisionOperation cop2;
				cop2.object1 = cop2.COLLISION_SET_ALL;
				///Setting object1 to "gripper" does not work
				//cop2.object1 = "gripper";
				cop2.object2 = collision_name_;
				cop2.operation = cop2.DISABLE;    //0 = Disable, 1 = Enable
				cop2.penetration_distance = 0.5;
				goalA.operations.collision_operations.push_back(cop2);
			}
		}

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

		ROS_INFO("Sending goal to move arm");
		//breakpoint();
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

	//int Manipulator::move_arm_tumble(geometry_msgs::Point go_to, int plan_id){
	int Manipulator::move_arm_tumble(geometry_msgs::Point go_to, int cluster_id, int cluster_op, int plan_id)
	{
		actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);
		move_arm.waitForServer();
		ROS_INFO("DUPLO: Connected to server");
		arm_navigation_msgs::MoveArmGoal goalA;

		goalA.motion_plan_request.group_name = "right_arm";
		goalA.motion_plan_request.num_planning_attempts = 3;
		goalA.motion_plan_request.planner_id = std::string("");
		goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
		goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

		if (cluster_id != 100) {
			arm_navigation_msgs::CollisionObject co;
			co.id = collision_name_;
			if (cluster_op == 0)
				co.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD; //0: add, 1: remove, 2: detach and add; 3: attach and remove
			else
				co.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE; //0: add, 1: remove, 2: detach and add; 3: attach and remove
			co.header.frame_id = "base_link";
			co.header.stamp = ros::Time::now();
			arm_navigation_msgs::Shape object;
			object.type = arm_navigation_msgs::Shape::BOX;  //0: sphere; 1: box; 2: cylinder; 3: mesh
			//arm_navigation_msgs::Shape::CYLINDER;
			object.dimensions.resize(3);
			object.dimensions[0] = bbx_.dimensions.x + 0.1; // need large padding in x direction if push is in x direction bcoz after push the hand will be in collision
			object.dimensions[1] = bbx_.dimensions.y + 0.1;
			object.dimensions[2] = bbx_.dimensions.z + 0.1;
			co.shapes.push_back(object);

			co.poses.push_back(bbx_.pose_stamped.pose);
			goalA.planning_scene_diff.collision_objects.push_back(co);

			if (cluster_op == 0) {
				arm_navigation_msgs::CollisionOperation cop2;
				cop2.object1 = cop2.COLLISION_SET_ALL;
				///Setting object1 to "gripper" does not work
				//cop2.object1 = "gripper";
				cop2.object2 = collision_name_;
				cop2.operation = cop2.DISABLE;    //0 = Disable, 1 = Enable
				cop2.penetration_distance = 0.5;
				goalA.operations.collision_operations.push_back(cop2);
			}
		}

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

	void Manipulator::breakpoint()
	{
		int user_input;
		std::cout << "Save the screen shot for sans_cluster_TTOCloud. Press 1 and then 'Enter' after you are done." << std::endl;
		std::cin >> user_input;
	}

}

int main(int argc, char **argv)
{
	//initialize the ROS node
	ros::init(argc, argv, "spread_server");
	ros::NodeHandle n;
	sort_duplos::Manipulator sp(n);

	ROS_INFO("DUPLO: Entered spread server");
	ros::spin();
	return 0;
}
