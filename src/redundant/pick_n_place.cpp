#include "pick_n_place.h"
//#include "pcd_from_kinect.h"

namespace sort_duplos{
	DuploGrasper::DuploGrasper (ros::NodeHandle & n): n_(n)
	{
		ROS_INFO("Starting grasp duplo node.");
		targetObjectPub_ = n_.advertise<sensor_msgs::PointCloud>("target_object",1);
		graspDuploServer_ = n_.advertiseService("grasp_duplo", &DuploGrasper::graspDuploCallback, this);
		getNewDataClient_ = n_.serviceClient<duplo_v1::Get_New_PCD>("get_new_pcd");

		//wait for detection client
		while ( !ros::service::waitForService(OBJECT_DETECTION_SERVICE_NAME, 
		                                      ros::Duration(2.0)) && n_.ok() ) 
			ROS_INFO("Waiting for object detection service to come up");
		if (!n_.ok()) exit(0);

		objectDetectionClient_ = n_.serviceClient<tabletop_object_detector::TabletopDetection>
			(OBJECT_DETECTION_SERVICE_NAME, true);

		//wait for collision map processing client
		while ( !ros::service::waitForService(COLLISION_PROCESSING_SERVICE_NAME, 
		                                      ros::Duration(2.0)) && n_.ok() )
			ROS_INFO("Waiting for collision processing service to come up");
		if (!n_.ok()) exit(0);

		collisionProcessingClient_ = n_.serviceClient<tabletop_collision_map_processing::TabletopCollisionMapProcessing>
			(COLLISION_PROCESSING_SERVICE_NAME, true);

		RESET_POSN.x = 0.6;	RESET_POSN.y = -0.5; RESET_POSN.z = 0;
		RED.x = 0.6;  RED.y = -0.5; RED.z = -0.1;  //Red bin
		BLUE.x = 0.6;  BLUE.y = -0.6; BLUE.z = -0.1;	//Blue bin
		GREEN.x = 0.6; GREEN.y = -0.7; GREEN.z = -0.1;  //Green bin

		TabletopDetection();
		CollisionMapProcessing();
	}

	DuploGrasper::~DuploGrasper (void)
	{
	}

	bool DuploGrasper::graspDuploCallback (duplo_v1::Grasp_Duplo::Request &req,
	                                       duplo_v1::Grasp_Duplo::Response &res)
	{
		//ROS_INFO("Publishing the point cloud to be picked up.");
		targetObject_ = req.pick_cluster;
		PickupObject();
		
		return true;
	}

	bool DuploGrasper::TabletopDetection()
	{
		ROS_INFO("PP: Calling tabletop detector");
		
		//we want recognized database objects returned
		//set this to false if you are using the pipeline without the database
		detectionCall_.request.return_clusters = true;
		//we want the individual object point clouds returned as well
		detectionCall_.request.return_models = false;
		if (!objectDetectionClient_.call(detectionCall_))
		{
			ROS_ERROR("PP: Tabletop detection service failed");
			return false;
		}
		if (detectionCall_.response.detection.result != 
		    detectionCall_.response.detection.SUCCESS)
		{
			ROS_ERROR("PP: Tabletop detection returned error code %d", 
			          detectionCall_.response.detection.result);
			return false;
		}
		if (detectionCall_.response.detection.clusters.empty() && 
		    detectionCall_.response.detection.models.empty() )
		{
			ROS_DEBUG("PP: The tabletop detector detected the table, but found no objects");
			//return false;
		}
		detectionCall_.response.detection.clusters.clear();
		return true;
	}

	bool DuploGrasper::CollisionMapProcessing()
	{
		ROS_INFO("PP: Calling collision map processing");
		
		//pass the result of the tabletop detection 
		processingCall_.request.detection_result = detectionCall_.response.detection;
		//ask for the exising map and collision models to be reset
		//processingCall_.request.reset_static_map = true;
		processingCall_.request.reset_collision_models = false;
		processingCall_.request.reset_attached_models = false;
		//ask for a new static collision map to be taken with the laser
		//after the new models are added to the environment
		//processingCall_.request.take_static_collision_map = false;
		//ask for the results to be returned in base link frame
		processingCall_.request.desired_frame = "base_link";
		if (!collisionProcessingClient_.call(processingCall_))
		{
			ROS_ERROR("PP: Collision map processing service failed");
			return false;
		}
		//the collision map processor returns instances of graspable objects
		if (processingCall_.response.graspable_objects.empty())
		{
			ROS_DEBUG("PP: Collision map processing returned no graspable objects");
			//return false;
		}
		return true;
	}

	bool DuploGrasper::PickupObject()
	{
		ROS_INFO("Calling the pickup action");

		actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> pickupClient_(PICKUP_ACTION_NAME, true);
		//wait for pickup client
		while(!pickupClient_.waitForServer(ros::Duration(2.0)) && n_.ok())
			ROS_INFO_STREAM("Waiting for action client " << PICKUP_ACTION_NAME);
		if (!n_.ok()) exit(0);  
		
		geometry_msgs::Point RESET_POSN;
		RESET_POSN.x = 0.6;	RESET_POSN.y = -0.5; RESET_POSN.z = 0;
		
		object_manipulation_msgs::GraspableObject object;
		sensor_msgs::PointCloud goal_pcd;
		sensor_msgs::PointCloud2 pick_cluster;
		//pickupGoal_.target = processingCall_.response.graspable_objects.at(0);
		//pickupGoal_.collision_object_name = processingCall_.response.collision_object_names.at(0);


		if (sensor_msgs::convertPointCloud2ToPointCloud(targetObject_[0],goal_pcd) == true)
		{
			ROS_INFO("PP: Frame Id of input point cloud cluster is: %s\n", goal_pcd.header.frame_id.c_str());
			ROS_INFO("PP: Target frame id is: %s\n", detectionCall_.response.detection.table.pose.header.frame_id.c_str());
			goal_pcd.header.frame_id = "base_link";
			goal_pcd.header.stamp = ros::Time::now();

			object.cluster = goal_pcd;
			object.reference_frame_id = "base_link";
			pickupGoal_.target = object;
			targetObjectPub_.publish(goal_pcd);
			ROS_INFO("PP: Set the goal target as a graspable object\n");
		} else {
			ROS_ERROR("PP: Conversion from pointcloud2 to pointcloud failed.\n");
			return false;
		}


		//pass the collision name of the table, also returned by the collision map processor
		pickupGoal_.collision_support_surface_name = 
			processingCall_.response.collision_support_surface_name;

		//Allow collisions with the table
		pickupGoal_.allow_gripper_support_collision = true;

		//pick up the object with the right arm
		pickupGoal_.arm_name = "right_arm";
		//specify the desired distance between pre-grasp and final grasp
		//pickupGoal_.desired_approach_distance = 0.1;
		//pickupGoal_.min_approach_distance = 0.05;
		//we will be lifting the object along the "vertical" direction
		//which is along the z axis in the base_link frame
		geometry_msgs::Vector3Stamped direction;
		direction.header.stamp = ros::Time::now();
		direction.header.frame_id = "base_link";
		direction.vector.x = 0;
		direction.vector.y = 0;
		direction.vector.z = 1;
		pickupGoal_.lift.direction = direction;
		//request a vertical lift of 10cm after grasping the object
		pickupGoal_.lift.desired_distance = 0.1;
		pickupGoal_.lift.min_distance = 0.05;
		//do not use tactile-based grasping or tactile-based lift
		pickupGoal_.use_reactive_lift = false;
		pickupGoal_.use_reactive_execution = false;
		//pickupGoal_.only_perform_feasibility_test = true;
		//pickupGoal_.ignore_collisions = false;

		//send the goal
		pickupClient_.sendGoal(pickupGoal_);
		while (!pickupClient_.waitForResult(ros::Duration(5.0)))
		{
			ROS_INFO("PP: Waiting for the pickup action...");
		}
		object_manipulation_msgs::PickupResult pickup_result = *(pickupClient_.getResult());
		if (pickupClient_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_ERROR("PP: The pickup action has failed with result code %d", 
			          pickup_result.manipulation_result.value);
			pickupSuccess_ = false;
			//return false;
		} 
		else 
		{
			ROS_INFO("PP: The pickup action succeeded.");   
			pickupSuccess_ = true;
		}
		if (pickupSuccess_) {
			PlaceObject();
		} else {
			move_arm(RESET_POSN);
		}
		//head_.lookAt("base_link", 0.2, 0.0, 1.0);
		head_.lookAt("base_link", 0.2, -0.1, 1.35);
		gripper_.open();
		ros::Duration(2).sleep();

		// Ask for new data 
		//ROS_INFO("PP: Requesting for new point cloud.");
		//kinectReader.newDataWanted_ = true;
	
		duplo_v1::Get_New_PCD getNewDataSrv;
		getNewDataSrv.request.question = true;

		ROS_INFO("PP: Requesting for new point cloud.");
		if (getNewDataClient_.call(getNewDataSrv)) {
			ROS_INFO("PP: Requested for new point cloud.");
			return true;
		} else {
			ROS_ERROR("PP: Failed to call service get new pcd.");
			return false;
		}
	
	} //end function PickupObject

	void DuploGrasper::PlaceObject()
	{
		geometry_msgs::Point big, medium, small;
		big = RED; 		medium = BLUE;		small = GREEN;
		
		pcl::PointCloud<pcl::PointXYZRGB> temp;
		fromROSMsg(targetObject_[0],temp);

		
		//Sorting by Color 
		// unpack rgb into r/g/b and find the avg r/g/b of the cluster
		size_t num_pts = temp.points.size();
		uint32_t rsum = 0,gsum = 0,bsum = 0;
		uint8_t r,g,b;
		for (size_t i = 0; i < num_pts; i++) {
			uint32_t rgb = *reinterpret_cast<int*>(&temp.points[i].rgb);
			rsum = rsum + ((rgb >> 16) & 0x0000ff);
			gsum = gsum + ((rgb >> 8)  & 0x0000ff);
			bsum = bsum + ((rgb)       & 0x0000ff);
		}
		r = std::floor(rsum/num_pts);
		g = std::floor(gsum/num_pts);
		b = std::floor(bsum/num_pts);
		ROS_INFO("PP: Avg RGB of cluster: %d, %d, %d",r,g,b);
		uint8_t max_col = std::max(r,g);
		max_col = std::max(max_col,b);
		if (max_col == r) {
			ROS_INFO("PP: Placing block into the red bin.");
			move_arm(RED);
		} else if (max_col == g) {
			ROS_INFO("PP: Placing block into the green bin.");
			move_arm(GREEN);
		} else if (max_col == b) {
			ROS_INFO("PP: Placing block into the blue bin.");
			move_arm(BLUE);
		}
	
/*
		//Sorting by size
		float longest_dim = findLongestDim(temp);
		ROS_INFO("PP: Longest dimension of cluster = %f", longest_dim);				   
		if (longest_dim > 0.09) {
			//Place in size A bin
			ROS_INFO("PP: Placing in big bin");
			move_arm(big);
		} else if (longest_dim >= 0.05 && longest_dim <= 0.08) {
			//Place in size B bin
			ROS_INFO("PP: Placing in medium bin");
			move_arm(medium);
		} else if (longest_dim < 0.05) {
			//Place in size C bin
			ROS_INFO("PP: Placing in small bin");
			move_arm(small);
		}
*/
		gripper_.open();
		//move_arm(reset_posn);			
	} //end function PlaceObject	

	float DuploGrasper::findLongestDim(pcl::PointCloud<pcl::PointXYZRGB> pcd)
	{ 
		vector<double> vecx, vecy, vecz;
		for (vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> >::iterator it1 = pcd.points.begin(); it1 != pcd.points.end(); ++it1) {
			vecx.push_back(it1->x);
			vecy.push_back(it1->y);
			vecz.push_back(it1->z);
		}

		vector<double>::iterator it_minx, it_miny, it_minz, it_maxx, it_maxy, it_maxz;
		it_minx = min_element(vecx.begin(), vecx.end());
		it_miny = min_element(vecy.begin(), vecy.end());
		it_minz = min_element(vecz.begin(), vecz.end());
		it_maxx = max_element(vecx.begin(), vecx.end());
		it_maxy = max_element(vecy.begin(), vecy.end());
		it_maxz = max_element(vecz.begin(), vecz.end());

		float dim1, dim2, dim3;
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

		dim1 = pow((pt_maxx.x - pt_minx.x),2) + pow((pt_maxx.y - pt_minx.y),2) + 
			pow((pt_maxx.z - pt_minx.z),2);
		dim2 = pow((pt_maxy.x - pt_miny.x),2) + pow((pt_maxy.y - pt_miny.y),2) + 
			pow((pt_maxy.z - pt_miny.z),2);
		dim3 = pow((pt_maxz.x - pt_minz.x),2) + pow((pt_maxz.y - pt_minz.y),2) + 
			pow((pt_maxz.z - pt_minz.z),2);

		float max_dim = std::max(dim1, dim2);
		max_dim = std::max(max_dim,dim3);
		return (sqrt(max_dim));
	}

} //end namespace

int main(int argc, char **argv)
{
	//initialize the ROS node
	ros::init(argc, argv, "grasp_duplo");
	ros::NodeHandle n;
	sort_duplos::DuploGrasper grasper(n);
	ros::spin();
	return 0;
}
