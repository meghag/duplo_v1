#include "spread.h"

namespace sort_duplos
{
	Spreader::Spreader(ros::NodeHandle & n): n_(n)
	{
		spread_service_called = false;
		spread_service_ = n_.advertiseService("spread_duplo", &Spreader::spreadCallback, this);
		marker_pub = n_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
		//coll_obj_pub = n_.advertise<arm_navigation_msgs::CollisionObject>("/collision_object", 1);
		spread_cloud_pub = n_.advertise<sensor_msgs::PointCloud2>("to_spread",1);
	
		shape[0] = visualization_msgs::Marker::CUBE;
		shape[1] = visualization_msgs::Marker::SPHERE;
		shape[2] = visualization_msgs::Marker::CYLINDER;
		shape[3] = visualization_msgs::Marker::CUBE;
		//shape[4] = visualization_msgs::Marker::CUBE;

		reset_posn.point.x = 0.68;	reset_posn.point.y = -0.5; reset_posn.point.z = 1;
		reset_posn.header.frame_id = "base_link";
	}

	Spreader::~Spreader() {}

	bool Spreader::spreadCallback(duplo_v1::Spread_Duplo::Request &req,
	                              duplo_v1::Spread_Duplo::Response &res)
	{
		spread_cloud_pub.publish(req.spread_cloud);
		ROS_INFO("DUPLO: Finding central cluster");
		pcl::PointCloud<pcl::PointXYZRGB> pcd;
		fromROSMsg(req.spread_cloud,pcd);
		path_ = find_circular_path(pcd);

		// Publish the central markers
		for (size_t i = 0; i < path_.size(); i++)
			marker_pub.publish(set_marker("base_link","central",i,shape[1],path_[i],0.02,0.0f,1.0f,0.0f,1.0));

		//Spread
		gripper.close();
		int move_count = 0;
		for (int plan_id = 1; plan_id <= 1; plan_id++) {
			for (size_t i = 0; i < path_.size(); i++) {
				if (move_arm_spread(path_[i],plan_id) != -1)
					move_count++;
			}
			/*		if (move_count > 3)
			break;
			else if (plan_id == 1)
			move_count = 0;
			*/
		}

		//	move_arm_spread(reset_posn.point,2);

		if (move_count > 3)
			res.done = true;
		else 
			res.done = false;

		spread_service_called = true;
		return true;
	}

	std::vector<geometry_msgs::Point> Spreader::find_circular_path(pcl::PointCloud<pcl::PointXYZRGB> pcd)
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
				//ROS_INFO("DUPLO: IOnside in contact for loop region %d",region_id);
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
		//int dirn = 1;
		//	for (size_t i = 0; i <= ceil(clusters.size()/2); i++) {
		//		PointCloud<PointXYZRGB> central;
		//		fromROSMsg(clusters[i],central);
		std::vector<geometry_msgs::Point> central_extent = find_extents(central);
		//geometry_msgs::Point center;
		center.x = central_extent[0].x + (central_extent[1].x - central_extent[0].x)/2-0.01;
		center.y = central_extent[2].y + (central_extent[3].y - central_extent[2].y)/2;
		center.z = central_extent[4].z + (central_extent[5].z - central_extent[4].z)/2;
		temp = center;
		temp.z += 0.1;
		path.push_back(temp);
		temp.z -= 0.1;
		path.push_back(center);

		if (abs(cloud_extent[1].x - center.x + 0.01) > abs(cloud_extent[0].x - center.x + 0.01)) {
			if (cloud_extent[0].x - 0.05 > 0.3) {
				temp.x -= 0.05;
			}
		}
		else if (cloud_extent[1].x + 0.05 < 0.85)
			temp.x += 0.05;
		path.push_back(temp);
		//temp.x = center.x;

		if (abs(cloud_extent[3].y - center.y) > abs(cloud_extent[2].y - center.y)) {
			if (cloud_extent[2].y - 0.05 > -0.35) {
				temp.y -= 0.05;
			}
		}
		else if (cloud_extent[3].y + 0.05 < 0.3) {
			temp.y += 0.05;
		}
		path.push_back(temp);

		temp.z += 0.1;
		path.push_back(temp);

		return path;
	}
}

int main(int argc, char **argv)
{
	//initialize the ROS node
	ros::init(argc, argv, "spread_server");
	ros::NodeHandle n;
	sort_duplos::Spreader sp(n);

	ROS_INFO("DUPLO: Entered spread server");
	ros::spin();
	return 0;
}