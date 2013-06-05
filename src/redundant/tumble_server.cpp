#include "tumble.h"

namespace sort_duplos
{
	Tumbler::Tumbler(ros::NodeHandle & n): n_(n)
	{
		client_collision_processing_ = n_.serviceClient<tabletop_collision_map_processing::TabletopCollisionMapProcessing>
			(COLLISION_PROCESSING_SERVICE_NAME, true);
		tumble_service_called = false;
		tumble_service_ = n_.advertiseService("tumble_duplo", &Tumbler::tumbleCallback, this);
		marker_pub = n_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
		coll_obj_pub = n_.advertise<arm_navigation_msgs::CollisionObject>("/collision_object", 1);
		tumble_cloud_pub = n_.advertise<sensor_msgs::PointCloud2>("to_tumble",1);

		shape[0] = visualization_msgs::Marker::CUBE;
		shape[1] = visualization_msgs::Marker::SPHERE;
		shape[2] = visualization_msgs::Marker::CYLINDER;
		shape[3] = visualization_msgs::Marker::CUBE;
		//shape[4] = visualization_msgs::Marker::CUBE;

		reset_posn.point.x = 0.68;	reset_posn.point.y = -0.5; reset_posn.point.z = 1;
		reset_posn.header.frame_id = "base_link";
	}

	Tumbler::~Tumbler() {}

	bool Tumbler::tumbleCallback(duplo_v1::Tumble_Duplo::Request &req,
	                     duplo_v1::Tumble_Duplo::Response &res)
	{
		tumble_cloud_pub.publish(req.tumble_cloud);
		ROS_INFO("DUPLO: Finding waypoints");
		pcl::PointCloud<pcl::PointXYZRGB> pcd;
		fromROSMsg(req.tumble_cloud,pcd);
		waypoints_ = find_waypoints(pcd);
		
		reset_posn.header.stamp = waypoints_[0].header.stamp;
		//waypoints.push_back(reset_posn);

		// Publish the waypoints markers
		/*	for (int i = 0; i < 4; i++)
		marker_pub.publish(set_marker("base_link","basic_shapes",i,shape[i],
		waypoints[i].point,0.05,0.0f,1.0f,0.0f,0.5));
		*/	
		//Tumble
		gripper.close();
		int move_count = 0;
		int plan_id;
		//	move_arm_tumble(waypoints[3].point);
		for (plan_id = 1; plan_id <= 2 ; plan_id++) {
			for (size_t i = 0; i < 3; i++) {
				if (move_arm_tumble(waypoints_[3*plan_id-3+i].point, plan_id) != -1)
					move_count++;
			}
			if (move_count >= 2)
				break;
			else if (plan_id != 2)
				move_count = 0;
		}

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
		//	move_arm_tumble(reset_posn.point, plan_id);
		//	head.lookAt("base_link", 0.2, 0.0, 1.0);
		if (move_count < 2)
			res.done = false;
		else
			res.done = true;

		tumble_service_called = true;
		return true;
	}	

	std::vector<geometry_msgs::PointStamped> Tumbler::find_waypoints(pcl::PointCloud<pcl::PointXYZRGB> pcd)
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

		marker_pub.publish(set_marker("base_link","extents",0,shape[1],xmin,0.02,0.0f,0.0f,1.0f,1.0));
		marker_pub.publish(set_marker("base_link","extents",1,shape[1],xmax,0.02,0.0f,0.0f,1.0f,1.0));
		marker_pub.publish(set_marker("base_link","extents",2,shape[1],ymin,0.02,0.0f,0.0f,1.0f,1.0));
		marker_pub.publish(set_marker("base_link","extents",3,shape[1],ymax,0.02,0.0f,0.0f,1.0f,1.0));
		marker_pub.publish(set_marker("base_link","extents",4,shape[1],zmin,0.02,0.0f,0.0f,1.0f,1.0));
		marker_pub.publish(set_marker("base_link","extents",5,shape[1],zmax,0.02,0.0f,0.0f,1.0f,1.0));

		geometry_msgs::PointStamped wp[12];

		wp[1].point.x = pt_maxz.x-0.01;
		wp[1].point.y = pt_maxz.y;
		wp[1].point.z = pt_maxz.z;//+0.01;
		for (int i = 0; i < 12; i++) {
			wp[i] = wp[1];
		}


		wp[0].point.x += 0.05;
		wp[2].point.x -= 0.05;
		wp[3].point.y += 0.05;
		wp[5].point.y -= 0.05;
		wp[6].point.y += 0.05;
		wp[8].point.y -= 0.05;
		wp[9].point.x += 0.05;
		wp[11].point.x -= 0.05;

		std::vector<geometry_msgs::PointStamped> waypoints;
		for (int i = 0; i < 12; i++) {
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
			marker_pub.publish(set_marker("base_link","waypoints",i,shape[i],wp[i].point,
			                              0.02,0.0f,1.0f,0.0f,1.0));
		}

		ROS_INFO("DUPLO: Exiting Finding waypoints");
		return waypoints;
	}

}

int main(int argc, char **argv)
{
	//initialize the ROS node
	ros::init(argc, argv, "tumble_server");
	ros::NodeHandle n;
	ROS_INFO("DUPLO: Entered tumble server");
	sort_duplos::Tumbler tm(n);
	ros::spin();
	return 0;
}

// Publish the allowed contacts marker
//marker_pub.publish(set_marker("base_link","allowed_contacts",4,3,
//                            waypoints[1].point,0.5,1.0f,0.0f,0.0f,0.3));

