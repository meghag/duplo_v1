#include <ros/ros.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <sensor_msgs/point_cloud_conversion.h>

#include <gripper.h>
#include <robothead.h>
//#include <move_arm.h>

using namespace std;

int move_arm(geometry_msgs::Point go_to);

bool place_duplo(sensor_msgs::PointCloud2 to_place)
{
	geometry_msgs::Point reset_posn;
	reset_posn.x = 0.6;	reset_posn.y = -0.5; reset_posn.z = 0.9;
	geometry_msgs::Point red;
	red.x = 0.6;  red.y = -0.5; red.z = 0.9;  //Red bin
	geometry_msgs::Point blue;
	blue.x = 0.6;  blue.y = -0.6; blue.z = 0.9;	//Blue bin
	geometry_msgs::Point green;
	green.x = 0.5; green.y = -0.6; green.z = 0.9;  //Green bin
	
	Gripper gripper;
	
	pcl::PointCloud<pcl::PointXYZRGB> temp;
	fromROSMsg(to_place,temp);
	
	// unpack rgb into r/g/b and find the avg r/g/b of the cluster
	size_t num_pts = temp.points.size();
	//ROS_INFO("PP: Number of points in cluster = %d", num_pts);
	uint32_t rsum = 0,gsum = 0,bsum = 0;
	uint8_t r,g,b;
	for (size_t i = 0; i < num_pts; i++) {
		uint32_t rgb = *reinterpret_cast<int*>(&temp.points[i].rgb);
		rsum = rsum + ((rgb >> 16) & 0x0000ff);
		gsum = gsum + ((rgb >> 8)  & 0x0000ff);
		bsum = bsum + ((rgb)       & 0x0000ff);
		//ROS_INFO("PP: r = %d, g= %d, b = %d", rsum, gsum, bsum);
	}
	r = std::floor(rsum/num_pts);
	g = std::floor(gsum/num_pts);
	b = std::floor(bsum/num_pts);
	ROS_INFO("PP: Avg RGB of cluster: %d, %d, %d",r,g,b);
	uint8_t max_col = std::max(r,g);
	max_col = std::max(max_col,b);
	if (max_col == r) {
		ROS_INFO("PP: Placing block into the red bin.");
		move_arm(red);
	} else if (max_col == g) {
		ROS_INFO("PP: Placing block into the green bin.");
		move_arm(green);
	} else if (max_col == b) {
		ROS_INFO("PP: Placing block into the blue bin.");
		move_arm(blue);
	}// else 
	gripper.open();
	
	return true;
}
	