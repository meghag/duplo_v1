#include <ros/ros.h>
#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace pcl;

std::vector<geometry_msgs::Point> find_extents(PointCloud<PointXYZRGB> pcd)
{ 
	//ROS_INFO("Inside find_extents");
	vector<double> vecx, vecy, vecz;
	//float minx, miny, minz, maxx, maxy, maxz;
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

	geometry_msgs::Point vertices[6];
	vertices[0].x = pt_minx.x;
	vertices[0].y = pt_minx.y;
	vertices[0].z = pt_minx.z;
	vertices[1].x = pt_maxx.x;
	vertices[1].y = pt_maxx.y;
	vertices[1].z = pt_maxx.z;
	vertices[2].x = pt_miny.x;
	vertices[2].y = pt_miny.y;
	vertices[2].z = pt_miny.z;
	vertices[3].x = pt_maxy.x;
	vertices[3].y = pt_maxy.y;
	vertices[3].z = pt_maxy.z;
	vertices[4].x = pt_minz.x;
	vertices[4].y = pt_minz.y;
	vertices[4].z = pt_minz.z;
	vertices[5].x = pt_maxz.x;
	vertices[5].y = pt_maxz.y;
	vertices[5].z = pt_maxz.z;
		
	std::vector<geometry_msgs::Point> extent;
	for (int i =0; i<6; i++)
		extent.push_back(vertices[i]);

	return extent;
}

bool incontact(pcl::PointCloud<PointXYZRGB> cloud1, pcl::PointCloud<PointXYZRGB> cloud2)
{
	//Concatenate the 2 clouds
	pcl::PointCloud<PointXYZRGB> cloud_concat = cloud1;
	cloud_concat += cloud2;
	pcl::PointCloud<PointXYZRGB>::Ptr concat(new PointCloud<PointXYZRGB>);
	*concat = cloud_concat;

	//pcl::io::savePCDFileASCII ("concat_2bricks.pcd", cloud_concat);

	//Do Euclidean clustering and find number of clusters
	// Creating the KdTree object for the search method of the extraction
	//Electric: pcl::KdTree<PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<PointXYZRGB>);
	// Fuerte:
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (concat);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance (0.005); // 0.5 cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (2000);
	ec.setSearchMethod (tree);
	ec.setInputCloud(concat);
	ec.extract (cluster_indices);
	if (cluster_indices.size() <= 1)
		//The two clusters are closer than 0.5 cm
		return true;
	else
		return false;
}
