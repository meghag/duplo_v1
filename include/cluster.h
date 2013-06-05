#ifndef CLUSTER_H 
#define CLUSTER_H

#include <ros/ros.h>
#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace pcl;

void
extract_color_clusters (const PointCloud<PointXYZRGB> &cloud, 
                        const std::vector<int> &indices,
                        const boost::shared_ptr<pcl::KdTree<PointXYZRGB> > &tree,
                        float tolerance, std::vector<PointIndices> &clusters,
                        unsigned int min_pts_per_cluster, 
                        unsigned int max_pts_per_cluster);
/* Fuerte::
extract_color_clusters (const PointCloud<PointXYZRGB> &cloud, 
                        const std::vector<int> &indices,
                        const boost::shared_ptr<pcl::search::KdTree<PointXYZRGB> > &tree,
                        float tolerance, std::vector<PointIndices> &clusters,
                        unsigned int min_pts_per_cluster, 
                        unsigned int max_pts_per_cluster);
*/
vector<sensor_msgs::PointCloud2> cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr o_cloud, int region_id);

vector<sensor_msgs::PointCloud2> cluster_regions(pcl::PointCloud<pcl::PointXYZRGB>::Ptr o_cloud, int region_id);

#endif