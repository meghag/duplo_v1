#include "incontact.h"

bool incontact(pcl::PointCloud<PointXYZRGB> cloud1, pcl::PointCloud<PointXYZRGB> cloud2)
{
	//Concatenate the 2 clouds
	pcl::PointCloud<PointXYZRGB> cloud_concat = cloud1;
	cloud_concat += cloud2;
	pcl::PointCloud<PointXYZRGB>::Ptr concat(new PointCloud<PointXYZRGB>);
	*concat = cloud_concat;

	pcl::io::savePCDFileASCII ("concat_2bricks.pcd", cloud_concat);

	//Do Euclidean clustering and find number of clusters
	// Creating the KdTree object for the search method of the extraction
	//Electric: pcl::KdTree<PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<PointXYZRGB>);
	// Fuerte:
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (concat);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance (0.02); // 1.5 cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (2000);
	ec.setSearchMethod (tree);
	ec.setInputCloud(concat);
	ec.extract (cluster_indices);
	if (cluster_indices.size() <= 1)
		//The two clusters are closer than 1.5 cm
		return true;
	else
		return false;
}
