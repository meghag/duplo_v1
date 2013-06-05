#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

using namespace std;
using namespace pcl;

float find_longest_dim(pcl::PointCloud<pcl::PointXYZRGB> pcd)
{ 
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

	std::vector<float> dim;
	dim.push_back(dim1);
	dim.push_back(dim2);
	dim.push_back(dim3);

	float max_dim = std::max(dim1, dim2);
	max_dim = std::max(max_dim,dim3);
	return (sqrt(max_dim));
}