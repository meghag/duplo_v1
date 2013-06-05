/*
 *  pcd_from_kinect.h
 *  
 *
 *  Created by Megha Gupta on 09/20/12.
 *  Copyright 2011 USC. All rights reserved.
 *
 */

#ifndef PCD_FROM_KINECT_H
#define PCD_FROM_KINECT_H

#include <string.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include "duplo_v1/Process_PCD.h"
#include "duplo_v1/Get_New_PCD.h"
//#include "grasp_duplo.h"

//const::std::string pcdDataTopic_ = "/camera/rgb/points";  //Electric
//const::std::string pcdDataTopic_ = "/narrow_stereo_textured/points2";
const::std::string pcdDataTopic_ = "/head_mount_kinect/depth_registered/points";		//Fuerte

namespace sort_duplos {
	class KinectDataReader 
	{
	public:
		KinectDataReader (ros::NodeHandle & n);
		~KinectDataReader (void);
		//friend class DuploGrasper;
		
	private:
		//Functions
		void pcdDataCallback(const::sensor_msgs::PointCloud2::ConstPtr &);
		bool getNewDataCallback(duplo_v1::Get_New_PCD::Request &req,
								duplo_v1::Get_New_PCD::Response &res);
		bool callProcessingService (void);
		
		//Variables
		ros::NodeHandle n_;
//		ros::Subscriber kinectDataSub_;
		ros::Publisher inputCloudPub_;
		ros::ServiceServer getNewDataServer_;
		ros::ServiceClient processDataClient_;
		message_filters::Subscriber<sensor_msgs::PointCloud2> pcdDataSub_;
		tf::TransformListener listener_;
		tf::MessageFilter<sensor_msgs::PointCloud2> * tfFilter_;
		
		sensor_msgs::PointCloud2 input_;
		//bool newDataArrived_;
		bool newDataWanted_;
		
	};
}

#endif
