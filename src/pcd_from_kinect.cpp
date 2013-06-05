#include "pcd_from_kinect.h"

namespace sort_duplos {
	KinectDataReader::KinectDataReader (ros::NodeHandle & n): n_(n)
	{
		ROS_INFO("Starting read pcd node.");
		getNewDataServer_ = n_.advertiseService("get_new_pcd", &KinectDataReader::getNewDataCallback, this);

		processDataClient_ = n_.serviceClient<duplo_v1::Process_PCD>("process_pcd");

		while (!ros::service::waitForService("process_pcd", ros::Duration(2.0)) && n_.ok() ) {
			ROS_ERROR("Waiting for point cloud processing service to come up");
		}
		if (!n_.ok()) exit(0);

		inputCloudPub_ = n_.advertise<sensor_msgs::PointCloud2>("Current_input",1);
		ROS_INFO("Started input cloud topic.");
		
		newDataWanted_ = true;
		pcdDataSub_.subscribe(n_,pcdDataTopic_,1);
		tfFilter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(pcdDataSub_, listener_, "base_link", 1);
		tfFilter_->registerCallback( boost::bind(&KinectDataReader::pcdDataCallback, this, _1) );
	}
	
	KinectDataReader::~KinectDataReader (void)
	{
	}
	
	void KinectDataReader::pcdDataCallback(const sensor_msgs::PointCloud2::ConstPtr& input)             
	{
		input_ = *input;
		if (newDataWanted_) {
			ROS_INFO("New data wanted.");
			int j;
			for (j = 0; j < 3; j++) {
				if (callProcessingService()) {
					j = 0;
					break;
				}
			}
			if (j == 3) {
				ROS_ERROR("Failed to call service process_pcd. Giving up permanently.");
				ros::shutdown();
			}
		}
	}	
	
	bool KinectDataReader::callProcessingService() 
	{
		ROS_INFO("Inside call processing service.");
		//Call Processing Service
		sensor_msgs::PointCloud2 input = input_;

		//Convert input to base_link frame before any processing
		ROS_INFO("seq = %d; frame id = %s", input.header.seq, input.header.frame_id.c_str());
		//input.header.stamp = ros::Time(0);
		
		//ROS_INFO("DUPLO: input before transformation: %u data points", input.height*input.width);
		if (pcl_ros::transformPointCloud("base_link",input,input,listener_)) {
			ROS_INFO("DUPLO: transformed the point cloud");
			pcl::PointCloud<pcl::PointXYZRGB> cloud;
			pcl::fromROSMsg(input,cloud);
			
			inputCloudPub_.publish(input);
			ROS_INFO("Point cloud:cloud size = %ld", (long int)cloud.points.size());
			
			duplo_v1::Process_PCD srv;
			srv.request.pcd_in = input;
			
			if (processDataClient_.call(srv))
			{
				ROS_INFO("Processing the point cloud.");
				return true;
			} else {
				ROS_ERROR("Failed to call service process_pcd. Trying again");
				return false;
			}
			newDataWanted_ = false;
		}
		return true;
	}

	bool KinectDataReader::getNewDataCallback(duplo_v1::Get_New_PCD::Request &req,
                 duplo_v1::Get_New_PCD::Response &res)
	{
		ROS_INFO("Inside new data wanted callback.");
		newDataWanted_ = req.question;
		ROS_INFO("Set newDataWanted_ to true.");
		res.answer = true;
		return true;
	}
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "pcd_from_kinect");
	ros::NodeHandle n;
	sort_duplos::KinectDataReader kinectReader(n);
	ros::MultiThreadedSpinner spinner(3);
	spinner.spin();
	//ros::spin();
	return 0;
}
