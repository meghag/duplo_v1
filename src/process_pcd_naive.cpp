#include "process_pcd_naive.h"

namespace sort_duplos {
	DataProcessor::DataProcessor(ros::NodeHandle & n): n_(n) 
	{
		processed_pointcloud_ = false;
		new_cloud_wanted_ = false;
		nocluster_count_ = 0;
		process_service_ = n_.advertiseService("process_pcd", &DataProcessor::processDataCallback, this);
		pub_active_region_ = n_.advertise<sensor_msgs::PointCloud2>("activeregion",1);
		marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
		regions_pub_ = n_.advertise<visualization_msgs::MarkerArray>("region_nums", 1);

		//ros::Subscriber sub = n.subscribe("/camera/rgb/points",1,got_point_cloud);
		//client_grasp_ = n_.serviceClient<duplo_v1::Grasp_Duplo>("grasp_duplo");
		client_manipulate_ = n_.serviceClient<duplo_v1::Manipulate_Duplo>("manipulate_duplo");
		//client_tumble_ = n_.serviceClient<duplo_v1::Tumble_Duplo>("tumble_duplo");
		reset_posn.x = 0.6;	reset_posn.y = -0.5; reset_posn.z = 1;

		while ( !ros::service::waitForService("get_new_pcd",ros::Duration(2.0)) && n.ok() ) 
			ROS_INFO("DUPLO: Waiting for object detection service to come up");
		if (!n.ok()) exit(0);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp1(new pcl::PointCloud<pcl::PointXYZRGB>);
		object_cloud_ = temp1;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZRGB>);
		planar_cloud_ = temp2;
		
		ROS_INFO("Ready to process input point cloud data.");
		client_newpcd_ = n.serviceClient<duplo_v1::Get_New_PCD>("get_new_pcd");
	}

	DataProcessor::~DataProcessor() {}

	void DataProcessor::pubCloud(const std::string &topic_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::string frame_id)
	{
	    ros::Publisher *cloud_pub;

	    if (cloud_publishers.find(topic_name) == cloud_publishers.end())
	    {

	        cloud_pub = new ros::Publisher();
	        *cloud_pub = n_.advertise<sensor_msgs::PointCloud2>(topic_name,0,true);

	        cloud_publishers.insert(std::pair<std::string, ros::Publisher*>(topic_name, cloud_pub ));
	        //std::cout << "created new publisher" << cloud_pub << std::endl;
	    }
	    else
	    {
	        cloud_pub = cloud_publishers.find(topic_name)->second;
	        //std::cout << "found pub on " << cloud_pub->getTopic() << ", reusing it" << std::endl;
	    }

	    sensor_msgs::PointCloud2 out; //in map frame

	    pcl::toROSMsg(*cloud,out);
	    out.header.frame_id = frame_id;
	    out.header.stamp = ros::Time::now();
	    cloud_pub->publish(out);

	    ROS_INFO("Published a cloud on topic %s", topic_name.c_str());
	    //ROS_INFO("published frame %s %i x %i points on %s", out.header.frame_id.c_str(), out.height, out.width, topic_name.c_str());
	}

	bool DataProcessor::processDataCallback(duplo_v1::Process_PCD::Request &req,
	                                        duplo_v1::Process_PCD::Response &res)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudz(new pcl::PointCloud<pcl::PointXYZRGB>);	//Filtered cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZRGB>); //Inliers after removing outliers
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr everything_else(new pcl::PointCloud<pcl::PointXYZRGB>); 

		pcl::fromROSMsg(req.pcd_in,*cloud);
		ROS_INFO("Inside process.");
		get_object_cloud(cloud,planar_cloud_,object_cloud_);
		processed_pointcloud_ = true;
		res.done = true;

		process();
		ROS_INFO("\n \n");
		ros::Duration(2).sleep();
		return true;
	}

	int DataProcessor::get_object_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_orig, 
	                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr planar_cloud,
	                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);	//Filtered cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr everything_else(new pcl::PointCloud<pcl::PointXYZRGB>);

		/****************** Filter out the non-table points ******************/
		pass_through_gen(pcd_orig,pcd_filtered,true,0,1.0,true,-0.6,0.6,true,0,0.8);
		//pcl::io::savePCDFileASCII ("scene3.pcd", *pcd_filtered);

		/* ********* Segmentation of the cloud into table and object clouds **********/
		string str1 = "table_try.pcd", str2 = "everything_else.pcd";
		planar_seg(pcd_filtered,planar_cloud,everything_else,str1.c_str(), str2.c_str());
		//planar_seg(pcd_filtered,planar_cloud,everything_else,"table_try.pcd","everything_else.pcd");
		pubCloud("planar_cloud", planar_cloud, "base_link");

		/*********** Find dimensions of table **********/
		std::vector<geometry_msgs::Point> extent = find_extents(*planar_cloud);
		table_extent_ = extent;

		/********** Now filter the object cloud based on table dimensions *********/
		pass_through_gen(everything_else,object_cloud,true,extent[0].x, extent[1].x-0.05,
		                 true,extent[2].y+0.01,extent[3].y-0.005,true,extent[5].z-0.01,extent[5].z+0.15);
		pubCloud("object_cloud", object_cloud, "base_link");

		//pcl::io::savePCDFileASCII ("object_try.pcd", *object_cloud);
		return (0);
	}

	void DataProcessor::process() 
	{
		vector<pcl::PointCloud<pcl::PointXYZRGB> > region_cloud;
		//vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > region_ptr;
		vector<geometry_msgs::Point> object_extent = find_extents(*object_cloud_);
		marker_pub_.publish(set_boundaries("base_link","obj_extent",1,
		                                  visualization_msgs::Marker::LINE_STRIP,visualization_msgs::Marker::ADD,
		                                  object_extent,0.005,1.0f,0.0f,0.0f,1.0));

		//Pick up every block
		duplo_v1::Manipulate_Duplo grasp_call;
		vector<sensor_msgs::PointCloud2> bricks = cluster(object_cloud_, 0);
		ROS_INFO("Need to pick up %zu bricks", bricks.size());

		if (bricks.size() == 0) {
			nocluster_count_++;
			new_cloud_wanted_ = true;
		} else {
			nocluster_count_ = 0;
			pick_n_place(bricks[0]);
			new_cloud_wanted_ = true;
		}

		if (nocluster_count_ == 3) {
			ROS_INFO("Table cleared!");
			ros::shutdown();
		}

		if (new_cloud_wanted_) {
			ROS_INFO("DUPLO: Asking for new point cloud.");
			//ros::Duration(2).sleep();

			duplo_v1::Get_New_PCD srv_newpcd;
			srv_newpcd.request.question = true;
			if (client_newpcd_.call(srv_newpcd))
				ROS_INFO("DUPLO: Requesting for new point cloud.");
			else {
				ROS_ERROR("Failed to call service get new pcd.");
				return;
			}
			new_cloud_wanted_ = false;
		}
	}  

	bool DataProcessor::pick_n_place(sensor_msgs::PointCloud2 to_pick)
	{
		//Pick up every block
		duplo_v1::Manipulate_Duplo grasp_call;
		grasp_call.request.target_cloud.push_back(to_pick);
		grasp_call.request.action = 0;
		grasp_call.request.table_extent = table_extent_;
		sensor_msgs::PointCloud2 active_region2 = to_pick;
		//toROSMsg(to_pick, active_region2);
		active_region2.header.frame_id = "base_link";
		pub_active_region_.publish(active_region2);

		ROS_INFO("DUPLO: Picking the brick....");
		if (client_manipulate_.call(grasp_call)) {
			if (grasp_call.response.done) {
				ROS_INFO("DUPLO: Placed the duplo");
				return true;
			} else {
				ROS_ERROR("Pick and place service called but grasping failed.");
				return false;
			}
		} else {
			ROS_ERROR("Failed to call grasp duplo service.");
			return false;
		}
		//new_cloud_wanted_ = true;
		return true;
	}
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "process_pcd_naive");
	ros::NodeHandle n;
	sort_duplos::DataProcessor dp(n);
	ros::spin();
}
