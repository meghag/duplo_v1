#include "process_pcd.h"

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
		pass_through_gen(pcd_orig,pcd_filtered,true,0,1.0,true,-0.6,0.6,true,0,1.0);
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
		                 true,extent[2].y+0.01,extent[3].y-0.005,true,extent[5].z-0.01,extent[5].z+0.11);
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

		state_.clear();
		regions_ = find_regions(object_cloud_);	
		if (regions_.size() == 0) {
			nocluster_count_++;
			new_cloud_wanted_ = true;
		} else {
			nocluster_count_ = 0;
			//sensor_msgs::PointCloud2 region_publish[4];
			for (size_t i = 0; i < regions_.size(); i++) {
				pcl::PointCloud<PointXYZRGB> temp_cloud;
				PointCloud<PointXYZRGB>::Ptr temp_ptr(new PointCloud<PointXYZRGB>);
				fromROSMsg(regions_[i],temp_cloud);
				*temp_ptr = temp_cloud;

				//Publish region
				stringstream ss;
				ss << "region_" <<  i;
				pubCloud(ss.str(), temp_ptr, "base_link");

				//ROS_INFO("test: Region %zu: %zu data points, also %zu", i+1,temp_cloud.points.size(),temp_ptr->points.size());
				region_cloud.push_back(temp_cloud);
				region_ptr.push_back(temp_ptr);
				//toROSMsg(region_cloud[i],region_publish[i]);
				//state_.push_back(make_pair(i+1,find_state(temp_ptr,i+1)));
				state_.push_back(make_pair(temp_ptr, find_state(temp_ptr,i+1)));
				marker_pub_.publish(set_boundaries("base_link","region_extent",i+1,
						visualization_msgs::Marker::LINE_STRIP,visualization_msgs::Marker::ADD,
						find_extents(temp_cloud),0.01,0.0f,0.0f,1.0f,1.0));
			}
			assign_manipulation_primitive();
		}

		if (nocluster_count_ == 10) {
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

	vector<sensor_msgs::PointCloud2> DataProcessor::find_regions(pcl::PointCloud<PointXYZRGB>::Ptr input)
	{
		vector<sensor_msgs::PointCloud2> regions;
		if (input->points.size() >= 50) {
			//Do Euclidean clustering and find number of clusters

			// Creating the KdTree object for the search method of the extraction
			//Electric: pcl::KdTree<PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<PointXYZRGB>);
			// Fuerte:  
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
			tree->setInputCloud (input);

			std::vector<pcl::PointIndices> cluster_indices;
			pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
			ec.setClusterTolerance (0.02); // 3 cm
			//ec.setMinClusterSize (150);
			ec.setMinClusterSize(50);
			ec.setMaxClusterSize (30000);
			ec.setSearchMethod (tree);
			ec.setInputCloud(input);
			ec.extract (cluster_indices);

			PCDWriter writer;


			ROS_INFO("Found %zu regions", cluster_indices.size());
			if (cluster_indices.size() > 0)
				regions.resize(cluster_indices.size());
			else {
				return regions;
			}

			int j = 0;
			for (vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
			{
				PointCloud<PointXYZRGB>::Ptr region_cluster(new PointCloud<PointXYZRGB>);

				/*************** Separating out and saving each cluster ***************/
				for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
					region_cluster->points.push_back (input->points[*pit]); //*
				cerr << "PointCloud representing the Region: " << region_cluster->points.size () << " data points." << endl;

				sensor_msgs::PointCloud2 cluster_msg;
				toROSMsg(*region_cluster,cluster_msg);
				//regions.push_back(cluster_msg);
				regions[j] = cluster_msg;

				//stringstream ss;
				//ss << "region_" <<  j << ".pcd";
				//writer.write<PointXYZRGB> (ss.str (), *region_cluster, false); 
				j++;
			}
		}
		return regions;
	}

	int DataProcessor::find_state(pcl::PointCloud<PointXYZRGB>::Ptr cloud, int region_id)
	{
		//Check if the point cloud is empty
		if (cloud->points.size() == 0) {
			//Empty region
			ROS_INFO("DUPLO: Region %d is empty",region_id);
			return 0;
		}

		//system("rosrun dynamic_reconfigure dynparam set_from_parameters camera_synchronizer_node projector_mode 1");
		//ROS_INFO("DUPLO: Calling clustering algo inside find_state for region %d",region_id);
		vector<sensor_msgs::PointCloud2> clusters = cluster_regions(cloud,region_id);
		vector<int> count;
		if (clusters.size() == 0 && cloud->points.size() < 100) {
			ROS_INFO("DUPLO: Region %d is empty.", region_id);
			return 0;
		} else if (clusters.size() == 0 && cloud->points.size() >= 100) {
			ROS_INFO("DUPLO: Region %d probably has one small block.", region_id);
			return 1;
		} 

		if (clusters.size() == 1) {
			ROS_INFO("DUPLO: Region %d has only one block => Uncluttered.", region_id);
			return 1;
		}

		/*size_t max_duplos = 15;
		
		for (size_t i = 0; i < min(max_duplos,clusters.size()); i++) 
			count.push_back(0);

		for (size_t i = 0; i < min(max_duplos,clusters.size())-1; i++) {
			for (size_t j = i+1; j < min(max_duplos, clusters.size()); j++) {
				//ROS_INFO("DUPLO: Inside in contact for loop region %d",region_id);
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

		for (size_t i = 0; i < count.size(); i++)
			ROS_INFO("DUPLO: Region %d: block %zu touches %d other bricks", region_id, i+1, count[i]);

		int max_count = *max_element(count.begin(), count.end());	
*/

		vector<geometry_msgs::Point> extent = find_extents(*cloud);
		/*if(max_count < 1 && (extent[5].z - extent[4].z < 0.035)) {
			//Classify the region as uncluttered
			ROS_INFO("DUPLO: Region %d is uncluttered",region_id);
			return 1;
		}*/

		ROS_INFO("DUPLO: Height of region %d = %f", region_id, extent[5].z - extent[4].z);
		if (extent[5].z - extent[4].z < 0.05) {
			//Height threshold was 5 cms earlier (before Jan 16, 2013)
			//Height is less than 4 cm. Classify as cluttered but low.
			ROS_INFO("DUPLO: Region %d is cluttered",region_id);
			return 2;
		} else {
			//Height is more than 4 cm. Classify as piled.
			ROS_INFO("DUPLO: Region %d is piled up",region_id);
			return 3;
		}
	}

	bool DataProcessor::mysort(std::pair<int,int> state1, std::pair<int,int> state2) {
		return (state1.second < state2.second);
	}
	
	void DataProcessor::assign_manipulation_primitive()
	{
		//sort(state_.begin(), state_.end(), mysort);
		std::sort(state_.begin(), state_.end(), 
		          boost::bind(&std::pair<PointCloud<PointXYZRGB>::Ptr, int>::second, _1) <
		          boost::bind(&std::pair<PointCloud<PointXYZRGB>::Ptr, int>::second, _2));

		visualization_msgs::MarkerArray region_marker_arr;
		region_marker_arr.markers.resize(state_.size());
		for (size_t i = 0; i < state_.size(); i++)
		{
			ROS_INFO("DUPLO: State of region %zu: %d", i, state_[i].second);
			stringstream ss;//create a stringstream
			ss << i;//add number to the stream
			region_marker_arr.markers[i] = set_marker("base_link", "regions", i, visualization_msgs::Marker::TEXT_VIEW_FACING,
					find_extents(*(state_[i].first))[0], 0.1, 0.0f, 0.0f, 1.0f, 1.0, ss.str());
		}
		regions_pub_.publish(region_marker_arr);

		for (size_t i = 0; i < state_.size(); i++) {
			if (state_[i].second == 1) {
				if (!pick_n_place((int)i)) {
					move_arm(reset_posn);
				}
				//break;
				//ros::Duration(1.0).sleep();
			} 		
			else if (state_[i].second == 2) {
				spread((int)i);
				//ros::Duration(1.0).sleep();
				//break;
			}					
			else if (state_[i].second == 3) {
				tumble((int)i);
				//ros::Duration(1.0).sleep();
				break;
			}	
			if (state_[state_.size()-1].second == 0) {
				new_cloud_wanted_ = true;
				//break;
			}
		}

		new_cloud_wanted_ = true;
		ROS_INFO("Through all states. Moving arm to reset position.");
		move_arm(reset_posn);
		head_.lookAt("base_link", 0.2, -0.1, 1.3);

		return;
	}

	bool DataProcessor::pick_n_place(int region_id)
	{
		//Pick up every block
		duplo_v1::Manipulate_Duplo grasp_call;
		//vector<sensor_msgs::PointCloud2> bricks = cluster(region_ptr[state_[region_id].first-1],region_id+1);
		vector<sensor_msgs::PointCloud2> bricks = cluster(state_[region_id].first,region_id);
		ROS_INFO("Need to pick up %zu bricks for region %d", bricks.size(), region_id);
		if (bricks.size() > 0)
			grasp_call.request.target_cloud = bricks;
		else {
			sensor_msgs::PointCloud2 cluster2;
			toROSMsg(*state_[region_id].first, cluster2);
			grasp_call.request.target_cloud.push_back(cluster2);
		}
		grasp_call.request.action = 0;
		grasp_call.request.table_extent = table_extent_;

		//pub_active_region_.publish(region_publish[state_[region_id].first-1]);
		sensor_msgs::PointCloud2 active_region2;
		//toROSMsg(*region_ptr[state_[region_id].first-1], active_region2);
		toROSMsg(*(state_[region_id].first), active_region2);
		active_region2.header.frame_id = "base_link";
		pub_active_region_.publish(active_region2);

		ROS_INFO("DUPLO: Picking the brick for region %d", region_id);
		if (client_manipulate_.call(grasp_call)) {
			if (grasp_call.response.done) {
				//ROS_INFO("DUPLO: Placed the duplo for region %d", state_[region_id].first);
				ROS_INFO("DUPLO: Placed the duplo for region %d", region_id);
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

	bool DataProcessor::spread(int region_id)
	{
		//Spread out the cluster
		duplo_v1::Manipulate_Duplo spread_call;
		//spread_call.request.target_cloud = regions_[state_[region_id].first-1];

		sensor_msgs::PointCloud2 spreadCloud2;
		toROSMsg(*(state_[region_id].first), spreadCloud2);
		spread_call.request.target_cloud.push_back(spreadCloud2);
		spread_call.request.action = 1;
		spread_call.request.table_extent = table_extent_;

		sensor_msgs::PointCloud2 active_region2 = spread_call.request.target_cloud[0];
		active_region2.header.frame_id = "base_link";
		pub_active_region_.publish(active_region2);

		//ROS_INFO("DUPLO: Spreading the bricks for region %d", state_[region_id].first);
		ROS_INFO("DUPLO: Spreading the bricks for region %d", region_id);
		if (client_manipulate_.call(spread_call))
			ROS_INFO("DUPLO: Spreading the bricks.");
		else 
			ROS_ERROR("Failed to call spread duplo service.");

		if (spread_call.response.done) {
			ROS_INFO("DUPLO: Spreading action succeeded.");	
			new_cloud_wanted_ = true;
			//			break;
		} else {
			ROS_INFO("DUPLO: Spreading action failed.");
			//					pick_n_place(region_id);
			new_cloud_wanted_ = true;
		}		
		//if ((uint)region_id == state_.size()-1) {
			//No more regions remaining
			//move_arm(reset_posn);
			//head.lookAt("base_link", 0.3, 0.0, 0.0);
		//}
		return true;
	}

	bool DataProcessor::tumble(int region_id)
	{
		//Tumble the pile
		duplo_v1::Manipulate_Duplo tumble_call;
		//tumble_call.request.target_cloud = regions_[state_[region_id].first-1];
		sensor_msgs::PointCloud2 tumbleCloud2;
		toROSMsg(*(state_[region_id].first), tumbleCloud2);
		tumble_call.request.target_cloud.push_back(tumbleCloud2);
		tumble_call.request.action = 2;
		tumble_call.request.table_extent = table_extent_;

		sensor_msgs::PointCloud2 active_region2 = tumble_call.request.target_cloud[0];
		active_region2.header.frame_id = "base_link";
		pub_active_region_.publish(active_region2);

		//ROS_INFO("DUPLO: Tumbling the pile for region %d", state_[region_id].first);
		ROS_INFO("DUPLO: Tumbling the pile for region %d", region_id);
		if (client_manipulate_.call(tumble_call))
			//ROS_INFO("DUPLO: Tumbled the pile for region %d", state_[region_id].first);
			ROS_INFO("DUPLO: Tumbled the pile for region %d", region_id);
		else 
			ROS_ERROR("Failed to call tumble duplo service.");

		if (tumble_call.response.done) {
			ROS_INFO("DUPLO: Tumbling action succeeded.");	
			new_cloud_wanted_ = true;
			//			break;
		} else {
			ROS_INFO("DUPLO: Tumbling action failed.");
			//spread(region_id);
		}		
		//if ((uint)region_id == state_.size()-1) {
			//No more regions remaining
			//move_arm(reset_posn);
			//head_.lookAt("base_link", 0.2, 0.0, 1.3);
		//}
		return true;
	}
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "process_pcd");
	ros::NodeHandle n;
	sort_duplos::DataProcessor dp(n);
	ros::spin();
}
