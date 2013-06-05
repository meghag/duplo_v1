#ifndef ROBOT_HEAD_H
#define ROBOT_HEAD_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class RobotHead
{
	private:
		PointHeadClient* point_head_client_;

	public:
		//! Action client initialization 
		RobotHead (void);
		~RobotHead (void);

		//! Points the high-def camera frame at a point in a given frame  
		void lookAt(std::string frame_id, double x, double y, double z);
};

#endif

