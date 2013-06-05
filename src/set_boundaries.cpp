#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace std;

visualization_msgs::Marker 
set_boundaries(std::string frame_id, string ns, int id, int shape, int action,
           vector<geometry_msgs::Point> extent, float scale, float r, float g, float b, float a)
{
	geometry_msgs::Point temp;
	visualization_msgs::Marker marker;

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = ns;
	marker.id = id;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = shape;

	// Set the marker action.  Options are ADD and DELETE
	marker.action = action;

	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = scale;
	marker.scale.y = 0.02;
	marker.scale.z = 0.02;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	temp.x = extent[0].x;
	temp.y = extent[2].y;
	temp.z = extent[5].z;
	marker.points.push_back(temp);
	temp.x = extent[1].x;
	marker.points.push_back(temp);
	temp.y = extent[3].y;
	marker.points.push_back(temp);
	temp.x = extent[0].x;
	marker.points.push_back(temp);
	temp.y = extent[2].y;
	marker.points.push_back(temp);

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;

	marker.lifetime = ros::Duration(0.0);

	return marker;
}
