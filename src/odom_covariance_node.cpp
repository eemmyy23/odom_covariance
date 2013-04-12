#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

ros::Publisher odom_pub;
bool changeCov;
double posX=99999, posY=99999, posZ=99999;
double rotX=99999, rotY=99999, rotZ=99999;
std::string frameId, childFrameId;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	nav_msgs::Odometry odom;

	//header
	odom.header = msg->header;
	if(!frameId.empty())
		odom.header.frame_id=frameId;

	//child_frame_id
	if(!childFrameId.empty())
		odom.child_frame_id = childFrameId;
	else
		odom.child_frame_id = msg->child_frame_id;

	//pose and twist
	odom.pose = msg->pose;
	odom.twist = msg->twist;
    if(changeCov == true)
    {
		odom.pose.covariance.elems[0]  = posX;
		odom.pose.covariance.elems[7]  = posY;
		odom.pose.covariance.elems[14] = posZ;
		odom.pose.covariance.elems[21] = rotX;
		odom.pose.covariance.elems[28] = rotY;
		odom.pose.covariance.elems[35] = rotZ;
		odom.twist.covariance = odom.pose.covariance;
	}

    //publish the new topic
	odom_pub.publish(odom);

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "odom_covariance_node");
	ros::NodeHandle n;

	ros::param::param("~changeCov", changeCov, false);
	ros::param::get("~posX", posX);
	ros::param::get("~posY", posY);
	ros::param::get("~posZ", posZ);
	ros::param::get("~rotX", rotX);
	ros::param::get("~rotY", rotY);
	ros::param::get("~rotZ", rotZ);

	frameId = (ros::param::get("~frame_id", frameId)) ? frameId : "";
    childFrameId = (ros::param::get("~child_frame_id", childFrameId)) ? childFrameId : "";

	ros::Subscriber sub = n.subscribe("inputOdom", 1, odomCallback);
	odom_pub = n.advertise<nav_msgs::Odometry>("outputOdom", 1);
	ros::spin();
	return 0;
}
