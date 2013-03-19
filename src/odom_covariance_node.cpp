#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

ros::Publisher odom_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	nav_msgs::Odometry odom;
	odom.child_frame_id = msg->child_frame_id;
    odom.header = msg->header;
    odom.pose = msg->pose;
    odom.twist = msg->twist;

    odom.pose.covariance.elems[0] = 0.01;
    odom.pose.covariance.elems[7] = 0.01;
    odom.pose.covariance.elems[14] = 99999;
    odom.pose.covariance.elems[21] = 99999;
    odom.pose.covariance.elems[28] = 99999;
    odom.pose.covariance.elems[35] = 0.01;
    odom.twist.covariance = odom.pose.covariance;

    odom_pub.publish(odom);


}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_covariance_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("inputOdom", 1, odomCallback);
	odom_pub = n.advertise<nav_msgs::Odometry>("outputOdom", 1);
	ros::spin();
	return 0;
}
