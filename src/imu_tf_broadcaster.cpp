#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

ros::Publisher rpy_pub;

std::string frameId;
sensor_msgs::Imu imu;
nav_msgs::Odometry odom;
double radToDegree = 57.2957795;

double firstYaw;
bool init=false;
int delay = 0;

double limitPi(double a){
	//constrain to -PI ... PI
	if (a > 180)
		a-=180;
	else if (a <-180)
		a+=180;
	return a;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	imu.header = msg->header;
	// Convert quaternion to RPY.
	    tf::Quaternion q;
	    double roll, pitch, yaw;
	    double limitYaw,deltaYaw;
	    tf::quaternionMsgToTF(msg->orientation, q);
	    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	    roll*=radToDegree;
	    pitch*=radToDegree;
	    yaw=yaw*radToDegree;
	    //limitYaw = limitPi(yaw);
//	    ROS_DEBUG("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);

	    //if (delay>0) delay--;

	    if (!init){
	    	firstYaw = yaw;
	    	//deltaYaw = firstYaw;
	    	ROS_INFO("start yaw = %lf",firstYaw);
	    	init = true;
	    }

	    //deltaYaw  = limitYaw-firstYaw;
	    //deltaYaw = yaw - firstYaw;
	    //if ( !(deltaYaw > -180 && deltaYaw < 180))
	    //	deltaYaw = yaw - 360 + firstYaw;
	    deltaYaw = yaw - firstYaw;
	    if(yaw - firstYaw < 0)
	    	deltaYaw += 360;
	    if (deltaYaw > 180)
	    	deltaYaw -= 360;

	    //ROS_INFO("%lf = %lf - %lf",deltaYaw,yaw,firstYaw);
	    geometry_msgs::Vector3 rpy;
	    rpy.x = yaw;
	    rpy.y = 0;
	    rpy.z = deltaYaw;
	    rpy_pub.publish(rpy);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	odom.pose.pose.position = msg->pose.pose.position;


}


int main(int argc, char** argv){
  ros::init(argc, argv, "imu_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  frameId = (ros::param::get("~frame_id", frameId)) ? frameId : "/odom";

  ros::Subscriber sub1 = node.subscribe("imu/data", 100, imuCallback);
  ros::Subscriber sub2 = node.subscribe("RosAria/pose", 100, odomCallback);
  rpy_pub = node.advertise<geometry_msgs::Vector3>("imu/rpy", 100);


  ros::Rate rate(10.0);
    while (node.ok()){
      ros::spinOnce();
      transform.setOrigin( tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z + 0.3) );
      transform.setRotation( tf::Quaternion(0, 0, 0) );
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "/base_imu"));
      rate.sleep();
    }
  return 0;
}
