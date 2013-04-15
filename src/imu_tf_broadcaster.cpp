#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

std::string frameId;
sensor_msgs::Imu imu;
nav_msgs::Odometry odom;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	imu.header = msg->header;
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
