#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

std::string frameId;
sensor_msgs::Imu imu;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	imu.header = msg->header;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

	tf::TransformBroadcaster br;
  	tf::Transform transform;

        transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z + 0.3) );
//        transform.setRotation( tf::Quaternion(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w) );
	transform.setRotation( tf::Quaternion(0, 0, 0) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frameId, imu.header.frame_id));

	ROS_INFO("new tf  frame_id=%s->child=%s",frameId.c_str(),imu.header.frame_id.c_str());

}


int main(int argc, char** argv){
  ros::init(argc, argv, "imu_tf_broadcaster");
  ros::NodeHandle node;

  frameId = (ros::param::get("~frame_id", frameId)) ? frameId : "/odom";

  ros::Subscriber sub1 = node.subscribe("imu/data", 100, imuCallback);
  ros::Subscriber sub2 = node.subscribe("RosAria/pose", 100, odomCallback);

  ros::spin();

  return 0;
}
