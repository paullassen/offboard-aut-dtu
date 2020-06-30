#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>

static geometry_msgs::PoseStamped in;
static geometry_msgs::PoseStamped out;

void mcCb(const geometry_msgs::PoseStamped::ConstPtr & msg){
	in = *msg;
	std::cout << in.pose.position.y << std::endl;
	std::cout << msg->pose.position.y << std::endl;
}

int main(int argc, char ** argv){

	ros::init(argc, argv, "test");
	ros::NodeHandle nh;
	geometry_msgs::TransformStamped trans;
	tf2_ros::Buffer tBuff;
	tf2_ros::TransformListener tfListener(tBuff);
	ros::Subscriber tf_sub = nh.subscribe<geometry_msgs::PoseStamped>
							("/test/mocap", 10, mcCb);
	ros::Publisher tf_pub = nh.advertise<geometry_msgs::PoseStamped>
							("test/trans", 10);
	ros::Publisher tfo_pub = nh.advertise<geometry_msgs::PoseStamped>
							("test/transo", 10);
	ros::Rate rate(100);
	while (nh.ok()){
		ros::spinOnce();
		trans = tBuff.lookupTransform("HexyBoi", "Robot_1/base_link", ros::Time(0));
		try{
		//tf2:doTransform(in, out, trans);
			tf2::doTransform(in, out, trans);
		}catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		tf_pub.publish(out);
		tfo_pub.publish(in);
		rate.sleep();
	}
	return 0;
}
