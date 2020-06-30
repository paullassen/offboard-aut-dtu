#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/Buffer.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>

geometry_msgs::PoseStamped in;

void mcCb(const geometry_msgs::PoseStamped::ConstPtr & msg){
	in = msg;
}

int main(int argc, char ** argv){

	ros::init(argc, argv, "test");
	ros::NodeHandle nh;
	geometry_msgs::PoseStamped out;
	geometry_msgs::TransformStamped trans;
	tf2_ros::Buffer tBuff;
	tf2_ros::TransformListener tfListener(tBuff);
	ros::Subscriber tf_sub = mh.subscribe<geometry_msgs::PoseStamped>
							("test/mocap", 10, mcCb);
	ros::Publisher tf_pub = nh.advertise<geometry_msgs::PoseStamped>
							("test/trans", 10);
	ros::Rate rate(100);
	while (nh.ok()){
		try{
			trans = tBuff.lookupTransform("/HexyBoi", "/Robot_1/base_link", ros::Time(0));
			tf2_geometry_msgs::do_transform(in, out, trans);
			tf_pub.publishers(out);
		}
		catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
	    ros::Duration(1.0).sleep();
	    continue;
		}
	}
	return 0;
}
	

}

