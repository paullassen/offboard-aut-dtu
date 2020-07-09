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
static geometry_msgs::TransformStamped trans;
ros::Publisher *tf_;
ros::Publisher *rpyin_;
ros::Publisher *rpyout_;
void mcCb(const geometry_msgs::PoseStamped::ConstPtr & msg){
	try{
		tf2::doTransform(*msg, out, trans);
	}catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
		ros::Duration(0.001).sleep();
	}
	tf_->publish(out);

	tf::Quaternion qin(	msg->pose.orientation.x,
						msg->pose.orientation.y,
					    msg->pose.orientation.z,
					    msg->pose.orientation.w );

	tf::Quaternion qout(out.pose.orientation.x,
						out.pose.orientation.y,
						out.pose.orientation.z,
						out.pose.orientation.w );
	tf::Matrix3x3 min(qin);
	tf::Matrix3x3 mout(qout);

	geometry_msgs::Point rpyin, rpyout;
	min.getRPY(rpyin.x, rpyin.y, rpyin.z);
	mout.getRPY(rpyout.x, rpyout.y, rpyout.z);
	rpyin.x *= 180/3.14159;
	rpyin.y *= 180/3.14159;
	rpyin.z *= 180/3.14159;
	rpyout.x *= 180/3.14159;
	rpyout.y *= 180/3.14159;
	rpyout.z *= 180/3.14159;
	rpyout.x += rpyout.x>0?-180:180;

	rpyin_->publish(rpyin);
	rpyout_->publish(rpyout);

}

int main(int argc, char ** argv){

	ros::init(argc, argv, "test");
	ros::NodeHandle nh;
	tf2_ros::Buffer tBuff;
	tf2_ros::TransformListener tfListener(tBuff);
	ros::Subscriber tf_sub = nh.subscribe<geometry_msgs::PoseStamped>
							("/test/mocap", 10, mcCb);
	ros::Publisher tf_pub = nh.advertise<geometry_msgs::PoseStamped>
							("test/trans", 10);
	ros::Publisher ri_pub = nh.advertise<geometry_msgs::Point>
							("test/rpyin", 10);
	ros::Publisher ro_pub = nh.advertise<geometry_msgs::Point>
							("test/rpyout", 10);
	tf_ = &tf_pub;
	rpyin_ = &ri_pub;
	rpyout_ = &ro_pub;
	ros::Rate rate(100);
	trans = tBuff.lookupTransform("HexyBoi", "Robot_1/base_link", ros::Time(0), ros::Duration(1.0));
	
	ros::spin();
	return 0;
}
