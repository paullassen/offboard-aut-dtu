#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "interface");
	ros::NodeHandle nh;

	ros::Publisher gy_pub = nh.advertise<std_msgs::Bool>
								("test/health/gyro", 10);
	ros::Publisher ac_pub = nh.advertise<std_msgs::Bool>
								("test/health/accel", 10);
	ros::Publisher mag_pub = nh.advertise<std_msgs::Bool>
								("test/health/mag", 10);
	ros::Publisher lev_pub = nh.advertise<std_msgs::Bool>
								("test/health/level", 10);
	ros::Publisher loc_pub = nh.advertise<std_msgs::Bool>
								("test/health/local", 10);
	ros::Publisher glob_pub = nh.advertise<std_msgs::Bool>
								("test/health/global", 10);
	ros::Publisher home_pub = nh.advertise<std_msgs::Bool>
								("test/health/home", 10);
	ros::Publisher bat_pub = nh.advertise<std_msgs::Float32>
								("test/health/battery", 10);
	
	std_msgs::Bool state;
	state.data = true;
	std_msgs::Float32 vol;
	vol.data = 13.2;

	ros::Rate rate(10.0);

	while(ros::ok())
	{
		ros::spinOnce();
		gy_pub.publish(state);
		ac_pub.publish(state);
		mag_pub.publish(state);
		lev_pub.publish(state);
		loc_pub.publish(state);
		glob_pub.publish(state);
		home_pub.publish(state);

		bat_pub.publish(vol);
		rate.sleep();
	}
	return 0;
}


