#include <ros/ros.h>
#include <offboard/Health.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>

#include <unistd.h>
#include <string.h>
#include <pthread.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/info/info.h>
#include "mavsdk_helper.h"
#include "uav_monitor.h"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

int main(int argc, char ** argv){
	UavMonitor uav;	
	
	ros::init(argc, argv, "interface");
	ros::NodeHandle nh;

	Mavsdk dc;
	std::string connection_url;
	ConnectionResult connection_result;
	// Get connection url
	if (argc == 2){
		connection_url = argv[1];
		connection_result = dc.add_any_connection(connection_url);
	}else{
		usage(argv[0]);
		return 1;
	}
	// Attempt connection
	if (connection_result != ConnectionResult::Success) {
		std::cout << "Connection Failed: " << connection_result << std::endl;
		return 1;
	}

	wait_until_discover(dc);
	System& system = dc.system();
	
	// Once connected initialise plugins
	auto action	= std::make_shared<Action>(system);
	auto info	= std::make_shared<Info>(system);
	auto offboard	= std::make_shared<Offboard>(system);
	auto telemetry	= std::make_shared<Telemetry>(system);

	// Create ROS publishers
	ros::Publisher health_pub = nh.advertise<offboard::Health>
									("test/health", 10);
	ros::Publisher att_pub = nh.advertise<geometry_msgs::PointStamped>
									("test/attitude", 10);
	ros::Publisher err_pub = nh.advertise<geometry_msgs::PointStamped>
									("test/err", 10);
	ros::Publisher erd_pub = nh.advertise<geometry_msgs::PointStamped>
									("test/erd", 10);
	ros::Publisher in_pub = nh.advertise<geometry_msgs::PointStamped>
									("test/att_in", 10);
	ros::Publisher thrust_pub = nh.advertise<std_msgs::Float32>
									("test/thrust", 10);
	ros::Rate rate(20.0);

	//Create ROS msgs
	//std_msgs::Bool state;
	//std_msgs::Float32 volt;
	offboard::Health health;
	geometry_msgs::PointStamped attitude;
	geometry_msgs::PointStamped err;
	geometry_msgs::PointStamped erd;
	geometry_msgs::PointStamped in;
	std_msgs::Float32 thrust;
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	
	// Start Battery Subscriber
	Telemetry::Result set_rate_result = telemetry->set_rate_battery(2.0);
	if (set_rate_result != Telemetry::Result::Success) {
	    std::cout << "Setting rate failed:" << set_rate_result<< std::endl;
	}	

	// Start Attitude Subscriber
	set_rate_result = telemetry->set_rate_attitude(20.0);
	if (set_rate_result != Telemetry::Result::Success) {
	    std::cout << "Setting rate failed:" << set_rate_result<< std::endl;
	}	


	telemetry->subscribe_health([&uav](Telemetry::Health health){
		uav.set_health(health);
	});

	telemetry->subscribe_battery([&uav](Telemetry::Battery battery){
		uav.set_battery(battery);		
	});

	telemetry->subscribe_attitude_euler([&uav](Telemetry::EulerAngle angle){
		uav.set_angle(angle);		
	});


	struct thread_data thread_args;
	thread_args.uav = &uav;
	thread_args.offboard = offboard;
	thread_args.action = action;
	thread_args.nh = &nh;

	pthread_t offboard_thread, callback_thread;

	pthread_create(&callback_thread, NULL, &UavMonitor::ros_run, (void *)&thread_args);

    Action::Result arm_result = action->arm();
	action_error_exit(arm_result, "Arming failed");
    std::cout << "Armed" << std::endl;

	pthread_create(&offboard_thread, NULL, &UavMonitor::offboard_control, (void *)&thread_args);
	
	std::cout << "Starting publishers ..." << std::endl;

	while(!uav.done){
		
		health = uav.health;

		header.stamp = ros::Time::now();

		attitude.point.x = uav.roll;
		attitude.point.y = uav.pitch;
		attitude.point.z = uav.yaw;
		attitude.header = header;

		err.point.x = uav.ex;
		err.point.y = uav.ey;
		err.point.z = uav.ez;
		err.header = header;

		erd.point.x = uav.edx;
		erd.point.y = uav.edy;
		erd.point.z = uav.edz;
		erd.header = header;

		in.point.x = uav.uav_roll;
		in.point.y = uav.uav_pitch;
		in.point.z = uav.uav_yaw;
		in.header = header;

		thrust.data = uav.uav_thrust;


		att_pub.publish(attitude);
		health_pub.publish(health);
		err_pub.publish(err);
		erd_pub.publish(erd);
		in_pub.publish(in);
		thrust_pub.publish(thrust);
		rate.sleep();
	}

	pthread_join(offboard_thread, NULL);
	
	std::cout << "Shutting down ROS ..." << std::endl;

	ros::shutdown();

	pthread_join(callback_thread, NULL);

	std::cout << "Ending the program ..." << std::endl;

	return 0;
}

				  
