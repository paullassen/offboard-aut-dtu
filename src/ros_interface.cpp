#include <ros/ros.h>
#include <offboard/Health.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>
#include <boost/bind.hpp>

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
	ros::Publisher att_pub = nh.advertise<geometry_msgs::Point>
									("test/attitude", 10);
	ros::Subscriber kill_switch = nh.subscribe<std_msgs::Bool>
									("test/kill", 10, boost::bind(killCb, _1, &uav));
	ros::Subscriber baseline_sub = nh.subscribe<std_msgs::Float32>
									("test/baseline",10, boost::bind(baselineCb, _1, &uav));
	ros::Rate rate(10.0);

	//Create ROS msgs
	//std_msgs::Bool state;
	//std_msgs::Float32 volt;
	offboard::Health health;
	geometry_msgs::Point attitude;
	
	
	// Start Battery Subscriber
	Telemetry::Result set_rate_result = telemetry->set_rate_battery(1.0);
	if (set_rate_result != Telemetry::Result::Success) {
	    std::cout << "Setting rate failed:" << set_rate_result<< std::endl;
	}	

	// Start Attitude Subscriber
	set_rate_result = telemetry->set_rate_attitude(1.0);
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

	void *offboard_args[3];
	offboard_args[0] = &uav;
	offboard_args[1] = &offboard;
	offboard_args[2] = &action;

    Action::Result arm_result = action->arm();
    //action_error_exit(arm_result, "Arming failed");
    std::cout << "Armed" << std::endl;

	pthread_t offboard_thread;
	pthread_create(&offboard_thread, NULL, offboard_control, offboard_args);
	while(!uav.done){
		
		ros::spinOnce();
		
		health.gyro = uav.gyro_cal;
		health.accel = uav.accel_cal;
		health.mag = uav.mag_cal;
		health.level = uav.level_cal;
		health.local = uav.local_ok;
		health.global = uav.global_ok;
		health.home = uav.home_ok;
		health.battery = uav.battery;

		attitude.x = uav.roll;
		attitude.y = uav.pitch;
		attitude.z = uav.yaw;

		att_pub.publish(attitude);
		health_pub.publish(health);
		rate.sleep();
	}
	pthread_join(offboard_thread, NULL);
	return 0;
}

				  