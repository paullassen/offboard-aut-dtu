#include <ros/ros.h>
#include <offboard/Health.h>
#include <offboard/ActuatorArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


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
	Mavsdk dc;
	std::string connection_url;
	ConnectionResult connection_result;
	/* Get connection url */
	if (argc == 2){
		connection_url = argv[1];
		connection_result = dc.add_any_connection(connection_url);
	}else{
		usage(argv[0]);
		return 1;
	}
	/* Attempt connection */
	if (connection_result != ConnectionResult::Success) {
		std::cout << "Connection Failed: " << connection_result << std::endl;
		return 1;
	}

	wait_until_discover(dc);
	System& system = dc.system();
	
	/* Once connected initialise plugins */
	auto action	= std::make_shared<Action>(system);
	auto info	= std::make_shared<Info>(system);
	auto offboard	= std::make_shared<Offboard>(system);
	auto telemetry	= std::make_shared<Telemetry>(system);
	
	/* start ROS */	
	ros::init(argc, argv, "interface");
	ros::NodeHandle nh;

	/* Create ROS publishers */
	ros::Publisher health_pub = nh.advertise<offboard::Health>
									("test/health", 10);
	ros::Publisher att_pub = nh.advertise<geometry_msgs::PointStamped>
									("test/attitude", 10);
	ros::Publisher matt_pub = nh.advertise<geometry_msgs::PointStamped>
									("test/mocap_att", 10);
	ros::Publisher err_pub = nh.advertise<geometry_msgs::PointStamped>
									("test/err", 10);
	ros::Publisher erd_pub = nh.advertise<geometry_msgs::PointStamped>
									("test/erd", 10);
	ros::Publisher eri_pub = nh.advertise<geometry_msgs::PointStamped>
									("test/eri", 10);
	ros::Publisher in_pub = nh.advertise<geometry_msgs::PointStamped>
									("test/att_in", 10);
	ros::Publisher thrust_pub = nh.advertise<std_msgs::Float32>
									("test/thrust", 10);
	ros::Publisher actuator_target_pub = nh.advertise<offboard::ActuatorArray>
									("test/actuator_target", 10);
	ros::Publisher actuator_status_pub = nh.advertise<offboard::ActuatorArray>
									("test/actuator_status", 10);
	ros::Rate rate(50.0);

	/* Create UavMonitor to handle controllers */
	UavMonitor uav;	

	/* Start Battery Subscriber */
	Telemetry::Result set_rate_result = telemetry->set_rate_battery(2.0);
	if (set_rate_result != Telemetry::Result::Success) {
	    std::cout << "Setting rate failed:" << set_rate_result<< std::endl;
	}	

	/* Start Attitude Subscriber */
	set_rate_result = telemetry->set_rate_attitude(100.0);
	if (set_rate_result != Telemetry::Result::Success) {
	    std::cout << "Setting rate failed:" << set_rate_result<< std::endl;
	}	
	
	/* Start Attitude Subscriber */
	set_rate_result = telemetry->set_rate_actuator_control_target(100);
	if (set_rate_result != Telemetry::Result::Success) {
	    std::cout << "Setting rate failed:" << set_rate_result<< std::endl;
	}	

	/* Start Attitude Subscriber */
	set_rate_result = telemetry->set_rate_actuator_output_status(100); 
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

	telemetry->subscribe_actuator_control_target([&uav](Telemetry::ActuatorControlTarget act){
		uav.set_actuator_target(act);
	});

	telemetry->subscribe_actuator_output_status([&uav](Telemetry::ActuatorOutputStatus aos){
		uav.set_actuator_status(aos);
	});

	/* struct to pass objects to threads */
	struct thread_data thread_args;
	thread_args.uav = &uav;
	thread_args.offboard = offboard;
	thread_args.action = action;
	thread_args.nh = &nh;
	
	/* structure for measuring loop time */
	struct duration timeStruct;
	initDuration(&timeStruct);
	
	/* Find transformation*/
	tf2_ros::Buffer tBuffer;
	tf2_ros::TransformListener tfListener(tBuffer);
	uav.transform = tBuffer.lookupTransform("HexyBoi", "Robot_1/base_link", ros::Time(0), ros::Duration(1.0));

	/* start listening for msgs */
	pthread_t offboard_thread, callback_thread;
	pthread_create(&callback_thread, NULL, &UavMonitor::ros_run, (void *)&thread_args);

	/* go into offboard control */
	pthread_create(&offboard_thread, NULL, &UavMonitor::offboard_control, (void *)&thread_args);
	
	std::cout << "Starting publishers ..." << std::endl;
	
	/*Create ROS msgs*/
	//std_msgs::Bool state;
	//std_msgs::Float32 volt;
	offboard::Health health;
	offboard::ActuatorArray actuator_targets;
	offboard::ActuatorArray actuator_status;
	geometry_msgs::PointStamped attitude;
	geometry_msgs::PointStamped mocap_att;
	geometry_msgs::PointStamped err;
	geometry_msgs::PointStamped erd;
	geometry_msgs::PointStamped eri;
	geometry_msgs::PointStamped in;
	
	std_msgs::Float32 thrust;
	
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	

	while(!uav.done){
		ros::Time start = ros::Time::now();	

		health = uav.health;

		header.stamp = ros::Time::now();

		attitude.point.x = uav.roll;
		attitude.point.y = uav.pitch;
		attitude.point.z = uav.yaw;
		attitude.header = header;

		mocap_att.point.x = uav.mocap_roll;
		mocap_att.point.y = uav.mocap_pitch;
		mocap_att.point.z = uav.mocap_yaw;
		mocap_att.header = header;

		err.point.x = uav.ex;
		err.point.y = uav.ey;
		err.point.z = uav.ez;
		err.header = header;

		erd.point.x = uav.edx;
		erd.point.y = uav.edy;
		erd.point.z = uav.edz;
		erd.header = header;

		eri.point.x = uav.eix;
		eri.point.y = uav.eiy;
		eri.point.z = uav.eiz;
		eri.header = header;

		in.point.x = uav.uav_roll;
		in.point.y = uav.uav_pitch;
		in.point.z = uav.uav_yaw;
		in.header = header;

		thrust.data = uav.uav_thrust;
		
		actuator_targets.actuator = uav.actuator_target;
		actuator_status.actuator = uav.actuator_status;

		att_pub.publish(attitude);
		matt_pub.publish(mocap_att);
		health_pub.publish(health);
		err_pub.publish(err);
		erd_pub.publish(erd);
		eri_pub.publish(eri);
		in_pub.publish(in);
		thrust_pub.publish(thrust);
		actuator_target_pub.publish(actuator_targets);
		actuator_status_pub.publish(actuator_status);
		ros::Time end = ros::Time::now();
		addDuration(end - start, &timeStruct);
		rate.sleep();
	}
	printDuration("Pub", &timeStruct);
	pthread_join(offboard_thread, NULL);
	
	std::cout << "Shutting down ROS ..." << std::endl;

	ros::shutdown();

	pthread_join(callback_thread, NULL);

	std::cout << "Ending the program ..." << std::endl;

	return 0;
}

				  
