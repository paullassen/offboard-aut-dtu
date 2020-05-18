#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>

#include <pthread.h>
#include <ncurses.h>
#include <unistd.h>
#include <string.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/info/info.h>
#include "mavsdk_helper.h"
#include "uav_monitor.h"

using namespace mavsdk;

void UavMonitor::kpCb(const geometry_msgs::Point::ConstPtr& msg){
	kpx = msg->x;
	kpy = msg->y;
	kpz = msg->z;
}

void UavMonitor::kdCb(const geometry_msgs::Point::ConstPtr& msg){
	kdx = msg->x;
	kdy = msg->y;
	kdz = msg->z;
}

void UavMonitor::targetCb(const geometry_msgs::Point::ConstPtr& msg){
    tx = msg->x;
    ty = msg->y;
    tz = msg->z;
}

void UavMonitor::mocapCb(const geometry_msgs::PoseStamped::ConstPtr& msg){

	if (offset_yaw == 0.0){
		//create quaternion
		tf::Quaternion q(
			msg->pose.orientation.x,
			msg->pose.orientation.y,
			msg->pose.orientation.z,
			msg->pose.orientation.w
			);
		//get rotation matrix
		tf::Matrix3x3 m(q);
		double off_r, off_p, off_y;
		//get r,p,y
		m.getRPY(off_r, off_p, off_y);
		//get offset
		offset_yaw = yaw - (float) off_y;
	}

    float last_x = x;
    float last_y = y;
    float last_z = z;

    x =  msg->pose.position.x;
    y = -msg->pose.position.y;
    z =  msg->pose.position.z;

    
    ros::Duration dt = msg->header.stamp - last_time;
    
    dx = (x-last_x)/dt.toSec();
    dy = (y-last_y)/dt.toSec();
    dz = (z-last_z)/dt.toSec();
    
    last_time = msg->header.stamp;

    calculate_error();
}

void UavMonitor::baselineCb(const std_msgs::Float32::ConstPtr& msg)
{
	baseline = msg->data;
	std::cerr << "BASELINE : " << baseline << "\r" <<std::flush;
}

void  UavMonitor::killCb(const std_msgs::Bool::ConstPtr& msg)
{
	kill = msg->data;
}

void UavMonitor::startCb(const std_msgs::Bool::ConstPtr& msg)
{
	start = msg->data;
}

//Health Functions
void UavMonitor::set_health(Telemetry::Health h){

	health.gyro		= h.is_gyrometer_calibration_ok;
	health.accel	= h.is_accelerometer_calibration_ok;
	health.mag		= h.is_magnetometer_calibration_ok;
	health.level	= h.is_level_calibration_ok;
	
	health.local	= h.is_local_position_ok;
	health.globe	= h.is_global_position_ok;
	health.home		= h.is_home_position_ok;
}

bool UavMonitor::get_health(){
	return health.gyro
			&& health.accel
			&& health.mag
			&& health.level;
}

// Battery Functions
void UavMonitor::set_battery(Telemetry::Battery bat){
	health.battery = bat.voltage_v;
}

float UavMonitor::get_battery(){
	return battery;
}

//Attitude Functions
void UavMonitor::set_angle(Telemetry::EulerAngle angle){
	roll = angle.roll_deg;
	pitch = angle.pitch_deg;
	yaw = angle.yaw_deg;
}

//Other Functions
void *UavMonitor::offboard_control(void *arg){
	
	std::cout << "Starting control thread ..." << std::endl;
    const std::string offb_mode = "ATTITUDE";
	
	struct thread_data *args = (struct thread_data *)arg;

	UavMonitor *m = args->uav;
	std::shared_ptr<mavsdk::Offboard> offboard = args->offboard;
	std::shared_ptr<mavsdk::Action> action = args->action;
	
	ros::Rate rate(20);

	Offboard::Attitude attitude;
	attitude.roll_deg	= 0.0f;
	attitude.pitch_deg	= -1.0f;
	attitude.yaw_deg		= 0.0f;
	attitude.thrust_value = 0.1f;

	offboard->set_attitude(attitude);
	Offboard::Result offboard_result = offboard->start();
    offboard_error_exit(offboard_result, "Offboard start failed");
    offboard_log(offb_mode, "Offboard started");

	while(!m->kill){
		attitude.thrust_value =	m->calculate_thrust();			
		attitude.roll_deg = m->calculate_roll();
		attitude.pitch_deg = m->calculate_pitch();
		attitude.yaw_deg = m->calculate_yaw();
		offboard->set_attitude(attitude);
		rate.sleep();
	}

	std::cout << "Zeroing control inputs ..." << std::endl;
	attitude.thrust_value = 0.0f;
	attitude.roll_deg	= 0.0f;
	attitude.pitch_deg	= 0.0f;
	attitude.yaw_deg	= 0.0f;
	offboard->set_attitude(attitude);

/*
	offboard_result = offboard->stop();
    offboard_error_exit(offboard_result, "Offboard stop failed: ");
	offboard_log(offb_mode, "Offboard stopped");
*/  
	sleep(1);
	std::cout << "Sending kill command ..." << std::endl;

	const Action::Result kill_result = action->kill();

	m->done = true;
	pthread_exit(NULL);
}

float UavMonitor::calculate_thrust(){
	float thrust = baseline;
	thrust += kpz * ez + kdz * edz;

	double scale = cos((double) roll * M_PI/180)*cos((double) pitch * M_PI/180);

	uav_thrust = thrust / abs((float)scale);
	return saturate_minmax(uav_thrust, 0, 1);
}

float UavMonitor::calculate_pitch(){
	double ky = (kpy * ey + kdy * edy) * 180/M_PI;
	double kx = (kpx * ex + kdx * edx) * 180/M_PI;

	double yaw_rad = (yaw-offset_yaw) * M_PI/180;
	uav_pitch = saturate(-kx * cos(yaw_rad) + ky * sin(yaw_rad), 6);
	return uav_pitch;
}

float UavMonitor::calculate_roll(){
	double ky = (kpy * ey + kdy * edy) * 180/M_PI;
	double kx = (kpx * ex + kdx * edx) * 180/M_PI;

	double yaw_rad = (yaw-offset_yaw) * M_PI/180;
	
	uav_roll = saturate(-kx * sin(yaw_rad) + ky * cos(yaw_rad), 6);
	return uav_roll;
}

float UavMonitor::calculate_yaw(){
	double yaw_rad = (yaw-offset_yaw) * M_PI/180;
	uav_yaw = (float) yaw_rad;
	return (float) yaw_rad;
}

float UavMonitor::saturate(double in, double minmax)
{
	return saturate_minmax(in, -minmax, minmax);
}

float UavMonitor::saturate_minmax(double in, double min, double max)
{
	if(in > max){
		return (float) max;
	} else if(in < min) {
		return (float) min;
	}

	return (float) in;
}

void UavMonitor::calculate_error(){
    ex = tx - x;
    ey = ty - y;
    ez = tz - z;

    edx = -dx;
    edy = -dy;
    edz = -dz;
}

void *UavMonitor::ros_run(void * arg){

	std::cout << "Starting Callbacks ..." << std::endl;
	struct thread_data *args = (struct thread_data *)arg;
	
	UavMonitor *uav = args->uav;
	ros::NodeHandle * nh = args->nh;


	ros::Subscriber kill_switch = nh->subscribe<std_msgs::Bool>
								("test/kill", 10, &UavMonitor::killCb, uav);
	ros::Subscriber baseline_sub = nh->subscribe<std_msgs::Float32>
								("test/baseline",10, &UavMonitor::baselineCb, uav);
    ros::Subscriber mocap_sub = nh->subscribe<geometry_msgs::PoseStamped>
                                ("test/mocap", 10, &UavMonitor::mocapCb, uav);
    ros::Subscriber target_sub = nh->subscribe<geometry_msgs::Point>
                                ("test/target", 10, &UavMonitor::targetCb, uav);
	ros::Subscriber start_sub = nh->subscribe<std_msgs::Bool>
								("test/start", 10, &UavMonitor::startCb, uav);
	ros::Subscriber kp_sub = nh->subscribe<geometry_msgs::Point>
								("test/kp", 10, &UavMonitor::kpCb, uav);
	ros::Subscriber kd_sub = nh->subscribe<geometry_msgs::Point>
								("test/kd", 10, &UavMonitor::kdCb, uav);
								
	ros::spin();

	pthread_exit(NULL);
}
