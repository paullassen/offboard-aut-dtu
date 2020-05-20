#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

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

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

//UavMonitor::UavMonitor(){}
inline void action_error_exit(Action::Result result, const std::string& message)
{
    if (result != Action::Result::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles Offboard's result
inline void offboard_error_exit(Offboard::Result result, const std::string& message)
{
    if (result != Offboard::Result::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles connection result
inline void connection_error_exit(ConnectionResult result, const std::string& message)
{
    if (result != ConnectionResult::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Logs during Offboard control
inline void offboard_log(const std::string& offb_mode, const std::string msg)
{
    std::cout << "[" << offb_mode << "] " << msg << std::endl;
}

void targetCb(const geometry_msgs::Point::ConstPtr& msg, UavMonitor *uav){
    uav->tx = msg->x;
    uav->ty = msg->y;
    uav->tz = msg->z;
}

void mocapCb(const geometry_msgs::PoseStamped::ConstPtr& msg, UavMonitor *uav){
    float last_x = uav->x;
    float last_y = uav->y;
    float last_z = uav->z;

    uav->x = msg->pose.position.x;
    uav->y = -msg->pose.position.y;
    uav->z = msg->pose.position.z;

    
    ros::Duration dt = msg->header.stamp - uav->last_time;
    
    uav->dx = (uav->x-last_x)/dt.toSec();
    uav->dy = (uav->y-last_y)/dt.toSec();
    uav->dz = (uav->z-last_z)/dt.toSec();
    
    uav->last_time = msg->header.stamp;

    uav->calculate_error();
}

void baselineCb(const std_msgs::Float32::ConstPtr& msg, UavMonitor *uav)
{
	uav->baseline = msg->data;
	std::cerr << "BASELINE : " << uav->baseline << "\r" <<std::flush;
}

void killCb(const std_msgs::Bool::ConstPtr& msg, UavMonitor *uav)
{
	uav->kill = msg->data;
	//std::cout << (msg->data ? "True\r" : "False\r") << std::endl;
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
void *offboard_control(void *arg){

    const std::string offb_mode = "ATTITUDE";

	void **args = (void **)arg;
	UavMonitor *m =(UavMonitor *)args[0];
	std::shared_ptr<mavsdk::Offboard> offboard = *(std::shared_ptr<mavsdk::Offboard> *)args[1];
	std::shared_ptr<mavsdk::Action> action = *(std::shared_ptr<mavsdk::Action> *)args[2];
	
	ros::Rate rate(20);

	Offboard::Attitude attitude;
	attitude.roll_deg	= 0.0f;
	attitude.pitch_deg	= 0.0f;
	attitude.yaw_deg	= 0.0f;
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

	offboard_result = offboard->stop();
    offboard_error_exit(offboard_result, "Offboard stop failed: ");
	offboard_log(offb_mode, "Offboard stopped");
    
	const Action::Result land_result = action->land();
	m->done = true;
	pthread_exit(NULL);
}

float UavMonitor::calculate_thrust(){
	float thrust = baseline;
	thrust += kpz * ez + kdz * edz;

	double scale = cos((double) roll * M_PI/180)*cos((double) pitch * M_PI/180);

	return thrust/((float)scale);
}

float UavMonitor::calculate_roll(){
	double ky = kpy * ey + kdy * edy;
	double kx = kpx * ex + kdx * edx;

	double yaw_deg = yaw * M_PI/180;

	return (float) (kx * cos(yaw_deg) + ky * sin(yaw_deg));
}

float UavMonitor::calculate_pitch(){
	double ky = kpy * ey + kdy * edy;
	double kx = kpx * ex + kdx * edx;

	double yaw_deg = yaw * M_PI/180;
	
	return (float) (-kx * sin(yaw_deg) + ky * cos(yaw_deg));
}

float UavMonitor::calculate_yaw(){
	double yaw_deg = yaw * M_PI/180;
	return (float) yaw_deg;
}

void UavMonitor::calculate_error(){
    ex = tx - x;
    ey = ty - y;
    ez = tz - z;

    edx = dx;
    edy = dy;
    edz = dz;
}
