#ifndef UAV_MONITOR_H
#define UAV_MONITOR_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/info/info.h>
#include "mavsdk_helper.h"


using namespace mavsdk;
class UavMonitor
{
	public:
		float ez = 0;
		float edz = 0;
		float kpz = 0;
		float kdz = 0;
		float baseline = 0.1;
		// Health
		bool gyro_cal = false;
		bool accel_cal = false;
		bool mag_cal = false;
		bool level_cal = false;
		bool local_ok = false;
		bool global_ok = false;
		bool home_ok = false;
		
		// Battery
		float battery = 0.0;

		// Orientation
		float roll = 0;
		float pitch = 0;
		float yaw = 0;	

		// Set Functions
		void set_health(Telemetry::Health);
		void set_battery(Telemetry::Battery);
		void set_angle(Telemetry::EulerAngle);

		float calculate_thrust();
		float calculate_roll();
		float calculate_pitch();
		float calculate_yaw();

		// Get Functions
		bool	get_health(void);
		float	get_battery(void);


		// Other Functions	
		void print();

		int ch = ' ';
		bool done = false;
		bool kill = false;
};

void killCb(const std_msgs::Bool::ConstPtr& msg, UavMonitor *);
void baselineCb(const std_msgs::Float32::ConstPtr& msg, UavMonitor *);
void *term_routine(void *arg);
void *term_monitor(void *arg);
void *offboard_control(void *arg);
void action_error_exit(Action::Result result, const std::string& message);

#endif
