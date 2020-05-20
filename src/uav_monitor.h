#ifndef UAV_MONITOR_H
#define UAV_MONITOR_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <offboard/Health.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
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

        ros::Time last_time;
        float x = 0;
        float y = 0;
        float z = 0;

        float dx = 0;
        float dy = 0;
        float dz = 0;

        float tx = 0;
        float ty = 0;
        float tz = 0;

		float ex = 0;
		float ey = 0;
		float ez = 0;

		float edx = 0;
		float edy = 0;
		float edz = 0;

        float eix = 0;
        float eiy = 0;
        float eiz = 0;

		float kpx = 0;
		float kpy = 0;
		float kpz = 0;

		float kdx = 0;
		float kdy = 0;
		float kdz = 0;

        float kix = 0;
        float kiy = 0;
        float kiz = 0;

		float baseline = 0.1;
		
		// Health
		offboard::Health health;

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
        void calculate_error();
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
void targetCb(const geometry_msgs::Point::ConstPtr& msg, UavMonitor *);

void mocapCb(const geometry_msgs::PoseStamped::ConstPtr& msg, UavMonitor *);
void *offboard_control(void *arg);

#endif
