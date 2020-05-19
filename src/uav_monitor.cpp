#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

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

void baselineCb(const std_msgs::Float32::ConstPtr& msg, UavMonitor *uav)
{
	uav->baseline = msg->data;
	std::cout << "BASELINE : " << uav->baseline << "\r" <<std::flush;
}
void killCb(const std_msgs::Bool::ConstPtr& msg, UavMonitor *uav)
{
	uav->done = msg->data;
	//std::cout << (msg->data ? "True\r" : "False\r") << std::endl;
}

//Health Functions
void UavMonitor::set_health(Telemetry::Health health){
	gyro_cal	= health.is_gyrometer_calibration_ok;
	accel_cal	= health.is_accelerometer_calibration_ok;
	mag_cal		= health.is_magnetometer_calibration_ok;
	level_cal	= health.is_level_calibration_ok;
	
	local_ok	= health.is_local_position_ok;
	global_ok	= health.is_global_position_ok;
	home_ok		= health.is_home_position_ok;
}

bool UavMonitor::get_health(){
	return gyro_cal && accel_cal && mag_cal && level_cal;
}

// Battery Functions
void UavMonitor::set_battery(Telemetry::Battery bat){
	battery = bat.voltage_v;
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
void UavMonitor::print(){	
		std::cout << "Gyro Calibration  : " 
				<< (gyro_cal ? "OK" : "Failed") << std::endl;
		std::cout << "Accel Calibration : "
				<< (accel_cal ? "OK" : "Failed") << std::endl;
		std::cout << "Mag Calibration   : "
				<< (mag_cal ? "OK" : "Failed") << std::endl;
		std::cout << "Level Calibration : "
				<< (level_cal ? "OK" : "Failed") << std::endl;
		std::cout << "Local Position    : "
				<< (local_ok ? "OK" : "Failed") << std::endl;
		std::cout << "Global Position   : "
				<< (global_ok ? "OK" : "Failed") << std::endl;
		std::cout << "Home Position     : "
				<< (home_ok ? "OK" : "Failed") << std::endl;
		std::cout << "Battery           : " 
				<< battery << "V" << std::endl;
		std::cout << "Roll              : "
				<< roll << "deg" << std::endl;
		std::cout << "Pitch             : "
				<< pitch << "deg" << std::endl;
		std::cout << "Yawi              : "
				<< yaw << "deg" << std::endl;
}

void *term_routine(void *arg){
	int row, col;

	UavMonitor *m =(UavMonitor *) arg;

	initscr();
	getmaxyx(stdscr, row, col);
	WINDOW *w = stdscr;// = newwin(20,col,0, 0);
	//cbreak(); // or raw();
	raw();
	nodelay(w, TRUE);
	
	noecho();
	
	while(!m->done){
		for(int i = 0; i < row -10; ++i)
		{
			mvhline(i,0,' ',col);
		}
		refresh();
		
		//Title
		attron(A_BOLD);
		mvhline(0,0,'=',col);
		char title[] = "UAV Monitor [Test]";
		mvprintw(1,(col-strlen(title))/2,title);
		mvhline(2,0,'=',col);
		//Left Column
		mvvline(3,0,'|',17);
		mvvline(3,12,'|',17);
		//Health
		mvprintw(7,3, "Health");
		attroff(A_BOLD);

		mvprintw(4 , 16, "Gyro");
		mvprintw(6 , 16, "Accel");
		mvprintw(8 , 16, "Mag");
		mvprintw(10, 16, "Level");
		mvprintw(4 , 22, (m->gyro_cal ? " : OK" : " : FAIL"));
		mvprintw(6 , 22, (m->accel_cal ? " : OK" : " : FAIL"));
		mvprintw(8 , 22, (m->mag_cal ? " : OK" : " : FAIL"));
		mvprintw(10, 22, (m->level_cal ? " : OK" : " : FAIL"));

		
		mvprintw(4 , 40, "Local");
		mvprintw(6 , 40, "Global");
		mvprintw(8 , 40, "Home");
		mvprintw(10, 40, "Battery");
		mvprintw(4 , 46, (m->local_ok ? " : OK" : " : FAIL"));
		mvprintw(6 , 46, (m->global_ok ? " : OK" : " : FAIL"));
		mvprintw(8 , 46, (m->home_ok ? " : OK" : " : FAIL"));
		mvprintw(10, 46, " : %0.4fV", m->get_battery());

		mvhline(12,0,'=',col);

		attron(A_BOLD);
		mvprintw(16,1,"Orientation");
		attroff(A_BOLD);

		mvprintw(14, 16, "Roll");
		mvprintw(16, 16, "Pitch");
		mvprintw(18, 16, "Yaw");
		mvprintw(14, 22, " : %0.3f deg",m->roll);
		mvprintw(16, 22, " : %0.3f deg",m->pitch);
		mvprintw(18, 22, " : %0.3f deg",m->yaw);
		
		mvprintw(14, 40, "dRoll");
		mvprintw(16, 40, "dPitch");
		mvprintw(18, 40, "dYaw");
		mvprintw(14, 46, " : %0.3f deg/s", m->roll);
		mvprintw(16, 46, " : %0.3f deg/s", m->roll);
		mvprintw(18, 46, " : %0.3f deg/s", m->roll);

		mvhline(20,0,'=',col);
		mvprintw(row-1,0,">> ");
		refresh();

		usleep(100000);
	}
	endwin();
	pthread_exit(NULL);
}

void *term_monitor(void *arg){
	UavMonitor *m = (UavMonitor *) arg;


	while(!m->done){
		char c;
		c=getch();
		if(isalnum(c))
		{
			m->ch = c;
			m->done = (tolower(m->ch) == 'q');
		}
		usleep(100000);
	}
	pthread_exit(NULL);
}

void *offboard_control(void *arg){

    const std::string offb_mode = "ATTITUDE";

	void **args = (void **)arg;
	UavMonitor *m =(UavMonitor *)args[0];
	std::shared_ptr<mavsdk::Offboard> offboard = *(std::shared_ptr<mavsdk::Offboard> *)args[1];
	std::shared_ptr<mavsdk::Action> action = *(std::shared_ptr<mavsdk::Action> *)args[2];
	

	Offboard::Attitude attitude;
	attitude.roll_deg = 0.0f;
	attitude.pitch_deg= 0.0f;
	attitude.yaw_deg = 0.0f;
	attitude.thrust_value = 0.1f;
	offboard->set_attitude(attitude);
	Offboard::Result offboard_result = offboard->start();
    offboard_error_exit(offboard_result, "Offboard start failed");
    offboard_log(offb_mode, "Offboard started");

	while(!m->done){
		attitude.thrust_value =	m->calculate_thrust();			
		attitude.roll_deg = m->calculate_roll();
		attitude.pitch_deg = m->calculate_pitch();
		attitude.yaw_deg = m->calculate_yaw();
		offboard->set_attitude(attitude);
	}

	offboard_result = offboard->stop();
    offboard_error_exit(offboard_result, "Offboard stop failed: ");
	offboard_log(offb_mode, "Offboard stopped");
    
	const Action::Result land_result = action->land();

	pthread_exit(NULL);
}

float UavMonitor::calculate_thrust(){
	float thrust = baseline;
	thrust += kpz * ez + kdz * edz;
	return thrust;
}

float UavMonitor::calculate_roll(){
	return 0.0;
}

float UavMonitor::calculate_pitch(){
	return 0.0;
}

float UavMonitor::calculate_yaw(){
	return 0.0;
}
