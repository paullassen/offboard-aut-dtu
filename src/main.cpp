#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>

#include <unistd.h>
#include <pthread.h>
#include <ncurses.h>
#include <string.h>

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
	UavMonitor *m =(UavMonitor *)  arg;

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

int main(int argc, char ** argv){
	pthread_t p0, p1;
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
	ros::Rate rate(10.0);

	//Create ROS msgs
	std_msgs::Bool state;
	std_msgs::Float32 volt;

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


	//pthread_create(&p0, NULL, term_routine, &uav);
	//pthread_create(&p1, NULL, term_monitor, &uav);
	while(!uav.done){
		
		state.data = uav.gyro_cal;
		gy_pub.publish(state);
		
		state.data = uav.accel_cal;
		ac_pub.publish(state);
		
		state.data = uav.mag_cal;
		mag_pub.publish(state);
		
		state.data = uav.level_cal;
		lev_pub.publish(state);
		
		state.data = uav.local_ok;
		loc_pub.publish(state);
		
		state.data = uav.global_ok;
		glob_pub.publish(state);
		
		state.data = uav.home_ok;
		home_pub.publish(state);
		
		volt.data = uav.battery;
		bat_pub.publish(volt);

		rate.sleep();
	}
	//pthread_join(p1, NULL);
	//pthread_join(p0, NULL);
	return 0;
}

				  
