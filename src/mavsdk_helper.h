#ifndef MAVSDK_HELPER_H
#define MAVSDK_HELPER_H

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


using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour
void action_error_exit(Action::Result result, const std::string& message);
void offboard_error_exit(Offboard::Result result, const std::string& message);
void offboard_log(const std::string& offb_mode, const std::string msg);
void connection_error_exit(ConnectionResult result, const std::string& message);
void usage(std::string bin_name);

void wait_until_discover(Mavsdk& dc);

#endif
