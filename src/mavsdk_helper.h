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
void usage(std::string bin_name);

void wait_until_discover(Mavsdk& dc);

#endif
