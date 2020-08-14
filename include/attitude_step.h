#ifndef ATTITUDE_STEP_H
#define ATTITUDE_STEP_H
#include "uav_monitor.h"
class AttitudeStep : public UavMonitor {
  float calculate_thrust() { return baseline; }

  float calculate_roll() { return tx; }

  float calculate_pitch() { return ty; }

  float calculate_yaw() { return yaw; }
};
#endif
