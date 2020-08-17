#ifndef ATTITUDE_STEP_H
#define ATTITUDE_STEP_H
#include "uav_monitor.h"
class AttitudeStep : public UavMonitor {
 public:
  virtual ~AttitudeStep() {}

  virtual float calculate_thrust() { return baseline; }

  virtual float calculate_roll() { return tx; }

  virtual float calculate_pitch() { return ty; }

  virtual float calculate_yaw() { return yaw; }
};
#endif
