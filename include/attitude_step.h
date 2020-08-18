#ifndef ATTITUDE_STEP_H
#define ATTITUDE_STEP_H
#include "uav_monitor.h"
class AttitudeStep : public UavMonitor {
 public:
  virtual ~AttitudeStep() {}
  virtual void set_attitude_targets(Offboard::Attitude* attitude) {
    float tmp;
    target.get(&(attitude->roll_deg), &(attitude->pitch_deg), &tmp);
    attitude->yaw_deg = rpy.get_z();
    attitude->thrust_value = baseline;
    uav_rpy.set(target);
    uav_rpy.set_z(attitude->yaw_deg);
  }
};
#endif
