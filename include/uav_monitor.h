#ifndef UAV_MONITOR_H
#define UAV_MONITOR_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <offboard/Health.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <semaphore.h>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/info/info.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include "mavsdk_helper.h"

#define LIST_SIZE 10
#define MANIPULATOR_MAXIMUM 605
#define MANIPULATOR_MINIMUM 405

template <class T>
class Triplet {
 private:
  T x;
  T y;
  T z;

 public:
  Triplet() : x(0), y(0), z(0) {}
  Triplet(T w_) : x(w_), y(w_), z(w_) {}
  Triplet(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}
  Triplet(const Triplet<float>& obj)
      : x(obj.get_x()), y(obj.get_y()), z(obj.get_z()) {}
  Triplet(const Triplet<double>& obj)
      : x(obj.get_x()), y(obj.get_y()), z(obj.get_z()) {}
  Triplet(const Triplet<int>& obj)
      : x(obj.get_x()), y(obj.get_y()), z(obj.get_z()) {}

  void set_x(T x_) { x = x_; }
  void set_y(T y_) { y = y_; }
  void set_z(T z_) { z = z_; }

  void set(T x_, T y_, T z_) {
    set_x(x_);
    set_y(y_);
    set_z(z_);
  }

  void set(const Triplet<T>& obj) {
    set_x(obj.get_x());
    set_y(obj.get_y());
    set_z(obj.get_z());
  }

  void set(const geometry_msgs::Point& obj) {
    set_x(obj.x);
    set_y(obj.y);
    set_z(obj.z);
  }

  T get_x(void) const { return x; }
  T get_y(void) const { return y; }
  T get_z(void) const { return z; }
  T get(float* x, float* y, float* z) {
    *x = get_x();
    *y = get_y();
    *z = get_z();
  }

  geometry_msgs::Point to_point() {
    geometry_msgs::Point result;
    result.x = get_x();
    result.y = get_y();
    result.z = get_z();
    return result;
  }

  void saturate(T minmax) { saturate(-minmax, minmax); }
  void saturate(T min, T max) {
    if (x > max) {
      set_x(max);
    } else if (x < min) {
      set_x(min);
    }
    if (y > max) {
      set_y(max);
    } else if (y < min) {
      set_y(min);
    }
    if (z > max) {
      set_z(max);
    } else if (z < min) {
      set_z(min);
    }
  }
  void saturate(T mx, T my, T mz) {
    if (x > mx) {
      set_x(mx);
    } else if (x < -mx) {
      set_x(-mx);
    }
    if (y > my) {
      set_y(my);
    } else if (y < -my) {
      set_y(-my);
    }
    if (z > mz) {
      set_z(mz);
    } else if (z < -mz) {
      set_z(-mz);
    }
  }

  void print() {
    std::cout << "\n-------------" << std::endl;
    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << y << std::endl;
    std::cout << "z: " << z << std::endl;
    std::cout << "-------------" << std::endl;
  }

  Triplet& operator+=(const Triplet& obj) {
    x += (T)obj.x;
    y += (T)obj.y;
    z += (T)obj.z;
    return *this;
  }

  Triplet operator+(const Triplet& obj) {
    Triplet<T> result(*this);
    result += obj;
    return result;
  }

  Triplet& operator-=(const Triplet& obj) {
    x -= (T)obj.x;
    y -= (T)obj.y;
    z -= (T)obj.z;
    return *this;
  }

  Triplet operator-(const Triplet& obj) {
    Triplet<T> result(*this);
    result -= obj;
    return result;
  }

  Triplet& operator*=(const Triplet& obj) {
    x *= (T)obj.x;
    y *= (T)obj.y;
    z *= (T)obj.z;
    return *this;
  }

  Triplet operator*(const Triplet& obj) {
    Triplet<T> result(*this);
    result *= obj;
    return result;
  }

  Triplet operator/=(const Triplet& obj) {
    x /= (T)obj.x;
    y /= (T)obj.y;
    z /= (T)obj.z;
    return *this;
  }

  Triplet operator/(double obj) {
    Triplet<T> div(obj);
    Triplet<T> result(*this);
    result /= div;
    return result;
  }
};

using namespace mavsdk;
class UavMonitor {
 public:
  UavMonitor() { sem_init(&begin, 0, 0); }
  virtual ~UavMonitor() {}
  uint64_t dur = 0;
  ros::Time last_time = ros::Time::now();

  int list_counter = 0;
  float x_list[LIST_SIZE] = {};
  float y_list[LIST_SIZE] = {};
  float z_list[LIST_SIZE] = {};
  Triplet<float> pos_list[LIST_SIZE] = {};
  ros::Time t_list[LIST_SIZE] = {};

  Triplet<float> position;
  Triplet<float> velocity;
  Triplet<float> target;

  Triplet<float> erp;  // Position Error
  Triplet<float> erd;  // Derivative Error
  Triplet<float> eri;  // Integrated Error

  Triplet<float> kp;  // Position Gain
  Triplet<float> kd;  // Derivative Gain
  Triplet<float> ki;  // Integrated Gain

  Triplet<double> mocap_attitude;
  Triplet<float> rpy;
  Triplet<float> uav_rpy;

  Triplet<float> trim;

  float baseline = 0.1;
  float uav_thrust = 0;

  float target_yaw = 0.0;
  float offset_yaw = 0.0;

  geometry_msgs::TransformStamped transform;
  geometry_msgs::TransformStamped yaw_transform;
  ros::ServiceClient manip_client;

  // Health
  offboard::Health health;
  // Battery
  float battery = 0.0;


  // Set Functions
  void set_health(Telemetry::Health);
  void set_battery(Telemetry::Battery);
  void set_angle(Telemetry::EulerAngle);

  // This function can be overrided in a derived class to insert a new
  // controller
  virtual void set_attitude_targets(Offboard::Attitude* attitude);
  void calculate_error();

  void set_trim();

  float saturate(double in, double minmax);
  float saturate_minmax(double in, double min, double max);

  // Get Functions
  bool get_health(void);
  float get_battery(void);

  // Other Functions
  void print();

  int ch = ' ';
  volatile bool done = false;
  volatile bool kill = false;
  sem_t begin;
  // Callback Functions
  void kpCb(const geometry_msgs::Point::ConstPtr& msg);
  void kdCb(const geometry_msgs::Point::ConstPtr& msg);
  void kiCb(const geometry_msgs::Point::ConstPtr& msg);
  void killCb(const std_msgs::Bool::ConstPtr& msg);
  void startCb(const std_msgs::Bool::ConstPtr& msg);
  void baselineCb(const std_msgs::Float32::ConstPtr& msg);
  void targetCb(const geometry_msgs::Point::ConstPtr& msg);
  void yawCb(const std_msgs::Float32::ConstPtr& msg);
  void mocapCb(const geometry_msgs::PoseStamped::ConstPtr& msg);

  // Manipulator Functions
  bool deploy_manipulator(void);
  bool retract_manipulator(void);
  bool command_manipulator(int);

  // Threads
  static void* offboard_control(void* arg);
  static void* ros_run(void* args);
};

/* Struct and Functions for getting timing data */
struct duration {
  ros::Duration cusum;
  ros::Duration last;
  ros::Duration max;
  ros::Duration min;
  int count;
};

void initDuration(struct duration* s) {
  s->cusum = ros::Duration(0);
  s->max = ros::Duration(0);
  s->min = ros::Duration(1);
  s->last = ros::Duration(0);
  s->count = 0;
};

void addDuration(ros::Duration d, struct duration* s) {
  s->count++;

  s->cusum += d;
  s->last = d;

  if (d > s->max) {
    s->max = d;
  }

  if (d < s->min) {
    s->min = d;
  }
};

void printDuration(const std::string& title, struct duration* s) {
  std::cout << "\n" << title << " Loop duration:" << std::endl;
  std::cout << "\t avg: " << s->cusum.toNSec() / (double)s->count << " nsec"
            << std::endl;
  std::cout << "\tlast: " << s->last.toNSec() << " nsec" << std::endl;
  std::cout << "\t max: " << s->max.toNSec() << " nsec" << std::endl;
  std::cout << "\t min: " << s->min.toNSec() << " nsec" << std::endl;
};

/* Struct for passing arguments between main function and threads */
struct thread_data {
  UavMonitor* uav;
  ros::NodeHandle* nh;
  std::shared_ptr<Offboard> offboard;
  std::shared_ptr<Action> action;
};
#endif
