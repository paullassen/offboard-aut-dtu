// ROS libraries
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// Standard C++ libraries
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <queue>
#include <thread>
// Standard C libraries
#include <ncurses.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
// MAVSDK libraries
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/info/info.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
// Application libraries
#include "mavsdk_helper.h"
#include "uav_monitor.h"

using namespace mavsdk;

void UavMonitor::kpCb(const geometry_msgs::Point::ConstPtr &msg) {
  kpx = msg->x;
  kpy = msg->y;
  kpz = msg->z;
}

void UavMonitor::kdCb(const geometry_msgs::Point::ConstPtr &msg) {
  kdx = msg->x;
  kdy = msg->y;
  kdz = msg->z;
}

void UavMonitor::kiCb(const geometry_msgs::Point::ConstPtr &msg) {
  kix = msg->x;
  kiy = msg->y;
  kiz = msg->z;
}

void UavMonitor::targetCb(const geometry_msgs::Point::ConstPtr &msg) {
  yaw_transform.transform.rotation =
      tf::createQuaternionMsgFromYaw(-target_yaw * M_PI / 180);

  tx = msg->x;
  ty = msg->y;
  tz = msg->z;
}

void UavMonitor::yawCb(const std_msgs::Float32::ConstPtr &msg) {
  target_yaw = msg->data;
}

void UavMonitor::mocapCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  // create quaternion
  geometry_msgs::PoseStamped mocap;
  try {
    tf2::doTransform(*msg, mocap, transform);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }

  tf::Quaternion q(mocap.pose.orientation.x, mocap.pose.orientation.y,
                   mocap.pose.orientation.z, mocap.pose.orientation.w);

  // get rotation matrix
  tf::Matrix3x3 m(q);
  // get r,p,y
  m.getRPY(mocap_roll, mocap_pitch, mocap_yaw);
  mocap_roll += mocap_roll > 0 ? -M_PI : M_PI;
  if ((ros::Time::now() - last_time) > ros::Duration(0.5)) {
    // get offset
    offset_yaw = (float)mocap_yaw * 180 / M_PI - yaw;
    last_time = ros::Time::now();
  }

  // Fill the list if it is not yet initialized
  if (x_list[0] == 0.0 && list_counter == 0) {
    for (int i = 0; i < LIST_SIZE; i++) {
      t_list[i] = ros::Time::now();
    }
  }
  list_counter = ++list_counter % LIST_SIZE;

  int prev = (list_counter + 1) % LIST_SIZE;
  x_list[list_counter] = mocap.pose.position.x;
  y_list[list_counter] = mocap.pose.position.y;
  z_list[list_counter] = -mocap.pose.position.z;
  t_list[list_counter] = msg->header.stamp;

  ros::Duration dt = t_list[list_counter] - t_list[prev];

  dx = (x_list[list_counter] - x_list[prev]) / dt.toSec();
  dy = (y_list[list_counter] - y_list[prev]) / dt.toSec();
  dz = (z_list[list_counter] - z_list[prev]) / dt.toSec();

  calculate_error();
}

void UavMonitor::baselineCb(const std_msgs::Float32::ConstPtr &msg) {
  baseline = msg->data;
  // std::cerr << "BASELINE : " << baseline << "\r" <<std::flush;
}

void UavMonitor::killCb(const std_msgs::Bool::ConstPtr &msg) {
  kill = msg->data;
}

void UavMonitor::startCb(const std_msgs::Bool::ConstPtr &msg) {
  begin = msg->data;
}

// Health Functions
void UavMonitor::set_health(Telemetry::Health h) {
  health.gyro = h.is_gyrometer_calibration_ok;
  health.accel = h.is_accelerometer_calibration_ok;
  health.mag = h.is_magnetometer_calibration_ok;
  health.level = h.is_level_calibration_ok;

  health.local = h.is_local_position_ok;
  health.globe = h.is_global_position_ok;
  health.home = h.is_home_position_ok;
}

bool UavMonitor::get_health() {
  return health.gyro && health.accel && health.mag && health.level;
}

// Battery Functions
void UavMonitor::set_battery(Telemetry::Battery bat) {
  health.battery = bat.voltage_v;
}

float UavMonitor::get_battery() { return battery; }

// Attitude Functions
void UavMonitor::set_angle(Telemetry::EulerAngle angle) {
  roll = angle.roll_deg;
  pitch = angle.pitch_deg;
  yaw = angle.yaw_deg;
}

void UavMonitor::set_actuator_target(Telemetry::ActuatorControlTarget act) {
  actuator_target = act.controls;
}

void UavMonitor::set_actuator_status(Telemetry::ActuatorOutputStatus aos) {
  actuator_status = aos.actuator;
}

bool UavMonitor::command_manipulator(int value) {
  dynamixel_workbench_msgs::DynamixelCommand srv;

  srv.request.command = "";
  srv.request.id = 4;
  srv.request.addr_name = "Goal_position";
  srv.request.value = value;

  return manip_client.call(srv);
}

bool UavMonitor::deploy_manipulator() {
  dynamixel_workbench_msgs::DynamixelCommand srv;
  return command_manipulator(MANIPULATOR_MAXIMUM);
}

bool UavMonitor::retract_manipulator() {
  dynamixel_workbench_msgs::DynamixelCommand srv;
  return command_manipulator(MANIPULATOR_MINIMUM);
}

// Other Functions
void *UavMonitor::offboard_control(void *arg) {
  std::cout << "Starting control thread ..." << std::endl;
  const std::string offb_mode = "ATTITUDE";

  struct thread_data *args = (struct thread_data *)arg;

  UavMonitor *m = args->uav;
  std::shared_ptr<mavsdk::Offboard> offboard = args->offboard;
  std::shared_ptr<mavsdk::Action> action = args->action;

  ros::Rate rate(100);

  Offboard::Attitude attitude;
  attitude.roll_deg = 0.0f;
  attitude.pitch_deg = -0.0f;
  attitude.yaw_deg = 0.0f;
  attitude.thrust_value = 0.1f;

  while (!m->begin) {
  }
  Action::Result arm_result = action->arm();
  action_error_exit(arm_result, "Arming failed");
  std::cout << "Armed" << std::endl;

  offboard->set_attitude(attitude);
  Offboard::Result offboard_result = offboard->start();
  offboard_error_exit(offboard_result, "Offboard start failed");
  offboard_log(offb_mode, "Offboard started");

  struct duration timeStruct;
  initDuration(&timeStruct);
  while (!m->kill) {
    ros::Time start = ros::Time::now();
    attitude.thrust_value = m->calculate_thrust();
    attitude.roll_deg = m->calculate_roll();
    attitude.pitch_deg = m->calculate_pitch();
    attitude.yaw_deg = m->calculate_yaw();
    offboard->set_attitude(attitude);
    ros::Time end = ros::Time::now();
    ros::Duration t = end - start;
    addDuration(t, &timeStruct);
    rate.sleep();
  }

  printDuration("Control", &timeStruct);

  std::cout << "Zeroing control inputs ..." << std::endl;
  attitude.thrust_value = 0.0f;
  attitude.roll_deg = 0.0f;
  attitude.pitch_deg = 0.0f;
  attitude.yaw_deg = 0.0f;
  offboard->set_attitude(attitude);

  /*
  //	This wasn't working and was crashing the program before killing the
  drone offboard_result = offboard->stop(); offboard_error_exit(offboard_result,
  "Offboard stop failed: "); offboard_log(offb_mode, "Offboard stopped");
  */
  sleep(1);
  std::cout << "Sending kill command ..." << std::endl;

  const Action::Result kill_result = action->kill();

  m->done = true;
  pthread_exit(NULL);
}

float UavMonitor::calculate_thrust() {
  float thrust = baseline;
  thrust += kpz * ez + kdz * edz + kiz * eiz;

  double scale =
      cos((double)roll * M_PI / 180) * cos((double)pitch * M_PI / 180);

  uav_thrust = thrust;  // / scale;
  return saturate_minmax(uav_thrust, 0, 0.50);
}

float UavMonitor::calculate_pitch() {
  double ky = (kpy * ey + kdy * edy + kiy * eiy);
  double kx = (kpx * ex + kdx * edx + kix * eix);

  double yaw_rad = (mocap_yaw)*M_PI / 180;
  // 2 degree offset (from data analysis)
  // uav_pitch = -2+saturate(-kx * cos(yaw_rad) + -ky * sin(yaw_rad), 6);
  uav_pitch = -2 + saturate(-kx, 6);
  return uav_pitch;
}

float UavMonitor::calculate_roll() {
  double ky = (kpy * ey + kdy * edy + kiy * eiy);
  double kx = (kpx * ex + kdx * edx + kix * eix);

  double yaw_rad = (mocap_yaw)*M_PI / 180;

  // uav_roll =  saturate(-kx * sin(yaw_rad) + ky * cos(yaw_rad), 6);
  uav_roll = saturate(ky, 6);
  return uav_roll;
}

float UavMonitor::calculate_yaw() {
  // double yaw_rad = (yaw+offset_yaw) * M_PI/180;
  uav_yaw = (float)target_yaw - offset_yaw;
  return (float)uav_yaw;
}

float UavMonitor::saturate(double in, double minmax) {
  return saturate_minmax(in, -minmax, minmax);
}

float UavMonitor::saturate_minmax(double in, double min, double max) {
  if (in > max) {
    return (float)max;
  } else if (in < min) {
    return (float)min;
  }

  return (float)in;
}

void UavMonitor::calculate_error() {
  geometry_msgs::Point error;
  geometry_msgs::Point error_transformed;
  error.x = tx - x_list[list_counter];
  error.y = ty - y_list[list_counter];
  error.z = tz - z_list[list_counter];

  tf2::doTransform(error, error_transformed, yaw_transform);

  geometry_msgs::Point derror;
  geometry_msgs::Point derror_transformed;
  derror.x = -dx;
  derror.y = -dy;
  derror.z = -dz;

  tf2::doTransform(derror, derror_transformed, yaw_transform);

  ex = error_transformed.x;
  ey = error_transformed.y;
  ez = error_transformed.z;

  edx = derror_transformed.x;
  edy = derror_transformed.y;
  edz = derror_transformed.z;

  if (begin) {
    eix += ex / 100;
    eiy += ey / 100;
    eiz += ez / 100;
  }
}

void *UavMonitor::ros_run(void *arg) {
  std::cout << "Starting Callbacks ..." << std::endl;
  struct thread_data *args = (struct thread_data *)arg;

  UavMonitor *uav = args->uav;
  ros::NodeHandle *nh = args->nh;

  ros::Subscriber kill_switch =
      nh->subscribe<std_msgs::Bool>("kill", 10, &UavMonitor::killCb, uav);
  ros::Subscriber start_sub =
      nh->subscribe<std_msgs::Bool>("start", 10, &UavMonitor::startCb, uav);
  ros::Subscriber kp_sub =
      nh->subscribe<geometry_msgs::Point>("kp", 10, &UavMonitor::kpCb, uav);
  ros::Subscriber kd_sub =
      nh->subscribe<geometry_msgs::Point>("kd", 10, &UavMonitor::kdCb, uav);
  ros::Subscriber ki_sub =
      nh->subscribe<geometry_msgs::Point>("ki", 10, &UavMonitor::kiCb, uav);

  ros::Subscriber baseline_sub = nh->subscribe<std_msgs::Float32>(
      "baseline", 10, &UavMonitor::baselineCb, uav);
  ros::Subscriber mocap_sub = nh->subscribe<geometry_msgs::PoseStamped>(
      "mocap", 10, &UavMonitor::mocapCb, uav);
  ros::Subscriber target_sub = nh->subscribe<geometry_msgs::Point>(
      "target", 10, &UavMonitor::targetCb, uav);
  ros::Subscriber yw_sub = nh->subscribe<std_msgs::Float32>(
      "yaw_target", 10, &UavMonitor::yawCb, uav);

  uav->manip_client =
      nh->serviceClient<dynamixel_workbench_msgs::DynamixelCommand>(
          "/dynamixel_workbench/dynamixel_command");
  ros::spin();

  pthread_exit(NULL);
}
