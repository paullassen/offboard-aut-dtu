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
#include <semaphore.h>
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
  kp.set(msg->x, msg->y, msg->z);
}

void UavMonitor::kdCb(const geometry_msgs::Point::ConstPtr &msg) {
  kd.set(msg->x, msg->y, msg->z);
}

void UavMonitor::kiCb(const geometry_msgs::Point::ConstPtr &msg) {
  ki.set(msg->x, msg->y, msg->z);
}

void UavMonitor::targetCb(const geometry_msgs::Point::ConstPtr &msg) {
  yaw_transform.transform.rotation =
      tf::createQuaternionMsgFromYaw(-target_yaw * M_PI / 180);
  target.set(msg->x, msg->y, msg->z);
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
  double r, p, y;
  m.getRPY(r, p, y);
  r += r > 0 ? -M_PI : M_PI;
  if ((ros::Time::now() - last_time) > ros::Duration(0.5)) {
    // get offset
    offset_yaw = (float)y * 180 / M_PI - rpy.get_z();
    last_time = ros::Time::now();
  }
  mocap_attitude.set(r, p, y);
  // Fill the list if it is not yet initialized
  if (pos_list[0].get_x() == 0.0 && list_counter == 0) {
    for (int i = 0; i < LIST_SIZE; i++) {
      t_list[i] = ros::Time::now();
    }
  }
  list_counter++;
  list_counter %= LIST_SIZE;
//  std::cout << list_counter << std::endl;

  int prev = (list_counter + 1) % LIST_SIZE;
  pos_list[list_counter].set(mocap.pose.position.x, mocap.pose.position.y,
                             -mocap.pose.position.z);
  t_list[list_counter] = msg->header.stamp;

  ros::Duration dt = t_list[list_counter] - t_list[prev];

  velocity.set((pos_list[list_counter] - pos_list[prev]) / dt.toSec());
//std::cout <<LIST_SIZE<<"\t" <<list_counter <<"\t"<<prev<<"\t" << t_list[list_counter].toNSec() -  t_list[prev].toNSec() << "\t" << velocity.get_x() << std::endl;
  

  calculate_error();
}

void UavMonitor::baselineCb(const std_msgs::Float32::ConstPtr &msg) {
  baseline = msg->data;
}

void UavMonitor::killCb(const std_msgs::Bool::ConstPtr &msg) {
  kill = msg->data;
}

void UavMonitor::startCb(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data) {
    sem_post(&begin);
  }
}

void UavMonitor::trimCb(const std_msgs::Bool::ConstPtr &msg) {
  if (!last_trim_msg && msg->data) {
    set_trim();
  }
  last_trim_msg = msg->data;
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
  rpy.set(angle.roll_deg, angle.pitch_deg, angle.yaw_deg);
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

  if (sem_wait(&(m->begin)) == -1) {
    std::cout << "Thread Sync Error. Aborting ... " << std::endl;
    m->done = true;
    pthread_exit(NULL);
  }

  Action::Result arm_result = action->arm();
  action_error_exit(arm_result, "Arming failed");
  std::cout << "Armed" << std::endl;

  offboard->set_attitude(attitude);
  Offboard::Result offboard_result = offboard->start();
  offboard_error_exit(offboard_result, "Offboard start failed");
  offboard_log(offb_mode, "Offboard started");
  std::cout << "Offboard" << std::endl;

  struct duration timeStruct;
  initDuration(&timeStruct);
  ros::Time start = ros::Time::now();
  while (!m->kill) {
    // Control Loop Timer
    ros::Time end = ros::Time::now();
    ros::Duration t = end - start;
    addDuration(t, &timeStruct);
    start = end;
    // Control Loop
    m->set_attitude_targets(&attitude);
    offboard->set_attitude(attitude);
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

void UavMonitor::set_attitude_targets(Offboard::Attitude *attitude) {
  Triplet<float> attitude_target(kp * erp + kd * erd + ki * eri);
  attitude_target *= Triplet<float> (-1,1,1);
  attitude_target.saturate(8, 8, 0.3);
  attitude_target.get(&(attitude->pitch_deg), &(attitude->roll_deg),
                      &(attitude->thrust_value));
  attitude->thrust_value += baseline;
  attitude->pitch_deg -= trim.get_x();
  attitude->roll_deg += trim.get_y();
  attitude->yaw_deg = target_yaw - offset_yaw;

  uav_thrust = attitude->thrust_value;
  uav_rpy.set_x(attitude->roll_deg);
  uav_rpy.set_y(attitude->pitch_deg);
  uav_rpy.set_z(attitude->yaw_deg);
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
  Triplet<float> tmp(target - pos_list[list_counter]);
  geometry_msgs::Point error(tmp.to_point());

  geometry_msgs::Point error_transformed;
  tf2::doTransform(error, error_transformed, yaw_transform);

  tmp.set(Triplet<float>(-1) * velocity);
  geometry_msgs::Point derror(tmp.to_point());

  geometry_msgs::Point derror_transformed;
  tf2::doTransform(derror, derror_transformed, yaw_transform);

  erp.set(error_transformed);
  erd.set(derror_transformed);

  int sval;
  sem_getvalue(&begin, &sval);
  if (sval > 0) {
    eri += (erp / 100);
    eri.saturate(6, 6, 6);
  } else {
	  eri.set(erp);
	  eri /= Triplet<float>(100, 100, 100);
  }
}

void UavMonitor::set_trim() {
  trim.set(eri);
  trim.set_z(0);
  eri.set_x(0);
  eri.set_y(0);
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
  ros::Subscriber trim_sub =
      nh->subscribe<std_msgs::Bool>("trim", 10, &UavMonitor::trimCb, uav);
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
