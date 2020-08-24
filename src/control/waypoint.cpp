#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <functional>
#include <iostream>
#include <list>
#include <string>
#include <utility>

#define UPDATE_RATE 50
#define MANIPULATOR_MINIMUM 300
#define MANIPULATOR_MAXIMUM 740


class Controller {
 public:
  ros::NodeHandle* nh;
  ros::ServiceClient manip_client;

  sensor_msgs::Joy joystick;
  geometry_msgs::Point target;
  std_msgs::Bool start, kill;
  std_msgs::Float32 baseline, yaw;

  ros::Publisher bl_pub;
  ros::Publisher tg_pub;
  ros::Publisher kl_pub;
  ros::Publisher st_pub;
  ros::Publisher yw_pub;

  ros::Time start_time;
  int targ_step = 0;
  float targ_time[12] = {0.00, 2.50, 5.00, 7.50, 10.0, 15.0,
                         17.5, 40.0, 45.0, 47.5, 50.0, 55.0};
  float x_targs[12] = {0.00, 0.00, 0.00,  0.00,  0.00,  0.25,
                       0.50, 0.50, -0.25, -0.25, -0.25, -0.25};
  float y_targs[12] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
                       0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
  float z_targs[12] = {0.00, 0.80, 0.80, 0.80, 0.80, 0.80,
                       0.80, 0.80, 0.80, 0.50, 0.50, 0.25};
  float yw_targs[12] = {90.0, 90.0, 90.0, 90.0, 90.0, 90.0,
                        90.0, 90.0, 90.0, 90.0, 90.0, 90.0};

  Controller(ros::NodeHandle* n) : nh(n) {
    //    bl_pub = nh->advertise<std_msgs::Float32>("baseline_", 10);
    yw_pub = nh->advertise<std_msgs::Float32>("yaw_target_", 10);
    st_pub = nh->advertise<std_msgs::Bool>("start_", 10);
    //   kl_pub = nh->advertise<std_msgs::Bool>("kill_", 10);
    tg_pub = nh->advertise<geometry_msgs::Point>("target_", 10);

    manip_client =
        nh->serviceClient<dynamixel_workbench_msgs::DynamixelCommand>(
            "dynamixel_workbench/dynamixel_command");

    initController();
  }

  void initController(void) {
    start_time = ros::Time::now();
    target.x = x_targs[0];
    target.y = y_targs[0];
    target.z = z_targs[0];
  }

  void update(void) {
    double length = sizeof(targ_time) / sizeof(targ_time[0]);
    if (ros::Time::now() - start_time > ros::Duration(targ_time[targ_step])) {
      targ_step++;
      if (targ_step < length) {
        std::cout << "step++" << std::endl;
        target.x = x_targs[targ_step];
        target.y = y_targs[targ_step];
        target.z = z_targs[targ_step];
        yaw.data = yw_targs[targ_step];
      }
    }
  }

  void publish(void) {
    //  bl_pub.publish(baseline);
    yw_pub.publish(yaw);
    //  kl_pub.publish(kill);
    st_pub.publish(start);
    tg_pub.publish(target);
  }
};

int main(int argc, char** argv) {
  std::cout << "Starting Node" << std::endl;
  ros::init(argc, argv, "auto_ctrl");
  ros::NodeHandle nh;

  std::cout << "Starting Controller" << std::endl;
  Controller controller(&nh);
  ros::Rate rate(UPDATE_RATE);

  std::cout << "Starting loop" << std::endl;
  while (ros::ok()) {
    ros::spinOnce();
    controller.update();
    controller.publish();
  }
  ros::shutdown();
  return 0;
}

