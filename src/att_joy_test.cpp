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

class Button {
 public:
  std::string name = "";
  std::function<void(void)> pressed;
  std::function<void(void)> released;
  int state = 0;
  int index = 0;

  Button(int i, std::string n)
      : name(n),
        state(0),
        index(i),
        pressed([this] { std::cout << this->name << " pressed" << std::endl; }),
        released(
            [this] { std::cout << this->name << " released" << std::endl; }) {}
  Button(int i, std::string n, std::function<void(void)> p)
      : name(n), state(0), index(i), pressed(p), released([this] {
          std::cout << this->name << "released" << std::endl;
        }) {}

  void update(int value) {
    if (state == 0 && value > 0) {
      pressed();
    } else if (state == 1 && value < 1) {
      released();
    }

    state = value;
  }
};

class Cross : public Button {
 public:
  std::function<void(void)> pressed1;
  std::function<void(void)> released1;

  Cross(int i, std::string n) : Button(i, n) {}
  Cross(int i, std::string n, std::function<void(void)> p) : Button(i, n, p) {}

  Cross(int i, std::string n, std::function<void(void)> p,
        std::function<void(void)> p1)
      : Button(i, n, p) {
    pressed1 = p1;
    released1 = [this] { std::cout << this->name << " released" << std::endl; };
  }

  void update(int value) {
    if (state >= 0 && value > 0) {
      pressed();
    } else if (state > 0 && value <= 0) {
      released();
    }

    if (state <= 0 && value < 0) {
      pressed1();
    } else if (state < 0 && value >= 0) {
      released1();
    }
  }
};

class Stick {
 public:
  int index;
  std::string name;
  float state;
  float max;
  float* target;

  Stick(int i, std::string n, float* rel)
      : index(i), name(n), state(0.0), target(rel), max(0.0) {}
  Stick(int i, std::string n, float* rel, int mx)
      : index(i), name(n), state(0.0), target(rel), max(mx) {}

  void update(float value) { state = value; }

  void update_relative(void) {
    float state_update = state / (UPDATE_RATE * 3);

    if (max > 0) {
      state_update *= max;
    }

    *target += state_update;

    if (max > 0) {
      if (*target > max) {
        *target = max;
      } else if (*target < -max) {
        *target = -max;
      }
    }
  }
};

class Controller {
 public:
  ros::NodeHandle* nh;
  ros::ServiceClient manip_client;

  sensor_msgs::Joy joystick;
  geometry_msgs::Point target;
  std_msgs::Bool start, kill;
  std_msgs::Float32 baseline, yaw;

  float relative_x;
  float relative_y;
  float relative_z;
  float relative_yaw;

  ros::Publisher bl_pub;
  ros::Publisher tg_pub;
  ros::Publisher kl_pub;
  ros::Publisher st_pub;
  ros::Publisher yw_pub;

  std::list<Button> buttons;
  std::list<Cross> cross;
  // std::list<Stick> stick;

  Controller(ros::NodeHandle* n) : nh(n) {
    bl_pub = nh->advertise<std_msgs::Float32>("baseline_", 10);
    yw_pub = nh->advertise<std_msgs::Float32>("yaw_target_", 10);
    st_pub = nh->advertise<std_msgs::Bool>("start_", 10);
    kl_pub = nh->advertise<std_msgs::Bool>("kill_", 10);
    tg_pub = nh->advertise<geometry_msgs::Point>("target_", 10);

    manip_client =
        nh->serviceClient<dynamixel_workbench_msgs::DynamixelCommand>(
            "dynamixel_workbench/dynamixel_command");

    initController();
  }

  void initController(void) {
    buttons.push_back(Button(0, "b", [this] { this->kill.data = true; }));
    buttons.push_back(Button(1, "a"));
    buttons.push_back(Button(2, "y", [this] { this->baseline.data -= 0.01; }));
    buttons.push_back(Button(3, "x", [this] { this->baseline.data += 0.01; }));
    buttons.push_back(Button(4, "Lb", [this] { this->yaw.data -= 2.5; }));
    buttons.push_back(Button(5, "Rb", [this] { this->yaw.data += 2.5; }));
    buttons.push_back(Button(6, "Lt", [this] { this->target.z -= 0.1; }));
    buttons.push_back(Button(7, "Rt", [this] { this->target.z += 0.1; }));
    buttons.push_back(Button(8, "minus", [this] {
      this->command_manipulator(MANIPULATOR_MINIMUM);
    }));
    buttons.push_back(Button(
        9, "plus", [this] { this->command_manipulator(MANIPULATOR_MAXIMUM); }));
    buttons.push_back(Button(10, "ls"));
    buttons.push_back(Button(11, "rs"));
    buttons.push_back(Button(12, "home", [this] { this->start.data = true; }));
    buttons.push_back(Button(13, "cap"));

    cross.push_back(Cross(
        4, "LR",
        [this] {
          this->target.x -= 25.0;
          if (this->target.x < -25.0) this->target.x = -25.0;
          this->reset_relative_target();
        },
        [this] {
          this->target.x += 25.0;
          if (this->target.x > 25.0) this->target.x = 25.0;
          this->reset_relative_target();
        }));
    cross.push_back(Cross(
        5, "UD",
        [this] {
          this->target.y -= 25.0;
          if (this->target.y < -25.0) this->target.y = -25.0;
          this->reset_relative_target();
        },
        [this] {
          this->target.y += 25.0;
          if (this->target.y > 25.0) this->target.y = 25.0;
          this->reset_relative_target();
        }));

    // stick.push_back(Stick(0, "lLR", &relative_y));
    // stick.push_back(Stick(1, "lUD", &relative_x));
    // stick.push_back(Stick(2, "rLR", &relative_yaw, 180));
    // stick.push_back(Stick(3, "rUD", &relative_z));
  }

  bool command_manipulator(int value) {
    dynamixel_workbench_msgs::DynamixelCommand srv;
    srv.request.command = "";
    srv.request.id = 4;
    srv.request.addr_name = "Goal_Position";
    srv.request.value = value;

    return manip_client.call(srv);
  }

  void reset_relative_target(void) {
    relative_x = 0;
    relative_y = 0;
  }

  // void set_relative_target(void) {
  //  for (Stick s : stick) {
  //    s.update_relative();
  //  }

  //  yaw.data = relative_yaw;

  //  if (relative_x == 0 && relative_y == 0) {
  //    geometry_msgs::Point relative_target;
  //    relative_target.x = relative_x;
  //    relative_target.y = relative_y;
  //    relative_target.z = relative_z;
  //    geometry_msgs::TransformStamped transform;
  //    transform.transform.rotation =
  //        tf::createQuaternionMsgFromYaw(relative_yaw * M_PI / 180);
  //    tf2::doTransform(relative_target, target, transform);
  //  }
  //}

  void joyCb(const sensor_msgs::Joy::ConstPtr& msg) {
    for (Button b : buttons) {
      b.update(msg->buttons[b.index]);
    }
    for (Cross c : cross) {
      c.update(msg->axes[c.index]);
    }
    // for (Stick s : stick) {
    //  s.update(msg->axes[s.index]);
    //}
  }

  void publish(void) {
    bl_pub.publish(baseline);
    yw_pub.publish(yaw);
    kl_pub.publish(kill);
    st_pub.publish(start);
    tg_pub.publish(target);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "joy_ctrl");
  ros::NodeHandle nh;

  Controller controller(&nh);
  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>(
      "joy", 10, &Controller::joyCb, &controller);

  ros::Rate rate(UPDATE_RATE);

  while (ros::ok()) {
    ros::spinOnce();
    controller.publish();
  }
  ros::shutdown();
  return 0;
}

