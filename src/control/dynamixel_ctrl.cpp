#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <ros/ros.h>
#include <string>

#define MANIPULATOR_MINIMUM 300
#define MANIPULATOR_MAXIMUM 740

bool command_manipulator(ros::ServiceClient manip_client, int value) {
  dynamixel_workbench_msgs::DynamixelCommand srv;
  srv.request.command = "";
  srv.request.id = 4;
  srv.request.addr_name = "Goal_Position";
  srv.request.value = value;

  return manip_client.call(srv);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "manip_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_node_handle("~");
  ros::ServiceClient manip_client;

    manip_client =
        nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>(
            "dynamixel_workbench/dynamixel_command");
  int pos, val;
  if (private_node_handle.getParam("position", pos)) {
	val = pos == 0 ? MANIPULATOR_MINIMUM : MANIPULATOR_MAXIMUM;
	return command_manipulator(manip_client, val);
	}

return 0;


}
