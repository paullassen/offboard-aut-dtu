#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/Thrust.h>
mavros_msgs::State current_state;
std_msgs::Float32 thrust;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void thrust_cb(const std_msgs::Float32::ConstPtr& msg){
    thrust = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Subscriber thrust_sub = nh.subscribe<std_msgs::Float32>
                                    ("test/baseline", 10, thrust_cb); 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                    ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                    ("mavros/setpoint_attitude/attitude", 10);
    ros::Publisher thrust_pub = nh.advertise<mavros_msgs::Thrust>
                                    ("mavros/setpoint_attitude/thrust", 10);
    ros::Publisher att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
                                    ("mavros/setpoint_raw/attitude",10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                    ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                    ("mavros/set_mode");

    ros::Rate rate(20.0);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    
    mavros_msgs::AttitudeTarget att;
    att.orientation.x = 0;
    att.orientation.y = 0;
    att.orientation.z = 0;
    att.orientation.w = 1;
    att.type_mask = 0x07;
    att.thrust = 0.6;
    att.header.stamp = ros::Time::now();
    /*
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    
    mavros_msgs::Thrust th;
    th.thrust = 0;
    */
    for(int i = 100; ros::ok() && i > 0; --i){
        //local_pos_pub.publish(pose);
        //thrust_pub.publish(th);
        att_pub.publish(att);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        /*
        if( current_state.mode != "OFFBOARD" && 
            (ros::Time::now() - last_request > ros::Duration(2.0)))
        {
            if( set_mode_client.call(offb_set_mode) && 
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
        last_request = ros::Time::now();
        }else{
            ROS_INFO(current_state.armed ? "armed" : "not armed");
            if(!current_state.armed)// &&
               // (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                ROS_INFO("Arming");
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
            last_request = ros::Time::now();
            }
        }
        */
        //th.thrust = thrust.data;
        //local_pos_pub.publish(pose);
        //thrust_pub.publish(th);
        att.header.stamp = ros::Time::now();
        att.thrust = thrust.data;
        att_pub.publish(att);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
