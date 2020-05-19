#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "att_test");
    ros::NodeHandle nh;
    
    ros::Publisher att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
                                ("mavros/setpoint_raw/attitude", 10);

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                    ("mavros/set_mode");
 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                     ("mavros/state", 10, state_cb);
    ros::Rate rate(20.0);

    mavros_msgs::AttitudeTarget att;

    att.header.stamp = ros::Time::now();
    att.orientation.x = 0;
    att.orientation.y = 0;
    att.orientation.z = 0;
    att.orientation.w = 1;
    att.type_mask = 0x07;
    att.thrust = 0.6;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected");
    ros::Time last_time = ros::Time::now();
    ros::spinOnce();
    while(ros::ok())
    {
        
        ros::spinOnce();
        if( current_state.armed &&
            current_state.mode != "OFFBOARD" && 
            ros::Time::now()-last_time > ros::Duration(2.0))
        {

            if( set_mode_client.call(offb_set_mode) && 
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_time = ros::Time::now();
        }
            ROS_INFO(current_state.armed ? "a " : "u ");
        
        att.header.stamp = ros::Time::now();
        att_pub.publish(att);

        rate.sleep();
    }
}
