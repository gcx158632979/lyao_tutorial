#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include "tf/transform_datatypes.h"
mavros_msgs::State current_state;
double truthx,truthy,truthz;
double yaw,pitch,roll;
ros::Publisher vel_pub;
ros::Publisher height_pub;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::Quaternion orientation = msg->pose.orientation;    
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)); 
    mat.getEulerYPR(yaw, pitch, roll);
    truthx=msg->pose.position.x;
    truthy=msg->pose.position.y;
    truthz=msg->pose.position.z;//height
}
void height_control(double desired_height)//PD control
{
    static double last_error = 0;
    double kp = 2.0, kd = 4;
    double error = desired_height - truthz;
    double output = kp * error + kd * (error - last_error);
    last_error = error;
    geometry_msgs::TwistStamped velcmd;
    velcmd.twist.linear.z = output;
    vel_pub.publish(velcmd);
    std_msgs::Float64 height;
    height.data = truthz;
    height_pub.publish(height);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,poseCallback);
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel",10);
    height_pub = nh.advertise<std_msgs::Float64>("/height",10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
	
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    bool start_flag=false;
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        if(!start_flag)
            local_pos_pub.publish(pose);
        if(abs(truthz-2)<=0.1)
            start_flag=true;
        if(start_flag)
        {
	    height_control(4);	
	}	
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
