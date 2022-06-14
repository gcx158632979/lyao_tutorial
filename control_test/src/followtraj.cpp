#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include "tf/transform_datatypes.h"//转换函数头文件
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
mavros_msgs::State current_state;
using namespace std;
double truthx,truthy,truthz;
double yaw,pitch,roll;
struct position{
    double x;
    double y;
    double z;
}current_position;

struct ref{
    double posex;
    double posey;
    double posez;
    double velx;
    double vely;
    double velz;
    double accx;
    double accy;
    double accz;
    double yaw;
};

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::Quaternion orientation = msg->pose.orientation;    
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)); 
    mat.getEulerYPR(yaw, pitch, roll);
    truthx=msg->pose.position.x;
    truthy=msg->pose.position.y;
    truthz=msg->pose.position.z;
    current_position.x=truthx;
    current_position.y=truthy;
    current_position.z=truthz;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel",10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher ref_pub = nh.advertise<mavros_msgs::PositionTarget>
        ("/mavros/setpoint_raw/local",10);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,poseCallback);

    std::vector<struct ref> _ref_list;
    mav_trajectory_generation::Vertex::Vector vertices;
    const int dimension = 4;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);
    start.makeStartOrEnd(Eigen::Vector4d(0,0,1.8,0), derivative_to_optimize);
    vertices.push_back(start);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(0.293,0.707,1.8,0));
    vertices.push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(1, 1,1.8,0));
    vertices.push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(1.293, 0.707,1.8,0));
    vertices.push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(2,0,1.8,0));
    vertices.push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(1.293, -0.707,1.8,0));
    vertices.push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(1, -1,1.8,0));
    vertices.push_back(middle);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(0.293, -0.707,1.8,0));
    vertices.push_back(middle);
    end.makeStartOrEnd(Eigen::Vector4d(0,0,1.8,0), derivative_to_optimize);
    vertices.push_back(end);
    std::vector<double> segment_times;
    const double v_max = 1.0;
    const double a_max = 1.0;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.solveLinear();
    mav_trajectory_generation::Segment::Vector segments;
    opt.getSegments(&segments);
    mav_trajectory_generation::Trajectory trajectory;
    opt.getTrajectory(&trajectory);
    mav_msgs::EigenTrajectoryPoint::Vector states;
    double sampling_interval = 0.02;
    bool success=false;
    success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);
    cout<<success<<endl;
    ofstream outfile("/home/ubuntu18/vicon.txt",ios::trunc);
    for(int i = 0;i<states.size();i++)
    {
        struct ref p;
        p.posex = states[i].position_W.x();
        p.posey = states[i].position_W.y();
        p.posez = states[i].position_W.z();
        p.velx = states[i].velocity_W.x();
        p.vely = states[i].velocity_W.y();
        p.velz = states[i].velocity_W.z();
        p.accx = states[i].acceleration_W.x();;
        p.accy = states[i].acceleration_W.y();;
        p.accz = states[i].acceleration_W.z();;
        p.yaw = 0;
        outfile.setf(ios::fixed);
        outfile<<p.posex<<" "<<p.posey<<" "<<p.posez<<" "<<p.velx<<" "<<p.vely<<" "<<p.velz<<" "<<p.accx<<" "<<p.accy<<" "<<p.accz<<" "<<p.yaw<<"\n";
        _ref_list.push_back(p);   
    }

    cout<<"list size is "<<_ref_list.size()<<std::endl;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);
 
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.8;
    /*pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = -0.7;
    pose.pose.orientation.w = -0.7;*/
    /*for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }*/

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
 
    ros::Time last_request = ros::Time::now();
    int _points_id=0;
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
        if(abs(current_position.z-1.8)<=0.2)
            start_flag=true;
        if(start_flag)
        {
            if (_points_id < (int)_ref_list.size()-1)
            {
                mavros_msgs::PositionTarget ref_target;
                ref_target.coordinate_frame = ref_target.FRAME_LOCAL_NED;
                ref_target.type_mask = 2048;
                ref_target.position.x = _ref_list[_points_id].posex;
                ref_target.position.y = _ref_list[_points_id].posey;
                ref_target.position.z = _ref_list[_points_id].posez;
                ref_target.velocity.x = _ref_list[_points_id].velx;
                ref_target.velocity.y = _ref_list[_points_id].vely;
                ref_target.velocity.z = _ref_list[_points_id].velz;
                ref_target.acceleration_or_force.x = _ref_list[_points_id].accx;
                ref_target.acceleration_or_force.y = _ref_list[_points_id].accy;
                ref_target.acceleration_or_force.z = _ref_list[_points_id].accz;
                ref_target.yaw = _ref_list[_points_id].yaw;
                ref_pub.publish(ref_target);
                std::cout<<_ref_list[_points_id].posex<<" "<<truthx<<endl;
                std::cout<<_ref_list[_points_id].posey<<" "<<truthy<<endl;
                std::cout<<_ref_list[_points_id].posez<<" "<<truthz<<endl;
                //std::cout<<"aaaaa"<<endl;
                std::cout<<"points_id = "<<_points_id<<std::endl;
             }
            else
	    {
		    pose.pose.position.x = 0;
		    pose.pose.position.y = 0;
		    pose.pose.position.z = -10;
		    local_pos_pub.publish(pose);
            }
            _points_id++;  
        }
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}
