#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>
#include <visualization_msgs/Marker.h> //marker
#include <std_msgs/String.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Twist.h>

// #include "rins_task_1/msg_face.h"

#include <sound_play/sound_play.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cstdlib>
// #include "waypoint.hpp"

#include "json.hpp"
// #include "waypoint.hpp"
#include <fstream>
using json = nlohmann::json;

std::vector<std::vector<float>> waypoints;

using namespace std;
using namespace cv;


int green_x = 0;
int green_y = 0;
bool green_found = false;



bool flag_arrived=false;
// int num_of_faces=0;
// bool new_face=false;

void cb_done(const actionlib::SimpleClientGoalState& state,
             const move_base_msgs::MoveBaseResultConstPtr& result)
{
    ROS_INFO("finished in state [%s]", state.toString().c_str());
    flag_arrived=!strcmp(state.toString().c_str(),"SUCCEEDED");    
}

void markerCallback(const visualization_msgs::Marker::ConstPtr& msg)
{

    green_x = msg->pose.position.x;
    green_y = msg->pose.position.y;
    green_found = true;
    // if (msg->markers.size() > num_of_faces) {
    //     new_face = true;
    //     num_of_faces = msg->markers.size();
    //     std::string cmd = "aplay /home/nejc/faks/RINS/ros/src/exercise6/halo.wav";
    //     std::system(cmd.c_str());
    // }
}

void cb_active()
{
    ROS_INFO("goal just went active");
}
 
void cb_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
    // ROS_INFO("[X]:%f [Y]:%f [W]: %f",feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w); 
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;

int main(int argc, char** argv) {

    ros::init(argc, argv, "main");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(2);
    spinner.start();


    move_base_client ac("move_base", true);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
    ros::Publisher parking_pub = n.advertise<std_msgs::String>("/park_command", 1000);
    ros::Publisher arm_pub = n.advertise<std_msgs::String>("arm_topic", 10);
    
    ros::Subscriber sub_green_point = n.subscribe("green_ring_point", 1000, &markerCallback);
    
    while(!ac.waitForServer(ros::Duration(20.0))){
       ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    // std::vector<waypoint> waypoints;
    // waypoints.push_back(waypoint(-0.85,1.5));
    // waypoints.push_back(waypoint(-0.85,0.1));
    // waypoints.push_back(waypoint(0.1,1.2));
    // waypoints.push_back(waypoint(-0.15,-0.3));
    // waypoints.push_back(waypoint(-0.85,0.1));

    std::ifstream i("/home/gal/ROS/src/exercise6/waypoints.json");
    json j;
    i >> j;    
    waypoints=j["waypoints"].get<std::vector<std::vector<float>>>();
	ros::Rate rate(0.4);

    for(size_t i=0;i<waypoints.size();i++) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = waypoints[i][0];
        goal.target_pose.pose.position.y = waypoints[i][1];
        goal.target_pose.pose.orientation.w=1;
        
        ROS_INFO("sending destination");
        ac.sendGoal(goal, &cb_done, &cb_active, &cb_feedback);
        ac.waitForResult();

        if (green_found) {
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = green_x;
            goal.target_pose.pose.position.y = green_y;
            goal.target_pose.pose.orientation.w=1;
            
            ROS_INFO("sending destination");
            ac.sendGoal(goal, &cb_done, &cb_active, &cb_feedback);
            ac.waitForResult();
            

            std_msgs::String msg_park;
            msg_park.data = "parking_on";
            std_msgs::String msg_arm;
            msg_park.data = "extend";


            parking_pub.publish(msg_park);
            arm_pub.publish(msg_arm);
            break;

        }

    
        if(flag_arrived) {
            int n_steps=10;

            

            for(int i=0;i<n_steps;i++) {
               geometry_msgs::Twist msg;
               msg.linear.x = 0;
                msg.angular.z = (4.0*3.14)/(double)n_steps;
                pub.publish(msg);
                rate.sleep();
            }
        }

        



        flag_arrived=false;
        ros::Duration(1).sleep();


        // if (num_of_faces == 3) {
        //     break;
        // }
    }

    ROS_INFO("FINISHED");

    return 0;

}
