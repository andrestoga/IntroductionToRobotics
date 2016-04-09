
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/LaserScan.h"
#include "SDL/SDL.h"
#include <sys/time.h>
#include <fstream>
#include <pthread.h>			//Multi threading concerning WIFI scanning
#include <unistd.h>

//#define PATH "/home/connor/logs/"
#define DTOR(d) ((d) * M_PI / 180)

using namespace std;


#include <termios.h>
#include <stdio.h>

static struct termios old, new_s;

/* Initialize new terminal i/o settings */

ros::Publisher gazebo_pub;

void poseCallback(gazebo_msgs::ModelStates msg)
{
	std::string robot_name = "pioneer3at";
	float x;
	float y;
	float offset_x = 10.58;
	float offset_y = 10.0;

	for (int i = 0 ; i < msg.name.size(); ++i)
		if (robot_name == msg.name[i])
		{
			msg.pose[i].position.x =  msg.pose[i].position.x + offset_x;
			msg.pose[i].position.y =  msg.pose[i].position.y + offset_y;
			gazebo_pub.publish(msg.pose[i]);
			std::cout<<"Gazebo Pose (" << msg.pose[i].position.x << ", " << msg.pose[i].position.y <<")\n";
		}
		else
			std::cout<<"Discarding " << msg.name[i] << endl;
}

int main(int argc, char** argv)
{
	std::cout<<"Program Started\n";
	struct timeval tv;


	double newspeed = 0, newturnrate = 0;
	//ramping robot motor speeds
	double u[2];
	double ulow[] = {0.15, 0.1};
	double uhigh[] = {0.4, 0.25};
	u[0]=ulow[0];
	u[1]=ulow[1];

  	//init the ROS node
	ros::init(argc, argv, "pose_converter");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 1000, poseCallback);
	gazebo_pub = nh.advertise<geometry_msgs::Pose>("/gazebo_pose", 1000);
	
	ros::Rate rate(10);
	ros::spin();
    return 0;
}
