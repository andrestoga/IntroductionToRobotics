#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <std_srvs/Empty.h>

ros::Publisher velocity_publisher;

//Method to move the robot straight
void move(double speed, double distance, bool isForward);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_cleaner");
	ros::NodeHandle nh;
	ros::ServiceClient reset = nh.serviceClient<std_srvs::Empty>("reset");

	std_srvs::Empty empty;
  	reset.call(empty);

	velocity_publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

	move(2.0, 1.0, 1);
}

void move(double speed, double distance, bool isForward)
{
	geometry_msgs::Twist vel_msg;

	if(isForward)
	{
		vel_msg.linear.x = abs(speed);
	}
	else
	{
		vel_msg.linear.x = -abs(speed);
	}

	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	// double t = distance / speed;

	//Get current time
	ros::WallTime t0 = ros::WallTime::now();

	//Publish the velocity on the /cmd_vel topic during certain time
	// while( (ros::WallTime::now().sec - start.sec) <= t )
	// {
	// 	velocity_publisher.publish(vel_msg);
	// }

	while(true)
	{
		velocity_publisher.publish(vel_msg);
		double distance = speed * (2);
	}
}