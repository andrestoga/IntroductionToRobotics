#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"

#define MIN_RANGE 1
#define FORWARD_SPEED_MPS .2
#define ROTATE_SPEED_RADPS M_PI/18
#define MIN_SCAN_ANGLE_RAD -40.0/180*M_PI
#define MAX_SCAN_ANGLE_RAD +40.0/180*M_PI

ros::Publisher commandPub;

// 0 = forward
// 1 = rotate
int state = 0;
ros::Duration rotateDuration;
ros::Time rotateStartTime;
int closestRange;

void scanMessageReceived(const sensor_msgs::LaserScan msg)
{
	float min = msg.range_min;
	float max = msg.range_max;
	// ROS_INFO_STREAM("Laser configuration");
	// ROS_INFO_STREAM("Min Range " << min << " Max Range " << max);
	// ROS_INFO_STREAM("Ranges");

	//TODO: Limit the range of the lidar sensor.
	//Converting min and max radians to steps in order to iterate over the array.
	unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg.angle_min) / msg.angle_increment);
	unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg.angle_min) / msg.angle_increment);

  	//Setting initial value to closest range.
	closestRange = msg.ranges[minIndex];

	for (int i = minIndex + 1; i < maxIndex; i++ )
	{
  		//Gathering the closest range.
		if (msg.ranges[i] >= min && msg.ranges[i] <= max)
		{
			if (msg.ranges[i] < closestRange)
			{
				closestRange = msg.ranges[i];
			}
		}
	}

	ROS_INFO_STREAM("Closest range: " << closestRange);

	// ROS_INFO_STREAM("Min angle: " << msg.angle_min*(180/M_PI) << "\nMax angle" << msg.angle_max*(180/M_PI));

	// If it is less than the allowed range, rotate to a random direction.
	if(closestRange < MIN_RANGE)
	{
		state = 1;		
	}
	else
	{
		state = 0;
	}

	// If it is less than the allowed range, rotate to a random direction.
	// if(closestRange < MIN_RANGE && state == 0)
	// {
	// 	rotateStartTime = ros::Time::now();
	// 	rotateDuration = ros::Duration(rand() % 5 + 1);

	// 	state = 1;

	// 	//TODO: Calculate the angle to rotate in the opposite direction
	// 	//TODO: Rotate that angle
	// }

	// if (state)
	// {
	// 	// Creating a message of type 'geometry_msgs'.
	// 	geometry_msgs::Twist msg;
	// 	//Assigning the linear and angular velocities.
	// 	msg.linear.x = 0;
	// 	msg.angular.z = ROTATE_SPEED_RADPS;
	// 	//Publishing the message to the topic.
	// 	commandPub.publish(msg);

	// 	ROS_INFO_STREAM("ROTATING");

	// 	//Checking if it has passed the duration of the rotation.
	// 	if((ros::Time::now().sec - rotateStartTime.sec) >= rotateDuration.sec)
	// 	{
	// 		// Creating a message of type 'geometry_msgs'.
	// 		geometry_msgs::Twist msg;
	// 		//Assigning the linear and angular velocities.
	// 		msg.linear.x = FORWARD_SPEED_MPS;
	// 		msg.angular.z = 0;
	// 		//Publishing the message to the topic.
	// 		commandPub.publish(msg);

	// 		state = 0;

	// 	}
	// }
}


void rotate (double angular_speed, double relative_angle, int dir)
{
	geometry_msgs::Twist vel_msg;
	   //set a random linear velocity in the x-axis
	   vel_msg.linear.x =0;
	   vel_msg.linear.y =0;
	   vel_msg.linear.z =0;
	   //set a random angular velocity in the y-axis
	   vel_msg.angular.x = 0;
	   vel_msg.angular.y = 0;

	   if (dir)
	   	vel_msg.angular.z = angular_speed;
	   else
	   	vel_msg.angular.z = -1*angular_speed;

	   // vel_msg.angular.z = -1*angular_speed;

	   double t0 = ros::Time::now().toSec();
	   double current_angle = 0.0;
	   ros::Rate loop_rate(1000);

	   // ROS_INFO_STREAM("Angular speed" << angular_speed << std::endl);

	   do{
		   commandPub.publish(vel_msg);
		   double t1 = ros::Time::now().toSec();
		   current_angle = angular_speed * (t1-t0);
		   ros::spinOnce();
		   loop_rate.sleep();
		   // cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<std::endl;
			// ROS_INFO_STREAM((t1-t0)<<", "<<current_angle <<", "<<relative_angle << ", " << vel_msg.angular.z << std::endl);
	   }while(current_angle<relative_angle);

	   vel_msg.angular.z =0;
	   commandPub.publish(vel_msg);
}

double degrees2radians(double angle_in_degrees)
{
	return angle_in_degrees *M_PI /180.0;
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"random");   
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/scan", 10, &scanMessageReceived);
	commandPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	// ros::spin();

	// Creating a message of type 'geometry_msgs'.
	geometry_msgs::Twist msgFor;
	//Assigning the linear and angular velocities.
	msgFor.linear.x = FORWARD_SPEED_MPS;
	msgFor.angular.z = 0;

	// Creating a message of type 'geometry_msgs'.
	geometry_msgs::Twist msgRot;
	//Assigning the linear and angular velocities.
	msgRot.linear.x = 0;
	msgRot.angular.z = ROTATE_SPEED_RADPS;

	//Set the loop at 10 HZ
	ros::Rate rate(10);

	while (ros::ok())
	{
		if (!state)
		{
			commandPub.publish(msgFor);
			ROS_INFO_STREAM("FORWARD");
		}
		else
		{
			// commandPub.publish(msgRot);
			ROS_INFO_STREAM("ROTATE");

			//Angular speed, relative angle, clockwise
			rotate( degrees2radians(10), degrees2radians(rand() % 180 + 91), rand() % 2);

		}

		// ROS_INFO_STREAM("FORWARD");
		ros::spinOnce();//Call this function to process ROS incoming messages.

		rate.sleep();//Sleep the rest of the cycle until 10 Hz
	}
}