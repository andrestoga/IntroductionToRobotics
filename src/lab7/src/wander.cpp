#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#define MIN_RANGE 1
#define FORWARD_SPEED_MPS .2
#define ROTATE_SPEED_RADPS M_PI/18
#define MIN_SCAN_ANGLE_RAD -40.0/180*M_PI
#define MAX_SCAN_ANGLE_RAD +40.0/180*M_PI

ros::Publisher commandPub;

// 0 = forward
// 1 = rotate
// 2 = stop
int state = 0;
ros::Duration rotateDuration;
ros::Time rotateStartTime;
int closestRange;

double t1 = 0.0;
double t2 = 0.0;
int flag = 0;

int once = 1;
float volume = 0.0f;

void stop()
{
	geometry_msgs::Twist vel_msg;

	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	commandPub.publish(vel_msg);
}

void poseMessageReceived(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{


	volume = msg->pose.covariance[0] + msg->pose.covariance[7] + msg->pose.covariance[14] + msg->pose.covariance[21] + msg->pose.covariance[28] + msg->pose.covariance[35];

	if (once)
	{
		t1 = ros::Time::now().toSec();
		once = 0;
	}

	t2 = ros::Time::now().toSec();

	//Completly localized
	//If the sum of the elements of the diagonal shrink below to 0.1
	//Completly lost if the sum of the elements of the diagonal are greater than 2.0

	if ((t2 - t1) > 5.0)
	{
		flag = 1;
	}

	ROS_INFO("Covariance value: %f  %f  %d", volume, t2 - t1, flag);

	if (flag)
	{
		if (volume < 0.1)
		{
			state = 2;
			ROS_INFO("Robot is localized!!!");
		}

		if (volume > 2.0)
		{
			ROS_INFO("Robot is lost!!!");
			state = 2;
		}
	}
}

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

	// ROS_INFO_STREAM("Closest range: " << closestRange);

	// ROS_INFO_STREAM("Min angle: " << msg.angle_min*(180/M_PI) << "\nMax angle" << msg.angle_max*(180/M_PI));

	// If it is less than the allowed range, rotate to a random direction.
	if(closestRange < MIN_RANGE)
	{
		if (state != 2)
		{
			state = 1;	
		}
	}
	else
	{
		if (state != 2)
		{
			state = 0;
		}
	}
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

		   if (state == 2)
		   {
		   	break;
		   }

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
	ros::init(argc,argv,"wander");

	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/scan", 10, &scanMessageReceived);
	ros::Subscriber sub_pose = nh.subscribe("/amcl_pose", 10, &poseMessageReceived);
	commandPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

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
		switch(state)
		{
			case 0:

			commandPub.publish(msgFor);

			break;

			case 1:

			rotate( degrees2radians(10), degrees2radians(rand() % 180 + 91), rand() % 2);

			break;

			case 2:

				stop();
				// ROS_INFO("Covariance value: %f  %f  %d", volume, t2 - t1, flag);

			break;

			default:

				ROS_INFO("Error!! the code shouldn't arrive here.");
		}

		if(state == 2)
		{
			break;
		}

		ros::spinOnce();//Call this function to process ROS incoming messages.

		rate.sleep();//Sleep the rest of the cycle until 10 Hz
	}
}