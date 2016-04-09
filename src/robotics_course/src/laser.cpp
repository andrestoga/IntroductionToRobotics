#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define MIN_RANGE 2.0
#define FORWARD_SPEED_MPS 1.0
#define ROTATE_SPEED_RADPS M_PI/2

void scanMessageReceived(const sensor_msgs::LaserScan msg)
{
	float min = msg.range_min;
	float max = msg.range_max;
	ROS_INFO_STREAM("Laser configuration");
	ROS_INFO_STREAM("Min Range " << min << " Max Range " << max);
	ROS_INFO_STREAM("Ranges");

	//TODO: Limit the range of the lidar sensor.

  	//Setting initial value to closest range.
	float closestRange = msg.ranges[0];

	for (int i = 0 ; i < msg.ranges.size() ; i++ )
	{
  		//Gathering the closest range.
		if (msg->ranges[i] < closestRange)
		{
			closestRange = msg->ranges[i];
		}
	}

  	//If it is less than the allowed range, rotate to a random direction.
	if(closestRange < MIN_RANGE)
	{
		//TODO: Calculate the angle to rotate in the opposite direction


		//TODO: Rotate that angle
		

		// Creating a message of type 'geometry_msgs'.
		geometry_msgs::Twist msg;
		//Assigning the linear and angular velocities.
		msg.linear.x = 0;
		msg.angular.z = ROTATE_SPEED_RADPS;
		//Publishing the message to the topic.
		commandPub.publish(msg);
	}
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"laser_subscriber");   
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/scan", 1000, &scanMessageReceived);
	ros::Publisher commandPub = nh.advertise<geometry_msgs::Twist>("RobotX/cmd_vel", 1);
	// ros::spin();

	//Set the loop at 10 HZ
	ros::Rate rate(10);

	while (ros::ok())
	{
		// Creating a message of type 'geometry_msgs'.
		geometry_msgs::Twist msg;
		//Assigning the linear and angular velocities.
		msg.linear.x = FORWARD_SPEED_MPS;
		msg.angular.z = 0;
		//Publishing the message to the topic.
		commandPub.publish(msg);
		

		ros::spinOnce();//Call this function to process ROS incoming messages.
		rate.sleep();//Sleep the rest of the cycle until 10 Hz

	}
}