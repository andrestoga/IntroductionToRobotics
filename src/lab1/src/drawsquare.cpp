#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <std_srvs/Empty.h>

#define MOVING_FORWARD 3.0
#define TURN_90 3.0

int state = 0;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "drawsquare");
	ros::NodeHandle nh;
	ros::Time rotateStartTime;//Start time of the rotation.
	ros::Duration rotateDuration;//Duration of the rotation.
	ros::ServiceClient reset = nh.serviceClient<std_srvs::Empty>("reset");

	std_srvs::Empty empty;
  	reset.call(empty);

	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

	ros::Rate rate(2);

	rotateStartTime = ros::Time::now();
	rotateDuration = ros::Duration(MOVING_FORWARD);

	while(ros::ok())
	{
		geometry_msgs::Twist msg;

		if (state == 0)
		{
			msg.angular.z = 0;
			msg.linear.x = .7;

			// Checking if it has passed the duration of moving forward.
			if((ros::Time::now().sec - rotateStartTime.sec) >= rotateDuration.sec)
			{
				state = 1;
				rotateStartTime = ros::Time::now();
				rotateDuration = ros::Duration(TURN_90);
			}
		}
		else
		{
			msg.angular.z = .5;
			msg.linear.x = 0;

			// Checking if it has passed the duration of the rotation.
			if((ros::Time::now().sec - rotateStartTime.sec) >= rotateDuration.sec)
			{
				state = 0;
				rotateStartTime = ros::Time::now();
				rotateDuration = ros::Duration(MOVING_FORWARD);
			}
		}

		pub.publish(msg);

		rate.sleep();
	}
}