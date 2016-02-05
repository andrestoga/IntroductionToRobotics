#include <ros/ros.h>
#include <turtlesim/Color.h>

void printcolor(const turtlesim::Color& msg)
{
	// ROS_INFO_STREAM("R: %d\n G: %d\n B: %d\n", msg.r, msg.g, msg.b);

	unsigned int r = msg.r;
	unsigned int g = msg.g;
	unsigned int b = msg.b;

	ROS_INFO_STREAM("R: " << r << "\n" << "G: " << g << "\n" << "B: " << b);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "printcolor");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("turtle1/color_sensor", 1000, &printcolor);

	ros::spin();
}