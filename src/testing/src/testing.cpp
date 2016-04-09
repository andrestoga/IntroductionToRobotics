#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testing");

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // ROS_INFO("%s", msg.data.c_str());



    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}