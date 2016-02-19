#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

#include <tf/tf.h>
#include <geometry_msgs/Transform.h>

std::string turtle_name;
ros::Publisher pub;

// void poseCallback(const turtlesim::PoseConstPtr& msg){
//   static tf::TransformBroadcaster br;
//   tf::Transform transform;
//   transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
//   tf::Quaternion q;
//   q.setRPY(0, 0, msg->theta);
//   transform.setRotation(q);
//   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
// }

void poseCallback(const turtlesim::PoseConstPtr& msg)
{
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);

  //Transform pose of turtle 2 with respect to turtle1

  //Set the transform to a Transform message
  geometry_msgs::Transform tran_msg;

  pub.publish(tran_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_broadcaster");

  if (argc != 2)
  {
    ROS_ERROR("need turtle name as argument");
    return -1;
  }

  turtle_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe( turtle_name + "/pose", 10, &poseCallback );

  pub = node.advertise<geometry_msgs::Transform>( turtle_name + "/wrt1", 1000 );

  ros::spin();

  return 0;
}