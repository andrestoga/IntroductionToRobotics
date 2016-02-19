#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

#include <tf/tf.h>
#include <geometry_msgs/Transform.h>

tf::Transform transform1;
tf::Transform transform2;

void poseCallback1(const turtlesim::PoseConstPtr& msg)
{
  // ROS_INFO_STREAM("Turtle1" << " :\n\tposition:\n\t\tx: " << msg->x << "\n\t\ty: " << msg->y <<
  // "\n\torientation: " << "\n\t\tYaw: " << msg->theta << "\n\n");

  transform1.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transform1.setRotation(q);
}

void poseCallback2(const turtlesim::PoseConstPtr& msg)
{
  // ROS_INFO_STREAM("Turtle2" << " :\n\tposition:\n\t\tx: " << msg->x << "\n\t\ty: " << msg->y <<
  // "\n\torientation: " << "\n\t\tYaw: " << msg->theta << "\n\n");

  transform2.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transform2.setRotation(q);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "displacement");

  ros::NodeHandle node;
  ros::Subscriber sub1 = node.subscribe( "turtle1/pose", 10, &poseCallback1 );
  ros::Subscriber sub2 = node.subscribe( "turtle2/pose", 10, &poseCallback2 );
  ros::Publisher pub = node.advertise<geometry_msgs::Transform>( "displacement/transform", 1000 );

  //Set the loop at 10 HZ
  ros::Rate rate(10);

  while (ros::ok())
  {
      ros::spinOnce();//Call this function to process ROS incoming messages.

      //Calculate the transformation
      tf::Transform transformTmp;
      // transformTmp = transform1.inverse() * transform2;

      tf::Vector3 t1Origin = transform1.getOrigin();
      tf::Vector3 t2Origin = transform2.getOrigin();

      tf::Quaternion q1 = transform1.getRotation();
      tf::Quaternion q2 = transform2.getRotation();

      transformTmp.setOrigin( tf::Vector3(t2Origin.getX() - t1Origin.getX(), t2Origin.getY() - t1Origin.getY(), 0.0) );
      tf::Quaternion qtemp;
      qtemp.setRPY(0, 0, q2.getAngle() - q1.getAngle());
      transformTmp.setRotation(qtemp);

      //Publish the transformation
      geometry_msgs::Transform tg;
      tf::transformTFToMsg(transformTmp, tg);
      pub.publish(tg);

      rate.sleep();//Sleep the rest of the cycle until 10 Hz
    }

    return 0;
  }