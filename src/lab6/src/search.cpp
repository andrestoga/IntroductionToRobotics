#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <tf/tf.h>

#include <complex>

#define MIN_RANGE 0.5
#define FORWARD_SPEED_MPS .2
#define ROTATE_SPEED_RADPS M_PI/18
#define ROTATE_SPEED_ANGPS 10
#define MIN_SCAN_ANGLE_RAD -40.0/180*M_PI
#define MAX_SCAN_ANGLE_RAD +40.0/180*M_PI
#define MOVE_LITTLE 2
#define GOAL 3
#define TURN_ITERATIONS 8
#define ESCAPE 5

ros::Publisher commandPub;

// States
//Rotate_Goal = 0
//Forward_Goal = 1
//Evade_Obstacle = 2
int state = 0;
ros::Duration rotateDuration;
ros::Time rotateStartTime;
float closestRange = 0.0f;
float theta = 0.0f;
double robotAngle = 0.0;
int obstacle = 0;
int goal = 0;
geometry_msgs::Vector3 position;

int Tries = 0;

//0 move to the left
//1 move up
//2 move to the right
int obstacleState = 0;
int alignToGoal = 1;

int count = TURN_ITERATIONS;


geometry_msgs::Vector3 east;
geometry_msgs::Vector3 west;
geometry_msgs::Vector3 north;
geometry_msgs::Vector3 south;

int moveUp = MOVE_LITTLE;

void stop();
void scanMessageReceived(const sensor_msgs::LaserScan msg);
float Calculate_Rotation_Angle(geometry_msgs::Vector3 unitVectorRobot, geometry_msgs::Vector3 unitVectorDirection);
void poseCallback(const geometry_msgs::Pose& pose);
void move(double speed, double distance, int isForward);
double getDistance(double x1, double y1, double x2, double y2);
void rotate(double angular_speed, double relative_angle, int dir);
double degrees2radians(double angle_in_degrees);
double radians2degrees(double angle_in_radians);
geometry_msgs::Vector3 convertToUnitVector(geometry_msgs::Vector3 vec);
int MoveDirection(int direction, geometry_msgs::Vector3 unitVectorRobot, int distance);

int main(int argc,char **argv)
{
	ros::init(argc,argv,"search");   
	ros::NodeHandle nh;
	ros::Subscriber sub_pose = nh.subscribe("/gazebo_pose", 10, &poseCallback);
	ros::Subscriber sub = nh.subscribe("/scan", 10, &scanMessageReceived);
	commandPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	// ros::spin();
	int direction = 0;

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

	int dir = 1;

	east.x = 1;
	east.y = 0;
	west.x = -1;
	west.y = 0;
	north.x = 0;
	north.y = 1;
	south.x = 0;
	south.y = -1;

	do
	{
		ros::spinOnce();

	}while(robotAngle == 0.0f);

	// rotate( degrees2radians(ROTATE_SPEED_ANGPS), degrees2radians(90), 1);

	// ROS_INFO("Angle of rotation: %f", radians2degrees(theta));

	//Set the loop at 10 HZ
	ros::Rate rate(10);

	// while(count--)
	// {
	// 	ROS_INFO("Angle of rotation: %f", radians2degrees(theta));

	// 	if (theta < 0)
	// 	{
	// 		dir = 0;
	// 		theta *= -1;
	// 	}
	// 	else
	// 	{
	// 		dir = 1;
	// 	}

	// 	rotate( degrees2radians(ROTATE_SPEED_ANGPS), theta, dir);
	// 	ros::spinOnce();
	// 	rate.sleep();
	// }

	// state = 2;

	geometry_msgs::Vector3 unitVectorRobot;

	while (ros::ok())
	{
		unitVectorRobot.x = cos(robotAngle);
		unitVectorRobot.y = sin(robotAngle);

		ROS_INFO("State: %d\nObstacle state: %d", state, obstacleState);

		//Rotate_Goal
		if (0 == state)
		{
			if(count--)
			{
				//Create a vector for the goal
				geometry_msgs::Vector3 goalVec;
				goalVec.x = GOAL - position.x;
				goalVec.y = GOAL - position.y;

				// goalVec = convertToUnitVector(goalVec);

				//Convert the vector goal to unit
				double norm = sqrt(pow(goalVec.x, 2) + pow(goalVec.y, 2));
				goalVec.x /= norm;
				goalVec.y /= norm;

				theta = Calculate_Rotation_Angle(unitVectorRobot, goalVec); 

				// ROS_INFO("Angle of rotation: %f", radians2degrees(theta));

				if (theta < 0)
				{
					dir = 0;
					theta *= -1;
				}
				else
				{
					dir = 1;
				}

				rotate( degrees2radians(ROTATE_SPEED_ANGPS), theta, dir);
			}
			else
			{
				count = TURN_ITERATIONS;
				state = 1;
				// break;
			}

	// 		// rotate( degrees2radians(ROTATE_SPEED_ANGPS), degrees2radians(theta), 1);

		}//Forward_Goal
		else if(1 == state)
		{
			// commandPub.publish(msgFor);

			double distance = getDistance(GOAL, GOAL, position.x, position.y);

			// ROS_INFO("Distance to move to the goal: %f\n", distance);

			move(FORWARD_SPEED_MPS, distance, 1);

			if (obstacle && alignToGoal != 0)
			{
				state = 2;
				ROS_INFO("Obstacle");
				// break;
			}
			else
			{
				stop();

				if ( alignToGoal )
				{
					state = 0;
					alignToGoal = 0;
					ROS_INFO("Aligning to the goal\nX: %f\nY: %f", position.x, position.y);
				}
				else
				{
						//Reached to the goal!!
					ROS_INFO("Goal reached!!\nFinal position:\nX: %f\nY: %f", position.x, position.y);
					break;
				}
			}

		}//Evade_Obstacle
		else if(2 == state)
		{
			// rotate( degrees2radians(ROTATE_SPEED_ANGPS), degrees2radians(90), 0);

			// move(FORWARD_SPEED_MPS, MOVE_LITTLE, 1);

			// while(obstacle)
			// {
			// 	direction ^= 1;

			// 	rotate( degrees2radians(ROTATE_SPEED_ANGPS), degrees2radians(180), direction);

			// 	move(FORWARD_SPEED_MPS, MOVE_LITTLE, 1);
			// }

			// if(MoveDirection(2, unitVectorRobot, MOVE_LITTLE) == 0)
			// {
			// 	break;
			// }


			if (0 == obstacleState)//Move to the left
			{
				state = MoveDirection(0, unitVectorRobot, MOVE_LITTLE);

				if (obstacle && state == 0)
				{
					obstacleState = 1;
					// state = 0;
				}
			}
			else if(1 == obstacleState)//Move up
			{
				state = MoveDirection(1, unitVectorRobot, moveUp);

				if (obstacle && state == 0)
				{
					obstacleState = 2;
					moveUp = MOVE_LITTLE;
					// state = 0;
				}
				else
				{
					moveUp += MOVE_LITTLE;
				}
			}
			else if(2 == obstacleState)//Move to the right
			{
				if (Tries < ESCAPE)
				{
					state = MoveDirection(2, unitVectorRobot, MOVE_LITTLE);

					if (obstacle && state == 0)
					{
						obstacleState = 0;
						Tries = 0;
					}

					if (state == 0)
					{
						Tries++;
					}
				}
				else
				{
					state = MoveDirection(3, unitVectorRobot, MOVE_LITTLE);
					ROS_INFO("Trying to moving down to escape");

					if (state == 0)
					{
						Tries = 0;
					}
				}
			}
		}

		// ROS_INFO_STREAM("FORWARD");
		ros::spinOnce();//Call this function to process ROS incoming messages.

		rate.sleep();//Sleep the rest of the cycle until 10 Hz
	}
}

int MoveDirection(int direction, geometry_msgs::Vector3 unitVectorRobot, int distance)
{
	double angleToRotate = 0.0;
	int dir = 1;

	switch(direction)
	{
		//Left
		case 0:

		angleToRotate = Calculate_Rotation_Angle(unitVectorRobot, west);

		break;

		//Up
		case 1:

		angleToRotate = Calculate_Rotation_Angle(unitVectorRobot, north);

		break;

		//Right
		case 2:

		angleToRotate = Calculate_Rotation_Angle(unitVectorRobot, east);

		break;

		//Down
		case 3:

		angleToRotate = Calculate_Rotation_Angle(unitVectorRobot, south);

		break;

		default:

		ROS_INFO("Error: Code shouldn't arrive here");

		break;
	}

	if (angleToRotate < 0)
	{
		dir = 0;
		angleToRotate *= -1;
	}
	else
	{
		dir = 1;
	}

	rotate( degrees2radians(ROTATE_SPEED_ANGPS), angleToRotate, dir);
	count--;

	if(0 == count)
	{
		count = TURN_ITERATIONS;
		move(FORWARD_SPEED_MPS, distance, 1);

		return 0;
	}
	else
	{
		return 2;
	}
}

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

void scanMessageReceived(const sensor_msgs::LaserScan msg)
{
	float min = msg.range_min;
	float max = msg.range_max;
	// ROS_INFO_STREAM("Laser configuration");
	// ROS_INFO_STREAM("Min Range " << min << " Max Range " << max);
	// ROS_INFO_STREAM("Ranges");

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

	// ROS_INFO_STREAM("Min angle: " << msg.angle_min*(180/M_PI) << "\nMax angle" << msg.angle_max*(180/M_PI));

	// If it is less than the allowed range, rotate to a random direction.
	if(closestRange < MIN_RANGE)
	{
		obstacle = 1;		
	}
	else
	{
		obstacle = 0;
	}

	// ROS_INFO_STREAM("Closest range: " << closestRange << "\nObstacle: " << obstacle);
}

geometry_msgs::Vector3 convertToUnitVector(geometry_msgs::Vector3 vec)
{
	double norm = sqrt(pow(vec.x, 2) + pow(vec.y, 2));
	vec.x /= norm;
	vec.y /= norm;

	return vec;
}

float Calculate_Rotation_Angle(geometry_msgs::Vector3 unitVectorRobot, geometry_msgs::Vector3 unitVectorDirection)
{
	float angle = 0.0;
	float dot = 0.0;
	float det = 0.0;

	// ROS_INFO("Angle of rotation: %f", robotAngle);
	// ROS_INFO("X = %f Y = %f", robotAngleVec.x, robotAngleVec.y);
	// ROS_INFO("X = %f Y = %f", goalVec.x, goalVec.y);

	//Calculate and return the angle between the two vectors
	dot = unitVectorRobot.x * unitVectorDirection.x + unitVectorRobot.y * unitVectorDirection.y;//dot product
	det = unitVectorRobot.x * unitVectorDirection.y - unitVectorRobot.y * unitVectorDirection.x;//determinant
	angle = atan2(det, dot);//atan2(y, x) or atan2(sin, cos)

	return angle;
}

void poseCallback(const geometry_msgs::Pose& pose)
{
	position.x = pose.position.x;
	position.y = pose.position.y;
	position.z = pose.position.z;

	tf::Quaternion q;
	tf::quaternionMsgToTF(pose.orientation, q);

    // the tf::Quaternion has a method to acess roll pitch and yaw
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
	geometry_msgs::Vector3 rpy;
	rpy.x = roll;
	rpy.y = pitch;
	rpy.z = yaw;

    // this Vector is then published:
    // ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f\nPosition: x= %f y= %f z= %f", rpy.x, rpy.y, rpy.z, position.x, position.y, position.z);

	// theta = yaw;

  // ROS_INFO_STREAM("Angle of the Robot: " << radians2degrees(rpy.z));

	if (rpy.z < 0)
	{
		// rpy.z += 2 * M_PI;
		robotAngle = 2 * M_PI + rpy.z;
	}
	else
	{
		robotAngle = rpy.z;
	}

	// ROS_INFO("Robot angle: %f", robotAngle);

    //Create a vector for the robot angle
	// geometry_msgs::Vector3 robotAngleVec;
	// robotAngleVec.x = cos(robotAngle);
	// robotAngleVec.y = sin(robotAngle);

	// //Create a vector for the goal
	// geometry_msgs::Vector3 goalVec;
	// goalVec.x = GOAL - position.x;
	// goalVec.y = GOAL - position.y;

	// // goalVec = convertToUnitVector(goalVec);

	// //Convert the vector goal to unit
	// double norm = sqrt(pow(goalVec.x, 2) + pow(goalVec.y, 2));
	// goalVec.x /= norm;
	// goalVec.y /= norm;

	// theta = Calculate_Rotation_Angle(robotAngleVec, goalVec);    

	// theta = Calculate_Rotation_Angle(rpy.z, position);
	// ROS_INFO("Angle of rotation: %f", theta);
}

void move(double speed, double distance, int isForward)
{
	geometry_msgs::Twist vel_msg;

   //set a random linear velocity in the x-axis
	if (isForward)
		vel_msg.linear.x = std::abs(speed);
	else
		vel_msg.linear.x = -std::abs(speed);

	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
   //set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);

	do{
		commandPub.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();

		// ROS_INFO("Current position: x = %f, y = %f\n", position.x, position.y);
		// ROS_INFO("Distance to the goal: %f\n", getDistance(GOAL, GOAL, position.x, position.y));

		if (obstacle)
		{
			stop();
			return;
		}

		loop_rate.sleep();
	   //cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	}while(current_distance<distance);

	vel_msg.linear.x =0;
	commandPub.publish(vel_msg);
}

double getDistance(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

void rotate(double angular_speed, double relative_angle, int dir)
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
	{
   		// vel_msg.angular.z = angular_speed;
		vel_msg.angular.z = std::abs(angular_speed);
	}
	else
	{
   		// vel_msg.angular.z = -1*angular_speed;
		vel_msg.angular.z = -std::abs(angular_speed);
	}

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
		// ROS_INFO_STREAM((t1-t0)<<", "<<current_angle <<", "<<relative_angle << ", " << vel_msg.angular.z <<", "<< angular_speed << std::endl);
	}while(current_angle<relative_angle);

	vel_msg.angular.z =0;
	commandPub.publish(vel_msg);
}

double degrees2radians(double angle_in_degrees)
{
	return angle_in_degrees * M_PI /180.0;
}

double radians2degrees(double angle_in_radians)
{
	return angle_in_radians * 180.0/M_PI;
}

// int main(int argc,char **argv)
// {
// 	ros::init(argc,argv,"search");   
// 	ros::NodeHandle nh;
// 	ros::Subscriber sub = nh.subscribe("/scan", 10, &scanMessageReceived);
// 	commandPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
// 	// ros::spin();
// 	int direction = 0;

// 	// Creating a message of type 'geometry_msgs'.
// 	geometry_msgs::Twist msgFor;
// 	//Assigning the linear and angular velocities.
// 	msgFor.linear.x = FORWARD_SPEED_MPS;
// 	msgFor.angular.z = 0;

// 	// Creating a message of type 'geometry_msgs'.
// 	geometry_msgs::Twist msgRot;
// 	//Assigning the linear and angular velocities.
// 	msgRot.linear.x = 0;
// 	msgRot.angular.z = ROTATE_SPEED_RADPS;

// 	move(FORWARD_SPEED_MPS, 10, 1);
// 	rotate( degrees2radians(ROTATE_SPEED_ANGPS), degrees2radians(90), 0);

// 	// //Set the loop at 10 HZ
// 	// ros::Rate rate(10);

// 	// while (ros::ok())
// 	// {



// 	// 	//Rotate_Goal
// 	// 	// if (!state)
// 	// 	// {
// 	// 	// 	rotate( degrees2radians(ROTATE_SPEED_ANGPS), degrees2radians(theta), 0);

// 	// 	// 	state = 1;

// 	// 	// }//Forward_Goal
// 	// 	// else if (state)
// 	// 	// {
// 	// 	// 	commandPub.publish(msgFor);

// 	// 	// 	//move(FORWARD_SPEED_MPS, getDistance(1, 1, double x2, double y2), 1);//TODO: 

// 	// 	// 	if (obstacle)
// 	// 	// 	{
// 	// 	// 		state = 2;
// 	// 	// 	}
// 	// 	// 	else
// 	// 	// 	{
// 	// 	// 		stop();
// 	// 	// 		//Reached to the goal!!
// 	// 	// 		return 0;
// 	// 	// 	}

// 	// 	// }//Evade_Obstacle
// 	// 	// else
// 	// 	// {
// 	// 	// 	rotate( degrees2radians(ROTATE_SPEED_ANGPS), degrees2radians(90), 0);

// 	// 	// 	move(FORWARD_SPEED_MPS, MOVE_LITTLE, 1);

// 	// 	// 	while(obstacle)
// 	// 	// 	{
// 	// 	// 		direction = direction ^ 1;

// 	// 	// 		rotate( degrees2radians(ROTATE_SPEED_ANGPS), degrees2radians(180), direction);

// 	// 	// 		move(FORWARD_SPEED_MPS, MOVE_LITTLE, 1);
// 	// 	// 	}

// 	// 	// 	state = 1;
// 	// 	// }

// 	// 	// ROS_INFO_STREAM("FORWARD");
// 	// 	ros::spinOnce();//Call this function to process ROS incoming messages.

// 	// 	rate.sleep();//Sleep the rest of the cycle until 10 Hz
// 	// }
// }