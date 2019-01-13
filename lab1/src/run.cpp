/* Reference -  Code snippets could match the ROS tutorials, as they are modified in order to fit the requirement of the assignment.
   Source file for controlling the evader Robot.
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include <time.h>

sensor_msgs::LaserScan laser ;
int mid,flag=0;


void base_scan_cb(const sensor_msgs::LaserScan::ConstPtr& las)
{
   flag=0;
   laser =*las;
   int length=laser.ranges.size();
   mid = length/2;
   for(int i=0;i<length;i+=19) //Checks the laser.ranges over the entire span of 180 degrees, with an interval of 20 units.
   {
    if(laser.ranges[i]<3)
    {
      flag=1;
      break;
    }
   }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>( "/robot_0/cmd_vel", 1000);
  ros::Subscriber sub = n.subscribe("/robot_0/base_scan",10, base_scan_cb);    //Subscribing to laser scans of the robot.

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    geometry_msgs::Twist msg;
   if(flag==1)
    {
     srand(time(NULL));
     int degree_random = (rand() % 360) + 1;
     float radian_random = degree_random/57.2958; //Random number generator to rotate the evader randomly in range [0,360] degrees.
     int odd_even_random = rand()%10;              
     if(odd_even_random % 2 == 0)
     msg.angular.z=radian_random;
     else
     msg.angular.z=radian_random;
     msg.linear.x = 0;
    }
    else
    {
     msg.linear.x = 2;
    }
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
   // ++count;
  }


  return 0;
}
