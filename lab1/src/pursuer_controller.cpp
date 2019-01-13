/*
Reference -  Code snippets could match the ROS tutorials, as they are modified in order to fit the requirements of the assignment.
Source file for making the pursuer follow the evader.
*/
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <sstream>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pursuer_controller");

  ros::NodeHandle node;

  ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("robot_1/cmd_vel", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0); //There is delay of 10 secs, as to wait for the availability of tf for the robots, so that transform can then be applied. So, the pursuer will start to pursue after 10 secs.
  ros::Duration(10.0).sleep();
  while (node.ok())
  {
    tf::StampedTransform transform;
    try
    {
     ros::Time now = ros::Time::now();
     ros::Time past = now - ros::Duration(1.0);  // The evader will approach the purseur's position with 1 sec time delay.
     listener.waitForTransform("/robot_1", now,
                              "/robot_0", past,
                              "/world", ros::Duration(1.0));
     listener.lookupTransform("/robot_1", now,
                             "/robot_0", past,
                             "/world", transform);
    }
    catch (tf::TransformException &ex) 
    {
     ROS_ERROR("%s",ex.what());
     ros::Duration(1.0).sleep();
     continue;
    }

    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4 * atan2(transform.getOrigin().y(),transform.getOrigin().x());
  //  ROS_INFO("y_co: %f ; x_co: %f",transform.getOrigin().y(),transform.getOrigin().x());
  //  ROS_INFO("\nAngular Velocity: %f", vel_msg.angular.z );
    float distance = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
    if(distance<1.5)
    vel_msg.linear.x=0;   // If the pursuer approaches too close to the evader, then it would stop inorder to avoid the collison between the two robots.
    else 
    vel_msg.linear.x =0.5 * distance; 
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};
