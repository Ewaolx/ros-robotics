//source for running bug2
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include <time.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

sensor_msgs::LaserScan laser ;
int mid,flag=0;
geometry_msgs::Twist vel_msg;
nav_msgs::Odometry nav_msg;

float rx,ry;
int wf=0;
int state  = 1; // 1 - GOAL SEEK  ; 0 - WALL FOLLOW
int vm_flag = 1;
int turn=0; // 1 - right turn ; 0 - left turn 
//std::string turtle_name;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
void visualization_marker_cb(const visualization_msgs::Marker::ConstPtr& mark)
{
  if(vm_flag == 1)
    return ;
 
  int f=0,slope_compare=0;
  float distance =0;
  //ROS_INFO("\n thes");

  
  if(mark->id==1)
  {
   
   int index=0,y_dis=1000;
   // Change state to GOAL_SEEK if there is no obstacle.
   if(mark->points.size()==0 && wf==2)
   {

    vm_flag = 1;
    wf = 0;
    state=1; 

   }
   int count_nx=-1;
   float slope=0;
   for(int i=0;i<mark->points.size();i+=2)
   {
    count_nx=0;
    float x1 = mark->points[i].y;
    float y1 = mark->points[i].x;
    float x2 = mark->points[i+1].y;
    float y2 = mark->points[i+1].x;
    float m = (y2-y1)/(x2-x1);
    float b_ = y1 - (m*x1);
    float a = -m,b = 1,c = -b_;
    float distance  = abs((a*rx)+(b*ry)+c)/pow((pow(a,2)+pow(b,2)),0.5);

      
     int d_flag=0;
    if((x1 < 1 && y1 < 0.2)||(x2 < 1 && y2 < 0.2) &&  wf!=0) //line coordinates to which robot is parallel and close moving.
    {

      slope  = m;
      distance  = abs((a*rx)+(b*ry)+c)/pow((pow(a,2)+pow(b,2)),0.5);
      d_flag=1;
      ROS_INFO("\ndistance : %f",distance);

    }
    ROS_INFO("\n wf=%d, d_flag =%d",wf,d_flag);

    if((distance < 0.9 && d_flag==1) || ((abs(x1)<0.2 && abs(y1)<0.2)||(abs(x2)<0.2 && abs(y2)<0.2)))
    {
     ROS_INFO("\n  Safety turn right"); 
     vel_msg.angular.z = -0.5;
     vel_msg.linear.x = 0 ; 
    }
    else if(distance > 0.9  && d_flag==1)
    {
     ROS_INFO("\n  Safety turn left"); 
     vel_msg.angular.z = 1.0;
     vel_msg.linear.x = 0 ; 
    }
  
    if(x1>0.2 || y2>0.2)  
    {
           count_nx++;
    }
    else if(abs(x1)>1 ||abs(x2)>1)
    {
      count_nx++;
    }
    float x_min=500,y_min=500;
    if(/*abs(x1)<abs(x2) &&*/ y1<y2)
    {
     x_min=abs(x1);
     y_min=y1;
    }
    else if(/*abs(x2)<abs(x1) && */y2<y1)
    {
     x_min=abs(x2);
     y_min=y2;
    } 
    ROS_INFO("\nx_min:%f",x_min);
    if(((x1 * x2 < 0) && (y1 < 0.9 || y2 < 0.9 )) || (x_min<0.1 && x_min!=0 && y_min < 0.1) )//corner case x1 && x2 both positive or negative but x1,y1 or x2,y2 below threshold
    {
         f=1;
         turn =1;
         vel_msg.angular.z = -0.5;  //turn right
         vel_msg.linear.x = 0 ;
         wf=1;
         ROS_INFO("\n  Normal turn right"); 
         break;
    }  
   
   if(abs(m-slope)>0.2 )
   {
    slope_compare=1;
   }
   

  // Makes left turn when there is more than threshold(0.2) change in slope.
   if((y1 > 0.5 && y2 > 0.5  && wf==2 ) ) //  if(i<=mark->points.size()-1 && slope_compare==1 && wf==2)
   {
    ROS_INFO("\nx1: %f,y1: %f,x2: %f,y2: %f",x1,y1,x2,y2);
    ROS_INFO("\n  Normal turn left"); 
     
    vel_msg.angular.z = 2.0;   //turn left
    vel_msg.linear.x=0;
    f=1;
    state=0;
   }

  }
   
   
   // state change to allow robot taking left turns.
   if(f==0)
   {
    if(wf==1)
      wf=2;
    vel_msg.angular.z = 0;
    vel_msg.linear.x = 0.4 ;

   }
     ROS_INFO("\n ------------");
  }
  
}

int main(int argc, char **argv)
{
  //turtle_name = argv[1];
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;
  ros::Publisher robot_vel = n.advertise<geometry_msgs::Twist>( "/cmd_vel", 1000);
 // ros::Publisher robot_pose = n.advertise<geometry_msgs::Twist>( "/odom", 1000);
  ros::Subscriber sub = n.subscribe("/visualization_marker",10, visualization_marker_cb);    

  //ros::Rate loop_rate(10);
  tf::TransformListener listener;
  
  float x1=-8.0,y1=-2.0,x2=4.5,y2=9.0;
  float m =(y2-y1)/(x2-x1);
  float b =y1 - (m*x1);

  ros::Rate rate(10.0);
  ros::Duration(3.0).sleep();
  while (ros::ok())
  {
    tf::StampedTransform transform,robot_in_world;
    

    try
    {
      listener.lookupTransform("/robot_0", "/goal",
                               ros::Time(0), transform);
      listener.lookupTransform("/world", "/robot_0",
                               ros::Time(0), robot_in_world);
    }
    catch (tf::TransformException &ex) 
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    

       float rx = robot_in_world.getOrigin().x();
       float ry = robot_in_world.getOrigin().y();


     //state change to GOAL_SEEK when robot moves close to the line joining start position and end position(goal).  
    if(state==0)
    {
       if(abs(ry - ((m*rx) + b)) < 0.5)
       {
        vm_flag = 1;
        wf = 0;
        state=1;
       }
    }

    // GOAL_SEEK state: robot rotates towards the goal, and then again changes its state to normal 
    if(vm_flag==1)
    { 
     vel_msg.angular.z = 6.0 * atan2(transform.getOrigin().y(),transform.getOrigin().x());
     vel_msg.linear.x = 0*0.2 *  sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
   //  ROS_INFO("\n an_vel:%f",vel_msg.angular.z);
     if((int)vel_msg.angular.z==0)
      { 
     //   ROS_INFO("\nwhy");
        vm_flag=0;
        //state=0;
      }
    }                      
    
    // Checks if it has reached the goal or not.
    ROS_INFO("Distance from goal:%f",pow((pow(rx-x2,2)+pow(ry-y2,2)),0.5));
    if(pow((pow(rx-x2,2)+pow(ry-y2,2)),0.5) < 0.4)
    {
         vel_msg.angular.z=0;
         vel_msg.angular.x=0;
         vm_flag=1;
         state=-2;
    }

    robot_vel.publish(vel_msg);


    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
