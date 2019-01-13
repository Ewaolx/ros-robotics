//Source for running a_star algorithm and making robot move along the formed path.
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>
#include <utility>
#include <stdlib.h>
#include <time.h> 
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <cmath>

using namespace std;
int row_size = 20;
int col_size = 18;
int vert_hor_dist = 10;
int diagonal_dist = 14;
int c_mid = 8;    // For mapping between grid and world map
int r_mid = 9; 

int tp_r,tp_c; //Final targets in grid

  
int grid[20][18] = {
  	                  {0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0},
                      {0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0},
                      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                      {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                      {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                      {0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0},
                      {0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0},
                      {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0},
                      {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1},
                      {0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1},
                      {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1},
                      {0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0},
                      {0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0},
                      {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
                      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                      {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
                      {0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0},
                      {0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0},
                      {0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0},
                      {0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1},
                     };

vector <pair <int,int> > path_coordinates;

ros::Publisher turtle_vel;

int state = 0; // switch between rotating and moving of robot.
void world_to_grid(int x,int y,int res[])
{
 int grid_r = int(r_mid - y);
 int grid_c = int(c_mid + x);
 if(grid[grid_r][grid_c] == 1)
 { 
  for(int i=grid_r-1;i<=grid_r+1;i++)
  {
   for(int j=grid_c-1;j<=grid_c+1;j++)	
   {
    if(grid[i][j] ==0)
    	res[0] = i;
        res[1] = j;
        return ;
   }
  } 
 }
 res[0] = grid_r ;
 res[1] = grid_c ;
}
void grid_to_world(int grid_r,int grid_c,int res[])
{
	int x = int(grid_c - c_mid);
	int y = int(r_mid - grid_r);
	res[0] = x;
	res[1] = y; 
}

void initialize_hvalue(int grid_value[20][18][3], int tp_r,int tp_c)
{
 for(int i=0;i<row_size;i++)
 {
  for(int j=0;j<col_size;j++)	
  {
   int dx = abs(tp_r - i);
   int dy = abs(tp_c - j); 	
   if(dx<dy)
   	grid_value[i][j][0] = (dx * diagonal_dist) + ((dy-dx)*vert_hor_dist);
   if(dy<dx)
   	grid_value[i][j][0] = (dy * diagonal_dist) + ((dx-dy)*vert_hor_dist);
   if(dy==dx)
   	grid_value[i][j][0] = dx * diagonal_dist;
  }
 }
}

void move(int grid[20][18],pair <int,int> parent_grid[20][18],int open_closed_grid[20][18],int grid_value[20][18][3],int cp_r,int cp_c)
{	
 for(int i=cp_r-1;i<=cp_r+1;i++)
 {
  for(int j=cp_c-1;j<=cp_c+1;j++)	
  {
  	if(i<0 || i>=row_size || j<0 || j>=col_size)   //Out of range check
  		continue;
  	if(grid[i][j] == 1 || open_closed_grid[i][j] == -1) //Obstacle or already iterated
  		continue;
    if(i == cp_r || j == cp_c)
    {
     if((open_closed_grid[i][j] == 1 && (vert_hor_dist + grid_value[cp_r][cp_c][1] < grid_value[i][j][1])) || open_closed_grid[i][j] == 0)
     {
      grid_value[i][j][1] = vert_hor_dist + grid_value[cp_r][cp_c][1];
      grid_value[i][j][2] = grid_value[i][j][0] + grid_value[i][j][1];

      parent_grid[i][j] = make_pair(cp_r,cp_c); 
     }	
    }
    else 
    {
     if((open_closed_grid[i][j] == 1 && (diagonal_dist + grid_value[cp_r][cp_c][1] < grid_value[i][j][1])) || open_closed_grid[i][j] == 0)
     {
      if(i == cp_r-1 && j == cp_c-1 && grid[cp_r][cp_c-1] == 1 && grid[cp_r-1][cp_c] == 1)
      {
        continue;  
      }
      if(i == cp_r+1 && j == cp_c+1 && grid[cp_r][cp_c+1] == 1 && grid[cp_r+1][cp_c] == 1)
      {
        continue;  
      }
      if(i == cp_r-1 && j == cp_c+1 && grid[cp_r-1][cp_c] == 1 && grid[cp_r][cp_c+1] == 1)
      {
        continue;  
      }
      if(i == cp_r+1 && j == cp_c-1 && grid[cp_r][cp_c-1] == 1 && grid[cp_r+1][cp_c] == 1)
      {
        continue;  
      }	
      grid_value[i][j][1] = diagonal_dist + grid_value[cp_r][cp_c][1];
      grid_value[i][j][2] = grid_value[i][j][0] + grid_value[i][j][1];

      parent_grid[i][j] = make_pair(cp_r,cp_c); 	
     } 
    }
    open_closed_grid[i][j] = 1;  //Adding to open list
  }	
 } 
}


                  
void command_robot(const nav_msgs::Odometry::ConstPtr& msg)
{	
 geometry_msgs::Pose2D pose2d;	
 pose2d.x = msg->pose.pose.position.x;
 pose2d.y = msg->pose.pose.position.y;

 tf::Quaternion q(
 msg->pose.pose.orientation.x,
 msg->pose.pose.orientation.y,
 msg->pose.pose.orientation.z,
 msg->pose.pose.orientation.w);
 tf::Matrix3x3 m(q);
 double roll, pitch, yaw;
 m.getRPY(roll, pitch, yaw);

 geometry_msgs::Twist vel_msg;
 float angle_to_goal = atan2(path_coordinates.back().second - pose2d.y,path_coordinates.back().first - pose2d.x);
 cout<<"pose2d.x: "<<pose2d.x<<" pose2d.y"<<pose2d.y<<"\n";
 //cout<<"path_coordinates - pose2d: "<<path_coordinates.back().second - pose2d.y<<" "<<path_coordinates.back().first - pose2d.x<<"\n";
 cout<<"angle_to_goal: "<<angle_to_goal<<"\n";
 float distance = sqrt(pow(path_coordinates.back().second - pose2d.y, 2) + pow(path_coordinates.back().first - pose2d.x, 2));

 if(abs(angle_to_goal - yaw)>0.05 && state == 0)
  {state = 0; cout<<"1\n";}
 else if(distance > 0.3 )
  {state = 1; cout<<"2\n";}
 else //if(distance < 0.3)
  {state = -1; cout<<"3\n";}
 
 int res[2];
 world_to_grid(pose2d.x,pose2d.y,res); 
 int pose_r_grid = res[0];
 int pose_c_grid = res[1];
 float distance_to_target = sqrt(pow(tp_r - pose_r_grid, 2) + pow(tp_c - pose_c_grid, 2));
 
 cout<<"Distance to target:"<<distance_to_target<<"\n";

 if(distance_to_target <= 1)   
 {
 	cout<<"Reached target\n";
 	vel_msg.linear.x = 0.0;
 	vel_msg.angular.z = 0.0;
 	turtle_vel.publish(vel_msg);
 	return;
 } 

 if (state == 0)
 {
 	cout<<"rotating:"<<abs(angle_to_goal - yaw)<<"\n";
 	vel_msg.linear.x = 0.0;
 	vel_msg.angular.z = 0.8;  //0.3 original
 }
 else if(state == 1)
 {
 	cout<<"moving distance"<<distance<<"\n";
 	vel_msg.linear.x = 0.5;
 	vel_msg.angular.z = 0.0;
 }
 else if(state == -1)
 {
  	cout<<"stopping"<<"\n";
 	vel_msg.linear.x = 0.0;
 	vel_msg.angular.z = 0.0;	
 	cout<<"pose of robot:"<<pose2d.x<<" "<<pose2d.y<<"\n";
 	path_coordinates.pop_back();
 	state = 0;
 	/*if(path_coordinates.size() == 0)
    {
     state = -1;
    }*/
 }
 turtle_vel.publish(vel_msg);
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "run_a_star");
  ros::NodeHandle n;

  double goalx,goaly;
  n.getParam("goalx",goalx);
  n.getParam("goaly",goaly);
  cout<<"The parameter is:"<<goalx<<"x"<<goaly;


  int open_closed_grid[20][18] = {0};
  int grid_value[20][18][3] = {0}; 

  int sp_r = 11;
  int sp_c = 0;
  int starting_r = sp_r;
  int starting_c = sp_c;
  int res[2]; 
  world_to_grid(goalx,goaly,res);
  tp_r = res[0];
  tp_c = res[1];

  cout<<"end goal: "<<tp_r<<"x"<<tp_c<<"\n"; 

  pair<int,int> parent_grid[20][18];

  initialize_hvalue(grid_value,tp_r,tp_c);

  cout<<"H value"<<"\n";
  for(int i=0;i<row_size;i++)
  {
  	for(int j=0;j<col_size;j++)
  	{
  		cout<<grid_value[i][j][0]<<" ";
  	}
  	cout<<"\n";
  }
   
  int count = 0,flag = 0; 

  while(true)
  {	
   cout<<"count: "<<count<<"\n";	
   cout<<"sp_r :"<<sp_r<<" sp_c:"<<sp_c;
   // if(count== 2)
   // {
   // 	break;
   // }	
   
   open_closed_grid[sp_r][sp_c] = -1;	  // Adding to closed list
   move(grid,parent_grid,open_closed_grid,grid_value,sp_r,sp_c);
   // cout<<"G value"<<"\n"; 
   // for(int i=0;i<row_size;i++)
   // {
  	// for(int j=0;j<col_size;j++)
  	// {
  	// 	cout<<grid_value[i][j][1]<<" ";
  	// }
  	// cout<<"\n";
   // }
   // cout<<"F value"<<"\n"; 
   // for(int i=0;i<row_size;i++)
   // {
  	// for(int j=0;j<col_size;j++)
  	// {
  	// 	cout<<grid_value[i][j][2]<<" ";
  	// }
  	// cout<<"\n";
   // }
   // cout<<"open_closed value"<<"\n"; 
   // for(int i=0;i<row_size;i++)
   // {
  	// for(int j=0;j<col_size;j++)
  	// {
  	// 	cout<<open_closed_grid[i][j]<<" ";
  	// }
  	// cout<<"\n";
   // }
   int temp_r, temp_c, temp_f = 1000000;
   for(int i=0;i<row_size;i++)
   {
   	for(int j=0;j<col_size;j++)
   	{
   	 if(open_closed_grid[i][j]==1)
   	 {
   	  //cout<<"Here: i:"<<i<<" j:"<<j<<"\n";	
   	  if(i == tp_r && j == tp_c)	
   	  {
   	  	// Target Reached
   	  	flag = 1;
   	  	break;
   	  }
   	  if(temp_f > grid_value[i][j][2])
   	  {
   	   temp_f = grid_value[i][j][2];
   	   temp_r = i;
   	   temp_c = j;
   	  }
   	 }	
   	}
    if(flag == 1)
     break; 
   }
   if(flag == 1)
   	break;
   sp_r = temp_r;
   sp_c = temp_c; 
   count++;
  }


   // cout<<"F value"<<"\n"; 
   // for(int i=0;i<row_size;i++)
   // {
  	// for(int j=0;j<col_size;j++)
  	// {
  	// 	cout<<grid_value[i][j][2]<<" ";
  	// }
  	// cout<<"\n";
   // }


   //  cout<<"parent_grid"<<"\n"; 
   // for(int i=0;i<row_size;i++)
   // {
  	// for(int j=0;j<col_size;j++)
  	// {
  	// 	cout<<parent_grid[i][j].first<<"x"<<parent_grid[i][j].second<<" ";
  	// }
  	// cout<<"\n";
   // }

   //cout<<"parent of 5 12:"<<parent_grid[5][12].first<<"x"<<parent_grid[5][12].second<<"\n";
   int path_grid[20][18] ={0};
   int tc = tp_c;
   int tr = tp_r;
   

   
//   int ii =0 ;
   while(tr != starting_r || tc != starting_c)
   {
   	// if(ii == 7)
   	// 	break;
   	// ii++;
   	path_grid[tr][tc] = 1;
   	int res[2];
   	grid_to_world(tr,tc,res);
    path_coordinates.push_back(make_pair(res[0],res[1]));
   	//cout<<"parent of "<<tr<<" "<<tc<<":";
    int temp_tr = tr;
    int temp_tc = tc;
   	tr = parent_grid[temp_tr][temp_tc].first;
   	tc = parent_grid[temp_tr][temp_tc].second;
   	//cout<<"is:"<<tr<<"x"<<tc<<"\n";
   }

   cout<<"Path"<<"\n"; 
   for(int i=0;i<row_size;i++)
   {
  	for(int j=0;j<col_size;j++)
  	{
  		cout<<path_grid[i][j]<<" ";
  	}
  	cout<<"\n";
   }

   // for(int i=0;i<path_coordinates.size();i++)
   // {
   // 	cout<<"path_coordinates:"<<path_coordinates[i].first<<"x"<<path_coordinates[i].second<<"\n";
   // }
   
 //  cout<<"test: "<<path_coordinates.back().first<<"x"<<path_coordinates.back().second<<"\n";

  turtle_vel = n.advertise<geometry_msgs::Twist>( "/cmd_vel", 1000);
  ros::Subscriber sub = n.subscribe("/base_pose_ground_truth",10,command_robot);    //Subscribing to laser scans of the robot.

  ros::Rate loop_rate(5);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}


  
  //parent_grid[0][0] = make_pair(1,10);
  //cout<<"parent_:"<<parent_grid[0][0].first<<"dede"<<parent_grid[0][0].second;

  // string text = "";
  // ifstream file;
  // file.open("/home/raj/catkin_ws/src/lab5/src/map.txt");
  // if(!file.is_open())
  // 	cout<<"Error while opening the file";
  // else
  // {
  // 	file>>text;
  // 	file>>text;
  // 	file>>text;
  // 	file>>text;
  // 	cout<<"file:"<<text;
  // }

  //file.close();
