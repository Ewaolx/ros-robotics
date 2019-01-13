// Change trajectory_file_save variable to chose location of saving trajectory.txt.

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"
#include <lab4/Motion.h>
#include <lab4/Observation.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include <time.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>
#include <utility>


#define foreach BOOST_FOREACH	
#define timesteps 179

const int x_ = 36;
const int y_ = 36;
const int theta_ = 37;
int timecounter =0;
char commands[timesteps][50]  = {0};
char trajectory_file_save[200] = "/home/raj/trajectory.txt";

std::pair <int,int> trajectory[89];
int trajectory_angle[89] = {0};
int t_ind = 0 ;

void find_max_element(double grid[x_][y_][theta_], int result[])
{
  double max =0.0;
  int ind_i = 0,ind_j = 0,ind_k = 0;
  for(int i=1;i<x_;i++)
  {
   for(int j=1;j<y_;j++)
   {
    for(int k=1;k<theta_;k++)
    {
      if(grid[i][j][k]>max)
      {
        max = grid[i][j][k];
        ind_i = i;
        ind_j = j;
        ind_k = k;
      }
    }
   }
  }
  result[0] = ind_i;
  result[1] = ind_j;
  result[2] = ind_k;
  result[3] = grid[ind_i][ind_j][ind_k];
  std::cout<<"\nMax prob : "<<grid[ind_i][ind_j][ind_k];
  std::cout<<"\n Max Indices: "<<ind_i<<" "<<ind_j<<" "<<ind_k;   
}

float gaussian_pdf(int x, int m, float s)
{
    static const float inv_sqrt_2pi = 0.3989422804014327;
    float a = (x - m) / s;
    return inv_sqrt_2pi / s * std::exp(-0.5f * a * a);
}
double constrainAngle(double x)
{
    x = fmod(x,360);
    if (x < 0)
        x += 360;
    return x;
}

double radians_to_degrees(double radians)
{
    double degrees = constrainAngle(radians * (180.0/3.141592653589793238463));
    return degrees;
}

int degrees_to_cells(double degrees)
{
  int cell = ceil(degrees/10.);
  return cell;
}

int metres_to_cells(double metre)
{
  int cell = ceil(metre/0.2);
  return cell;
}

void  normalize(double grid[x_][y_][theta_],double sum)
{
  //double cs = 0;
  for(int i=1;i<x_;i++)
  {
   for(int j=1;j<y_;j++)
   {
    for(int k=1;k<theta_;k++)
    {
      grid[i][j][k]/= sum;    
    } 
   }
  }    
}
void initialize_grid_uniform(double grid[x_][y_][theta_])
{
  double t = 0.000022676;
  grid[0][0][0] = 0;
  for(int i=1;i<x_;i++)
  {
   for(int j=1;j<y_;j++)
   {
    for(int k=1;k<theta_;k++)
    {
     grid[i][j][k] = t; 
    }
   }
  }
}


void initialize_grid(double grid[x_][y_][theta_])
{
  // Initial mean values for Gaussian noise estimation
  int x_mean = 12;
  int y_mean = 28;
  int theta_mean = ceil(200.52/10.  );
  float standard_dev_x = .4;
  float standard_dev_y = .4;
  float standard_dev_theta = .4;
  double pc=0;
  for(int i=1;i<x_;i++)
  {
   for(int j=1;j<y_;j++)
   {
    for(int k=1;k<theta_;k++)
    {
      float p_x = gaussian_pdf(i,x_mean,standard_dev_x);
      float p_y = gaussian_pdf(j,y_mean,standard_dev_y);
      float p_theta = gaussian_pdf(k,theta_mean,standard_dev_theta);
      grid[i][j][k] = p_x * p_y * p_theta ; 
      pc+=grid[i][j][k];
    }  
   } 
  }
  normalize(grid,pc);
  std::cout<<"\n probability at 12,28,21 :"<<grid[12][28][21];  
  int r[4] = {0};
  //find_max_element(grid,r);  
}
void copy_array(double dest[x_][y_][theta_],double source[x_][y_][theta_])
{
  for(int i=1;i<x_;i++)
  {
   for(int j=1;j<y_;j++)
   {
    for(int k=1;k<theta_;k++)
    {
     dest[i][j][k] = source[i][j][k]; 
    }
   }
  }    
}

void observation_step(double grid[x_][y_][theta_], double observation[],std::pair <double,double> landmarks[])
{
 double temp_grid[x_][y_][theta_] = {0};
 copy_array(temp_grid,grid); 
  

 float standard_dev_range = .5;
 float standard_dev_angle = .5;

 int range_mean_cell  = metres_to_cells(observation[1]);
 double angle_mean = radians_to_degrees(observation[2]);
 int angle_mean_cell = degrees_to_cells(angle_mean);
 

 std::cout<<"\n observation[0]: "<<observation[0];


 std::cout<<"\n range: "<<observation[1];
 std::cout<<"\n range_mean_cell: "<<range_mean_cell;
 std::cout<<"\n angle: "<<observation[2];
 std::cout<<"\n angle_mean_cell: "<<angle_mean_cell;
   

 double landmark_x = landmarks[int(observation[0])].first;
 double landmark_y = landmarks[int(observation[0])].second;

 int landmark_x_cell = metres_to_cells(landmark_x); 
 int landmark_y_cell = metres_to_cells(landmark_y); 
 double sum =0;

for(int i_=1;i_<x_;i_++)
 {
  for(int j_=1;j_<y_;j_++)
  {
   for(int k_=1;k_<theta_;k_++)
   {
    if(grid[i_][j_][k_]==0)
    { 
     continue;
    } 
    int range_cell = ceil(pow(((pow(i_- landmark_x_cell,2))+(pow(j_- landmark_y_cell,2))),0.5));
    double angle = radians_to_degrees(atan2((j_- landmark_y_cell),(i_- landmark_x_cell)));
    
    int angle_cell = degrees_to_cells(angle);

    float p1 = gaussian_pdf(range_cell,range_mean_cell,standard_dev_range);
    float p2 = gaussian_pdf(angle_cell,angle_mean_cell,standard_dev_angle);
   
    grid[i_][j_][k_] *= (p1 * p2);
    sum+= grid[i_][j_][k_];
   }
  }
 }
 if(sum==0)
 {
   //initialize_grid_uniform(grid);
   copy_array(grid,temp_grid);
 }
 else
 {
  normalize(grid,sum); 
 }

 int res[4] = {0};
 
 find_max_element(grid,res);  

 trajectory[t_ind] = std::make_pair(res[0],res[1]);
 trajectory_angle[t_ind] = res[2]; 
 t_ind++;
  
}

void motion_step(double grid[x_][y_][theta_], double motion[])
{
  float standard_dev_trans = .5;
  float standard_dev_rot1 = .5;
  float standard_dev_rot2 = .5;
  std::cout<<"\nIn motion_step:"<<motion[0]<<"----"<<motion[1]<<"----"<<motion[2]; 

  double rot1_mean = radians_to_degrees(motion[0]);
  int rot1_mean_cell = degrees_to_cells(rot1_mean);
  double rot2_mean = radians_to_degrees(motion[2]);
  int rot2_mean_cell = degrees_to_cells(rot2_mean);

  int trans_mean_cell = metres_to_cells(motion[1]);
  //trans_mean_cell = 15;
  std::cout<<"\n rot1_mean :"<<rot1_mean ;
  std::cout<<"\n rot1_mean_cell :"<<rot1_mean_cell ;
  std::cout<<"\n rot2_mean :"<<rot2_mean ;
  std::cout<<"\n rot2_mean_cell :"<<rot2_mean_cell ;
  std::cout<<"\n trans_mean_cell :"<<trans_mean_cell ;
  double sum =0;


  double temp_grid[x_][y_][theta_] = {0};

  //copy_array(temp_grid,grid);

  for(int i_=1;i_<x_;i_++)
  {
   for(int j_=1;j_<y_;j_++)
   {
    for(int k_=1;k_<theta_;k_++)
    { 
     for(int i=1;i<x_;i++)
     {
      for(int j=1;j<y_;j++)
      {
       for(int k=1;k<theta_;k++)
       {
        if(grid[i][j][k]==0)
        {
          sum+=temp_grid[i_][j_][k_];
          continue;
        }
        int trans_cell = ceil(pow(((pow(i_-i,2))+(pow(j_-j,2))),0.5));
        double temp_rot1 = radians_to_degrees(atan2(j_-j,i_-i)); 
        double rot1 = constrainAngle(temp_rot1 - (k*10));   //degrees
        double rot2 = constrainAngle((k_*10) - (k*10) - rot1);
        
        int rot1_cell = degrees_to_cells(rot1);  
        int rot2_cell = degrees_to_cells(rot2);  

        float p_trans = gaussian_pdf(trans_cell,trans_mean_cell,standard_dev_trans);
        float p_rot1 = gaussian_pdf(rot1_cell,rot1_mean_cell,standard_dev_rot1);
        float p_rot2 = gaussian_pdf(rot2_cell,rot2_mean_cell,standard_dev_rot2);
 
        temp_grid[i_][j_][k_] += ((p_trans * p_rot1 * p_rot2) * grid[i][j][k]);
        sum+=temp_grid[i_][j_][k_];
       } 
      }  
     } 
    }  
   }  
  }

  copy_array(grid,temp_grid);

  normalize(grid,sum);

  std::cout<<"\n sumx: "<<sum;
}

void bayes_filter(double grid[x_][y_][theta_],double motion[][3],double observation[][3],std::pair <double,double> landmarks[])
{
  int time_ =1;
  int end_time = timesteps-1;
  for (int i=1;i<=end_time;i++)
  {
    std::cout<<"\n i: --------------------------------------------------->"<<i;
    if(!strcmp(commands[i],"Motion"))
    {
      motion_step(grid,motion[i]);
    }
    else if(!strcmp(commands[i],"Observation"))
    {
      observation_step(grid,observation[i],landmarks);
    }

  }

}

void assign_landmarks(std::pair <double,double> l[])
{
 l[0] = std::make_pair(1.25,5.25);
 l[1] = std::make_pair(1.25,3.25);
 l[2] = std::make_pair(1.25,1.25);
 l[3] = std::make_pair(4.25,1.25);
 l[4] = std::make_pair(4.25,3.25);
 l[5] = std::make_pair(4.25,5.25);
}


int main(int argc, char **argv)
{ 
  double motion[timesteps][3] = {0}; 
  double observation[timesteps][3] = {0}; 
  std::cout<<"\n Initially : "<<observation[0][0];

  std::pair <double,double> landmarks[6];
  assign_landmarks(landmarks);

  double grid[x_][y_][theta_];
  
  //  Reading the bag file and storing the contents.
  rosbag::Bag bag;
  bag.open("/home/raj/catkin_ws/src/lab4/src/grid.bag", rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("Movements"));
  topics.push_back(std::string("Observations"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));


  int time_ = 1;  //Timesteps to store the value at respective indices for motion and observation data.
  foreach(rosbag::MessageInstance const m, view)
  {
        lab4::Motion::ConstPtr s = m.instantiate<lab4::Motion>();
        if (s != NULL)
        {
          tf::Quaternion q1(s->rotation1.x, s->rotation1.y,s->rotation1.z,s->rotation1.w);
          tf::Matrix3x3 m1(q1);
          double roll1, pitch1, yaw1;
          m1.getRPY(roll1, pitch1, yaw1);

          tf::Quaternion q2(s->rotation2.x, s->rotation2.y,s->rotation2.z,s->rotation2.w);
          tf::Matrix3x3 m2(q2);
          double roll2, pitch2, yaw2;
          m2.getRPY(roll2, pitch2, yaw2);

          motion[time_][0]=(double)yaw1;
          motion[time_][1]=(double)s->translation;
          motion[time_][2]=(double)yaw2;

          strcpy(commands[time_],"Motion");
          time_++;
        }
        lab4::Observation::ConstPtr ss = m.instantiate<lab4::Observation>();
        if (ss != NULL)
        {
          
          tf::Quaternion q(ss->bearing.x, ss->bearing.y,ss->bearing.z,ss->bearing.w);
          tf::Matrix3x3 m(q);
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);

          observation[time_][0]=(double)ss->tagNum;
          observation[time_][1]=(double)ss->range;
          observation[time_][2]=(double)yaw;

          strcpy(commands[time_],"Observation");
          time_++;
        }
    }

  bag.close();
  
  initialize_grid(grid);
  bayes_filter(grid,motion,observation,landmarks);

  
  FILE * pFile;
  char name [100];
  int ts = 2;
  pFile = fopen (trajectory_file_save,"w");
  for (int i=0 ; i<89 ; i++)
  {
     fprintf (pFile, "After Time_step: %d, x: %d, y: %d, z: %d\n",ts,trajectory[i].first,trajectory[i].second,trajectory_angle[i]);
     ts+=2;
  }
  fclose (pFile);

  ros::init(argc, argv, "read");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  while(ros::ok())
  {
    visualization_msgs::Marker boundary, traj;
    boundary.header.frame_id = traj.header.frame_id = "/my_frame";
    boundary.header.stamp = traj.header.stamp = ros::Time::now();
    boundary.ns = traj.ns =  "trajectory_and_grid";
    boundary.action = traj.action =  visualization_msgs::Marker::ADD;
    boundary.pose.orientation.w = traj.pose.orientation.w = 1.0;

    boundary.id = 0;
    traj.id = 1;

    boundary.type = visualization_msgs::Marker::LINE_STRIP;
    traj.type = visualization_msgs::Marker::LINE_STRIP;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    boundary.scale.x = 0.1;
    traj.scale.x = 0.1;


    // Line strip is blue
    boundary.color.b = 1.0;
    boundary.color.a = 1.0;

    traj.color.b = 1.0;
    traj.color.a = 1.0;

    for(int i=0;i<89;i++)
    {
     geometry_msgs::Point p;
     p.x = trajectory[i].first;
     p.y = trajectory[i].second; 
     traj.points.push_back(p);     
    }

    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    geometry_msgs::Point p3;
    geometry_msgs::Point p4;
    geometry_msgs::Point p5;

    p1.x = 1;
    p1.y = 1;
    p2.x = 35;
    p2.y = 1;
    p3.x = 35;
    p3.y = 35;
    p4.x = 1;
    p4.y = 35;
    p5.x = 1;
    p5.y = 1;

    boundary.points.push_back(p1);
    boundary.points.push_back(p2);
    boundary.points.push_back(p3);
    boundary.points.push_back(p4);
    boundary.points.push_back(p5);
        
    marker_pub.publish(traj);
    marker_pub.publish(boundary);

    r.sleep();
  }

  std::cout<<"\n";
  return 0;

}



 /* while (ros::ok())
  {
   
    visualization_msgs::Marker points, line_strip, marker;
    points.header.frame_id = line_strip.header.frame_id = "/my_frame";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns =  "points_and_lines";
    points.action = line_strip.action =  visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Create the vertices for the points and lines
    /*for (uint32_t i = 0; i < 100; ++i)
    {
      float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
      float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

      geometry_msgs::Point p;
      p.x = (int32_t)i - 50;
      p.y = y;
      p.z = z;

      points.points.push_back(p);
      line_strip.points.push_back(p);

      // The line list needs two points for each line
      p.z += 1.0;


    }*//*

  for(int i_=1;i_<x_;i_++)
  {
   for(int j_=1;j_<y_;j_++)
   {
    for(int k_=1;k_<theta_;k_++)
    {
      if(grid[i_][j_][k_]!=0.0)
      {
        geometry_msgs::Point p;
        p.x = i_;
        p.y = j_;
        p.z = 0;
        points.points.push_back(p);
        break;
      }  
    }  
   }
  }  
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    geometry_msgs::Point p3;
    geometry_msgs::Point p4;
    geometry_msgs::Point p5;

    p1.x = 1;
    p1.y = 1;
    p2.x = 35;
    p2.y = 1;
    p3.x = 35;
    p3.y = 35;
    p4.x = 1;
    p4.y = 35;
    p5.x = 1;
    p5.y = 1;

    line_strip.points.push_back(p1);
    line_strip.points.push_back(p2);
    line_strip.points.push_back(p3);
    line_strip.points.push_back(p4);
    line_strip.points.push_back(p5);

    
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "points_and_lines";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.z = 3;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.text="VISUALIZATION TXT MARKER TEST";
    marker_pub.publish(marker);
    
    ros::Duration(30).sleep();

    marker.text="Change";

    marker_pub.publish(marker);
    ros::Duration(30).sleep();
        
    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    r.sleep();
  }*/