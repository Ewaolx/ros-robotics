// Source for running perception program.
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>
#include <utility>
#include <stdlib.h>
#include <time.h> 
#include <math.h>

using namespace std;
sensor_msgs::LaserScan laser ;
int mid,flag=0,length=-9;

float x_pos[361],y_pos[361];


void base_scan_cb(const sensor_msgs::LaserScan::ConstPtr& las)
{
   for(int i=0;i<361;i++)
   {
      x_pos[i]=0;
      y_pos[i]=0;
   }
   ROS_INFO("Hello: ");
   //flag=0;
   laser =*las;
   length=laser.ranges.size();
   mid = length/2;
   ROS_INFO("\n length: %d",length);
   int c=0;
   for(int i=0;i<361;i++)
   {
     if(laser.ranges[i]>=3.0)
     continue;
     double rad = i * (1.57/180);
     x_pos[c]=laser.ranges[i]*sin(rad);
     y_pos[c]=(-1)*laser.ranges[i]*cos(rad); 
     //myvector.push_back(make_pair(x_pos[c],y_pos[c])); 
     c++;
   }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/base_scan",10, base_scan_cb);    //Subscribing to laser scans of the robot.
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate loop_rate(5);
 // ros::Rate r(30);
   
  while (ros::ok())
  {
    visualization_msgs::Marker points,line_strip;
    points.header.frame_id= "/base_link";
    points.header.stamp= ros::Time::now();
    points.ns = "points_and_lines";
    points.action= visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    

    line_strip.header.frame_id= "/robot_0";
    line_strip.header.stamp= ros::Time::now();
    line_strip.ns = "points_and_lines";
    line_strip.action= visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.01;
    points.scale.y = 0.01;

    line_strip.scale.x = 0.05;
    //line_strip.scale.y = 0.05;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;
    
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    
    vector <pair <float,float> > myvector;
    for(int i=0;i<361;i++)
    {
      geometry_msgs::Point p;
      p.x = x_pos[i];
      p.y = y_pos[i];
      if(p.x != 0 && p.y != 0)
      {
       myvector.push_back(make_pair(p.x,p.y));
       points.points.push_back(p);
      }
      p.z = 0;
       
     
     //points.points.push_back(p);
    
    }
    ROS_INFO("\n here ----- length: %d",myvector.size());
    int count_inliers =0;
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
     srand (time(NULL));
    while(myvector.size()>20)
    {
     int k=10; 
     int ind_a,ind_b;
     
     vector <int> records_points ;
     int count =0;
     for(int kk=0;kk<k;kk++)
     {
      count_inliers=0;
      
      int a = rand() % myvector.size();  
      int b = rand() % myvector.size();
      while(a==b)
      b = rand() % myvector.size();
      ROS_INFO("\na:-------->%d,b:-------->%d, kk------------->%d",a,b,kk);
      
      float x1_variable= roundf(myvector[a].first*100)/100;
      float y1_variable= roundf(myvector[a].second*100)/100;
      float x2_variable= roundf(myvector[b].first*100)/100;
      float y2_variable= roundf(myvector[b].second*100)/100;
      ROS_INFO("\nx1:-------->%f,y1:-------->%f, x2------------->%f,y2------------->%f",x1_variable,y1_variable,x2_variable,y2_variable); 
      //float m = (myvector[b].second - myvector[a].second)/(myvector[b].first - myvector[a].first);
      
      float distance_threshold=10;
      vector <int>index ;
      float x_diff = abs(abs(x2_variable) - abs(x1_variable));
      float y_diff = abs(abs(y2_variable) - abs(y1_variable));
      if(x_diff <=0.05)
      {
        for(int i=0;i<myvector.size();i++)
        {
         float x =roundf(myvector[i].first*100)/100;
         float y =roundf(myvector[i].second*100)/100;
         if(abs(x-x1_variable)<0.05 || abs(x-x2_variable)<0.05)
         {
          index.push_back(i);
          count_inliers++;
         }
        }
      }
      else if(y_diff <=0.05)
      {
        for(int i=0;i<myvector.size();i++)
        {
         float x =roundf(myvector[i].first*100)/100;
         float y =roundf(myvector[i].second*100)/100;
         if(abs(y-y1_variable)<0.05 || abs(y-y2_variable)<0.05)
         {
          index.push_back(i);
          count_inliers++;
         }
        }
      }
      else
      {
       float m = (y2_variable-y1_variable)/(x2_variable-x1_variable); 
       float b_intercept = y1_variable-(x1_variable * m);
       ROS_INFO("SLOPE: %f",m);
       for(int i=0; i<myvector.size();i++)
       {
        float x =roundf(myvector[i].first*100)/100;
        float y =roundf(myvector[i].second*100)/100;
        float distance =  abs(y-((m*x)+b_intercept))*100;
        ROS_INFO("Valueee:--> %f I--- %d",distance,i);
        if(distance <= distance_threshold)
        {
         index.push_back(i);
         count_inliers++;
        }
       } 
      }
      if(count_inliers>count)
      {
       ROS_INFO("count_inliers : %d , count : %d",count_inliers,count);
       count=count_inliers;
       records_points=index;
       ind_a = a;
       ind_b = b;
      }
      //index.clear();
     }
      
     ROS_INFO("\n count inliers : %d",count);
     ROS_INFO("\n myvector size before droping inliers : %d",myvector.size());  
    /* float p1xx=myvector[ind_a].first;
     float p1yy=myvector[ind_a].second;
     float p2xx=myvector[ind_b].first;
     float p2yy=myvector[ind_b].second;*/
     /*p1.x = myvector[ind_a].first;
     p1.y = myvector[ind_a].second;
     p2.x = myvector[ind_b].first;
     p2.y = myvector[ind_b].second;*/
    /* p1.x=p1xx;
     p1.y=p1yy;
     p2.x=p2xx;
     p2.y=p2yy;
     line_strip.points.push_back(p1);
     line_strip.points.push_back(p2); */
     
     // Dropping the inlier points by splitting them into inliers(myvector2) and outliers(myvector1)  
     vector <pair <float,float> > myvector1;
     vector <pair <float,float> > myvector2; 
     for(int i=0;i<myvector.size();i++)
     {
         int f=0;
         if(i==ind_a || i==ind_b)
         continue;
         for(int j=0;j<records_points.size();j++)    
         {
          if(i==records_points[j])
          {
            myvector2.push_back(make_pair(myvector[i].first,myvector[i].second));
            f=1;
            break;
          }
         }
         if(f==0)
         myvector1.push_back(make_pair(myvector[i].first,myvector[i].second));
     }
     myvector=myvector1; 
     ROS_INFO("myvector size after droping inliers : %d",myvector.size());
       
     // Finding the farthest two points by iterating over all points in the vecctor twice  
     float x1 = myvector2[0].first;
     float y1 = myvector2[0].second;
     float dis=0;
     int inde=0;
     for(int i=1;i<myvector2.size();i++)
     {
        float distance;
        float x2 = myvector2[i].first;
        float y2 = myvector2[i].second; 
        distance=pow((x2-x1),2)+pow((y2-y1),2);
        if(distance > dis)
        { 
         dis = distance;
         inde=i;
        }
     }
     x1 = myvector2[inde].first;
     y1 = myvector2[inde].second;
     dis=0;
     for(int i=0;i<myvector2.size();i++)
     {
        float distance;
        float x2 = myvector2[i].first;
        float y2 = myvector2[i].second; 
        distance=pow((x2-x1),2)+pow((y2-y1),2);
        if(distance > dis)
        { 
         dis = distance;
         inde=i;
        }
     }
     
    /* p1.x=p1xx;
     p1.y=p1yy;
     p2.x=p2xx;
     p2.y=p2yy;*/
     p1.x=x1;
     p1.y=y1;
     p2.x=myvector2[inde].first;
     p2.y=myvector2[inde].second;
     line_strip.points.push_back(p1);
     line_strip.points.push_back(p2); 


     //int erase_count=0;
    // break;
    /* myvector.erase(myvector.begin()+ind_a-erase_count);
     erase_count++;
     myvector.erase(myvector.begin()+ind_b-erase_count);*/
     /*erase_count++;
     for(int i=0;i<records_points.size();i++)
     {
      myvector.erase(myvector.begin()+records_points[i]-erase_count);
      erase_count++;
     }*/
   //  break;
     ROS_INFO("\n myvector size AFTER ERASE : %d",myvector.size());
    }//end of while()
    
    marker_pub.publish(points);
    //myvector.clear();
    //ROS_INFO("\n myvector size AFTER  CLEAR : %d",myvector.size());
    marker_pub.publish(line_strip);
    ROS_INFO("\n Coming till here -------------------<<<<<>>>>>>>>>>>>>>");
    ros::spinOnce();
    loop_rate.sleep();
    
  }
}
