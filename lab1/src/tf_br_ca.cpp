/*
Reference -  Code snippets could match the ROS tutorials, as they are modified in order to fit the requirement of the assignment.
Source file for publishing the tf of both the robots to the world frame. Creates a link between the world frame and both the robots.
*/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>

std::string turtle_name;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y, 0.0) );
  double roll,pitch,yaw;
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation,q);
  // Extracting the yaw angle from the given quaternion.
  tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
  yaw=angles::normalize_angle_positive(yaw);
  q.setRPY(0, 0, yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
  turtle_name = argv[1];
  ROS_INFO("name of turtle: %s",turtle_name.c_str());
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/base_pose_ground_truth", 10, &poseCallback); //Subscribing to the pose of the robot in order to map it to the world frame using tf.

  ros::spin();
  return 0;
};
