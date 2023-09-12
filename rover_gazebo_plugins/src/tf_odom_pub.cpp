#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <string>
#include <sstream>
#include <iostream>
#include <stdio.h>
using namespace std;

string robot_ns;
ros::Publisher pose_pub;

class RoverOdomTF
{
  // public: RoverOdomTF(string robot_ns);
  // public: void callback(nav_msgs::OdometryPtr msg);
  public: 
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time;
    string robot_ns;
    // std::unique_ptr<ros::NodeHandle> rosNode;
    ros::NodeHandle node;
    ros::Publisher odom_pub;
    ros::Subscriber odom_sub;
    geometry_msgs::TransformStamped odom_trans;

};

// RoverOdomTF::RoverOdomTF(string robot_ns)
// {
//   // this->robot_ns = robot_ns;
//   // this->rosNode.reset(new ros::NodeHandle(robot_ns));
//   odom_sub = node.subscribe("/"+robot_ns+"/odom", 10, &RoverOdomTF::callback, this);
// }

void callback(nav_msgs::OdometryPtr msg)
{
  static tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time;
  geometry_msgs::TransformStamped odom_trans;

  // cout<<"get"<<endl;
  current_time = ros::Time::now();  
  odom_trans.header.stamp = current_time;
  
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = robot_ns+"/chassis";

  odom_trans.transform.translation.x = msg->pose.pose.position.x;
  odom_trans.transform.translation.y = msg->pose.pose.position.y;
  odom_trans.transform.translation.z = msg->pose.pose.position.z;
  odom_trans.transform.rotation = msg->pose.pose.orientation;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.frame_id = robot_ns+"/chassis";
  pose_msg.header.stamp = current_time;
  pose_msg.pose = msg->pose;
  pose_pub.publish(pose_msg);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "tf_odom_pub");
  stringstream ss;
  ss << argv[1];
  robot_ns = ss.str();
  // RoverOdomTF rover_odom(robot_ns);
  ros::NodeHandle node;
  ros::Subscriber odom_sub = node.subscribe("/"+robot_ns+"/odom", 10, &callback);
  pose_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/"+robot_ns+"/pose",10);

  ros::spin();
  return 0;
}