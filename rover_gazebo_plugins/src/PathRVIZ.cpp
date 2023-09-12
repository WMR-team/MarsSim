#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <cmath>

ros::Publisher marker_pub;
// ros::Publisher marker_pub2;
int i = 0;
visualization_msgs::Marker line_strip;
// visualization_msgs::Marker line_strip2;
int freq = 100;

void Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  // i ++;
  if(i%freq==0)
  {
    line_strip.header.frame_id = "odom";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "points_and_lines";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    // line_strip.id = 1;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;


    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    // Line strip is blue
    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = msg->pose.pose.position.x;
    p.y = msg->pose.pose.position.y;
    p.z = msg->pose.pose.position.z;

    line_strip.points.push_back(p);

    marker_pub.publish(line_strip);
  }

}

// void Callback2(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
//   if(i%freq==0)
//   {
//     line_strip2.header.frame_id = "odom";
//     line_strip2.header.stamp = ros::Time::now();
//     line_strip2.ns = "points_and_lines";
//     line_strip2.action = visualization_msgs::Marker::ADD;
//     line_strip2.pose.orientation.w = 1.0;

//     // line_strip.id = 1;

//     line_strip2.type = visualization_msgs::Marker::LINE_STRIP;


//     // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
//     line_strip2.scale.x = 0.1;

//     // Line strip is blue
//     line_strip2.color.r = 1.0;
//     line_strip2.color.a = 1.0;

//     geometry_msgs::Point p;
//     p.x = msg->pose.position.x;
//     p.y = msg->pose.position.y;
//     p.z = msg->pose.position.z;

//     line_strip2.points.push_back(p);

//     marker_pub2.publish(line_strip2);
//   }

// }

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  std::string robot_ns;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  // marker_pub2 = n.advertise<visualization_msgs::Marker>("slam_marker", 10);
  ros::Subscriber odom_sub = n.subscribe("/zhurong_mars_rover/pose", 10, &Callback);
  // ros::Subscriber slam_sub = n.subscribe("/orb_slam2_rgbd/pose", 10, &Callback2);
  
  ros::spin();
  return 0;
}

