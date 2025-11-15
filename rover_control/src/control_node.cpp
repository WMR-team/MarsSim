#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//front to back
float f2b=0.548;
//wheel r 0.137
float r=0.137;
//left to right
float l2r=0.5;

// 回调函数，处理接收到的消息并发布新消息
void messageCallback(const geometry_msgs::Twist::ConstPtr& msg,ros::Publisher* pub_lf_steer,ros::Publisher* pub_rf_steer,
     ros::Publisher* pub_lf, ros::Publisher* pub_rf, ros::Publisher* pub_lb, ros::Publisher* pub_rb) 
{
    ROS_INFO("recive:%f,%f\n", msg->linear.x,msg->angular.z);
    
    // 创建新消息
    std_msgs::Float64 msg_lf,msg_rf,msg_lb,msg_rb,msg_lf_steer,msg_rf_steer;
    float R=0;
    float vx=msg->linear.x,wz=msg->angular.z;

    //+:zuo,-:you
    if(wz==0)
    {
        msg_lf_steer.data=0;
        msg_rf_steer.data=0;
        msg_lf.data=vx/r;
        msg_rf.data=vx/r;
        msg_lb.data=vx/r;
        msg_rb.data=vx/r;
    }
    else if(abs(vx/wz)>0.8126)
    {
        R=vx/wz;
        msg_lf_steer.data=atan(f2b/(R-l2r/2));
        msg_rf_steer.data=atan(f2b/(R+l2r/2));
        msg_lf.data=abs(wz*sqrt((R-l2r/2)*(R-l2r/2)+f2b*f2b)/r)*vx/abs(vx);
        msg_rf.data=abs(wz*sqrt((R+l2r/2)*(R+l2r/2)+f2b*f2b)/r)*vx/abs(vx);
        msg_lb.data=abs(wz*(R-l2r/2)/r)*vx/abs(vx);
        msg_rb.data=abs(wz*(R+l2r/2)/r)*vx/abs(vx);
    }
    else
    {
        msg_lf_steer.data=0;
        msg_rf_steer.data=0;
        msg_lf.data=0;
        msg_rf.data=0;
        msg_lb.data=0;
        msg_rb.data=0;
        ROS_INFO("turning radius too small");
    }
        
    // 发布新消息
    pub_lf_steer->publish(msg_lf_steer);
    pub_rf_steer->publish(msg_rf_steer);
    pub_lf->publish(msg_lf);
    pub_rf->publish(msg_rf);
    pub_lb->publish(msg_lb);
    pub_rb->publish(msg_rb);

    // ROS_INFO("R:%f",R);
    ROS_INFO("pub:%f  %f\n%f  %f  %f  %f\n",
        msg_lf_steer.data, msg_rf_steer.data, msg_lf.data, msg_rf.data, msg_lb.data, msg_rb.data);
    
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "control_node");
    
    // 创建节点句柄
    ros::NodeHandle nh;
    
    ros::Publisher pub_lf_steer = nh.advertise<std_msgs::Float64>("/hunter_rover/fr_steer_left_joint_controller/command", 10);
    ros::Publisher pub_rf_steer = nh.advertise<std_msgs::Float64>("/hunter_rover/fr_steer_right_joint_controller/command", 10);
    ros::Publisher pub_lf = nh.advertise<std_msgs::Float64>("/hunter_rover/fr_left_joint_controller/command", 10);
    ros::Publisher pub_rf = nh.advertise<std_msgs::Float64>("/hunter_rover/fr_right_joint_controller/command", 10);
    ros::Publisher pub_lb = nh.advertise<std_msgs::Float64>("/hunter_rover/re_left_joint_controller/command", 10);
    ros::Publisher pub_rb = nh.advertise<std_msgs::Float64>("/hunter_rover/re_right_joint_controller/command", 10);
    
    // 创建订阅者，订阅/input_topic并绑定回调函数
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>(
        "cmd_vel", 
        10, 
        boost::bind(messageCallback, _1, &pub_lf_steer, &pub_rf_steer,&pub_lf,&pub_rf,&pub_lb,&pub_rb)
    );
    
    ROS_INFO("Init OK\n");
    
    // 进入循环处理回调
    ros::spin();
    
    return 0;
}

