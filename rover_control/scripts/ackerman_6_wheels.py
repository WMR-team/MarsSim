#! /usr/bin/env python3

import time
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool, SetBoolResponse


class ZhurongMarsRoverControl(object):
    def __init__(self, NameSpace = ""):		
     
        rospy.init_node("zhurong_control_node", anonymous=True)
        self.rate = rospy.Rate(100.0)
        self.pub = rospy.Publisher('/gazebo/wheel_cmd', Float64, queue_size=10)
        rospy.loginfo("ZhurongRoverControl Initialising...")
        self.control_msg=Float64()

        self.h = 0.652  # 1/2 width
        self.l = 0.775  # 1/2 length
        self.r = 0.15
  
        self.cam_pitch = 0
        self.cam_yaw = 0
        self.zhurong_publishers = {}
        self.controller_ns = NameSpace
        self.controller_command = "command"
        self.controllers_list = [   
                                    "front_wheel_L_joint_velocity_controller",
                                    "front_wheel_R_joint_velocity_controller",
                                    "middle_wheel_L_joint_velocity_controller",
                                    "middle_wheel_R_joint_velocity_controller",
                                     "back_wheel_L_joint_velocity_controller",
                                    "back_wheel_R_joint_velocity_controller",
                                    # "suspension_arm_B2_L_joint_position_controller",
                                    # "suspension_arm_B2_R_joint_position_controller",
                                    "suspension_arm_B_L_joint_position_controller",
                                    "suspension_arm_B_R_joint_position_controller",
                                    "suspension_arm_F_L_joint_position_controller",
                                    "suspension_arm_F_R_joint_position_controller",
                                    
                                    "suspension_steer_F_L_joint_position_controller",
                                    "suspension_steer_F_R_joint_position_controller",
                                    "suspension_steer_M_L_joint_position_controller",
                                    "suspension_steer_M_R_joint_position_controller",
                                    "suspension_steer_B_L_joint_position_controller",
                                    "suspension_steer_B_R_joint_position_controller",
         
                                    "PTZYaw_joint_position_controller",
                                    "PTZPitch_joint_position_controller"
                                ]

        for controller_name in self.controllers_list:
            if len(self.controller_ns) > 0:
                topic_name = "/"+self.controller_ns+"/"+controller_name+"/"+self.controller_command
            else:
                topic_name = "/"+controller_name+"/"+self.controller_command
            self.zhurong_publishers[controller_name] = rospy.Publisher(
                topic_name,
                Float64,
                queue_size=1)
    
        self.cmd_vel_msg = TwistStamped()
        cmd_vel_topic = "/mars_environment/cmd_vel"
        rospy.Subscriber(cmd_vel_topic, TwistStamped, self.cmd_vel_callback)
        rospy.Subscriber("/wheel_LF_cmd", TwistStamped, self.wheel_LF_cmd_callback)
        rospy.Subscriber("/wheel_RF_cmd", TwistStamped, self.wheel_RF_cmd_callback)
        rospy.Subscriber("/wheel_LM_cmd", TwistStamped, self.wheel_LM_cmd_callback)
        rospy.Subscriber("/wheel_RM_cmd", TwistStamped, self.wheel_RM_cmd_callback)
        rospy.Subscriber("/wheel_LB_cmd", TwistStamped, self.wheel_LB_cmd_callback)
        rospy.Subscriber("/wheel_RB_cmd", TwistStamped, self.wheel_RB_cmd_callback)
        rospy.Subscriber('/mars_environment/cam_ctl', Float64, self.cam_ctl_callback)
        rospy.Service('/init_controller', SetBool, self.init_response)
        self.control_msg.data=0
        self.pub.publish(self.control_msg)
        
        self.init_publisher_variables()
        self.wait_publishers_to_be_ready()		
        self.init_state()
  
        rospy.logwarn("ZhurongMarsRoverControl...READY")

    def cmd_vel_callback(self, msg):
        # print('received!!!')
        self.body_velocity = msg.twist.linear.x
        self.body_omega = msg.twist.angular.z
        self.move_with_cmd_vel()
        
    def wheel_LF_cmd_callback(self, msg):
        self.wheel_velocity_msg[0].data = msg.twist.linear.x
        self.wheel_publisher[0].publish(self.wheel_velocity_msg[0])
        self.wheel_steer_msg[0].data = msg.twist.angular.z
        self.steer_publisher[0].publish(self.wheel_steer_msg[0])
        
    def wheel_RF_cmd_callback(self, msg):
        self.wheel_velocity_msg[1].data = msg.twist.linear.x
        self.wheel_publisher[1].publish(self.wheel_velocity_msg[1])
        self.wheel_steer_msg[1].data = msg.twist.angular.z
        self.steer_publisher[1].publish(self.wheel_steer_msg[1])
        
    def wheel_LM_cmd_callback(self, msg):
        self.wheel_velocity_msg[2].data = msg.twist.linear.x
        self.wheel_publisher[2].publish(self.wheel_velocity_msg[2])
        self.wheel_steer_msg[2].data = msg.twist.angular.z
        self.steer_publisher[2].publish(self.wheel_steer_msg[2])
        
    def wheel_RM_cmd_callback(self, msg):
        self.wheel_velocity_msg[3].data = msg.twist.linear.x
        self.wheel_publisher[3].publish(self.wheel_velocity_msg[3])
        self.wheel_steer_msg[3].data = msg.twist.angular.z
        self.steer_publisher[3].publish(self.wheel_steer_msg[3])
        
    def wheel_LB_cmd_callback(self, msg):
        self.wheel_velocity_msg[4].data = msg.twist.linear.x
        self.wheel_publisher[4].publish(self.wheel_velocity_msg[4])
        self.wheel_steer_msg[4].data = msg.twist.angular.z
        self.steer_publisher[4].publish(self.wheel_steer_msg[4])
        
    def wheel_RB_cmd_callback(self, msg):
        self.wheel_velocity_msg[5].data = msg.twist.linear.x
        self.wheel_publisher[5].publish(self.wheel_velocity_msg[5])
        self.wheel_steer_msg[5].data = msg.twist.angular.z
        self.steer_publisher[5].publish(self.wheel_steer_msg[5])

    def cam_ctl_callback(self, msg):
        self.cam_yaw = msg.data
        self.set_navcam_angle()
    
    def init_response(self, request):
        # self.init_publisher_variables()
        self.init_state()
        self.control_msg.data=0
        self.pub.publish(self.control_msg)
        # print('init success!!!!')
        return SetBoolResponse(success = True, message = 'initial rover controllers!')

    def wait_publishers_to_be_ready(self):

        rate_wait = rospy.Rate(10)
        for controller_name, publisher_obj in self.zhurong_publishers.items():
            publisher_ready = False
            while not publisher_ready:
                rospy.logwarn("Checking Publisher for ==>"+str(controller_name))
                pub_num = publisher_obj.get_num_connections()
                publisher_ready = (pub_num > 0)
                rate_wait.sleep()
            rospy.loginfo("Publisher ==>" + str(controller_name) + "...READY")

    def init_publisher_variables(self):
        """
        We create variables for more pythonic access access to publishers
        and not need to access any more
        :return:
        """
        # Get the publishers for wheel speed
        
        self.wheel_publisher = []
        self.steer_publisher = []
        for i in range(6):
            self.wheel_publisher.append(self.zhurong_publishers[self.controllers_list[i]])
            self.steer_publisher.append(self.zhurong_publishers[self.controllers_list[-8+i]])
        # Get the publishers for suspension
        # self.suspension_arm_B2_L = self.zhurong_publishers[self.controllers_list[6]]
        # self.suspension_arm_B2_R = self.zhurong_publishers[self.controllers_list[7]]
        self.suspension_arm_B_L = self.zhurong_publishers[self.controllers_list[6]]
        self.suspension_arm_B_R = self.zhurong_publishers[self.controllers_list[7]]
        self.suspension_arm_F_L = self.zhurong_publishers[self.controllers_list[8]]
        self.suspension_arm_F_R = self.zhurong_publishers[self.controllers_list[9]]
  
        self.cam_yaw_ctl_publisher = self.zhurong_publishers[self.controllers_list[-2]]
        self.cam_pitch_ctl_publisher = self.zhurong_publishers[self.controllers_list[-1]]

        # Init Messages
        self.wheel_velocity_msg=[]
        self.wheel_steer_msg=[]
        for i in range(6):
            self.wheel_velocity_msg.append(Float64())
            self.wheel_steer_msg.append(Float64())

        # self.suspension_arm_B2_L_pos_msg = Float64()
        # self.suspension_arm_B2_R_pos_msg = Float64()
        self.suspension_arm_B_L_pos_msg = Float64()
        self.suspension_arm_B_R_pos_msg = Float64()
        self.suspension_arm_F_L_pos_msg = Float64()
        self.suspension_arm_F_R_pos_msg = Float64()
  
        self.cam_yaw_msg = Float64()
        self.cam_pitch_msg = Float64()

    def init_state(self):
        self.set_suspension_mode("standard")
        self.set_turning_radius(np.zeros(6))
        self.set_wheels_speed(np.zeros(6))
        self.set_navcam_angle()

    def set_suspension_mode(self, mode_name):

        if mode_name == "standard":

            # self.suspension_arm_B2_L_pos_msg.data = -0
            # self.suspension_arm_B2_R_pos_msg.data = -0
            self.suspension_arm_B_L_pos_msg.data = -0
            self.suspension_arm_B_R_pos_msg.data = -0
            self.suspension_arm_F_L_pos_msg.data = 0
            self.suspension_arm_F_R_pos_msg.data = 0

            # self.suspension_arm_B2_L.publish(self.suspension_arm_B2_L_pos_msg)
            # self.suspension_arm_B2_R.publish(self.suspension_arm_B2_R_pos_msg)
            self.suspension_arm_B_L.publish(self.suspension_arm_B_L_pos_msg)
            self.suspension_arm_B_R.publish(self.suspension_arm_B_R_pos_msg)
            self.suspension_arm_F_L.publish(self.suspension_arm_F_L_pos_msg)
            self.suspension_arm_F_R.publish(self.suspension_arm_F_R_pos_msg)

    def set_turning_radius(self, turn_radius):
        for i in range(6):
            self.wheel_steer_msg[i].data = turn_radius[i]
            self.steer_publisher[i].publish(self.wheel_steer_msg[i])

    def set_wheels_speed(self, turning_speed):
        """
        Sets the turning speed in radians per second
        :param turning_speed: In radians per second
        :return:
        """
        # TODO: turning_speed for each wheel should change based on ackerman.
        for i in range(6):
            self.wheel_velocity_msg[i].data = turning_speed[i]
            self.wheel_publisher[i].publish(self.wheel_velocity_msg[i])
   
    def set_navcam_angle(self):
        
        self.cam_yaw_msg.data = self.cam_yaw
        self.cam_pitch_msg.data = self.cam_pitch
        self.cam_yaw_ctl_publisher.publish(self.cam_yaw_msg.data)
        self.cam_pitch_ctl_publisher.publish(self.cam_pitch_msg.data)

    def move_forwards(self):
        self.body_velocity = 0.3
        self.body_omega = 0
        self.move_with_cmd_vel()
        print('forward')

    def move_backwards(self):
        self.body_velocity = -0.3
        self.body_omega = 0
        self.move_with_cmd_vel()
        print('backward')

    def move_slow_forwards(self):
        self.body_velocity = 0.1
        self.body_omega = 0
        self.move_with_cmd_vel()
        print('slow forward')
  
    def move_slow_backwards(self):
        self.body_velocity = -0.1
        self.body_omega = 0
        self.move_with_cmd_vel()
        print('slow backward')

    def move_turn_left(self):
        self.body_velocity = 0.3
        self.body_omega = 0.08
        self.move_with_cmd_vel()

    def move_turn_right(self):
        self.body_velocity = 0.3
        self.body_omega = -0.08
        self.move_with_cmd_vel()

    def move_turn_stop(self):
        self.body_velocity = 0
        self.body_omega = 0
        self.move_with_cmd_vel()
  
    def cam_pitch_ctl_1(self):
        self.cam_pitch += 0.1
        self.set_navcam_angle()

    def cam_pitch_ctl_2(self):
        self.cam_pitch -= 0.1
        self.set_navcam_angle()

    def cam_yaw_ctl_1(self):
        self.cam_yaw += 0.1
        self.set_navcam_angle()
  
    def cam_yaw_ctl_2(self):
        self.cam_yaw -= 0.1
        self.set_navcam_angle()
 
    def move_with_cmd_vel(self):
        if self.body_omega == 0:
            theta = np.zeros(6)
            self.set_turning_radius(theta)
            vel_arr = np.ones(6) * self.body_velocity/self.r
            self.set_wheels_speed(vel_arr)
        else:
            turning_radius = self.body_velocity/self.body_omega
            r_arr = np.zeros(6) 
            r_arr[0] = np.sqrt((turning_radius-self.h)**2+self.l**2)
            r_arr[1] = np.sqrt((turning_radius+self.h)**2+self.l**2)
            r_arr[2] = abs(turning_radius-self.h)
            r_arr[3] = abs(turning_radius+self.h)
            r_arr[4] = r_arr[0]
            r_arr[5] = r_arr[1]
            vel_arr = abs(self.body_omega) * r_arr / self.r
            if self.body_velocity < 0:
                vel_arr = -vel_arr
            elif self.body_velocity == 0 and self.body_omega < 0:
                vel_arr = -vel_arr
            # print(vel_arr)

            theta = np.zeros(6)
            theta[0] = np.arctan(self.l/(turning_radius-self.h))
            theta[1] = np.arctan(self.l/(turning_radius+self.h))
            theta[2] = 0
            theta[3] = 0
            theta[4] = -theta[0]
            theta[5] = -theta[1]
            
            if turning_radius>=0 and abs(turning_radius)<self.h:
                vel_arr[0] = -vel_arr[0]
                vel_arr[2] = -vel_arr[2]
                vel_arr[4] = -vel_arr[4]

            elif turning_radius<0 and abs(turning_radius)<self.h:
                vel_arr[1] = -vel_arr[1]
                vel_arr[3] = -vel_arr[3]
                vel_arr[5] = -vel_arr[5]

            # print(theta)

            self.set_turning_radius(theta)
            self.set_wheels_speed(vel_arr)
    
    def wait_for_keyboard_ctl(self, x):

        if(x=='w'):
            self.move_forwards()      
            self.control_msg.data=2
            self.pub.publish(self.control_msg)
        elif(x=='s'):
            self.move_backwards()
            self.control_msg.data=-2
            self.pub.publish(self.control_msg)
        elif(x=='a'):
            self.move_turn_left()
            self.control_msg.data=2
            self.pub.publish(self.control_msg)
        elif(x=='d'):
            self.move_turn_right()
            self.control_msg.data=2
            self.pub.publish(self.control_msg)
        elif(x=='p'):
            self.move_turn_stop()
            self.control_msg.data=0
            self.pub.publish(self.control_msg)
        elif(x=='k'):
            self.move_slow_forwards()
            self.control_msg.data=0.6
            self.pub.publish(self.control_msg)
        elif(x=='l'):
            self.move_slow_backwards()
            self.control_msg.data=-0.6
            self.pub.publish(self.control_msg)
        elif(x=='z'):
            self.cam_pitch_ctl_1()
        elif(x=='x'):
            self.cam_pitch_ctl_2()

        elif(x=='c'):
            self.cam_yaw_ctl_1()
        elif(x=='v'):
            self.cam_yaw_ctl_2()
            
        # self.rate.sleep()



if __name__ == "__main__":
    
    zhurong_mars_rover_control = ZhurongMarsRoverControl()
    # rover2_control = ZhurongMarsRoverControl("rover_2")
    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        x=input()
        zhurong_mars_rover_control.wait_for_keyboard_ctl(x)
        # rover2_control.wait_for_keyboard_ctl(x)
        rate.sleep()
