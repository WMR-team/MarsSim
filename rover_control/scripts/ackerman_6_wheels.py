#! /usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse


class ZhurongMarsRoverControl(object):
    HALF_WIDTH = 0.652  # 1/2 width
    HALF_LENGTH = 0.775  # 1/2 length
    WHEEL_RADIUS = 0.15

    def __init__(self, NameSpace=""):

        rospy.init_node("zhurong_control_node", anonymous=True)
        self.controller_ns = NameSpace
        self.rate = rospy.Rate(100.0)
        rospy.loginfo("ZhurongRoverControl Initialising...")

        self.h = self.HALF_WIDTH
        self.l = self.HALF_LENGTH
        self.r = self.WHEEL_RADIUS

        # defined to track camera state
        self.cam_pitch = 0
        self.cam_yaw = 0

        self._setup_ros_interfaces()
        self._reset_to_initial_state()

        rospy.loginfo("ZhurongMarsRoverControl...READY")

    def _setup_ros_interfaces(self):
        self.init_publishers()
        self.wait_publishers_to_be_ready()
        self.init_msgs()  # TODO: do I need to init msgs? I can directly create msg and publish without init.
        self.init_subscribers()

    def _reset_to_initial_state(self):
        self.set_suspension_mode("standard")
        self.set_turning_radius(np.zeros(6))
        self.set_wheels_speed(np.zeros(6))
        self.set_navcam_angle(self.cam_pitch, self.cam_yaw)

    def init_subscribers(self):
        rospy.Subscriber("/wheel_LF_cmd", Twist, lambda msg: self.wheel_cmd_callback(msg, 0))
        rospy.Subscriber("/wheel_RF_cmd", Twist, lambda msg: self.wheel_cmd_callback(msg, 1))
        rospy.Subscriber("/wheel_LM_cmd", Twist, lambda msg: self.wheel_cmd_callback(msg, 2))
        rospy.Subscriber("/wheel_RM_cmd", Twist, lambda msg: self.wheel_cmd_callback(msg, 3))
        rospy.Subscriber("/wheel_LB_cmd", Twist, lambda msg: self.wheel_cmd_callback(msg, 4))
        rospy.Subscriber("/wheel_RB_cmd", Twist, lambda msg: self.wheel_cmd_callback(msg, 5))

        cmd_vel_topic = "/mars_environment/cmd_vel"  # TODO: move this topic name to a parameter server or launch file     
        rospy.Subscriber(cmd_vel_topic, Twist, self.cmd_vel_callback)
        cmd_camera_yaw_topic = "/mars_environment/cam_yaw_ctl"  # TODO: move this topic name to a parameter server or launch file
        rospy.Subscriber(cmd_camera_yaw_topic, Float64, self.cam_yaw_callback)
        cmd_camera_pitch_topic = "/mars_environment/cam_pitch_ctl"  # TODO: move this topic name to a parameter server or launch file
        rospy.Subscriber(cmd_camera_pitch_topic, Float64, self.cam_pitch_callback)
    
    def init_publishers(self):
        """
        We create variables for more pythonic access access to publishers
        and not need to access any more
        :return:
        """

        # Get the publishers for wheel speed
        self.wheel_controller_list = [
            "front_wheel_L_joint_velocity_controller",
            "front_wheel_R_joint_velocity_controller",
            "middle_wheel_L_joint_velocity_controller",
            "middle_wheel_R_joint_velocity_controller",
            "back_wheel_L_joint_velocity_controller",
            "back_wheel_R_joint_velocity_controller",
        ]
        self.wheel_publishers = [
            rospy.Publisher(self._assemble_topic_name(controller_name), Float64, queue_size=1)
            for controller_name in self.wheel_controller_list
        ]

        self.steer_controller_list = [
            "suspension_steer_F_L_joint_position_controller",
            "suspension_steer_F_R_joint_position_controller",
            "suspension_steer_M_L_joint_position_controller",
            "suspension_steer_M_R_joint_position_controller",
            "suspension_steer_B_L_joint_position_controller",
            "suspension_steer_B_R_joint_position_controller",
        ]
        self.steer_publishers = [
            rospy.Publisher(self._assemble_topic_name(controller_name), Float64, queue_size=1)
            for controller_name in self.steer_controller_list
        ]

        # Get the publishers for suspension
        self.suspension_controller_list = [
            # "suspension_arm_B2_L_joint_position_controller",
            # "suspension_arm_B2_R_joint_position_controller",
            "suspension_arm_B_L_joint_position_controller",
            "suspension_arm_B_R_joint_position_controller",
            "suspension_arm_F_L_joint_position_controller",
            "suspension_arm_F_R_joint_position_controller",
        ]
        # self.suspension_arm_B2_L = rospy.Publisher(self._assemble_topic_name(self.suspension_controller_list[-6]), Float64, queue_size=1)
        # self.suspension_arm_B2_R = rospy.Publisher(self._assemble_topic_name(self.suspension_controller_list[-5]), Float64, queue_size=1)
        self.suspension_arm_B_L = rospy.Publisher(self._assemble_topic_name(self.suspension_controller_list[-4]), Float64, queue_size=1)
        self.suspension_arm_B_R = rospy.Publisher(self._assemble_topic_name(self.suspension_controller_list[-3]), Float64, queue_size=1)
        self.suspension_arm_F_L = rospy.Publisher(self._assemble_topic_name(self.suspension_controller_list[-2]), Float64, queue_size=1)
        self.suspension_arm_F_R = rospy.Publisher(self._assemble_topic_name(self.suspension_controller_list[-1]), Float64, queue_size=1)
        self.suspension_publishers = [
            self.suspension_arm_B_L,
            self.suspension_arm_B_R,
            self.suspension_arm_F_L,
            self.suspension_arm_F_R,
        ]

        # Get the publisher for navigation camera
        self.camera_controller_list = [
            "PTZYaw_joint_position_controller",
            "PTZPitch_joint_position_controller",
        ]
        self.cam_yaw_ctl_publisher = rospy.Publisher(self._assemble_topic_name(self.camera_controller_list[-2]), Float64, queue_size=1)
        self.cam_pitch_ctl_publisher = rospy.Publisher(self._assemble_topic_name(self.camera_controller_list[-1]), Float64, queue_size=1)
        self.camera_publishers = [
            self.cam_yaw_ctl_publisher,
            self.cam_pitch_ctl_publisher,
        ]

    def _assemble_topic_name(self, controller_name):
        self.controller_command = "command"

        if len(self.controller_ns) > 0:
            topic_name = (
                "/"
                + self.controller_ns
                + "/"
                + controller_name
                + "/"
                + self.controller_command
            )
        else:
            topic_name = (
                "/" + controller_name + "/" + self.controller_command
            )
        return topic_name
    
    def init_msgs(self):

        self.wheel_velocity_msg = [Float64() for _ in range(6)]
        self.wheel_steer_msg = [Float64() for _ in range(6)]

        # self.suspension_arm_B2_L_pos_msg = Float64()
        # self.suspension_arm_B2_R_pos_msg = Float64()
        self.suspension_arm_B_L_pos_msg = Float64()
        self.suspension_arm_B_R_pos_msg = Float64()
        self.suspension_arm_F_L_pos_msg = Float64()
        self.suspension_arm_F_R_pos_msg = Float64()

        self.cam_yaw_msg = Float64()
        self.cam_pitch_msg = Float64()

    def wait_publishers_to_be_ready(self):
        rate_wait = rospy.Rate(10)
        for publisher_obj in self.wheel_publishers + self.steer_publishers + self.suspension_publishers + self.camera_publishers:
            publisher_ready = False
            while not publisher_ready:
                rospy.loginfo(
                    "Checking Publisher for ==>" + str(publisher_obj.resolved_name)
                )
                pub_num = publisher_obj.get_num_connections()
                publisher_ready = pub_num > 0
                rate_wait.sleep()
            rospy.loginfo("Publisher ==>" + str(publisher_obj.resolved_name) + "...READY")
    
    def cmd_vel_callback(self, msg: Twist):
        rospy.logdebug('cmd vel received!')
        self.body_velocity = msg.linear.x
        self.body_omega = msg.angular.z
        self.move_with_cmd_vel()

    def wheel_cmd_callback(self, msg: Twist, wheel_index):
        self.wheel_velocity_msg[wheel_index].data = msg.linear.x
        self.wheel_publishers[wheel_index].publish(self.wheel_velocity_msg[wheel_index])
        self.wheel_steer_msg[wheel_index].data = msg.angular.z
        self.steer_publishers[wheel_index].publish(self.wheel_steer_msg[wheel_index])

    def cam_yaw_callback(self, msg):
        self.cam_yaw = msg.data
        self.set_navcam_angle(self.cam_pitch, self.cam_yaw)

    def cam_pitch_callback(self, msg):
        self.cam_pitch = msg.data
        self.set_navcam_angle(self.cam_pitch, self.cam_yaw)

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

        else:
            rospy.logwarn("Unsupported suspension mode: %s", mode_name)

    def set_turning_radius(self, turn_radius):
        for i in range(6):
            self.wheel_steer_msg[i].data = turn_radius[i]
            self.steer_publishers[i].publish(self.wheel_steer_msg[i])

    def set_wheels_speed(self, turning_speed):
        """
        Sets the turning speed in radians per second
        :param turning_speed: In radians per second
        :return:
        """
        # TODO: turning_speed for each wheel should change based on ackerman.
        for i in range(6):
            self.wheel_velocity_msg[i].data = turning_speed[i]
            self.wheel_publishers[i].publish(self.wheel_velocity_msg[i])

    def set_navcam_angle(self, pitch=0, yaw=0):
        """
        Set the navigation camera angle by publishing yaw and pitch control messages.

        This method updates the camera's yaw and pitch angles by packaging the current
        yaw and pitch values into ROS messages and publishing them to their respective
        control topics. The actual camera movement is handled by the subscribers to
        these published topics.

        Returns:
            None
        """

        self.cam_pitch_msg.data = pitch
        self.cam_yaw_msg.data = yaw
        self.cam_yaw_ctl_publisher.publish(self.cam_yaw_msg.data)
        self.cam_pitch_ctl_publisher.publish(self.cam_pitch_msg.data)

    def move_with_cmd_vel(self):
        if self.body_omega == 0:
            theta = np.zeros(6)
            self.set_turning_radius(theta)
            vel_arr = np.ones(6) * self.body_velocity / self.r
            self.set_wheels_speed(vel_arr)
        else:
            turning_radius = self.body_velocity / self.body_omega
            r_arr = np.zeros(6)
            r_arr[0] = np.sqrt((turning_radius - self.h) ** 2 + self.l**2)
            r_arr[1] = np.sqrt((turning_radius + self.h) ** 2 + self.l**2)
            r_arr[2] = abs(turning_radius - self.h)
            r_arr[3] = abs(turning_radius + self.h)
            r_arr[4] = r_arr[0]
            r_arr[5] = r_arr[1]
            vel_arr = abs(self.body_omega) * r_arr / self.r
            if self.body_velocity < 0:
                vel_arr = -vel_arr
            elif self.body_velocity == 0 and self.body_omega < 0:
                vel_arr = -vel_arr
            rospy.logdebug(vel_arr)

            theta = np.zeros(6)
            theta[0] = np.arctan(self.l / (turning_radius - self.h))
            theta[1] = np.arctan(self.l / (turning_radius + self.h))
            theta[2] = 0
            theta[3] = 0
            theta[4] = -theta[0]
            theta[5] = -theta[1]
            rospy.logdebug(theta)

            if turning_radius >= 0 and abs(turning_radius) < self.h:
                vel_arr[0] = -vel_arr[0]
                vel_arr[2] = -vel_arr[2]
                vel_arr[4] = -vel_arr[4]

            elif turning_radius < 0 and abs(turning_radius) < self.h:
                vel_arr[1] = -vel_arr[1]
                vel_arr[3] = -vel_arr[3]
                vel_arr[5] = -vel_arr[5]

            self.set_turning_radius(theta)
            self.set_wheels_speed(vel_arr)

if __name__ == "__main__":
    ZhurongMarsRoverControl()
    rospy.spin()
