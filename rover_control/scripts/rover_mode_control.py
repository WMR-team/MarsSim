#!/usr/bin/env python3
# 双模式火星车轮速控制节点
# mode=0: 行驶模式(阿克曼转向)
# mode=1: 辨识模式(6轮独立转速控制)
# 订阅话题: /rover_mode_control (RoverControlMode)

import rospy
import numpy as np
from std_msgs.msg import Float64
from rover_msgs.msg import RoverControlMode


class RoverModeControl:

    WHEEL_NAMES = [
        "front_wheel_L_joint",
        "front_wheel_R_joint",
        "middle_wheel_L_joint",
        "middle_wheel_R_joint",
        "back_wheel_L_joint",
        "back_wheel_R_joint",
    ]

    STEER_NAMES = [
        "suspension_steer_F_L_joint",
        "suspension_steer_F_R_joint",
        "suspension_steer_M_L_joint",
        "suspension_steer_M_R_joint",
        "suspension_steer_B_L_joint",
        "suspension_steer_B_R_joint",
    ]

    def __init__(self, ns="zhurong_mars_rover"):
        rospy.init_node("rover_mode_control_node", anonymous=True)
        self.ns = ns
        self.r = 0.15    # 轮子半径 m
        self.h = 0.652   # 轮距 m
        self.l = 0.775   # 轴距 m

        # 轮速发布器
        self.wheel_pubs = []
        for name in self.WHEEL_NAMES:
            topic = f"/{ns}/{name}_velocity_controller/command"
            self.wheel_pubs.append(rospy.Publisher(topic, Float64, queue_size=1))

        # 转向角发布器
        self.steer_pubs = []
        for name in self.STEER_NAMES:
            topic = f"/{ns}/{name}_position_controller/command"
            self.steer_pubs.append(rospy.Publisher(topic, Float64, queue_size=1))

        # 初始化状态
        self._stop_all()

        rospy.Subscriber("/rover_mode_control", RoverControlMode, self._callback)
        rospy.loginfo(f"[RoverModeControl] Ready. ns={ns}, topic=/rover_mode_control")

    def _stop_all(self):
        zero = Float64()
        zero.data = 0.0
        for p in self.wheel_pubs:
            p.publish(zero)
        for p in self.steer_pubs:
            p.publish(zero)

    def _callback(self, msg):
        if msg.mode == RoverControlMode.DRIVE_MODE:
            self._drive_mode(msg.speed, msg.omega)
        elif msg.mode == RoverControlMode.IDENTIFY_MODE:
            self._identify_mode(msg.wheel_speeds)
        else:
            rospy.logwarn(f"[RoverModeControl] 未知mode: {msg.mode}")

    def _drive_mode(self, speed, omega):
        # 阿克曼转向, 复用原始运动学
        if omega == 0:
            steer = np.zeros(6)
            vel = np.ones(6) * speed / self.r
        else:
            R = speed / omega
            r_arr = np.array([
                np.sqrt((R - self.h)**2 + self.l**2),
                np.sqrt((R + self.h)**2 + self.l**2),
                abs(R - self.h),
                abs(R + self.h),
                r_arr[0],
                r_arr[1],
            ])
            vel = np.sign(speed) * abs(omega) * r_arr / self.r

            steer = np.zeros(6)
            steer[0] = np.arctan(self.l / (R - self.h))
            steer[1] = np.arctan(self.l / (R + self.h))
            steer[4] = -steer[0]
            steer[5] = -steer[1]

            if 0 <= R < self.h:
                vel[0] = -vel[0]
                vel[2] = -vel[2]
                vel[4] = -vel[4]
            elif -self.h < R < 0:
                vel[1] = -vel[1]
                vel[3] = -vel[3]
                vel[5] = -vel[5]

        self._publish(vel, steer)

    def _identify_mode(self, wheel_speeds):
        # 辨识模式: 直接设置6轮独立转速, 转向角置零
        vel = np.array(wheel_speeds)
        steer = np.zeros(6)
        self._publish(vel, steer)

    def _publish(self, vel, steer):
        for i in range(6):
            msg_v = Float64()
            msg_v.data = vel[i]
            self.wheel_pubs[i].publish(msg_v)

            msg_s = Float64()
            msg_s.data = steer[i]
            self.steer_pubs[i].publish(msg_s)


if __name__ == "__main__":
    RoverModeControl("zhurong_mars_rover")
    rospy.spin()
