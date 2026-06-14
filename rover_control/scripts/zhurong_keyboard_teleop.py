#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class ZhurongKeyboardTeleop(object):
    LINEAR_SPEED = 0.3
    SLOW_LINEAR_SPEED = 0.1
    TURN_RATE = 0.08
    CAMERA_STEP = 0.1

    def __init__(self):
        rospy.init_node("zhurong_keyboard_teleop", anonymous=True)

        self.cmd_vel_publisher = rospy.Publisher(
            "/mars_environment/cmd_vel", Twist, queue_size=1
        )
        self.wheel_cmd_publisher = rospy.Publisher(
            "/gazebo/wheel_cmd", Float64, queue_size=1
        )
        self.cam_yaw_publisher = rospy.Publisher(
            "/mars_environment/cam_ctl", Float64, queue_size=1
        )
        self.cam_pitch_publisher = rospy.Publisher(
            "/mars_environment/cam_pitch_ctl", Float64, queue_size=1
        )

        self.cam_yaw = 0.0
        self.cam_pitch = 0.0

        rospy.loginfo("Zhurong keyboard teleop initialized.")

    def _publish_motion(self, linear_x, angular_z, wheel_cmd):
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_x
        cmd_msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(cmd_msg)

        wheel_msg = Float64()
        wheel_msg.data = wheel_cmd
        self.wheel_cmd_publisher.publish(wheel_msg)

    def _publish_camera_state(self):
        yaw_msg = Float64()
        yaw_msg.data = self.cam_yaw
        self.cam_yaw_publisher.publish(yaw_msg)

        pitch_msg = Float64()
        pitch_msg.data = self.cam_pitch
        self.cam_pitch_publisher.publish(pitch_msg)

    def _handle_key(self, key):
        if key == "w":
            self._publish_motion(self.LINEAR_SPEED, 0.0, 2.0)
        elif key == "s":
            self._publish_motion(-self.LINEAR_SPEED, 0.0, -2.0)
        elif key == "a":
            self._publish_motion(self.LINEAR_SPEED, self.TURN_RATE, 2.0)
        elif key == "d":
            self._publish_motion(self.LINEAR_SPEED, -self.TURN_RATE, 2.0)
        elif key == "p":
            self._publish_motion(0.0, 0.0, 0.0)
        elif key == "k":
            self._publish_motion(self.SLOW_LINEAR_SPEED, 0.0, 0.6)
        elif key == "l":
            self._publish_motion(-self.SLOW_LINEAR_SPEED, 0.0, -0.6)
        elif key == "z":
            self.cam_pitch += self.CAMERA_STEP
            self._publish_camera_state()
        elif key == "x":
            self.cam_pitch -= self.CAMERA_STEP
            self._publish_camera_state()
        elif key == "c":
            self.cam_yaw += self.CAMERA_STEP
            self._publish_camera_state()
        elif key == "v":
            self.cam_yaw -= self.CAMERA_STEP
            self._publish_camera_state()
        else:
            rospy.logwarn("Unsupported key: %s", key)

    def spin(self):
        rospy.loginfo(
            "Keyboard teleop ready: w/s/a/d move, p stop, k/l slow move, z/x pitch, c/v yaw"
        )

        while not rospy.is_shutdown():
            try:
                key = input().strip().lower()
            except EOFError:
                rospy.loginfo("Keyboard teleop input closed.")
                break

            if not key:
                continue

            self._handle_key(key[0])


if __name__ == "__main__":
    ZhurongKeyboardTeleop().spin()