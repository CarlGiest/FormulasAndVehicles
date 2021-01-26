#!/usr/bin/env python
import rospy
import threading
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Pose, Vector3
import numpy as np


class rotControlNode():
    def __init__(self):
        rospy.init_node("rot_control")

        self.data_lock = threading.RLock()
        self.strategy = "search"

        self.search_thrust = 0.3

        self.roll_thrust = 0.0
        self.pitch_thrust = 0.0
        self.yaw_thrust = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.yaw_setpoint = 0.5 * np.pi
        self.pitch_setpoint = 0.0
        self.roll_setpoint = 0.0

        self.tags_x = 1.0
        self.tags_z = -0.5
        self.tags_y = 3.35

        self.rot_p_gain = 0.085
        self.rot_i_gain = 0.0
        self.rot_d_gain = 0.0

        self.roll_pub = rospy.Publisher("roll",
                                        Float64,
                                        queue_size=1)
        self.pitch_pub = rospy.Publisher("pitch",
                                         Float64,
                                         queue_size=1)
        self.yaw_pub = rospy.Publisher("yaw",
                                       Float64,
                                       queue_size=1)

        self.depth_sub = rospy.Subscriber("/nav_controller/angle",
                                          Vector3,
                                          self.rot_callback,
                                          queue_size=1)

        self.strategy_sub = rospy.Subscriber("strategy",
                                             String,
                                             self.strategy_callback,
                                             queue_size=1)

        self.depth_sub = rospy.Subscriber("robot_pos",
                                          Pose,
                                          self.pos_callback,
                                          queue_size=1)

        # Reconfigure Options via subscriber
        self.search_thrust_sub = rospy.Subscriber("search_thrust",
                                                  Float64,
                                                  self.search_thrust_callback,
                                                  queue_size=1)

    def publish(self):
        # rospy.loginfo("Hallo")
        msg_roll = Float64()
        msg_roll.data = self.roll_thrust
        self.roll_pub.publish(msg_roll)

        msg_pitch = Float64()
        msg_pitch.data = self.pitch_thrust
        self.pitch_pub.publish(msg_pitch)

        msg_yaw = Float64()
        msg_yaw.data = self.yaw_thrust
        self.yaw_pub.publish(msg_yaw)

    def strategy_callback(self, msg):
        self.strategy = msg.data
        self.control()

    def pos_callback(self, msg):
        self.tags_x = 1.0
        self.tags_z = -0.28 - 0.2
        self.yaw_setpoint = 0.5 * np.pi - np.arctan((msg.position.x - self.tags_x) /
                                                    (msg.position.y - self.tags_y))
        # rospy.logwarn(self.yaw_setpoint)
        self.yaw_setpoint = 0.5 * np.pi
        self.pitch_setpoint = -np.arctan((msg.position.z - self.tags_z) /
                                         (msg.position.y - self.tags_y))
        self.pitch_setpoint = 0.0

    def rot_callback(self, msg):
        self.roll = msg.x
        self.pitch = msg.y
        self.yaw = msg.z
        # rospy.loginfo(self.yaw)
        self.control()

    def search_thrust_callback(self, msg):
        with self.data_lock:
            if msg.data >= 0:
                self.search_thrust = msg.data
            else:
                rospy.logwarn("Received negative turning thrust")
            self.control()

    # might be redundant if only self.run is used
    def control(self):
        if self.strategy == "search":
            # rospy.logwarn("searching")
            self.roll_thrust = 0.0
            self.pitch_thrust = 0.0
            self.yaw_thrust = self.search_thrust
            self.yaw_thrust = (self.rot_p_gain * (self.yaw_setpoint - self.yaw))
            self.publish()
            return
        elif self.strategy == "follow":
            # rospy.logwarn("following")
            self.roll_thrust = (self.rot_p_gain * (self.roll_setpoint - self.roll))
            self.pitch_thrust = (self.rot_p_gain * (self.pitch_setpoint - self.pitch))
            self.yaw_thrust = (self.rot_p_gain * (self.yaw_setpoint - self.yaw))
            self.publish()
        else:
            return

    def run(self):
        rospy.spin()


def main():
    node = rotControlNode()
    node.run()


if __name__ == "__main__":
    main()
