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
        self.search_thrust = 0.09
        self.dirLock = False

        self.yaw_thrust = 0.0
        self.yaw = 0.0
        self.yaw_setpoint = 0.0

        self.rot_p_gain = 0.08
        self.rot_i_gain = 0.0
        self.rot_d_gain = 0.0

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

        # Reconfigure Options via subscriber
        self.search_thrust_sub = rospy.Subscriber("search_thrust",
                                                  Float64,
                                                  self.search_thrust_callback,
                                                  queue_size=1)

        self.rot_control_frequency = 20.0
        rospy.Timer(rospy.Duration(1.0/self.rot_control_frequency), self.rot_control)

    def strategy_callback(self, msg):
        self.strategy = msg.data
        if self.strategy != "search" and not self.dirLock:
            if self.yaw <= 0.1:
                self.yaw_setpoint = 0.0
            elif 0.1 < self.yaw <= np.pi/2:
                self.yaw_setpoint = 1.5707
            elif np.pi/2 < self.yaw <= np.pi:
                self.yaw_setpoint = np.pi
            elif np.pi < self.yaw <= 3*np.pi/2:
                self.yaw_setpoint = 3*np.pi/2
            else:
                self.yaw_setpoint = 0.0
            self.dirLock = True

    def rot_callback(self, msg):
        self.roll = msg.x
        self.pitch = msg.y
        self.yaw = msg.z

    def search_thrust_callback(self, msg):
        with self.data_lock:
            self.search_thrust = msg.data

    # might be redundant if only self.run is used
    def rot_control(self, *args):
        rospy.loginfo(self.yaw_setpoint)
        if not self.dirLock:
            self.roll_thrust = 0.0
            self.pitch_thrust = 0.0
            self.yaw_thrust = self.search_thrust
        else:
            self.yaw_thrust = (self.rot_p_gain * (self.yaw_setpoint - self.yaw))
        self.publish()

    def publish(self):
        msg_yaw = Float64()
        msg_yaw.data = self.yaw_thrust
        self.yaw_pub.publish(msg_yaw)

    def run(self):
        rospy.spin()


def main():
    node = rotControlNode()
    node.run()


if __name__ == "__main__":
    main()