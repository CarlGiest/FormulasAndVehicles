#!/usr/bin/env python
import rospy
import threading
from std_msgs.msg import Float64, Int16
from geometry_msgs.msg import Vector3, Pose
from nav_msgs.msg import Odometry
import numpy as np


class NavigationNode():
    def __init__(self):
        rospy.init_node("navigationNode")
        self.data_lock = threading.RLock()

        self._useGT = rospy.get_param("~useGT", False)

        self.ring_sub = rospy.Subscriber("ring_pos",
                                       Pose,
                                       self.ringCallback,
                                       queue_size=1)
        # use ground truth localization or not
        if not self._useGT:                              
            self.pos_sub = rospy.Subscriber("robot_pos",
                                            Pose,
                                            self.posCallback,
                                            queue_size=1)
        else:
            self.gt_sub = rospy.Subscriber("/ground_truth/state",
                                           Odometry,
                                           self.gtCallback,
                                           queue_size=1)
        self.num_tag = rospy.Subscriber("number_tags", 
                                        Int16,
                                        self.numTagsCallback,
                                        queue_size=1)
        self.setpoint_pub = rospy.Publisher("pos_setpoint",
                                            Pose,
                                            queue_size=1)

        self.x = np.array([0.7, 2.0, -0.5])
        self.xr = np.zeros(3)
        self.numTags = 0
        self.setpoint = np.zeros(3)

    def ringCallback(self, msg):
        with self.data_lock:
            self.xr[0] = msg.position.x
            self.xr[1] = msg.position.y
            self.xr[2] = msg.position.z

    def posCallback(self, msg):
        with self.data_lock:
            self.x[0] = msg.position.x
            self.x[1] = msg.position.y
            self.x[2] = msg.position.z

    def gtCallback(self, msg):
        with self.data_lock:
            self.x[0] = msg.pose.pose.position.x
            self.x[1] = msg.pose.pose.position.y
            self.x[2] = msg.pose.pose.position.z

    def numTagsCallback(self, msg):
        with self.data_lock:
            self.numTags = msg.data

    def getSetpoint(self):
        _tol = 0.1
        # first move to correct x and z position,
        # then move towards
        # finally, move up
        if abs(self.xr[0]) > _tol or abs(self.xr[2]) > _tol:
            self.setpoint[0] = self.x[0] - self.xr[0]
            self.setpoint[2] = self.x[2] - self.xr[2]
        elif abs(self.xr[1]) > _tol:
            self.setpoint = self.x - self.xr
        else:
            self.setpoint = self.x
            self.setpoint[2] = -0.1
        msg = Pose()
        msg.position.x = self.setpoint[0]
        msg.position.y = self.setpoint[1]
        msg.position.z = self.setpoint[2]
        self.setpoint_pub.publish

def main():
    node = NavigationNode()
    rospy.spin()

if __name__ == "__main__":
    main()
