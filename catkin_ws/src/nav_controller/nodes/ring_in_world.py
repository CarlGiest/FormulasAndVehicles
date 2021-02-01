#!/usr/bin/env python
import tf.transformations as tf
import numpy as np
import rospy
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3, Pose


class ringTranslator():
    def __init__(self):
        rospy.init_node("ring_translator")

        self.pose_sub = rospy.Subscriber("/ground_truth/state", Odometry,
                                         self.simCallback, queue_size=1)

        self.rob_sub = rospy.Subscriber("robot_pos_ls", Pose, self.rob_callback, queue_size=1)

        self.ring_sub = rospy.Subscriber("ring_pos",
                                         Pose,
                                         self.pos_callback,
                                         queue_size=1)

        self.ring_pub = rospy.Publisher("ringWorld",
                                         Pose,
                                         queue_size=1)

        self.euler = np.zeros(3)
        self.pos = np.zeros(3)
        self.ring_abs = np.zeros(3)
        self.ring_rel = np.zeros(3)

    def simCallback(self, msg):
        orient = msg.pose.pose.orientation
        quat = np.array([orient.x, orient.y, orient.z, orient.w])
        self.euler = tf.euler_from_quaternion(quat, axes='sxyz')

    def rob_callback(self, msg):
        pos = msg.position
        self.pos = np.array([pos.x, pos.y, pos.z])
    
    def pos_callback(self, msg):
        pos = msg.position
        self.ring_rel = np.array([-pos.x, pos.y, -pos.z])-\
                        np.array([0.0, -0.1, -0.1])
        self.transform()

    def transform(self):
        # R = tf.euler_matrix(-self.euler[0], -self.euler[1], -self.euler[2], 'sxyz')
        # print(self.euler)
        yaw = -(self.euler[2]-np.pi/2)
        R = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw),  np.cos(yaw), 0],
                      [0, 0, 1]])
        ring_rel_tank = np.matmul(R, self.ring_rel)
        self.ring_abs = ring_rel_tank + self.pos #- \
           # np.array([0.2, -0.25, 0.2])
        # print(self.ring_abs)
        # print(ring_rel_tank, self.ring_rel)
        self.publish()

    def publish(self):
        msg = Pose()
        msg.position.x = self.ring_abs[0]
        msg.position.y = self.ring_abs[1]
        msg.position.z = self.ring_abs[2]
        self.ring_pub.publish(msg)

    def run(self):
        rospy.spin()


def main():
    node = ringTranslator()
    node.run()


if __name__ == '__main__':
    main()
