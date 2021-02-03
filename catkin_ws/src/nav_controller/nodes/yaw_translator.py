#!/usr/bin/env python
import tf.transformations as tf
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3


class YawTranslator():
    def __init__(self):
        rospy.init_node("yaw_translator")

        self.pose_sub = rospy.Subscriber("/ground_truth/state", Odometry,
                                             self.simCallback, queue_size=1)

        self.angle_pub = rospy.Publisher("/nav_controller/angle",
                                         Vector3,
                                         queue_size=1)

        self.angle = Vector3()
        self.angle.x = 0.0
        self.angle.y = 0.0
        self.angle.z = 0.0

    def simCallback(self, msg):
        orient = msg.pose.pose.orientation
        quat = np.array([orient.x, orient.y, orient.z, orient.w])
        euler = tf.euler_from_quaternion(quat, axes='sxyz')
        self.angle.x = euler[0]
        self.angle.y = euler[1]
        self.angle.z = euler[2]
        self.publish()

    def publish(self):
        angleMsg = Vector3()
        angleMsg.x = self.angle.x
        angleMsg.y = self.angle.y
        angleMsg.z = self.angle.z
        self.angle_pub.publish(angleMsg)

    def run(self):
        rospy.spin()


def main():
    node = YawTranslator()
    node.run()


if __name__ == '__main__':
    main()
