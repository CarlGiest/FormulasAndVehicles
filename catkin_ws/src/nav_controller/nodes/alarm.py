#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose


class alarmNode():
    def __init__(self):
        rospy.init_node("alarmNode")
        self.tagNumber = 0
        self.tag_threshold = 3

        self.ring_sub = rospy.Subscriber("ringWorld",
                                         Pose,
                                         self.alarm_callback,
                                         queue_size=1)
        self.setpoint_sub = rospy.Subscriber("number_tags",
                                             Int16,
                                             self.tag_callback,
                                             queue_size=1)

    def tag_callback(self, msg):
        self.tagNumber = msg.data
        if self.tagNumber > 0 and self.tagNumber <= self.tag_threshold:
            rospy.loginfo("Maybe I found something")


    def alarm_callback(self, msg):
        self.actual_pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        if self.tagNumber > self.tag_threshold:
            rospy.logwarn("Alarm!!!!!!!!!     Object found at:")
            rospy.logwarn(self.actual_pos)

    def run(self):
        rospy.spin()


def main():
    node = alarmNode()
    node.run()


if __name__ == '__main__':
    main()