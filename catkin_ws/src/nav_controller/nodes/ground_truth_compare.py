#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Int16
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


class groundTruthTestNode():
    def __init__(self):
        rospy.init_node("ground_truth_compare")
        self.pos = Pose()
        self.pos.position.x = 0.0
        self.pos.position.y = 0.0
        self.pos.position.z = 0.0
        self.pos.orientation.x = 0.0
        self.pos.orientation.y = 0.0
        self.pos.orientation.z = 0.0
        self.pos.orientation.w = 0.0

        self.number_tags = 4

        self.range_sub = rospy.Subscriber("/ground_truth/state",
                                          Odometry,
                                          self.posCallback,
                                          queue_size=1)
        self.pos_pub = rospy.Publisher("robot_pos",
                                       Pose,
                                       queue_size=1)
        self.err_x_pub = rospy.Publisher("pos_err_x",
                                         Float64,
                                         queue_size=1)
        self.err_y_pub = rospy.Publisher("pos_err_y",
                                         Float64,
                                         queue_size=1)
        self.err_z_pub = rospy.Publisher("pos_err_z",
                                         Float64,
                                         queue_size=1)
        self.tag_pub = rospy.Publisher("number_tags",
                                       Int16,
                                       queue_size=1)

    def posCallback(self, msg):
        # rospy.loginfo("Hallo1")
        self.pos.position.x = msg.pose.pose.position.x
        self.pos.position.y = msg.pose.pose.position.y
        self.pos.position.z = msg.pose.pose.position.z
        self.pos.orientation.x = msg.pose.pose.orientation.x
        self.pos.orientation.y = msg.pose.pose.orientation.y
        self.pos.orientation.z = msg.pose.pose.orientation.z
        self.pos.orientation.w = msg.pose.pose.orientation.w
        self.pos_pub.publish(self.pos)
        msg = Int16()
        msg.data = self.number_tags
        self.tag_pub.publish(msg)
       

    def run(self):
        rospy.spin()


def main():
    rospy.loginfo("Hallo0")
    node = groundTruthTestNode()
    node.run()


if __name__ == "__main__":
    main()
