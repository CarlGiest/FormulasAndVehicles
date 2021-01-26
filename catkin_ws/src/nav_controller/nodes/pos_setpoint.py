#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
import numpy as np


class PosSetpointNode():
    def __init__(self):
        rospy.init_node("pos_setpoint")

        self.setpoint_pub = rospy.Publisher("pos_setpoint",
                                            Pose,
                                            queue_size=1)

    def run(self):
        mode = 'const'

        if mode == 'const':
            rate = rospy.Rate(50.0)
            while not rospy.is_shutdown():
                msg1 = Pose()
                # fill msg with example setpoint
                msg1.position.x = 1.0
                msg1.position.y = 3.0
                msg1.position.z = -0.5
                self.setpoint_pub.publish(msg1)

                rate.sleep()
        elif mode == 'sin':
            rate = rospy.Rate(10.0)
            while not rospy.is_shutdown():
                xs = np.linspace(0, 2*np.pi, num=200)
                for x in xs:
                    msg = Pose()
                    # fill msg with example setpoint
                    msg.data = 0.2*np.sin(x) - 0.4
                    self.setpoint_pub.publish(msg)
                    rate.sleep()
        elif mode == 'step':
            rate = rospy.Rate(10.0)
            while not rospy.is_shutdown():
                xs = np.linspace(0, 2*np.pi, num=400)
                for x in xs:
                    msg1 = Pose()
                    msg1.position.x = 1.0 + 0.5*float(x < np.pi)
                    msg1.position.y = 1.0 + 1.0*float(x < np.pi)
                    msg1.position.z = -0.5 - 0.5*float(x < np.pi)
                    self.setpoint_pub.publish(msg1)
                    rate.sleep()
        else:
            pass
            # TODO: add ros logger error


def main():
    node = PosSetpointNode()
    node.run()


if __name__ == "__main__":
    main()
