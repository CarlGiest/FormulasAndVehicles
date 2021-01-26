#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import numpy as np


class DepthSetpointNode():
    def __init__(self):
        rospy.init_node("depth_setpoints")

        self.setpoint_pub = rospy.Publisher("depth_setpoint",
                                            Float64,
                                            queue_size=1)

    def run(self):
        mode = 'step'

        if mode == 'const':
            rate = rospy.Rate(50.0)
            while not rospy.is_shutdown():
                msg = Float64()
                # fill msg with example setpoint
                msg.data = -0.5
                self.setpoint_pub.publish(msg)
                rate.sleep()
        elif mode == 'sin':
            rate = rospy.Rate(10.0)
            while not rospy.is_shutdown():
                xs = np.linspace(0, 2*np.pi, num=200)
                for x in xs:
                    msg = Float64()
                    # fill msg with example setpoint
                    msg.data = 0.2*np.sin(x) - 0.4
                    self.setpoint_pub.publish(msg)
                    rate.sleep()
        elif mode == 'step':
            rate = rospy.Rate(5.0)
            while not rospy.is_shutdown():
                xs = np.linspace(0, 2*np.pi, num=200)
                for x in xs:
                    msg = Float64()
                    # fill msg with example setpoint
                    msg.data = 0.4*float(x < np.pi) - 0.6
                    self.setpoint_pub.publish(msg)
                    rate.sleep()
        else:
            pass
            # TODO: add ros logger error


def main():
    node = DepthSetpointNode()
    node.run()


if __name__ == "__main__":
    main()
