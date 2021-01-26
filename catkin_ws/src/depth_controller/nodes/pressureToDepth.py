#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import FluidPressure

depth_buf = [-0.5] * 5
depth_buf_len = 5


def pressure_callback(msg, depth_pub):
    pascal_per_meter = 9.81*1000.0
    depth_tmp = - (msg.fluid_pressure - 101325) / pascal_per_meter

    depth_buf.append(depth_tmp)
    if len(depth_buf) > depth_buf_len:
        depth_buf.pop(0)
    depth = sum(depth_buf) / len(depth_buf)

    depth_msg = Float64()
    # depth_raw_msg = Float64()
    # depth_msg.data = depth
    depth_msg.data = depth  # offset, water level is zero
    # depth_raw_msg.data = depth_tmp + 10
    depth_pub.publish(depth_msg)
    # depth_raw_pub.publish(depth_raw_msg)


def main():
    rospy.init_node("depth_calculator")

    depth_pub = rospy.Publisher("depth", Float64, queue_size=1)
    # depth_raw_pub = rospy.Publisher("depth_raw", Float64, queue_size=1)
    rospy.Subscriber("pressure", FluidPressure,
                     pressure_callback,
                     (depth_pub))
    rospy.spin()


if __name__ == "__main__":
    main()
