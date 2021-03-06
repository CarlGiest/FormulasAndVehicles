#!/usr/bin/env python
import rospy
import threading
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Pose
from numpy import sign


class transControlNode():
    def __init__(self):
        rospy.init_node("trans_control")

        self.data_lock = threading.RLock()
        self.strategy = "search"

        self.vorsteuerung = 0.07
        # Parameter xy
        self.xy_p_gain = 0.17
        self.xy_i_gain = 0.1
        self.xy_d_gain = 0.13
        self.stichFaktor = 1.5
        # Parameter z
        self.vertical_p_gain = 0.2
        self.vertical_i_gain = 0.1
        self.vertical_d_gain = 0.05

        self.thrust = 0.0
        self.lateral_thrust = 0.0
        self.vertical_thrust = self.vorsteuerung

        self.setpoint_buf_len = 5
        self.i_buf_len = 10
        self.i_buf = Pose()
        self.i_buf.position.x = [0.0] * self.i_buf_len
        self.i_buf.position.y = [0.0] * self.i_buf_len
        self.i_buf.position.z = [0.0] * self.i_buf_len
        self.setpoint_buf = Pose()
        self.setpoint_buf.position.x = [0.0] * self.setpoint_buf_len
        self.setpoint_buf.position.y = [0.0] * self.setpoint_buf_len
        self.setpoint_buf.position.z = [0.0] * self.setpoint_buf_len
        self.sensor_time = rospy.get_time()
        self.pos = Pose()
        self.pos.position.x = 0.0
        self.pos.position.y = 0.0
        self.pos.position.z = 0.0

        self.thrust_pub = rospy.Publisher("thrust",
                                          Float64,
                                          queue_size=1)
        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                   Float64,
                                                   queue_size=1)
        self.lateral_thrust_pub = rospy.Publisher("lateral_thrust",
                                                  Float64,
                                                  queue_size=1)

        self.setpoint_sub = rospy.Subscriber("ring_pos",
                                             Pose,
                                             self.pos_callback,
                                             queue_size=1)

        self.strategy_sub = rospy.Subscriber("strategy",
                                             String,
                                             self.strategy_callback,
                                             queue_size=1)

        # Reconfigure Options via subscriber
        self.kp_dyn_sub = rospy.Subscriber("kp_xy", Float64,
                                           self.kp_xy_callback, queue_size=1)
        self.ki_dyn_sub = rospy.Subscriber("ki_xy", Float64,
                                           self.ki_xy_callback, queue_size=1)
        self.kd_dyn_sub = rospy.Subscriber("kd_xy", Float64,
                                           self.kd_xy_callback, queue_size=1)
        # Reconfigure Options via subscriber
        self.kp_sta_sub = rospy.Subscriber("kp_vert", Float64,
                                           self.kp_vert_callback, queue_size=1)
        self.ki_sta_sub = rospy.Subscriber("ki_vert", Float64,
                                           self.ki_vert_callback, queue_size=1)
        self.kd_sta_sub = rospy.Subscriber("kd_vert", Float64,
                                           self.kd_vert_callback, queue_size=1)
        self.vor_static_sub = rospy.Subscriber("vor", Float64,
                                               self.vor_callback, queue_size=1)
        
        self.trans_control_frequency = 10.0
        
        rospy.Timer(rospy.Duration(1.0/self.trans_control_frequency), self.trans_control)

    def strategy_callback(self, msg):
        self.strategy = msg.data

    def kp_xy_callback(self, msg):
        with self.data_lock:
            if msg.data >= 0:
                self.xy_p_gain = msg.data
            else:
                rospy.logwarn("Received negative Kp")

    def ki_xy_callback(self, msg):
        with self.data_lock:
            if msg.data >= 0:
                self.xy_i_gain = msg.data
            else:
                rospy.logwarn("Received negative Ki")

    def kd_xy_callback(self, msg):
        with self.data_lock:
            if msg.data >= 0:
                self.xy_d_gain = msg.data
            else:
                rospy.logwarn("Received negative Kd")

    def kp_vert_callback(self, msg):
        with self.data_lock:
            if msg.data >= 0:
                self.vertical_p_gain = msg.data
            else:
                rospy.logwarn("Received negative dyn Kp")

    def ki_vert_callback(self, msg):
        with self.data_lock:
            if msg.data >= 0:
                self.vertical_i_gain = msg.data
            else:
                rospy.logwarn("Received negative dyn Ki")

    def kd_vert_callback(self, msg):
        with self.data_lock:
            if msg.data >= 0:
                self.vertical_d_gain = msg.data
            else:
                rospy.logwarn("Received self.vertical_thrustnegative dyn Kd")

    def vor_callback(self, msg):
        with self.data_lock:
            self.vorsteuerung = msg.data

    def pos_callback(self, msg):
        with self.data_lock:
            if self.strategy == "search":
                #rospy.loginfo("search")
                self.pos.position.x = -msg.position.x
                self.pos.position.y = msg.position.y
                self.pos.position.z = -msg.position.z
            elif self.strategy == "approach":
                #rospy.loginfo("approach")
                self.pos.position.x = -msg.position.x
                self.pos.position.y = msg.position.y-0.2
                self.pos.position.z = -msg.position.z
            elif self.strategy == "stich":
                #rospy.loginfo("stich")
                self.pos.position.x = -msg.position.x
                self.pos.position.y = msg.position.y
                self.pos.position.z = -msg.position.z
            elif self.strategy == "rescue":
                #rospy.loginfo("rescue")
                self.pos.position.x = 0.0
                self.pos.position.y = 0.0
                self.pos.position.z = 0.5
            self.sensor_time = rospy.get_time()

    def setBuffer(self, buf, msgAppend, len):
        for i in range(0, len-1):
            buf.x[i] = buf.x[i+1]
        for i in range(0, len-1):
            buf.y[i] = buf.y[i+1]
        for i in range(0, len-1):
            buf.z[i] = buf.z[i+1]
        buf.x[-1] = msgAppend.x
        buf.y[-1] = msgAppend.y
        buf.z[-1] = msgAppend.z
        return buf
        
    def trans_control(self, *args):
        if rospy.get_time() - self.sensor_time > 2:
            rospy.logwarn("Sensor Timeout")
            self.thrust = 0.0
            self.lateral_thrust = 0.0
            self.vertical_thrust = self.vorsteuerung
            self.publish()
            return
        elif self.strategy == "search":
            self.thrust = 0.0
            self.lateral_thrust = 0.0
            self.vertical_thrust = self.vorsteuerung
            self.publish()
            return
        
        i_buf_Append = Pose()
        i_buf_Append.position.x = self.pos.position.x
        i_buf_Append.position.y = self.pos.position.y
        i_buf_Append.position.z = self.pos.position.z
        self.i_buf.position = self.setBuffer(self.i_buf.position,
                                             i_buf_Append.position,
                                             self.i_buf_len)

        self.setGains(True)
        # rospy.loginfo(self.pos.position)
        self.lateral_thrust = -self.getThrust(self.pos.position.x,
                                      self.i_buf.position.x, False)
        self.setGains(True)
        self.thrust = self.getThrust(self.pos.position.y,
                                     self.i_buf.position.y, False)
        self.setGains(False)
        self.vertical_thrust = self.getThrust(self.pos.position.z,
                                              self.i_buf.position.z, True)

        self.publish()

    def getThrust(self, pos, i_buf, vor_activate):
        thrust = (self.act_p_gain * pos
                  + self.act_i_gain * sum(i_buf) * (1.0/self.trans_control_frequency)
                  + self.act_d_gain * (i_buf[-1] - i_buf[-2]))
        if vor_activate:
            thrust += self.vorsteuerung
        if abs(thrust) > 1:
            thrust = sign(thrust)
        return thrust

    def setGains(self, dirXY):
        if dirXY:
            if self.strategy == "stich":
                self.act_p_gain = self.xy_p_gain * self.stichFaktor
                self.act_i_gain = self.xy_i_gain * self.stichFaktor
                self.act_d_gain = self.xy_d_gain * self.stichFaktor
            else:
                self.act_p_gain = self.xy_p_gain
                self.act_i_gain = self.xy_i_gain
                self.act_d_gain = self.xy_d_gain
        else:
            self.act_p_gain = self.vertical_p_gain
            self.act_i_gain = self.vertical_i_gain
            self.act_d_gain = self.vertical_d_gain

    def publish(self):
        # rospy.loginfo("Hallo")
        msg_thrust = Float64()
        msg_thrust.data = self.thrust
        self.thrust_pub.publish(msg_thrust)

        msg_vertical_thrust = Float64()
        msg_vertical_thrust.data = self.vertical_thrust
        self.vertical_thrust_pub.publish(msg_vertical_thrust)

        msg_lateral_thrust = Float64()
        msg_lateral_thrust.data = self.lateral_thrust
        self.lateral_thrust_pub.publish(msg_lateral_thrust)

    def run(self):
        rospy.spin()


def main():
    node = transControlNode()
    node.run()


if __name__ == "__main__":
    main()
