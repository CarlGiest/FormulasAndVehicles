#!/usr/bin/env python
import rospy
import threading
from std_msgs.msg import Float64
# from mavros_msgs.msg import MotorSetpoint
from mavros_msgs.srv import CommandBool
from numpy import sign


class depthControllerNode():
    def __init__(self):
        rospy.init_node("depthController")

        self.arm_vehicle()

        self.data_lock = threading.RLock()

        self.roll_pub = rospy.Publisher("roll",
                                        Float64,
                                        queue_size=1)
        self.pitch_pub = rospy.Publisher("pitch",
                                         Float64,
                                         queue_size=1)
        self.yaw_pub = rospy.Publisher("yaw",
                                       Float64,
                                       queue_size=1)
        self.thrust_pub = rospy.Publisher("thrust",
                                          Float64,
                                          queue_size=1)
        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                   Float64,
                                                   queue_size=1)
        self.lateral_thrust_pub = rospy.Publisher("lateral_thrust",
                                                  Float64,
                                                  queue_size=1)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.thrust = 0.0
        self.vertical_thrust = 0.0
        self.lateral_thrust = 0.0

        self.setpoint_sub = rospy.Subscriber("depth_setpoint",
                                             Float64,
                                             self.on_setpoint,
                                             queue_size=1)
        # self.setpointMotor_sub = rospy.Subscriber("mavros/setpoint_motor/setpoint",
        #                                      MotorSetpoint,
        #                                      self.on_setpoint_,
        #                                      queue_size=1)

        self.depth_sub = rospy.Subscriber("depth", Float64,
                                          self.depth_callback,
                                          queue_size=1)

        rospy.Timer(rospy.Duration(secs=5), self.control)

        """ # Parameter Theory -> not working
        self.Kp_krit = 2.32
        self.T_krit = 2.8
        self.p_gain = 0.6 * self.Kp_krit
        self.i_gain = self.p_gain / (0.5 * self.T_krit)
        self.d_gain = 0.125 * self.p_gain * self.T_krit """
        # Parameter static
        self.static_p_gain = 0.4
        self.static_i_gain = 0.5
        self.static_d_gain = 0.1
        self.static_vorsteuerung = -0.05
        # Dynamic Parameter
        self.dynamic_p_gain = 0.8
        self.dynamic_i_gain = 0.2
        self.dynamic_d_gain = 0.4
        self.dynamic_vorsteuerung_up = 0.05
        self.dynamic_vorsteuerung_down = -0.1

        # Reconfigure Options via subscriber
        self.kp_dyn_sub = rospy.Subscriber("kp_dyn", Float64,
                                           self.kp_dyn_callback, queue_size=1)
        self.ki_dyn_sub = rospy.Subscriber("ki_dyn", Float64,
                                           self.ki_dyn_callback, queue_size=1)
        self.kd_dyn_sub = rospy.Subscriber("kd_dyn", Float64,
                                           self.kd_dyn_callback, queue_size=1)
        # Reconfigure Options via subscriber
        self.kp_sta_sub = rospy.Subscriber("kp_sta", Float64,
                                           self.kp_sta_callback, queue_size=1)
        self.ki_sta_sub = rospy.Subscriber("ki_sta", Float64,
                                           self.ki_sta_callback, queue_size=1)
        self.kd_sta_sub = rospy.Subscriber("kd_sta", Float64,
                                           self.kd_sta_callback, queue_size=1)

        self.i_buf = [0.0] * 10
        self.setpoint_buf = [0.0] * 5
        self.depth_setpoint = -0.5
        self.depth = self.depth_setpoint
        self.depth_old = self.depth
        self.depth_buf = [self.depth]
        self.depth_buffer_len = 5
        self.sensor_time = rospy.get_time()

    def publish(self):
        msg_roll = Float64()
        msg_roll.data = self.roll
        self.roll_pub.publish(msg_roll)

        msg_pitch = Float64()
        msg_pitch.data = self.pitch
        self.pitch_pub.publish(msg_pitch)

        msg_yaw = Float64()
        msg_yaw.data = self.yaw
        self.yaw_pub.publish(msg_yaw)

        msg_thrust = Float64()
        msg_thrust.data = self.thrust
        self.thrust_pub.publish(msg_thrust)

        msg_vertical_thrust = Float64()
        msg_vertical_thrust.data = self.vertical_thrust
        self.vertical_thrust_pub.publish(msg_vertical_thrust)

        msg_lateral_thrust = Float64()
        msg_lateral_thrust.data = self.lateral_thrust
        self.lateral_thrust_pub.publish(msg_lateral_thrust)

    def on_setpoint(self, msg):
        with self.data_lock:
            if not self.isRegion(msg.data):
                self.depth_setpoint = msg.data
                self.setpoint_buf.append(self.depth_setpoint)
                self.setpoint_buf.pop(0)
            self.control()

    def kp_dyn_callback(self, msg):
        with self.data_lock:
            self.dynamic_p_gain = msg.data

    def ki_dyn_callback(self, msg):
        with self.data_lock:
            self.dynamic_i_gain = msg.data

    def kd_dyn_callback(self, msg):
        with self.data_lock:
            self.dynamic_d_gain = msg.data

    def kp_sta_callback(self, msg):
        with self.data_lock:
            self.static_p_gain = msg.data

    def ki_sta_callback(self, msg):
        with self.data_lock:
            self.static_i_gain = msg.data

    def kd_sta_callback(self, msg):
        with self.data_lock:
            self.static_d_gain = msg.data

    def depth_callback(self, msg):
        with self.data_lock:
            self.depth = msg.data
            self.sensor_time = rospy.get_time()
            self.control()

    # might be redundant if only self.run is used
    def control(self, *args):
        if rospy.get_time() - self.sensor_time > 5:
            rospy.logwarn("Sensor Timeout")
            self.vertical_thrust = 0
            self.publish()
            return

        if self.isStatic():
            self.act_p_gain = self.static_p_gain
            self.act_i_gain = self.static_i_gain
            self.act_d_gain = self.static_d_gain
            self.act_vorsteuerung = self.static_vorsteuerung
        else:
            self.act_p_gain = self.dynamic_p_gain
            self.act_i_gain = self.dynamic_i_gain
            self.act_d_gain = self.dynamic_d_gain
            if self.depth_setpoint > self.depth:
                self.act_vorsteuerung = self.dynamic_vorsteuerung_up
            else:
                self.act_vorsteuerung = self.dynamic_vorsteuerung_down

        self.i_buf.append(self.depth_setpoint - self.depth)
        self.i_buf.pop(0)
        self.vertical_thrust = \
            self.act_p_gain * (self.depth_setpoint - self.depth) +\
            self.act_i_gain * sum(self.i_buf) / len(self.i_buf) +\
            self.act_d_gain * (self.i_buf[-4] - self.i_buf[-1])\
            + self.act_vorsteuerung
        # self.depth_old = self.depth_setpoint - self.depth

        if abs(self.vertical_thrust) > 1:
            self.vertical_thrust = sign(self.vertical_thrust)

        if self.isRegion(self.depth) == 1:
            self.vertical_thrust = -0.1
        elif self.isRegion(self.depth) == -1:
            self.vertical_thrust = 0.1

        self.publish()

    def isRegion(self, setpoint):
        if setpoint > -0.1:
            return 1
        elif setpoint < -0.8:
            return -1
        else:
            return 0

    def isStatic(self):
        if self.i_buf[-4] == self.i_buf[-1]:
            return True
        else:
            return True

    def arm_vehicle(self):
        # wait until the arming serivce becomes available
        rospy.wait_for_service("mavros/cmd/arming")
        # connect to the service
        arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        # call the service to arm the vehicle until service call was successfull
        while not arm(True).success:
            rospy.logwarn("Could not arm vehicle. Keep trying.")
            rospy.sleep(1.0)
        rospy.loginfo("Armed successfully.")

    def run(self):
        rospy.spin()


def main():
    node = depthControllerNode()
    node.run()


if __name__ == "__main__":
    main()
