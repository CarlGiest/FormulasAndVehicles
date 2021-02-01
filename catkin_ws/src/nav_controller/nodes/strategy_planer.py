#!/usr/bin/env python
import rospy
import numpy as np
import threading
from std_msgs.msg import Int16, String
from mavros_msgs.srv import CommandBool
from geometry_msgs.msg import Pose


class strategyNode():
    def __init__(self):
        rospy.init_node("strategy_planer")
        self.data_lock = threading.RLock()
        self.tagNumber = 0
        self.tag_threshold = 3
        self.pos_threshold = 0.1
        self.pos_threshold_y= 0.1
        self.actual_pos = np.array([10.0, 10.0, 10.0])
        self.lock = False

        self.strategy_pub = rospy.Publisher("strategy",
                                            String,
                                            queue_size=1)

        self.setpoint_sub = rospy.Subscriber("number_tags",
                                             Int16,
                                             self.tag_callback,
                                             queue_size=1)

        self.doArm = rospy.get_param("~doArm", False)
        if self.doArm:
            self.arm_vehicle()

        self.setpoint_sub = rospy.Subscriber("ring_pos",
                                             Pose,
                                             self.pos_callback,
                                             queue_size=1)
    
        self.strategy_frequency = 5.0
        rospy.Timer(rospy.Duration(1.0/self.strategy_frequency), self.update_strategy)

    def tag_callback(self, msg):
        # with self.data_lock:
        self.tagNumber = msg.data

    def pos_callback(self, msg):
        #with self.data_lock:
        self.actual_pos[0] = msg.position.x
        self.actual_pos[1] = msg.position.y
        self.actual_pos[2] = msg.position.z

    def update_strategy(self, *args):
        msg = String()
        if self.lock:
            msg.data = "rescue"
            self.strategy_pub.publish(msg)
            return
        if self.tagNumber < self.tag_threshold:
            msg.data = "search"
        elif abs(self.actual_pos[0]) > self.pos_threshold or abs(self.actual_pos[2]) > self.pos_threshold:
            msg.data = "approach"
        elif self.actual_pos[1] > self.pos_threshold_y:
            msg.data = "stich"
        else:
            msg.data = "rescue"
            self.lock = True
        self.strategy_pub.publish(msg)
        

    def arm_vehicle(self):
        # wait until the arming serivce becomes available
        rospy.wait_for_service("mavros/cmd/arming")
        # connect to the service
        arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        # call the service to arm the vehicle until call was successfull
        while not arm(True).success:
            rospy.logwarn("Could not arm vehicle. Keep trying.")
            rospy.sleep(1.0)
        rospy.loginfo("Armed successfully.")

    def run(self):
        rospy.spin()


def main():
    node = strategyNode()
    node.run()


if __name__ == "__main__":
    main()
