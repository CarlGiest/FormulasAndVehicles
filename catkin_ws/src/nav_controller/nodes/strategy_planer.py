#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, String
from mavros_msgs.srv import CommandBool


class strategyNode():
    def __init__(self):
        rospy.init_node("strategy_planer")
        self.tagNumber = 0
        self.tag_threshold = 3

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

    def tag_callback(self, msg):
        if self.tagNumber == msg.data:
            self.update_strategy()
            return
        else:
            self.tagNumber = msg.data
            self.update_strategy()

    def update_strategy(self):
        msg = String()
        if self.tagNumber < self.tag_threshold:
            msg.data = "search"
        elif self.tagNumber > self.tag_threshold:
            msg.data = "follow"
        else:
            return
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
        self.update_strategy()

    def run(self):
        rospy.spin()


def main():
    node = strategyNode()
    node.run()


if __name__ == "__main__":
    main()
