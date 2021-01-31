#!/usr/bin/env python
import rospy
import threading
from std_msgs.msg import Float64, Int16
from geometry_msgs.msg import Vector3, Pose
from apriltag_ros.msg import AprilTagDetectionArray
from range_sensor.msg import RangeMeasurementArray
from nav_msgs.msg import Odometry
import tf.transformations as trans
from scipy.optimize import minimize, least_squares
import numpy as np
from numpy.linalg import norm

p = []
tank_bound_lower = np.array([0.0, 0.0, -1.0])
tank_bound_upper = np.array([1.6, 3.35, 0.0])

for j in range(9):
    for i in range(7):
        p.append(np.array([1.56+i*0.25, 0.06+j*0.39375, -1.3]))

p = np.array(p)

class localizationNode():
    def __init__(self):
        rospy.init_node("localizationNode")
        self.data_lock = threading.RLock()
        self.range_sub = rospy.Subscriber("ranges", RangeMeasurementArray, self.rangeCallback, queue_size=1)
        self.pos_pub = rospy.Publisher("robot_pos_ls", Pose, queue_size=1)
        #self.tag_num_pub = rospy.Publisher("number_tags", Int16, queue_size=1)
        self.x0 = np.zeros(3)
        self.avg_buf = []
        self.avg_dist_buf = [[] for _ in range(63)]
        self.avg_buf_len = 3
        self.avg_buf_len_dist = 5

    def rangeCallback(self, msg):
        with self.data_lock:
            dists = np.zeros(63)
            for measure in msg.measurements:
                if measure.id >= 5:
                    id = measure.id-4
                    dists[id-1] = measure.range
            #tagNumerMsg = Int16()
            #tagNumerMsg.data = len([1 for dist in dists if dist != 0])
            #self.tag_num_pub.publish(tagNumerMsg)
            for i in range(63):
                if dists[i] != 0:
                    self.avg_dist_buf[i].append(dists[i])
                if len(self.avg_dist_buf[i]) > self.avg_buf_len_dist:
                    self.avg_dist_buf[i].pop(0)
                if len(self.avg_dist_buf[i]) > 0:
                    dists[i] = sum(self.avg_dist_buf[i]) / \
                            len(self.avg_dist_buf[i])
            self.x0 = self.optimization(dists, self.x0)
            self.avg_buf.append(self.x0)
            if len(self.avg_buf) > self.avg_buf_len:
                self.avg_buf.pop(0)
            self.x0 = sum(self.avg_buf) / len(self.avg_buf)
            
            poseMsg = Pose()
            poseMsg.position.x = self.x0[0]
            poseMsg.position.y = self.x0[1]
            poseMsg.position.z = self.x0[2]
            self.pos_pub.publish(poseMsg)

    # def depth_callback(self, msg):
    #     with self.data_lock:
    #         self.depth = msg.data

    # def gt_callback(self, msg):
    #     with self.data_lock:
    #         self.z_gt = msg.pose.pose.position.z+0.08

    def optimization(self, dists, x0):
        def objective_function(x):
            return np.array([norm(p[i]-x)-dists[i]
                            for i in range(63) if dists[i] != 0])
        return least_squares(objective_function, x0, ftol=1e-7,
                             bounds=(tank_bound_lower, tank_bound_upper)).x


def main():
    node = localizationNode()
    rospy.spin()


if __name__ == "__main__":
    main()
