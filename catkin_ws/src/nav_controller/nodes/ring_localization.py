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

__radius_ring__ = np.sqrt(2.0*0.12**2.0)

def node_at_angle(phi):
    r = __radius_ring__
    return r*np.array([np.cos(phi), 0.0, np.sin(phi)])

tag1 = node_at_angle(np.pi/4.0)
tag2 = node_at_angle(3.0*np.pi/4.0)
tag3 = node_at_angle(-3.0*np.pi/4.0)
tag4 = node_at_angle(-np.pi/4.0)
p = np.array([tag1, tag2, tag3, tag4])
# tank_bound_lower = np.array([-1.6, 0.0, -1.0])
# tank_bound_upper = np.array([1.6, 3.35, 1.0])
tank_bound_lower = np.array([-4.0, 0.0, -1.0])
tank_bound_upper = np.array([4.0, 4.0, 1.0])

hook_offset = np.array([0.0, -0.1, -0.1])

class localizationNode():
    def __init__(self):
        rospy.init_node("ringLocalizationNode")
        self.data_lock = threading.RLock()
        self.range_sub = rospy.Subscriber("ranges", RangeMeasurementArray, self.rangeCallback, queue_size=1)
        self.pos_pub = rospy.Publisher("ring_pos", Pose, queue_size=1)
        self.tag_num_pub = rospy.Publisher("number_tags", Int16, queue_size=1)
        self.x0 = np.array([0.0, 1.0, 0.0])
        self.avg_buf = []
        self.avg_dist_buf = [[] for _ in range(4)]
        self.avg_buf_len = 3
        self.avg_buf_len_dist = 5

    def rangeCallback(self, msg):
        with self.data_lock:
            dists = np.zeros(4)
            for measure in msg.measurements:
                if measure.id < 5:
                    id = measure.id
                    dists[id-1] = measure.range
            tagNumerMsg = Int16()
            tagNumerMsg.data = len([1 for dist in dists if dist != 0])
            self.tag_num_pub.publish(tagNumerMsg)
            for i in range(4):
                if dists[i] != 0:
                    self.avg_dist_buf[i].append(dists[i])
                if len(self.avg_dist_buf[i]) > self.avg_buf_len_dist:
                    self.avg_dist_buf[i].pop(0)
                if len(self.avg_dist_buf[i]) > 0:
                    dists[i] = sum(self.avg_dist_buf[i]) / \
                            len(self.avg_dist_buf[i])
            if len(msg.measurements) < 3:
                return
            self.x0 = self.optimization(dists, self.x0)
            # self.avg_buf.append(self.x0)
            # if len(self.avg_buf) > self.avg_buf_len:
            #     self.avg_buf.pop(0)
            # self.x0 = sum(self.avg_buf) / len(self.avg_buf)
            
            poseMsg = Pose()
            poseMsg.position.x = self.x0[0] + hook_offset[0]
            poseMsg.position.y = self.x0[1] + hook_offset[1]
            poseMsg.position.z = self.x0[2] + hook_offset[2]
            self.pos_pub.publish(poseMsg)

    def optimization(self, dists, x0):
        def objective_function(x):
            return np.array([norm(p[i]-x)-dists[i]
                            for i in range(4) if dists[i] != 0])
        return least_squares(objective_function, x0, ftol=1e-7,
                             bounds=(tank_bound_lower, tank_bound_upper)).x
        # return least_squares(objective_function, x0, ftol=1e-7).x

def main():
    node = localizationNode()
    rospy.spin()

if __name__ == "__main__":
    main()
