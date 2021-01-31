#!/usr/bin/env python
import rospy
import threading
from std_msgs.msg import Float64, Int16
from geometry_msgs.msg import Vector3, Pose
from apriltag_ros.msg import AprilTagDetectionArray
from range_sensor.msg import RangeMeasurementArray
from nav_msgs.msg import Odometry
import tf.transformations as trans
# from scipy.optimize import minimize, least_squares
import numpy as np
from numpy.linalg import norm

# tag1 = np.array([0.0, 0.0, 0.0])
# tag2 = np.array([0.0, 0.0, 1.0])
# tag3 = np.array([0.0, 1.0, 0.0])
# tag4 = np.array([0.0, 1.0, 1.0])
# tag1 = np.array([0.5, 3.35, -0.5])
# tag2 = np.array([1.1, 3.35, -0.5])
# tag3 = np.array([0.5, 3.35, -0.9])
# tag4 = np.array([1.1, 3.35, -0.9])
tag1 = np.array([0.7, 3.35, -0.28])
tag2 = np.array([1.3, 3.35, -0.28])
tag3 = np.array([0.7, 3.35, -0.28])
tag4 = np.array([1.3, 3.35, -0.28])
p = [tag1, tag2, tag3, tag4]
tank_bound_lower = np.array([0.0, 0.0, -1.0])
tank_bound_upper = np.array([1.6, 3.35, 0.0])


xref = np.array([t[0] for t in p])
yref = np.array([t[1] for t in p])
zref = np.array([t[2] for t in p])

doX0_avg = True
doDist_avg = True


class localizationNode():
    def __init__(self):
        rospy.init_node("localizationNode")
        self.data_lock = threading.RLock()

        self.range_sub = rospy.Subscriber("ranges", RangeMeasurementArray, self.rangeCallback, queue_size=1)
        self.depth_sub = rospy.Subscriber("depth", Float64,
                                          self.depth_callback,
                                          queue_size=1)
        self.groundTruth_sub = rospy.Subscriber("/ground_truth/state", Odometry, self.gt_callback, queue_size=1)
        self.tag_num_pub = rospy.Publisher("number_tags", Int16, queue_size=1)
        self.z_gt = 0.0
        self.pos_pub = rospy.Publisher("robot_pos", Pose, queue_size=1)
        self.x0 = np.zeros(3)
        self.Sigma0 = np.diag([0.01, 0.01, 0.01])
        self.avg_buf = []
        self.avg_dist1_buf = []
        self.avg_dist2_buf = []
        self.avg_dist3_buf = []
        self.avg_dist4_buf = []
        self.avg_buf_len = 10
        self.avg_buf_len_dist = 10
        self.y = 0.0

    def rangeCallback(self, msg):
        with self.data_lock:
            dists = np.zeros(4)
            if len(msg.measurements) < 2:
                # rospy.loginfo('Got to few tags')
                return
            for measure in msg.measurements:
                id = measure.id
                dists[id-1] = measure.range
            tagNumerMsg = Int16()
            tagNumerMsg.data = len([1 for dist in dists if dist != 0])
            self.tag_num_pub.publish(tagNumerMsg)
            if doDist_avg:
                if len(self.avg_dist1_buf) > self.avg_buf_len_dist:
                    if dists[0] != 0:
                        self.avg_dist1_buf.pop(0)
                if len(self.avg_dist2_buf) > self.avg_buf_len_dist:
                    if dists[1] != 0:
                        self.avg_dist2_buf.pop(0)
                if len(self.avg_dist3_buf) > self.avg_buf_len_dist:
                    if dists[2] != 0:
                        self.avg_dist3_buf.pop(0)
                if len(self.avg_dist4_buf) > self.avg_buf_len_dist:
                    if dists[3] != 0:
                        self.avg_dist4_buf.pop(0)

                if len(self.avg_dist1_buf) == 0:
                    if dists[0] != 0:
                        self.avg_dist1_buf.append(dists[0])
                if len(self.avg_dist2_buf) == 0:
                    if dists[1] != 0:
                        self.avg_dist2_buf.append(dists[1])
                if len(self.avg_dist3_buf) == 0:
                    if dists[2] != 0:
                        self.avg_dist3_buf.append(dists[2])
                if len(self.avg_dist4_buf) == 0:
                    if dists[3] != 0:
                        self.avg_dist4_buf.append(dists[3])
                else:
                    d1_p = self.avg_dist1_buf[-1]
                    d2_p = self.avg_dist2_buf[-1]
                    d3_p = self.avg_dist3_buf[-1]
                    d4_p = self.avg_dist4_buf[-1]
                    if dists[0] != 0:
                        self.avg_dist1_buf.append(np.clip(dists[0], d1_p-0.3, d1_p+0.3))
                    if dists[1] != 0:
                        self.avg_dist2_buf.append(np.clip(dists[1], d2_p-0.3, d2_p+0.3))
                    if dists[2] != 0:
                        self.avg_dist3_buf.append(np.clip(dists[2], d3_p-0.3, d3_p+0.3))
                    if dists[3] != 0:
                        self.avg_dist4_buf.append(np.clip(dists[3], d4_p-0.3, d4_p+0.3))
                dists[0] = sum(self.avg_dist1_buf) / len(self.avg_dist1_buf)
                dists[1] = sum(self.avg_dist2_buf) / len(self.avg_dist2_buf)
                dists[2] = sum(self.avg_dist3_buf) / len(self.avg_dist3_buf)
                dists[3] = sum(self.avg_dist4_buf) / len(self.avg_dist4_buf)

                # print('dists', dists)
                # print(self.avg_dist1_buf)

                # for i in range(4):
                #     if dists[i] == 0:
                #         continue
                #     if len(self.avg_buf_2D[i]) > self.avg_buf_len:
                #         self.avg_buf_2D[i].pop(0)
                #     self.avg_buf_2D[i].append(dists[i])
                #     if i==1:
                #         print(dists[i], dists[0], dists[2], dists[3])
                #         print(self.avg_buf_2D[i])
                #     dists[i] = sum(self.avg_buf_2D[i]) / len(self.avg_buf_2D[i])
            # self.x0 = optimization_approach(dists, self.x0)
            # self.x0 = optimization_approach(dists, np.array([0.7, 2.0, -0.5]))
            # self.x0 = optimization_approach_depth(dists, self.x0, self.depth)
            # self.x0 = optimization_approach_depth(dists, np.array([0.7,2.0,-0.5]), self.z_gt)
            (self.x0, self.Sigma0) = kalmanP(dists, self.depth * 1.0e4, self.x0, self.Sigma0)

        if doX0_avg:
            if len(self.avg_buf) > self.avg_buf_len:
                self.avg_buf.pop(0)
            self.avg_buf.append(self.x0)

            x0 = sum(self.avg_buf) / len(self.avg_buf)
        else:
            x0 = self.x0

        poseMsg = Pose()
        poseMsg.position.x = x0[0]
        poseMsg.position.y = x0[1]
        poseMsg.position.z = x0[2]
        self.pos_pub.publish(poseMsg)

    def depth_callback(self, msg):
        with self.data_lock:
            self.depth = msg.data
            # self.sensor_time = rospy.get_time()

    def gt_callback(self, msg):
        with self.data_lock:
            self.z_gt = msg.pose.pose.position.z+0.08


def kalmanP(dists, pressure, x0, Sigma0):
    # Parameters

    Q = np.diag([0.01, 0.01, 0.01])     # system noise covariance

    # Output and measurement noise covariance matrix calculation for up to 4 AprilTag distances and the pressure sensor reading
    iter = np.array([0, 1, 2, 3])[dists != 0]
    C = np.zeros((iter.shape[0] + 1, 3))
    measurement_covs = np.array(iter.shape[0] + 1)

    for i in iter:
        C[i,0] = (x0[0] - p[i][0]) / dists[i]
        C[i,1] = (x0[1] - p[i][1]) / dists[i]
        C[i,2] = (x0[2] - p[i][2]) / dists[i]

        measurement_covs[i] = 0.1  # covariance of a distance measurement
    C[iter.shape[0] + 1, 0] = 0
    C[iter.shape[0] + 1, 1] = 0
    C[iter.shape[0] + 1, 2] = 1.0e-4      # pressure divided by pascals per meter 

    measurement_covs[iter.shape[0] + 1] = 0.2  # covariance of a depth measurement. Should account for the offset between camera and pressure sensor

    R = np.diag(measurement_covs)

    # Predicted state. For the P Kalman filter, the predicted state (position) is the same as the last position.
    x_pred = x0

    # Predicted covariance Sigma. For the P Kalman filter, the system noise covariance is simply added.
    Sigma_pred = Sigma0 + Q

    # Calculation of predicted measurements

    h = np.zeros(iter.shape[0] + 1)
    for i in iter:
        h[i] = np.sqrt((x_pred[0] - p[i][0])^2 + (x_pred[1] - p[i][1])^2 + (x_pred[2] - p[i][2])^2)
    h[iter.shape[0] + 1] = pressure / 1.0e4

    # Correction

    K = Sigma_pred * np.transpose(C) * np.inv(C * Sigma_pred * np.transpose(C) + R)
    x1 = x_pred + K * ([dists[dists!=0], pressure] - h)
    Sigma1 = (np.eye(3) - K*C) * Sigma_pred

    return x1, Sigma1


def main():
    node = localizationNode()
    rospy.spin()


if __name__ == "__main__":
    main()
