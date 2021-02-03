#!/usr/bin/env python
import rospy
import threading
from std_msgs.msg import Float64, Int16
from geometry_msgs.msg import Vector3, Pose
from apriltag_ros.msg import AprilTagDetectionArray
from range_sensor.msg import RangeMeasurementArray, RangeMeasurement
from nav_msgs.msg import Odometry
import tf.transformations as trans
# from scipy.optimize import minimize, least_squares
import numpy as np
from numpy.linalg import norm,inv

p = []
tank_bound_lower = np.array([0.0, 0.0, -1.3])
tank_bound_upper = np.array([2.0, 4.0, 0.0])

for j in range(9):
    for i in range(7):
        p.append(np.array([1.56-i*0.25, 0.06+j*0.39375, -1.3]))
range_buf = np.zeros(len(p))
tank_bound_lower = np.array([0.0, 0.0, -1.0])
tank_bound_upper = np.array([1.6, 3.35, 0.0])


xref = np.array([t[0] for t in p])
yref = np.array([t[1] for t in p])
zref = np.array([t[2] for t in p])

class localizationNode():
    def __init__(self):
        rospy.init_node("localizationNode", log_level = rospy.DEBUG)
        self.data_lock = threading.RLock()

        self.range_sub = rospy.Subscriber("ranges", RangeMeasurementArray, self.rangeCallback, queue_size=1)
        self.depth_sub = rospy.Subscriber("depth", Float64,
                                          self.depth_callback,
                                          queue_size=1)
        self.groundTruth_sub = rospy.Subscriber("/ground_truth/state", Odometry, self.gt_callback, queue_size=1)
        self.z_gt = 0.0
        self.pos_pub = rospy.Publisher("robot_pos", Pose, queue_size=1)
        self.range_est_pub = rospy.Publisher("range_estimates", RangeMeasurementArray, queue_size=1)
        self.range_meas_pub = rospy.Publisher("range_measurements", RangeMeasurementArray, queue_size=1)
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
            dists = np.zeros(63)
            for measure in msg.measurements:
                if measure.id >= 5:
                    id = measure.id-4
                    dists[id-1] = measure.range
            
            (self.x0, self.Sigma0) = kalmanP(dists, self.depth * 1.0e4, self.x0, self.Sigma0)

        x0 = self.x0

        poseMsg = Pose()
        poseMsg.position.x = x0[0]
        poseMsg.position.y = x0[1]
        poseMsg.position.z = x0[2]

        # Output the estimated range measurement based on the position estimate
        range_est_array_msg = RangeMeasurementArray()
        for i in range(len(p)):
            x_diff = x0[0] - p[i][0]
            y_diff = x0[1] - p[i][1]
            z_diff = x0[2] - p[i][2]
            range_est_msg = RangeMeasurement()
            range_est_msg.id = i
            range_est_msg.range = np.sqrt(x_diff**2 + y_diff**2 + z_diff**2)

            range_est_array_msg.measurements.append(range_est_msg)

        # Output the true range measurements. When no new measurement is received, the last value is held.
        range_meas_array_msg = RangeMeasurementArray()
        for i in range(len(p)-1):
            range_meas_msg = RangeMeasurement()
            range_meas_msg.id = i
            if(dists[i]):
                range_meas_msg.range = dists[i]
                range_buf[i] = dists[i]
            else:
                range_meas_msg.range = range_buf[i]

            range_meas_array_msg.measurements.append(range_meas_msg)

        self.range_est_pub.publish(range_est_array_msg)
        self.range_meas_pub.publish(range_meas_array_msg)
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
    iter = np.array(range(dists.shape[0]))[dists != 0]
    num_tags = iter.shape[0]
    C = np.zeros((num_tags + 1, 3))
    measurement_covs = np.zeros(num_tags + 1)

    for i in range(num_tags):
        C[i,0] = (x0[0] - p[iter[i]][0]) / dists[iter[i]]
        C[i,1] = (x0[1] - p[iter[i]][1]) / dists[iter[i]]
        C[i,2] = (x0[2] - p[iter[i]][2]) / dists[iter[i]]

        measurement_covs[i] = 0.1  # covariance of a distance measurement
    C[num_tags, 0] = 0
    C[num_tags, 1] = 0
    C[num_tags, 2] = 1.0e-4      # pressure divided by pascals per meter 

    measurement_covs[num_tags] = 0.2  # covariance of a depth measurement. Should account for the offset between camera and pressure sensor

    R = np.diag(measurement_covs)

    # Predicted state. For the P Kalman filter, the predicted state (position) is the same as the last position.
    x_pred = x0

    # Predicted covariance Sigma. For the P Kalman filter, the system noise covariance is simply added.
    Sigma_pred = Sigma0 + Q

    # Calculation of predicted measurements

    h = np.zeros(num_tags + 1)
    for i in range(num_tags):
        h[i] = np.sqrt(np.float_power((x_pred[0] - p[iter[i]][0]),2) + np.float_power((x_pred[1] - p[iter[i]][1]),2) + np.float_power((x_pred[2] - p[iter[i]][2]),2))
    h[num_tags] = pressure / 1.0e4

    # Correction

    K = Sigma_pred.dot(np.transpose(C)).dot(inv(C.dot(Sigma_pred).dot(np.transpose(C)) + R))
    z = np.append(dists[dists!=0], [pressure])

    x1 = x_pred + np.matmul(K,(z - h))
    Sigma1 = (np.eye(3) - K.dot(C)).dot(Sigma_pred)

    return x1, Sigma1


def main():
    node = localizationNode()
    rospy.spin()


if __name__ == "__main__":
    main()
