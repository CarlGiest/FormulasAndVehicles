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

# tag1 = np.array([0.0, 0.0, 0.0])
# tag2 = np.array([0.0, 0.0, 1.0])
# tag3 = np.array([0.0, 1.0, 0.0])
# tag4 = np.array([0.0, 1.0, 1.0])
# tag1 = np.array([0.5, 3.35, -0.5])
# tag2 = np.array([1.1, 3.35, -0.5])
# tag3 = np.array([0.5, 3.35, -0.9])
# tag4 = np.array([1.1, 3.35, -0.9])
ring = np.array([1.0, 3.0, -0.5])
tag_dist = 0.12
tag1 = np.array([ring[0]+tag_dist, ring[1], ring[2]+tag_dist])
tag2 = np.array([ring[0]-tag_dist, ring[1], ring[2]+tag_dist])
tag3 = np.array([ring[0]+tag_dist, ring[1], ring[2]-tag_dist])
tag4 = np.array([ring[0]-tag_dist, ring[1], ring[2]-tag_dist])
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
            self.x0 = optimization_approach(dists, np.array([0.7, 2.0, -0.5]))
            # self.x0 = optimization_approach_depth(dists, self.x0, self.depth)
            # self.x0 = optimization_approach_depth(dists, np.array([0.7,2.0,-0.5]), self.z_gt)

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

# def optimization_approach(dists, x0):
#     def func(x1):
#         def myfunc(x, y, z):
#             return np.linalg.norm([(xref[i] - x)**2 + (yref[i] - y)**2
#             + (zref[i] - z)**2 - dists[i]**2
#             for i, dist in enumerate(dists) if dist != 0])
#         return myfunc(*x1)
#     return minimize(func, x0, tol=1e-7).x


def optimization_approach(dists, x0):
    def func(x1):
        def myfunc(x):
            return norm([norm(p[i]-x)-dists[i] for i in range(4) if dists[i] != 0])
        return myfunc(x1)

    def func_ls(x1):
        def myfunc(x):
            return np.array([norm(p[i]-x)-dists[i] for i in range(4) if dists[i] != 0])
        return myfunc(x1)
    return minimize(func, x0, tol=1e-7).x
    # return least_squares(func_ls, x0, ftol=1e-7, bounds=(tank_bound_lower, tank_bound_upper)).x


def optimization_approach_depth(dists, x0, z):
    x0 = x0[[0, 1]]

    def func(x1):
        def myfunc(x, y):
            x_vec = np.array([x, y, z])
            # return np.linalg.norm([(xref[i] - x)**2 + (yref[i] - y)**2
            # + (zref[i] - z)**2 - dists[i]**2
            # for i, dist in enumerate(dists) if dist != 0])
            return norm([norm(p[i] - x_vec)-dists[i] for i in range(4) if dists[i] != 0])
        return myfunc(*x1)
    xmin = minimize(func, x0, tol=1e-7).x
    return np.array([xmin[0], xmin[1], z])


def trilateration(dists):
    # print(dists != 0)
    N = float(sum(dists != 0))
    iter = np.array([0, 1, 2, 3])[dists != 0]
    if N < 3.0:
        # rospy.loginfo('got to few distances')
        pass
    # Step 1:
    a = 1/N * sum([p[i]*np.dot(p[i], p[i]) - dists[i]**2 * p[i] for i in iter])
    # a = 1/N * sum([p[i]*p[i].T*p[i] - dists[i]**2 * p[i] for i in iter])
    B = 1/N * sum([
        -2*np.array([
            p[i][j]*p[i] for j in range(3)
            ]).T
        - np.dot(p[i], p[i])*np.eye(3) +
        dists[i]**2 * np.eye(3) for i in iter
                   ])
    c = 1/N * sum([p[i] for i in iter])
    f = a + np.matmul(B, c) + 2*np.dot(c, c.T) * c
    fd = np.array([f[i]-f[-1] for i in range(len(f)-1)])
    H = -2/N*sum([np.array([p[i][j]*p[i] for j in range(3)]).T for i in iter]) +\
        np.array([c[j]*c for j in range(3)]).T
    Hd = np.array([H[i]-H[-1] for i in range(len(H)-1)])
    Q, U = np.linalg.qr(Hd)

    # Step 2:
    qtq = -1/N * sum([np.dot(p[i], p[i]) for i in range(3)]) +\
        1/N * sum([dists[i]**2 for i in range(3)]) +\
        np.dot(c, c)

    # Step 3:
    # print(Q.T.shape, fd.shape)

    V = np.matmul(Q.T, fd)
    alpha = U[0, 1]*V[1]/(U[0, 0]*U[1, 1]) - V[0]/U[0, 0]
    beta = U[0, 1]*U[1, 2] / (U[0, 0]*U[1, 1]) - U[0, 2]/U[0, 0]
    gamma = V[1]/U[1, 1]
    delta = U[1, 2]/U[1, 1]

    # a2*q^2 + b2*q + c2 = 0
    a2 = beta**2 + delta**2 + 1
    b2 = 2 * (alpha*beta + gamma*delta)
    c2 = alpha**2 + gamma**2 - qtq
    q3_1 = (-b2 + np.sqrt(b2**2 - 4*a2*c2))/(2*a2)
    q3_2 = (-b2 - np.sqrt(b2**2 - 4*a2*c2))/(2*a2)

    # Step 4:
    q1_1 = alpha+beta*q3_1
    q1_2 = alpha+beta*q3_2
    q2_1 = -gamma - delta*q3_1
    q2_2 = -gamma - delta*q3_2

    # Step 5:
    p0_1 = np.array([q1_1, q2_1, q3_1]) + c
    p0_2 = np.array([q1_2, q2_2, q3_2]) + c

    # Step 6:
    # Choose one of the two p0's
    # TODO
    p0 = p0_1

    return p0


def main():
    node = localizationNode()
    rospy.spin()


if __name__ == "__main__":
    main()
