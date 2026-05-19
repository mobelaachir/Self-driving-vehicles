#!/usr/bin/env python3
"""
- Sensorfusie Node (Extended Kalman Filter)
Combineert odometry (voorspelling) en visuele correctie (update)
tot een robuuste pose-schatting.

State vector: [x, y, theta]
Publiceert:   /<veh>/fusion_node/pose
"""
import os
import math
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import tf_conversions


class FusionNode(DTROS):
    def __init__(self, node_name):
        super(FusionNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)

        veh = os.environ.get('VEHICLE_NAME', 'duckiebot21')
        self._veh = veh

        # ── EKF state 
        # State: [x, y, theta]
        self.mu = np.zeros(3)           # mean
        self.sigma = np.eye(3) * 0.01  # covariance

        # Process noise (Q) — hoeveel we odometry vertrouwen
        self.Q = np.diag([0.01, 0.01, 0.005])

        # Measurement noise (R) — hoeveel we visie vertrouwen
        self.R = np.diag([0.05, 0.05])

        # Vorige odometry pose voor delta berekening
        self.prev_odom_x     = None
        self.prev_odom_y     = None
        self.prev_odom_theta = None

        # Publishers
        self.pub_fused = rospy.Publisher(
            f"/{veh}/fusion_node/pose", Odometry, queue_size=10)
        self.pub_pose_stamped = rospy.Publisher(
            f"/{veh}/fusion_node/pose_stamped", PoseStamped, queue_size=10)

        # Subscribers
        rospy.Subscriber(f"/{veh}/odometry/pose",
                         Odometry, self.cb_odometry)
        rospy.Subscriber(f"/{veh}/slam_node/correction",
                         Float32MultiArray, self.cb_visual_correction)

        rospy.loginfo(f"[{node_name}] Fusie node (EKF) klaar voor '{veh}'")
        rospy.loginfo(f"  Publiceert -> /{veh}/fusion_node/pose")

    # ── EKF Prediction step (odometry) 

    def cb_odometry(self, msg):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        q  = msg.pose.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        ot = math.atan2(siny, cosy)

        if self.prev_odom_x is None:
            self.prev_odom_x, self.prev_odom_y, self.prev_odom_theta = ox, oy, ot
            return

        # Delta beweging
        dx     = ox - self.prev_odom_x
        dy     = oy - self.prev_odom_y
        dtheta = ot - self.prev_odom_theta

        self.prev_odom_x, self.prev_odom_y, self.prev_odom_theta = ox, oy, ot

        # ── EKF predict 
        theta = self.mu[2]

        # Motion model: x' = x + dx*cos(theta) - dy*sin(theta)
        self.mu[0] += dx * math.cos(theta) - dy * math.sin(theta)
        self.mu[1] += dx * math.sin(theta) + dy * math.cos(theta)
        self.mu[2] += dtheta
        self.mu[2]  = math.atan2(math.sin(self.mu[2]), math.cos(self.mu[2]))

        # Jacobian van motion model (F)
        F = np.array([
            [1, 0, -dx*math.sin(theta) - dy*math.cos(theta)],
            [0, 1,  dx*math.cos(theta) - dy*math.sin(theta)],
            [0, 0,  1]
        ])

        # Covariance predict
        self.sigma = F @ self.sigma @ F.T + self.Q

        rospy.loginfo_throttle(1,
            f"[EKF predict] x={self.mu[0]:.3f}  y={self.mu[1]:.3f}  "
            f"θ={math.degrees(self.mu[2]):.1f}°")

        self._publish()

    # ── EKF Update step (visuele correctie) 

    def cb_visual_correction(self, msg):
        if len(msg.data) < 2:
            return

        # Meting: optische flow dx, dy (pixel space → meter, geschaalde schatting)
        pixel_scale = 0.001   # ruwweg: 1 pixel flow ≈ 1mm beweging
        z = np.array([
            msg.data[0] * pixel_scale,
            msg.data[1] * pixel_scale
        ])

        # Measurement model H: we meten x en y beweging
        H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])

        # Kalman gain
        S = H @ self.sigma @ H.T + self.R
        K = self.sigma @ H.T @ np.linalg.inv(S)

        # Innovatie
        z_pred = H @ self.mu
        innovation = z - z_pred

        # ── EKF update 
        self.mu    = self.mu + K @ innovation
        self.mu[2] = math.atan2(math.sin(self.mu[2]), math.cos(self.mu[2]))
        self.sigma = (np.eye(3) - K @ H) @ self.sigma

        rospy.loginfo_throttle(2,
            f"[EKF update] innovatie=({innovation[0]:.4f}, {innovation[1]:.4f})")

        self._publish()

    def _publish(self):
        now = rospy.Time.now()
        q   = tf_conversions.transformations.quaternion_from_euler(0, 0, self.mu[2])

        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = "map"
        odom.child_frame_id  = "base_link"
        odom.pose.pose.position.x    = self.mu[0]
        odom.pose.pose.position.y    = self.mu[1]
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Covariance (6x6, we vullen x,y,theta)
        cov = [0.0] * 36
        cov[0]  = self.sigma[0,0]
        cov[1]  = self.sigma[0,1]
        cov[5]  = self.sigma[0,2]
        cov[6]  = self.sigma[1,0]
        cov[7]  = self.sigma[1,1]
        cov[11] = self.sigma[1,2]
        cov[30] = self.sigma[2,0]
        cov[31] = self.sigma[2,1]
        cov[35] = self.sigma[2,2]
        odom.pose.covariance = cov

        self.pub_fused.publish(odom)

        ps = PoseStamped()
        ps.header = odom.header
        ps.pose   = odom.pose.pose
        self.pub_pose_stamped.publish(ps)


if __name__ == '__main__':
    node = FusionNode(node_name='fusion_node')
    rospy.spin()
