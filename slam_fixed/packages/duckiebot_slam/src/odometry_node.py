#!/usr/bin/env python3
"""
- Odometry Node
Schat de positie (x, y, theta) van de Duckiebot op basis van
wheel encoder data met een differential drive kinematisch model.
Publiceert op: /<veh>/odometry/pose
"""
import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf_conversions


class OdometryNode(DTROS):
    def __init__(self, node_name):
        super(OdometryNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)

        veh = os.environ.get('VEHICLE_NAME', 'duckiebot21')
        self._veh = veh

        # Differential drive parameters (Duckiebot DB21)
        self.wheel_radius   = 0.0318   # meter
        self.wheel_baseline = 0.1      # meter (afstand tussen wielen)
        self.ticks_per_rev  = 135      # encoder ticks per rotatie

        # Robot pose
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # Encoder state
        self.prev_left  = None
        self.prev_right = None

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Publishers
        self.pub_odom = rospy.Publisher(
            f"/{veh}/odometry/pose", Odometry, queue_size=10)
        self.pub_pose = rospy.Publisher(
            f"/{veh}/odometry/pose_stamped", PoseStamped, queue_size=10)

        # Subscribers — beide encoder topics voor compatibiliteit
        rospy.Subscriber(f"/{veh}/left_wheel_encoder_node/tick",
                         WheelEncoderStamped, self.cb_left)
        rospy.Subscriber(f"/{veh}/right_wheel_encoder_node/tick",
                         WheelEncoderStamped, self.cb_right)
        rospy.Subscriber(f"/{veh}/left_wheel_encoder_driver_node/tick",
                         WheelEncoderStamped, self.cb_left)
        rospy.Subscriber(f"/{veh}/right_wheel_encoder_driver_node/tick",
                         WheelEncoderStamped, self.cb_right)

        rospy.loginfo(f"[{node_name}] Odometry node klaar voor '{veh}'")
        rospy.loginfo(f"  Publiceert -> /{veh}/odometry/pose")

    def cb_left(self, msg):
        if self.prev_left is not None:
            self._update(msg.data - self.prev_left, 0)
        self.prev_left = msg.data

    def cb_right(self, msg):
        if self.prev_right is not None:
            self._update(0, msg.data - self.prev_right)
        self.prev_right = msg.data

    def _update(self, dl, dr):
        """Differential drive kinematisch model."""
        dist_l = (dl / self.ticks_per_rev) * 2 * math.pi * self.wheel_radius
        dist_r = (dr / self.ticks_per_rev) * 2 * math.pi * self.wheel_radius

        dist   = (dist_l + dist_r) / 2.0
        dtheta = (dist_r - dist_l) / self.wheel_baseline

        # Pose update
        self.theta += dtheta
        self.x     += dist * math.cos(self.theta)
        self.y     += dist * math.sin(self.theta)

        rospy.loginfo_throttle(1,
            f"[Odometry] x={self.x:.3f}m  y={self.y:.3f}m  "
            f"θ={math.degrees(self.theta):.1f}°")

        self._publish()

    def _publish(self):
        now = rospy.Time.now()
        q   = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta)

        # Odometry message
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"
        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.pub_odom.publish(odom)

        # PoseStamped
        ps = PoseStamped()
        ps.header = odom.header
        ps.pose   = odom.pose.pose
        self.pub_pose.publish(ps)

        # TF transform odom -> base_link
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = "odom"
        t.child_frame_id  = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def get_pose(self):
        return self.x, self.y, self.theta


if __name__ == '__main__':
    node = OdometryNode(node_name='odometry_node')
    rospy.spin()
