#!/usr/bin/env python3
"""
Path Publisher Node
Verzamelt PoseStamped berichten en publiceert ze als nav_msgs/Path
zodat RViz een trajectory kan tonen.
"""
import os
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from duckietown.dtros import DTROS, NodeType


class PathPublisherNode(DTROS):
    def __init__(self, node_name):
        super(PathPublisherNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)

        veh = os.environ.get('VEHICLE_NAME', 'duckiebot21')
        self._veh = veh

        self.path = Path()
        self.path.header.frame_id = "odom"

        # Publisher
        self.pub_path = rospy.Publisher(
            f"/{veh}/odometry/path", Path, queue_size=10, latch=True)

        # Subscriber
        rospy.Subscriber(
            f"/{veh}/odometry/pose_stamped",
            PoseStamped, self.cb_pose, queue_size=10)

        rospy.loginfo(f"[{node_name}] Path publisher klaar voor '{veh}'")
        rospy.loginfo(f"  Publiceert -> /{veh}/odometry/path")

    def cb_pose(self, msg):
        msg.header.frame_id = "odom"
        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(msg)

        # Max 2000 poses bijhouden
        if len(self.path.poses) > 2000:
            self.path.poses = self.path.poses[-2000:]

        self.pub_path.publish(self.path)


if __name__ == '__main__':
    node = PathPublisherNode(node_name='path_publisher_node')
    rospy.spin()
