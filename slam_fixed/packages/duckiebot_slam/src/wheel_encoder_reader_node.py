#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped


class WheelEncoderReaderNode(DTROS):
    def __init__(self, node_name):
        super(WheelEncoderReaderNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'duckiebot')

        self._ticks_left = None
        self._ticks_right = None

        self.sub_left = rospy.Subscriber(
            f"/{self._vehicle_name}/left_wheel_encoder_node/tick",
            WheelEncoderStamped,
            self.callback_left
        )
        self.sub_right = rospy.Subscriber(
            f"/{self._vehicle_name}/right_wheel_encoder_node/tick",
            WheelEncoderStamped,
            self.callback_right
        )
        rospy.loginfo(f"[{node_name}] Subscribed to encoder topics for '{self._vehicle_name}'")

    def callback_left(self, data):
        if self._ticks_left is None:
            rospy.loginfo(f"Left encoder resolution: {data.resolution}")
        self._ticks_left = data.data

    def callback_right(self, data):
        if self._ticks_right is None:
            rospy.loginfo(f"Right encoder resolution: {data.resolution}")
        self._ticks_right = data.data

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._ticks_left is not None and self._ticks_right is not None:
                rospy.loginfo_throttle(1, f"Ticks [L={self._ticks_left}, R={self._ticks_right}]")
            rate.sleep()


if __name__ == '__main__':
    node = WheelEncoderReaderNode(node_name='wheel_encoder_reader_node')
    node.run()
