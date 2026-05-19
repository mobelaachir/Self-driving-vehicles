#!/usr/bin/env python3

import os
import rospy
import cv2
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class CameraReaderNode(DTROS):
    def __init__(self, node_name):
        super(CameraReaderNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.VISUALIZATION
        )
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'duckiebot')
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._bridge = CvBridge()

        self.sub = rospy.Subscriber(
            self._camera_topic,
            CompressedImage,
            self.callback,
            queue_size=1,
            buff_size=2**24  # nodig voor grote compressed images
        )
        rospy.loginfo(f"[{node_name}] Subscribed to {self._camera_topic}")

    def callback(self, msg):
        try:
            image = self._bridge.compressed_imgmsg_to_cv2(msg)
            h, w = image.shape[:2]
            rospy.loginfo_throttle(5, f"Received image: {w}x{h}")
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")


if __name__ == '__main__':
    node = CameraReaderNode(node_name='camera_reader_node')
    rospy.spin()
