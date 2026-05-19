#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped

class TwistControlNode(DTROS):
    def __init__(self, node_name):
        super(TwistControlNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.GENERIC
        )
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'duckiebot')
        self._twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self._publisher = rospy.Publisher(self._twist_topic, Twist2DStamped, queue_size=1)
        rospy.loginfo(f"Publisher created for {self._twist_topic}")

    def send_cmd(self, v, omega, duration):
        msg = Twist2DStamped(v=v, omega=omega)
        self._publisher.publish(msg)
        rospy.sleep(duration)
        self.stop()

    def stop(self):
        stop_msg = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop_msg)

    def on_shutdown(self):
        self.stop()

if __name__ == '__main__':
    node = TwistControlNode(node_name='twist_control_node')
    rospy.sleep(1)
    rospy.loginfo("Sending twist command...")
    node.send_cmd(0.2, 0.0, 2.0) # 0.2 m/s vooruit voor 2 seconden
    rospy.loginfo("Stopped.")
    rospy.spin()
