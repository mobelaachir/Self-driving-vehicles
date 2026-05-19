#!/usr/bin/env python3
import os
import sys
import tty
import termios
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped

HELP = """
Duckiebot Keyboard Control
--------------------------
W : vooruit
S : achteruit
A : links draaien
D : rechts draaien
SPATIE : stoppen
Q : afsluiten
--------------------------
"""

SPEED = 0.4
TURN  = 0.3

KEY_BINDINGS = {
    'w': ( SPEED,  SPEED),
    's': (-SPEED, -SPEED),
    'a': (-TURN,   TURN),
    'd': ( TURN,  -TURN),
    ' ': (0.0,    0.0),
}

class WheelControlNode(DTROS):
    def __init__(self, node_name):
        super(WheelControlNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.GENERIC
        )
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'duckiebot21')
        self._wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(
            self._wheels_topic, WheelsCmdStamped, queue_size=1
        )
        rospy.loginfo(f"Publisher created for {self._wheels_topic}")

    def publish(self, left, right):
        msg = WheelsCmdStamped()
        msg.vel_left  = left
        msg.vel_right = right
        self._publisher.publish(msg)

    def stop(self):
        self.publish(0.0, 0.0)

    def on_shutdown(self):
        self.stop()

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    node = WheelControlNode(node_name='wheel_control_node')
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    print(f"Verbonden met: /{node._vehicle_name}/wheels_driver_node/wheels_cmd")
    rate = rospy.Rate(10)
    left, right = 0.0, 0.0
    try:
        while not rospy.is_shutdown():
            key = get_key(settings)
            if key == 'q':
                rospy.loginfo("Afsluiten...")
                break
            elif key in KEY_BINDINGS:
                left, right = KEY_BINDINGS[key]
                action = {'w':'Vooruit','s':'Achteruit','a':'Links','d':'Rechts',' ':'Stop'}
                rospy.loginfo(f"{action.get(key, key)} -> L={left:.1f} R={right:.1f}")
            node.publish(left, right)
            rate.sleep()
    except Exception as e:
        rospy.logerr(f"Fout: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        rospy.loginfo("Gestopt.")

if __name__ == '__main__':
    main()
