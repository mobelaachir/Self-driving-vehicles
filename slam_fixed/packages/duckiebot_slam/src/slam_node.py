#!/usr/bin/env python3
"""
- Vision-based Monocular SLAM Node
Detecteert en volgt visuele kenmerken (ORB features) over frames.
Bouwt een kaart van feature points en gedetecteerde objecten.
Publiceert:
  /<veh>/slam_node/features/compressed  — camera met features
  /<veh>/slam_node/map/compressed        — 2D kaart
  /<veh>/slam_node/corrections           — visuele correctie voor fusie
"""
import os
import math
import rospy
import cv2
import numpy as np
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge


class SLAMNode(DTROS):
    def __init__(self, node_name):
        super(SLAMNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)

        veh = os.environ.get('VEHICLE_NAME', 'duckiebot21')
        self._veh = veh
        self._bridge = CvBridge()

        # ORB feature detector
        self.orb = cv2.ORB_create(500)
        self.bf  = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Feature tracking state
        self.prev_gray        = None
        self.prev_keypoints   = None
        self.prev_descriptors = None
        self.prev_pts         = None  # voor Lucas-Kanade tracking

        # Kaart
        self.map_points   = []   # (world_x, world_y, strength)
        self.trajectory   = [(0.0, 0.0)]

        # Pose (ontvangen van odometry/fusie)
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # Visuele bewegingsschatting
        self.visual_dx    = 0.0
        self.visual_dy    = 0.0
        self.frame_count  = 0

        # Kaart canvas
        self.map_size = 600
        self.scale    = 150   # pixels per meter
        self.map_img  = np.zeros((self.map_size, self.map_size, 3), dtype=np.uint8)

        # Publishers
        self.pub_features = rospy.Publisher(
            f"/{veh}/slam_node/features/compressed", CompressedImage, queue_size=1)
        self.pub_map = rospy.Publisher(
            f"/{veh}/slam_node/map/compressed", CompressedImage, queue_size=1)
        self.pub_correction = rospy.Publisher(
            f"/{veh}/slam_node/correction", Float32MultiArray, queue_size=1)

        # Subscribers
        rospy.Subscriber(f"/{veh}/camera_node/image/compressed",
                         CompressedImage, self.cb_camera, queue_size=1, buff_size=2**24)
        rospy.Subscriber(f"/{veh}/fusion_node/pose",
                         Odometry, self.cb_pose)

        rospy.loginfo(f"[{node_name}] SLAM node klaar voor '{veh}'")
        rospy.loginfo(f"  features -> /{veh}/slam_node/features/compressed")
        rospy.loginfo(f"  map      -> /{veh}/slam_node/map/compressed")

    def cb_pose(self, msg):
        """Ontvang gefuseerde pose van fusion_node."""
        self.x     = msg.pose.pose.position.x
        self.y     = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        self.theta = math.atan2(siny, cosy)
        if not self.trajectory or (self.x, self.y) != self.trajectory[-1]:
            self.trajectory.append((self.x, self.y))

    def cb_camera(self, msg):
        try:
            frame = self._bridge.compressed_imgmsg_to_cv2(msg)
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.frame_count += 1

            # ── ORB detectie
            keypoints, descriptors = self.orb.detectAndCompute(gray, None)

            # ── Lucas-Kanade optical flow tracking 
            flow_viz = frame.copy()
            if self.prev_gray is not None and self.prev_pts is not None and len(self.prev_pts) > 0:
                lk_params = dict(winSize=(15,15), maxLevel=2,
                                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
                curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                    self.prev_gray, gray, self.prev_pts, None, **lk_params)

                good_new = curr_pts[status == 1]
                good_old = self.prev_pts[status == 1]

                # Teken flow vectoren
                for new, old in zip(good_new, good_old):
                    a, b = int(new[0]), int(new[1])
                    c, d = int(old[0]), int(old[1])
                    cv2.arrowedLine(flow_viz, (c,d), (a,b), (0, 255, 255), 1, tipLength=0.3)
                    cv2.circle(flow_viz, (a,b), 2, (0, 200, 0), -1)

                # Schat visuele beweging via gemiddelde flow
                if len(good_new) > 5:
                    flow = good_new - good_old
                    self.visual_dx = float(np.median(flow[:,0]))
                    self.visual_dy = float(np.median(flow[:,1]))

                    # Publiceer correctie voor fusie node
                    corr = Float32MultiArray()
                    corr.data = [self.visual_dx, self.visual_dy]
                    self.pub_correction.publish(corr)

                self.prev_pts = good_new.reshape(-1, 1, 2) if len(good_new) > 0 else None
            else:
                # Initialiseer tracking punten vanuit ORB keypoints
                if keypoints:
                    self.prev_pts = np.array(
                        [[kp.pt] for kp in keypoints[:100]], dtype=np.float32)

            # ── Match ORB features voor kaartpunten
            if (self.prev_descriptors is not None and
                    descriptors is not None and len(descriptors) > 10):
                matches = sorted(
                    self.bf.match(self.prev_descriptors, descriptors),
                    key=lambda m: m.distance)
                good = [m for m in matches[:40] if m.distance < 50]
                for m in good:
                    pt = keypoints[m.trainIdx].pt
                    # Zet pixel naar wereld coordinaten (vereenvoudigd)
                    wx = self.x + (pt[0] - 320) / 800.0 * math.cos(self.theta)
                    wy = self.y + (pt[0] - 320) / 800.0 * math.sin(self.theta)
                    strength = 1.0 - m.distance / 100.0
                    self.map_points.append((wx, wy, max(0.1, strength)))

            self.prev_gray        = gray.copy()
            self.prev_keypoints   = keypoints
            self.prev_descriptors = descriptors

            # ── Features visualisatie
            # ORB hoekpunten als kleine kruisjes
            for kp in keypoints:
                x, y = int(kp.pt[0]), int(kp.pt[1])
                cv2.drawMarker(flow_viz, (x,y), (0,255,0), cv2.MARKER_CROSS, 6, 1)

            # HUD
            cv2.rectangle(flow_viz, (0,0), (320,85), (0,0,0), -1)
            cv2.putText(flow_viz, f"ORB features : {len(keypoints)}",
                        (8,22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 1)
            cv2.putText(flow_viz, f"Map landmarks: {len(self.map_points)}",
                        (8,44), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,150,0), 1)
            cv2.putText(flow_viz, f"Flow dx={self.visual_dx:+.1f} dy={self.visual_dy:+.1f}",
                        (8,66), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,200,255), 1)
            cv2.putText(flow_viz, f"Frame: {self.frame_count}",
                        (8,84), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180,180,180), 1)

            feat_msg = self._bridge.cv2_to_compressed_imgmsg(flow_viz)
            feat_msg.header.stamp = rospy.Time.now()
            self.pub_features.publish(feat_msg)

            # ── Kaart publiceren 
            self._redraw_map()
            self._publish_map()

        except Exception as e:
            rospy.logerr_throttle(5, f"[SLAM] Camera error: {e}")

    def _world_to_px(self, wx, wy):
        cx = cy = self.map_size // 2
        return (int(cx + wx * self.scale), int(cy - wy * self.scale))

    def _redraw_map(self):
        # Achtergrond + grid
        self.map_img[:] = (15, 15, 15)
        cx = cy = self.map_size // 2
        for d in range(-4, 5):
            px = cx + int(d * self.scale)
            py = cy + int(d * self.scale)
            if 0 <= px < self.map_size:
                cv2.line(self.map_img, (px,0),(px,self.map_size),(40,40,40),1)
            if 0 <= py < self.map_size:
                cv2.line(self.map_img, (0,py),(self.map_size,py),(40,40,40),1)
            if 0 <= px < self.map_size:
                cv2.putText(self.map_img, f"{d}m",
                            (px+2, cy-2), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (60,60,60), 1)

        # Landmarks (kleurgecodeerd op sterkte)
        for (mx, my, strength) in self.map_points[-800:]:
            pt = self._world_to_px(mx, my)
            if 0 <= pt[0] < self.map_size and 0 <= pt[1] < self.map_size:
                r = int(255 * (1-strength))
                b = int(255 * strength)
                cv2.circle(self.map_img, pt, 1, (b, 80, r), -1)

        # Trajectory
        for i in range(1, len(self.trajectory)):
            pt1 = self._world_to_px(*self.trajectory[i-1])
            pt2 = self._world_to_px(*self.trajectory[i])
            if all(0 <= v < self.map_size for v in [pt1[0],pt1[1],pt2[0],pt2[1]]):
                cv2.line(self.map_img, pt1, pt2, (0, 200, 0), 2)

        # Startpunt
        start = self._world_to_px(0.0, 0.0)
        cv2.drawMarker(self.map_img, start, (0,255,255), cv2.MARKER_STAR, 16, 2)
        cv2.putText(self.map_img, "START",
                    (start[0]+8, start[1]+5), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,255,255), 1)

        # Huidige positie
        curr = self._world_to_px(self.x, self.y)
        if 0 <= curr[0] < self.map_size and 0 <= curr[1] < self.map_size:
            cv2.circle(self.map_img, curr, 8, (0,0,255), -1)
            end = (int(curr[0] + 20*math.cos(self.theta)),
                   int(curr[1] - 20*math.sin(self.theta)))
            cv2.arrowedLine(self.map_img, curr, end, (0,180,255), 2, tipLength=0.35)

        # HUD
        dist = math.hypot(self.x, self.y)
        cv2.rectangle(self.map_img, (0,0),(240,90),(0,0,0),-1)
        cv2.putText(self.map_img, f"x = {self.x:+.3f} m",
                    (8,18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1)
        cv2.putText(self.map_img, f"y = {self.y:+.3f} m",
                    (8,36), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1)
        cv2.putText(self.map_img, f"hdg = {math.degrees(self.theta):+.1f} deg",
                    (8,54), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(200,200,255),1)
        cv2.putText(self.map_img, f"dist={dist:.3f}m  pts={len(self.map_points)}",
                    (8,72), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(150,200,150),1)
        cv2.putText(self.map_img, f"flow dx={self.visual_dx:+.1f} dy={self.visual_dy:+.1f}",
                    (8,88), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0,200,255),1)

    def _publish_map(self):
        try:
            msg = self._bridge.cv2_to_compressed_imgmsg(self.map_img)
            msg.header.stamp = rospy.Time.now()
            self.pub_map.publish(msg)
        except Exception as e:
            rospy.logerr_throttle(5, f"[SLAM] Map error: {e}")


if __name__ == '__main__':
    node = SLAMNode(node_name='slam_node')
    rospy.spin()
