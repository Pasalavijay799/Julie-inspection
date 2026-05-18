#!/usr/bin/env python3
"""
ZED 2i 3D Safety Zone Viewer — Camera-frame approach, no TF needed.

Works entirely in the ZED left camera optical frame (Z = depth forward,
X = right, Y = down). The camera is FIXED on its stand, so we define the
safety cylinder relative to the camera:

    ROBOT_X_CAM, ROBOT_Y_CAM, ROBOT_Z_CAM  — robot base in camera space

Tune these three values so the cylinder sits on the physical robot base.
A quick way: look at the depth value at the robot base on-screen and use
that as ROBOT_Z_CAM. Set X/Y to match the robot's lateral and vertical
position in the frame.

Zone:
  Cylinder radius = ZONE_RADIUS  m  (horizontal on XZ floor plane)
  Cylinder height = ZONE_HEIGHT  m  (along Y axis downward, Y-down frame)
"""

import threading
import warnings
warnings.filterwarnings("ignore")   # suppress sm_120 / scipy version warnings

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge

import cv2
import numpy as np
from ultralytics import YOLO

# ──────────────────────────────────────────────────────────────────────────────
# TUNE THESE  ← adjust so the cylinder is drawn on the real robot base
# All in metres, camera optical frame (Z = depth from camera)
# ──────────────────────────────────────────────────────────────────────────────
ROBOT_X_CAM  = -0.10   # robot base lateral offset from camera axis (pos = right)
ROBOT_Y_CAM  =  0.40   # robot base vertical offset  (pos = down in optical frame)
ROBOT_Z_CAM  =  1.80   # robot base depth from camera (metres — read off from screen)

ZONE_RADIUS  =  0.60   # ← tight radius around robot footprint (~60 cm)
ZONE_HEIGHT  =  2.20   # cylinder height (metres, upward)
N_RING_PTS   =  36     # wireframe resolution

DEPTH_PATCH  =  5      # pixels: patch radius for stable depth sampling


# ──────────────────────────────────────────────────────────────────────────────
# Geometry — all in camera optical frame
# ──────────────────────────────────────────────────────────────────────────────
def inside_cylinder_cam(Xp, Yp, Zp) -> bool:
    """Person at camera-frame (Xp,Yp,Zp) — is their base inside the cylinder?"""
    if not np.isfinite(Zp) or Zp <= 0:
        return False
    # Horizontal distance on the XZ floor plane from robot base
    horiz = np.sqrt((Xp - ROBOT_X_CAM)**2 + (Zp - ROBOT_Z_CAM)**2)
    return horiz <= ZONE_RADIUS  # height check optional; works without Y check too


def cam_to_pixel(X, Y, Z, fx, fy, cx, cy):
    """Project camera-frame 3D point to image pixel."""
    if Z <= 0:
        return None
    u = int(fx * X / Z + cx)
    v = int(fy * Y / Z + cy)
    return (u, v)


def draw_cylinder_cam(frame, fx, fy, cx, cy, color):
    """
    Draw the safety cylinder wireframe directly in camera frame —
    no TF needed since the cylinder IS defined in camera frame.
    """
    h, w = frame.shape[:2]

    def ok(pt):
        return pt is not None and 0 <= pt[0] < w and 0 <= pt[1] < h

    angles = np.linspace(0, 2 * np.pi, N_RING_PTS, endpoint=False)

    # Y levels of cylinder (camera optical frame: Y positive = downward)
    y_top = ROBOT_Y_CAM - ZONE_HEIGHT   # top of cylinder (above robot)
    y_bot = ROBOT_Y_CAM                  # bottom at robot base height

    for y_lvl in [y_top, y_bot]:
        ring = []
        for a in angles:
            X = ROBOT_X_CAM + ZONE_RADIUS * np.cos(a)
            Z = ROBOT_Z_CAM + ZONE_RADIUS * np.sin(a)
            ring.append(cam_to_pixel(X, y_lvl, Z, fx, fy, cx, cy))
        for i in range(len(ring)):
            p1, p2 = ring[i], ring[(i + 1) % len(ring)]
            if ok(p1) and ok(p2):
                cv2.line(frame, p1, p2, color, 2)

    # Vertical pillars every 4th point
    for i in range(0, N_RING_PTS, 4):
        a   = angles[i]
        X   = ROBOT_X_CAM + ZONE_RADIUS * np.cos(a)
        Z   = ROBOT_Z_CAM + ZONE_RADIUS * np.sin(a)
        pb  = cam_to_pixel(X, y_bot, Z, fx, fy, cx, cy)
        pt2 = cam_to_pixel(X, y_top, Z, fx, fy, cx, cy)
        if ok(pb) and ok(pt2):
            cv2.line(frame, pb, pt2, color, 2)

    # Draw robot base marker (crosshair)
    center_px = cam_to_pixel(ROBOT_X_CAM, ROBOT_Y_CAM, ROBOT_Z_CAM, fx, fy, cx, cy)
    if ok(center_px):
        cv2.drawMarker(frame, center_px, (255, 255, 0),
                       cv2.MARKER_CROSS, 20, 2)


# ──────────────────────────────────────────────────────────────────────────────
# ROS 2 Node
# ──────────────────────────────────────────────────────────────────────────────
class SafetyCameraViewer(Node):

    def __init__(self):
        super().__init__('safety_camera_viewer')

        self.bridge = CvBridge()

        # YOLO on GPU
        self.get_logger().info("Loading YOLOv8n model on RTX 5050 GPU...")
        self.yolo = YOLO("yolov8n.pt")
        self.yolo.to('cuda')
        self.get_logger().info("YOLO ready.")

        # Shared state
        self.lock      = threading.Lock()
        self.rgb       = None
        self.depth     = None
        self.cam_info  = None

        # ROS publisher
        self.safety_pub = self.create_publisher(Bool, '/safety_override', 10)

        # Subscribers
        self.create_subscription(Image,      '/zed/zed_node/left/image_rect_color',
                                  self._rgb_cb,  1)
        self.create_subscription(Image,      '/zed/zed_node/depth/depth_registered',
                                  self._dep_cb,  1)
        self.create_subscription(CameraInfo, '/zed/zed_node/left/camera_info',
                                  self._info_cb, 1)

        # Background thread for YOLO+render — decoupled from ROS spin
        # This ensures the LATEST frame is ALWAYS processed without any timer lag
        self._running = True
        self._thread  = threading.Thread(target=self._process_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"Safety viewer ready — cylinder r={ZONE_RADIUS}m around "
            f"(X={ROBOT_X_CAM}, Y={ROBOT_Y_CAM}, Z={ROBOT_Z_CAM})m in cam frame."
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _rgb_cb(self, msg):
        with self.lock:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _dep_cb(self, msg):
        with self.lock:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _info_cb(self, msg):
        with self.lock:
            self.cam_info = msg

    # ── Stable depth sample ───────────────────────────────────────────────────
    def _sample_depth(self, dmap, u, v):
        h, w = dmap.shape[:2]
        r = DEPTH_PATCH
        patch = dmap[max(0, v-r):min(h, v+r), max(0, u-r):min(w, u+r)].flatten()
        valid = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 20.0)]
        return float(np.median(valid)) if len(valid) else None

    # ── Background processing loop (runs in thread, always latest frame) ──────
    def _process_loop(self):
        while self._running:
            self._process()

    # ── Main processing step ──────────────────────────────────────────────────
    def _process(self):
        with self.lock:
            if self.rgb is None or self.depth is None or self.cam_info is None:
                return
            frame = self.rgb.copy()
            dmap  = self.depth.copy()
            info  = self.cam_info

        # Camera intrinsics from K matrix
        fx, fy = info.k[0], info.k[4]
        cx, cy = info.k[2], info.k[5]

        # ── YOLO inference (GPU) ───────────────────────────────────────────
        results = self.yolo(frame, classes=[0], verbose=False)

        zone_breached = False

        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                if float(box.conf[0]) < 0.40:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # Sample depth at FEET (bottom-centre of box)
                fu, fv  = (x1 + x2) // 2, y2
                depth_m = self._sample_depth(dmap, fu, fv)

                person_inside = False
                depth_txt     = "depth?"

                if depth_m is not None:
                    depth_txt = f"Z={depth_m:.2f}m"
                    # Back-project to camera frame
                    Xp = (fu - cx) * depth_m / fx
                    Yp = (fv - cy) * depth_m / fy
                    Zp = depth_m

                    if inside_cylinder_cam(Xp, Yp, Zp):
                        person_inside = True
                        zone_breached = True

                # Draw box
                bc = (0, 0, 255) if person_inside else (255, 165, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), bc, 3)
                label = ("IN ZONE  " if person_inside else "") + depth_txt
                cv2.putText(frame, label, (x1, max(y1 - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, bc, 2)
                cv2.circle(frame, (fu, fv), 5, bc, -1)

        # ── Publish safety state ───────────────────────────────────────────
        msg = Bool()
        msg.data = zone_breached
        self.safety_pub.publish(msg)

        # ── Draw cylinder wireframe (camera frame, no TF) ─────────────────
        wire_col = (0, 0, 255) if zone_breached else (0, 255, 0)
        draw_cylinder_cam(frame, fx, fy, cx, cy, wire_col)

        # ── HUD banner ─────────────────────────────────────────────────────
        bar = (0, 0, 180) if zone_breached else (0, 130, 0)
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 70), bar, -1)
        txt = ("SAFETY OVERRIDE — HUMAN IN ZONE — ROBOT STOPPED"
               if zone_breached else "ZONE CLEAR — ROBOT RUNNING")
        cv2.putText(frame, txt, (12, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)

        # Tuning hint overlay
        cv2.putText(frame,
                    f"Robot base @ X={ROBOT_X_CAM:.2f} Y={ROBOT_Y_CAM:.2f} Z={ROBOT_Z_CAM:.2f}m",
                    (10, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow("ZED 2i  —  3D Cylinder Safety Zone", frame)
        cv2.waitKey(1)


# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = SafetyCameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node._thread.join(timeout=2.0)
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
