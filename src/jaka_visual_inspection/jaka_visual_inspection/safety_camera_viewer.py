#!/usr/bin/env python3
"""
ZED 2i Real-Time Safety Zone Viewer
=====================================
Safety detection: YOLO-ONLY (no depth-based triggering — too noisy).
YOLO correctly identifies humans, never triggers on robot arm or sensor noise.

Debounce:
  ENTER_FRAMES consecutive YOLO detections  → trigger safety stop
  LEAVE_FRAMES consecutive clear frames     → resume robot

Display: smooth 30+ FPS live video — YOLO overlay is async (never blocks display).
"""

import threading
import time
import warnings
warnings.filterwarnings("ignore")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge

import cv2
import numpy as np
from ultralytics import YOLO

# ──────────────────────────────────────────────────────────────────────────────
#  ZONE CONFIG  (camera optical frame — Z=depth, X=right, Y=down)
#  Set ROBOT_Z_CAM to the depth value shown at the yellow crosshair on screen.
# ──────────────────────────────────────────────────────────────────────────────
ROBOT_X_CAM = -0.10   # metres (+ = right of camera centre)
ROBOT_Y_CAM =  0.40   # metres (+ = below camera centre)
ROBOT_Z_CAM =  1.80   # metres — depth from camera to robot base

ZONE_RADIUS  =  1.00   # safety cylinder radius (metres)
ZONE_HEIGHT  =  2.20   # cylinder height (metres)
N_RING_PTS   =  30     # wireframe ring segments
DEPTH_PATCH  =   5     # px: depth patch radius for stable sampling
YOLO_INFER_W = 640     # YOLO input width (640 = best accuracy/speed balance)
YOLO_CONF    = 0.50    # minimum detection confidence

# Debounce: prevents glitching on single-frame noise
ENTER_FRAMES = 3   # YOLO must detect person inside zone for N frames to STOP
LEAVE_FRAMES = 5   # YOLO must see NO person for N frames to RESUME


# ──────────────────────────────────────────────────────────────────────────────
#  Geometry helpers
# ──────────────────────────────────────────────────────────────────────────────
def inside_cylinder(Xp: float, Zp: float) -> bool:
    return np.sqrt((Xp - ROBOT_X_CAM)**2 + (Zp - ROBOT_Z_CAM)**2) <= ZONE_RADIUS


def cam_to_px(X, Y, Z, fx, fy, cx, cy):
    if Z <= 0:
        return None
    return int(fx * X / Z + cx), int(fy * Y / Z + cy)


def draw_cylinder(frame, fx, fy, cx, cy, color):
    h, w = frame.shape[:2]
    def ok(p):
        return p and 0 <= p[0] < w and 0 <= p[1] < h
    angles = np.linspace(0, 2 * np.pi, N_RING_PTS, endpoint=False)
    rc = ZONE_RADIUS * np.cos(angles)
    rs = ZONE_RADIUS * np.sin(angles)
    y_top = ROBOT_Y_CAM - ZONE_HEIGHT
    y_bot = ROBOT_Y_CAM
    for y_lvl in (y_bot, y_top):
        ring = [cam_to_px(ROBOT_X_CAM + rc[i], y_lvl,
                          ROBOT_Z_CAM + rs[i], fx, fy, cx, cy)
                for i in range(N_RING_PTS)]
        for i in range(N_RING_PTS):
            p1, p2 = ring[i], ring[(i + 1) % N_RING_PTS]
            if ok(p1) and ok(p2):
                cv2.line(frame, p1, p2, color, 2)
    for i in range(0, N_RING_PTS, 4):
        pb = cam_to_px(ROBOT_X_CAM + rc[i], y_bot,
                       ROBOT_Z_CAM + rs[i], fx, fy, cx, cy)
        pt = cam_to_px(ROBOT_X_CAM + rc[i], y_top,
                       ROBOT_Z_CAM + rs[i], fx, fy, cx, cy)
        if ok(pb) and ok(pt):
            cv2.line(frame, pb, pt, color, 2)
    ctr = cam_to_px(ROBOT_X_CAM, ROBOT_Y_CAM, ROBOT_Z_CAM, fx, fy, cx, cy)
    if ok(ctr):
        cv2.drawMarker(frame, ctr, (0, 255, 255), cv2.MARKER_CROSS, 24, 2)


def sample_depth(dmap, u, v):
    h, w = dmap.shape[:2]
    r = DEPTH_PATCH
    patch = dmap[max(0,v-r):min(h,v+r), max(0,u-r):min(w,u+r)].flatten()
    valid = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 20.0)]
    return float(np.median(valid)) if len(valid) else None


# ──────────────────────────────────────────────────────────────────────────────
#  Shared state
# ──────────────────────────────────────────────────────────────────────────────
class SharedState:
    def __init__(self):
        self.lock          = threading.Lock()
        self.raw_frame     = None
        self.depth         = None
        self.cam_info      = None
        self.overlay       = None   # latest annotated frame from YOLO worker
        self.zone_breached = False  # debounced safety state
        self.new_raw       = threading.Event()


# ──────────────────────────────────────────────────────────────────────────────
#  YOLO worker — detects humans, applies debounce, updates overlay
# ──────────────────────────────────────────────────────────────────────────────
def yolo_worker(state: SharedState, yolo: YOLO, safety_pub, running: threading.Event):
    enter_count = 0   # consecutive frames WITH person in zone
    leave_count = 0   # consecutive frames WITHOUT person in zone
    breached    = False

    while running.is_set():
        if not state.new_raw.wait(timeout=0.1):
            continue
        state.new_raw.clear()

        with state.lock:
            if state.raw_frame is None or state.cam_info is None:
                continue
            frame = state.raw_frame.copy()
            dmap  = state.depth.copy() if state.depth is not None else None
            info  = state.cam_info

        fx, fy = info.k[0], info.k[4]
        cx, cy = info.k[2], info.k[5]
        h_full, w_full = frame.shape[:2]

        # ── YOLO inference ────────────────────────────────────────────────
        scale  = YOLO_INFER_W / w_full
        h_inf  = int(h_full * scale)
        small  = cv2.resize(frame, (YOLO_INFER_W, h_inf),
                            interpolation=cv2.INTER_LINEAR)
        results = yolo(small, classes=[0], verbose=False, imgsz=YOLO_INFER_W)

        person_in_zone = False   # this frame only

        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                if float(box.conf[0]) < YOLO_CONF:
                    continue
                x1, y1, x2, y2 = (int(v / scale) for v in box.xyxy[0])
                x1 = max(0, min(x1, w_full-1))
                y1 = max(0, min(y1, h_full-1))
                x2 = max(0, min(x2, w_full-1))
                y2 = max(0, min(y2, h_full-1))

                fu, fv  = (x1 + x2) // 2, y2
                depth_m = sample_depth(dmap, fu, fv) if dmap is not None else None
                inside  = False
                dep_txt = "depth?"

                if depth_m is not None:
                    dep_txt = f"Z={depth_m:.2f}m"
                    Xp = (fu - cx) * depth_m / fx
                    if inside_cylinder(Xp, depth_m):
                        inside = person_in_zone = True

                bc = (0, 0, 255) if inside else (255, 165, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), bc, 3)
                cv2.putText(frame, ("IN ZONE  " if inside else "") + dep_txt,
                            (x1, max(y1 - 10, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, bc, 2)
                cv2.circle(frame, (fu, fv), 6, bc, -1)

        # ── Debounce logic ────────────────────────────────────────────────
        if person_in_zone:
            enter_count += 1
            leave_count  = 0
            if enter_count >= ENTER_FRAMES and not breached:
                breached = True
        else:
            leave_count += 1
            enter_count  = 0
            if leave_count >= LEAVE_FRAMES and breached:
                breached = False

        # ── Publish debounced safety state ────────────────────────────────
        msg = Bool(); msg.data = breached
        safety_pub.publish(msg)

        # ── Draw cylinder ─────────────────────────────────────────────────
        draw_cylinder(frame, fx, fy, cx, cy,
                      (0, 0, 255) if breached else (0, 255, 0))

        # ── HUD banner ────────────────────────────────────────────────────
        bar = (0, 0, 200) if breached else (0, 140, 0)
        cv2.rectangle(frame, (0, 0), (w_full, 70), bar, -1)
        status = ("HUMAN IN ZONE — ROBOT STOPPED"
                  if breached else "ZONE CLEAR — ROBOT RUNNING")
        cv2.putText(frame, status, (12, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        # debounce state indicator (small, bottom-left)
        dbg = f"enter:{enter_count}/{ENTER_FRAMES}  leave:{leave_count}/{LEAVE_FRAMES}"
        cv2.putText(frame, dbg, (10, h_full - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (140, 140, 140), 1)

        with state.lock:
            state.overlay      = frame
            state.zone_breached = breached


# ──────────────────────────────────────────────────────────────────────────────
#  ROS Node — receives frames only, no safety logic here
# ──────────────────────────────────────────────────────────────────────────────
class SafetyCameraViewer(Node):
    def __init__(self, state: SharedState):
        super().__init__('safety_camera_viewer')
        self.bridge     = CvBridge()
        self.state      = state
        self.safety_pub = self.create_publisher(Bool, '/safety_override', 10)

        self.create_subscription(Image,      '/zed/zed_node/left/image_rect_color',
                                  self._rgb_cb,  1)
        self.create_subscription(Image,      '/zed/zed_node/depth/depth_registered',
                                  self._dep_cb,  1)
        self.create_subscription(CameraInfo, '/zed/zed_node/left/camera_info',
                                  self._info_cb, 1)
        self.get_logger().info(
            f"Safety viewer ready  |  zone r={ZONE_RADIUS}m  "
            f"debounce enter={ENTER_FRAMES} leave={LEAVE_FRAMES} frames")

    def _rgb_cb(self, msg):
        rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        with self.state.lock:
            self.state.raw_frame = rgb
        self.state.new_raw.set()

    def _dep_cb(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        with self.state.lock:
            self.state.depth = depth

    def _info_cb(self, msg):
        with self.state.lock:
            self.state.cam_info = msg


# ──────────────────────────────────────────────────────────────────────────────
#  Main — imshow MUST be on main thread (Linux X11)
# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    state   = SharedState()
    running = threading.Event()
    running.set()

    rclpy.init(args=args)
    node = SafetyCameraViewer(state)

    node.get_logger().info("Loading YOLOv8n on GPU...")
    yolo = YOLO("yolov8n.pt")
    yolo.to('cuda')
    node.get_logger().info("YOLO ready.")

    worker = threading.Thread(
        target=yolo_worker,
        args=(state, yolo, node.safety_pub, running),
        daemon=True
    )
    worker.start()

    WIN = "ZED 2i  —  3D Cylinder Safety Zone"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 720)

    fps_buf = []
    t_prev  = time.perf_counter()

    try:
        while rclpy.ok() and running.is_set():
            rclpy.spin_once(node, timeout_sec=0.002)

            with state.lock:
                display = (state.overlay if state.overlay is not None
                           else state.raw_frame)

            if display is not None:
                t_now = time.perf_counter()
                fps_buf.append(1.0 / max(t_now - t_prev, 1e-6))
                t_prev = t_now
                if len(fps_buf) > 30:
                    fps_buf.pop(0)
                fps = sum(fps_buf) / len(fps_buf)

                disp = display.copy()
                cv2.putText(disp, f"{fps:.0f} FPS",
                            (disp.shape[1] - 120, 48),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 0), 2)
                cv2.imshow(WIN, disp)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        running.clear()
        state.new_raw.set()
        worker.join(timeout=3.0)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
