#!/usr/bin/env python3
"""
Safety Zone Node — 3D boundary around robot using ZED 2i body tracking.

Subscribes to ZED body tracking, checks if any person is within a
configurable cylindrical boundary around the robot, and publishes
Bool(True) on /safety_override to pause the robot.

Also publishes a MarkerArray for RViz visualization of the safety zone.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from zed_msgs.msg import ObjectsStamped

import tf2_ros
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # noqa: F401 — registers PointStamped transform


class SafetyZoneNode(Node):
    """Monitors ZED body tracking and triggers safety override when person enters zone."""

    def __init__(self):
        super().__init__('safety_zone_node')

        # ---- Configurable parameters ----
        self.declare_parameter('safety_radius', 1.0)      # meters
        self.declare_parameter('safety_height', 2.0)       # meters
        self.declare_parameter('robot_center_x', 0.0)      # world frame X
        self.declare_parameter('robot_center_y', 0.0)      # world frame Y
        self.declare_parameter('check_rate_hz', 10.0)

        self.safety_radius = self.get_parameter('safety_radius').value
        self.safety_height = self.get_parameter('safety_height').value
        self.robot_x = self.get_parameter('robot_center_x').value
        self.robot_y = self.get_parameter('robot_center_y').value

        self.get_logger().info(
            f'Safety zone: radius={self.safety_radius}m, '
            f'height={self.safety_height}m, '
            f'center=({self.robot_x}, {self.robot_y})'
        )

        # ---- TF2 ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Subscribers ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.body_sub = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/body_trk/skeletons',
            self._body_callback,
            qos,
        )

        # ---- Publishers ----
        self.safety_pub = self.create_publisher(Bool, '/safety_override', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/safety_zone_marker', 10)

        # ---- State ----
        self.person_inside = False
        self.last_detection_time = self.get_clock().now()

        # ---- Timer for zone visualization ----
        rate = self.get_parameter('check_rate_hz').value
        self.create_timer(1.0 / rate, self._publish_markers)

        # ---- Timer for timeout (no detections = safe) ----
        self.create_timer(0.5, self._check_timeout)

        self.get_logger().info('Safety zone node started — waiting for body tracking data...')

    def _body_callback(self, msg: ObjectsStamped):
        """Check if any detected body is inside the safety zone."""
        self.last_detection_time = self.get_clock().now()

        anybody_inside = False

        for obj in msg.objects:
            # obj.position is [x, y, z] in the camera optical frame
            pos = obj.position  # float32[3]

            # Skip invalid positions (NaN or zero)
            if math.isnan(pos[0]) or math.isnan(pos[1]) or math.isnan(pos[2]):
                continue

            # Transform position from camera frame to world frame
            world_pos = self._transform_to_world(
                pos[0], pos[1], pos[2], msg.header
            )
            if world_pos is None:
                continue

            # Check if inside cylinder
            dx = world_pos[0] - self.robot_x
            dy = world_pos[1] - self.robot_y
            dist_2d = math.sqrt(dx * dx + dy * dy)

            if dist_2d < self.safety_radius and 0.0 <= world_pos[2] <= self.safety_height:
                anybody_inside = True
                self.get_logger().warn(
                    f'PERSON DETECTED inside zone! dist={dist_2d:.2f}m '
                    f'pos=({world_pos[0]:.2f}, {world_pos[1]:.2f}, {world_pos[2]:.2f})',
                    throttle_duration_sec=1.0,
                )
                break

        # Publish safety override
        msg_out = Bool()
        msg_out.data = anybody_inside
        self.safety_pub.publish(msg_out)

        if anybody_inside and not self.person_inside:
            self.get_logger().error('⚠ SAFETY OVERRIDE: Person entered zone — STOPPING robot')
        elif not anybody_inside and self.person_inside:
            self.get_logger().info('✓ Safety zone clear — resuming robot')

        self.person_inside = anybody_inside

    def _transform_to_world(self, x, y, z, header):
        """Transform a 3D point from camera frame to world frame."""
        pt = PointStamped()
        pt.header = header
        pt.point.x = float(x)
        pt.point.y = float(y)
        pt.point.z = float(z)

        try:
            pt_world = self.tf_buffer.transform(pt, 'world', timeout=rclpy.duration.Duration(seconds=0.1))
            return [pt_world.point.x, pt_world.point.y, pt_world.point.z]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF transform failed: {e}', throttle_duration_sec=5.0
            )
            return None

    def _check_timeout(self):
        """If no body tracking data for 2 seconds, assume safe."""
        elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if elapsed > 2.0 and self.person_inside:
            self.get_logger().info('No body tracking data for 2s — clearing safety override')
            self.person_inside = False
            msg = Bool()
            msg.data = False
            self.safety_pub.publish(msg)

    def _publish_markers(self):
        """Publish cylinder marker showing the safety zone in RViz."""
        markers = MarkerArray()

        # Cylinder for safety zone
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'safety_zone'
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = self.robot_x
        m.pose.position.y = self.robot_y
        m.pose.position.z = self.safety_height / 2.0
        m.pose.orientation.w = 1.0
        m.scale.x = self.safety_radius * 2.0  # diameter
        m.scale.y = self.safety_radius * 2.0
        m.scale.z = self.safety_height

        if self.person_inside:
            # RED when person is inside
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 0.3
        else:
            # GREEN when safe
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.15

        m.lifetime.sec = 0  # persistent
        markers.markers.append(m)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyZoneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
