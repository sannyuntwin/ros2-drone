#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint


class TrackingNode(Node):
    """
    Camera-based object tracking node.

    Subscribes to a camera topic, tracks a target with an OpenCV tracker,
    and publishes PX4 velocity setpoints to keep the target centred in frame.

    Topics
    ------
    Subscribed:
      /<image_topic>   sensor_msgs/Image  — raw camera frames
      ~/enable         std_msgs/Bool      — True to start tracking, False to stop

    Published:
      /fmu/in/offboard_control_mode   px4_msgs/OffboardControlMode
      /fmu/in/trajectory_setpoint     px4_msgs/TrajectorySetpoint
      ~/debug_image                   sensor_msgs/Image  — annotated frame
    """

    def __init__(self):
        super().__init__('tracking_node')

        self.declare_parameter('tracker_type', 'CSRT')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('gain_lateral', 0.005)   # image-px → m/s lateral
        self.declare_parameter('gain_vertical', 0.003)  # image-px → m/s vertical
        self.declare_parameter('max_velocity', 2.0)     # m/s clamp

        self._tracker_type = self.get_parameter('tracker_type').value
        image_topic = self.get_parameter('image_topic').value
        self._gain_lat = self.get_parameter('gain_lateral').value
        self._gain_vert = self.get_parameter('gain_vertical').value
        self._max_vel = self.get_parameter('max_velocity').value

        self._bridge = CvBridge()
        self._tracker = None
        self._tracking = False

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, image_topic, self._image_cb, img_qos)
        self.create_subscription(Bool, '~/enable', self._enable_cb, 10)

        self._offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self._trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self._debug_pub = self.create_publisher(Image, '~/debug_image', 10)

        self.get_logger().info(
            f'TrackingNode ready ({self._tracker_type}). '
            'Publish std_msgs/Bool True to ~/enable to start.')

    # ── enable / disable ───────────────────────────────────────────────────

    def _enable_cb(self, msg: Bool):
        self._tracking = msg.data
        if not self._tracking:
            self._tracker = None
        self.get_logger().info(f'Tracking {"enabled" if self._tracking else "disabled"}')

    # ── image pipeline ─────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        if not self._tracking:
            self._publish_debug(frame)
            return

        if self._tracker is None:
            self._init_tracker(frame, w, h)
            return

        ok, bbox = self._tracker.update(frame)
        if ok:
            self._draw_bbox(frame, bbox)
            self._publish_velocity(bbox, w, h)
        else:
            self.get_logger().warn('Tracker lost target — reinitialising next frame')
            self._tracker = None
            cv2.putText(frame, 'TARGET LOST', (w // 2 - 80, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        self._publish_debug(frame)

    def _init_tracker(self, frame: np.ndarray, w: int, h: int):
        # Initialise on the central 20 % of the frame
        roi_w, roi_h = int(w * 0.2), int(h * 0.2)
        x = (w - roi_w) // 2
        y = (h - roi_h) // 2
        bbox = (x, y, roi_w, roi_h)

        self._tracker = self._make_tracker()
        self._tracker.init(frame, bbox)
        self.get_logger().info(f'Tracker initialised on centre ROI {bbox}')

    def _make_tracker(self):
        t = self._tracker_type.upper()
        if t == 'CSRT':
            return cv2.TrackerCSRT_create()
        if t == 'KCF':
            return cv2.TrackerKCF_create()
        if t == 'MOSSE':
            return cv2.legacy.TrackerMOSSE_create()
        self.get_logger().warn(f'Unknown tracker "{t}", falling back to CSRT')
        return cv2.TrackerCSRT_create()

    # ── drawing ────────────────────────────────────────────────────────────

    def _draw_bbox(self, frame: np.ndarray, bbox):
        x, y, bw, bh = (int(v) for v in bbox)
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        cx, cy = x + bw // 2, y + bh // 2
        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
        cv2.putText(frame, 'TRACKING', (x, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # ── velocity setpoint ──────────────────────────────────────────────────

    def _publish_velocity(self, bbox, frame_w: int, frame_h: int):
        x, y, bw, bh = bbox
        cx = x + bw / 2
        cy = y + bh / 2

        # Error: positive → target is right-of / below frame centre
        err_x = cx - frame_w / 2
        err_y = cy - frame_h / 2

        # NED frame: +y = right, +z = down
        vy = float(np.clip(self._gain_lat * err_x, -self._max_vel, self._max_vel))
        vz = float(np.clip(self._gain_vert * err_y, -self._max_vel, self._max_vel))

        now = self._now_us()

        ocm = OffboardControlMode()
        ocm.position = False
        ocm.velocity = True
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        ocm.timestamp = now
        self._offboard_pub.publish(ocm)

        sp = TrajectorySetpoint()
        sp.velocity = [0.0, vy, vz]
        sp.yaw = float('nan')
        sp.timestamp = now
        self._trajectory_pub.publish(sp)

    # ── debug image ────────────────────────────────────────────────────────

    def _publish_debug(self, frame: np.ndarray):
        if self._debug_pub.get_subscription_count() > 0:
            self._debug_pub.publish(
                self._bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

    # ── helpers ────────────────────────────────────────────────────────────

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
