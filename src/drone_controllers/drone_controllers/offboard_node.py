#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import Enum, auto

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from std_msgs.msg import String


class State(Enum):
    IDLE    = auto()
    TAKEOFF = auto()
    HOVER   = auto()
    MISSION = auto()
    LAND    = auto()


class OffboardNode(Node):
    """PX4 offboard controller: IDLE → TAKEOFF → HOVER → MISSION → HOVER → LAND."""

    def __init__(self):
        super().__init__('offboard_node')

        self.declare_parameter('target_altitude',          -5.0)
        self.declare_parameter('takeoff_counter_threshold', 10)
        self.declare_parameter('altitude_threshold',        0.3)
        self.declare_parameter('square_side',              10.0)
        self.declare_parameter('waypoint_radius',           0.5)

        self._target_alt    = self.get_parameter('target_altitude').value
        self._arm_threshold = self.get_parameter('takeoff_counter_threshold').value
        self._alt_threshold = self.get_parameter('altitude_threshold').value
        self._square_side   = self.get_parameter('square_side').value
        self._wp_radius     = self.get_parameter('waypoint_radius').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._offboard_pub   = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self._trajectory_pub = self.create_publisher(TrajectorySetpoint,  '/fmu/in/trajectory_setpoint',   qos)
        self._command_pub    = self.create_publisher(VehicleCommand,       '/fmu/in/vehicle_command',       qos)
        self._state_pub      = self.create_publisher(String, '~/state', 10)

        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._local_pos_cb, qos)
        self.create_subscription(VehicleStatus,        '/fmu/out/vehicle_status_v4',         self._status_cb,    qos)
        self.create_subscription(String, '~/cmd', self._cmd_cb, 10)

        self._local_pos      = VehicleLocalPosition()
        self._vehicle_status = VehicleStatus()
        self._counter        = 0
        self._state          = State.IDLE
        self._waypoints      = self._build_square()
        self._wp_index       = 0

        self.create_timer(0.1, self._timer_cb)
        self.get_logger().info('OffboardNode started — state: IDLE')

    # ── waypoints ──────────────────────────────────────────────────────────

    def _build_square(self):
        s = self._square_side
        z = self._target_alt
        return [
            [s,   0.0, z],  # leg 1: North
            [s,   s,   z],  # leg 2: East
            [0.0, s,   z],  # leg 3: South
            [0.0, 0.0, z],  # leg 4: West (home)
        ]

    # ── subscriptions ──────────────────────────────────────────────────────

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self._local_pos = msg

    def _status_cb(self, msg: VehicleStatus):
        self._vehicle_status = msg

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'land' and self._state in (State.TAKEOFF, State.HOVER, State.MISSION):
            self._transition(State.LAND)
        elif cmd == 'mission' and self._state == State.HOVER:
            self._wp_index = 0
            self._transition(State.MISSION)
        elif cmd == 'hover' and self._state == State.MISSION:
            self._transition(State.HOVER)

    # ── state machine ──────────────────────────────────────────────────────

    def _transition(self, new_state: State):
        self.get_logger().info(f'State: {self._state.name} → {new_state.name}')
        self._state = new_state
        if new_state == State.LAND:
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info('Land command sent')

    def _timer_cb(self):
        self._publish_offboard_heartbeat()
        self._publish_state()

        if self._state == State.IDLE:
            self._counter += 1
            if self._counter >= self._arm_threshold:
                self._engage_offboard()
                self._arm()
                self._transition(State.TAKEOFF)

        elif self._state == State.TAKEOFF:
            self._publish_position_setpoint(0.0, 0.0, self._target_alt)
            if (self._local_pos.z != 0.0 and
                    abs(self._local_pos.z - self._target_alt) < self._alt_threshold):
                self._transition(State.HOVER)

        elif self._state == State.HOVER:
            self._publish_position_setpoint(0.0, 0.0, self._target_alt)

        elif self._state == State.MISSION:
            wp = self._waypoints[self._wp_index]
            self._publish_position_setpoint(*wp)

            dist = math.sqrt(
                (self._local_pos.x - wp[0]) ** 2 +
                (self._local_pos.y - wp[1]) ** 2 +
                (self._local_pos.z - wp[2]) ** 2
            )
            if dist < self._wp_radius:
                self._wp_index += 1
                if self._wp_index >= len(self._waypoints):
                    self.get_logger().info('Mission complete — returning to HOVER')
                    self._wp_index = 0
                    self._transition(State.HOVER)
                else:
                    wp_next = self._waypoints[self._wp_index]
                    self.get_logger().info(
                        f'Waypoint {self._wp_index}/{len(self._waypoints)} — '
                        f'heading to ({wp_next[0]:.1f}, {wp_next[1]:.1f}, {wp_next[2]:.1f})'
                    )

        # LAND: PX4 handles descent; heartbeat keeps us in offboard

    # ── publishers ─────────────────────────────────────────────────────────

    def _publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.position  = True
        msg.timestamp = self._now_us()
        self._offboard_pub.publish(msg)

    def _publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position  = [x, y, z]
        msg.yaw       = 0.0
        msg.timestamp = self._now_us()
        self._trajectory_pub.publish(msg)

    def _publish_vehicle_command(self, command: int, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = self._now_us()
        self._command_pub.publish(msg)

    def _publish_state(self):
        msg = String()
        msg.data = self._state.name
        self._state_pub.publish(msg)

    # ── commands ───────────────────────────────────────────────────────────

    def _arm(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def _engage_offboard(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard mode engaged')

    def land(self):
        if self._state != State.LAND:
            self._transition(State.LAND)

    # ── helpers ────────────────────────────────────────────────────────────

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        try:
            node.land()
            deadline = node.get_clock().now().nanoseconds + 1_000_000_000
            while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
