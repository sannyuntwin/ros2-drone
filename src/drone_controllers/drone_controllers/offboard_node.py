#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class OffboardNode(Node):
    """PX4 offboard controller: arm, take off to a fixed altitude, then hover."""

    def __init__(self):
        super().__init__('offboard_node')

        self.declare_parameter('target_altitude', -5.0)  # NED: negative = up
        self.declare_parameter('takeoff_counter_threshold', 10)

        self._target_alt = self.get_parameter('target_altitude').value
        self._arm_threshold = self.get_parameter('takeoff_counter_threshold').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self._trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self._command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_pos_cb, qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._status_cb, qos)

        self._local_pos = VehicleLocalPosition()
        self._vehicle_status = VehicleStatus()
        self._counter = 0

        self.create_timer(0.1, self._timer_cb)
        self.get_logger().info('OffboardNode started')

    # ── callbacks ──────────────────────────────────────────────────────────

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self._local_pos = msg

    def _status_cb(self, msg: VehicleStatus):
        self._vehicle_status = msg

    # ── timer ──────────────────────────────────────────────────────────────

    def _timer_cb(self):
        self._publish_offboard_heartbeat()

        if self._counter == self._arm_threshold:
            self._engage_offboard()
            self._arm()

        if self._counter <= self._arm_threshold:
            self._counter += 1

        self._publish_position_setpoint(0.0, 0.0, self._target_alt)

    # ── publishers ─────────────────────────────────────────────────────────

    def _publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self._offboard_pub.publish(msg)

    def _publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = self._now_us()
        self._trajectory_pub.publish(msg)

    def _publish_vehicle_command(self, command: int, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self._command_pub.publish(msg)

    # ── commands ───────────────────────────────────────────────────────────

    def _arm(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def _disarm(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def _engage_offboard(self):
        # param2=6 selects offboard mode in PX4 custom_mode
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard mode engaged')

    def land(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Land command sent')

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
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
