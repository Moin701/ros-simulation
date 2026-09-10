#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def classify(vx: float, w: float) -> str:
    if vx < -0.05:
        return 'RECOVER_BACKUP'
    if abs(vx) <= 0.01 and abs(w) <= 0.01:
        return 'IDLE_STATIONARY'
    if vx > 0.05 and abs(w) <= 0.08:
        return 'STRAIGHT_FORWARD'
    if vx > 0.05 and w > 0.08:
        return 'CURVING_LEFT'
    if vx > 0.05 and w < -0.08:
        return 'CURVING_RIGHT'
    if abs(vx) <= 0.03 and abs(w) > 0.08:
        return 'IN_PLACE_SPIN'
    # Falls between the named bands (e.g. vx in (0.03, 0.05]) - not one of
    # the six requested states, so surfaced explicitly instead of silently
    # matching the wrong bucket.
    return 'TRANSITIONING'


class ManeuverTelemetryNode(Node):
    def __init__(self):
        super().__init__('maneuver_telemetry_node')
        self.last_maneuver = None
        self.create_subscription(Twist, '/cmd_vel_raw', self._cmd_vel_callback, 10)

    def _cmd_vel_callback(self, msg: Twist) -> None:
        vx = msg.linear.x
        vy = msg.linear.y
        w = msg.angular.z

        state = classify(vx, w)
        if state != self.last_maneuver:
            self.last_maneuver = state
            self.get_logger().info(
                f"[MANEUVER TELEMETRY] Active State: {state} | vx={vx:.2f} m/s, w={w:.2f} rad/s"
            )
        # vy is extracted per spec but unused in classification - on this
        # topic specifically it's always ~0.0 anyway, since FollowPath's
        # min_vel_y/max_vel_y are locked to 0.0 (see nav2_params.yaml).
        del vy


def main(args=None):
    rclpy.init(args=args)
    node = ManeuverTelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
