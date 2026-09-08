#!/usr/bin/env python3
"""
lidar_docker.py — Reactive LiDAR-Only U-Turn & Docking Controller (v2)

Engineering changes vs v1:
  1. Narrowed confinement sectors: Left [+80°→+100°], Right [-100°→-80°]
  2. 15-cycle consecutive confinement guard (was 3) to eliminate open-area false triggers
  3. 15-second cooldown timer after EXIT→IDLE to prevent re-entry during exit drift
  4. Exit threshold raised to L & R > 0.65m for 10 consecutive cycles
  5. Anti-collision strafe bumped to vy = -0.05 m/s (was -0.04)
  6. Closed-loop turn completion: rotation >= 80° AND |d_RF - d_RR| < 0.03m (wall-parallel)
  7. 6-second watchdog timeout on both TURN states to prevent stalling
  8. All dynamic ray indexing, median filtering, and sim-clock safety retained
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from action_msgs.srv import CancelGoal


# ---------------------------------------------------------------------------
# Tuning constants — all in one place
# ---------------------------------------------------------------------------

# --- Sector angles (radians) ---
# Confinement sectors narrowed to ±[80°..100°] (±[1.396..1.745]) to avoid
# diagonal walls and open-area false positives
LEFT_CONF_START  =  1.396   # +80°
LEFT_CONF_END    =  1.745   # +100°
RIGHT_CONF_START = -1.745   # -100°
RIGHT_CONF_END   = -1.396   # -80°

RF_START = -1.047   # -60°
RF_END   = -0.785   # -45°
RR_START = -2.356   # -135°
RR_END   = -2.094   # -120°
FRONT_START = -0.262  # -15°
FRONT_END   =  0.262  # +15°

# --- Confinement trigger ---
CONFINEMENT_THRESHOLD = 0.40   # m
CONFINEMENT_CYCLES    = 15     # consecutive control-loop cycles at 20 Hz = 0.75 s

# --- Exit trigger ---
EXIT_THRESHOLD  = 0.65   # m
EXIT_CYCLES     = 10     # consecutive cycles to confirm open space

# --- Cooldown after EXIT ---
COOLDOWN_SEC    = 15.0   # s — disarms confinement trigger after leaving U-channel

# --- PD gains ---
KP_YAW  = 1.2
KD_YAW  = 0.15
KP_LAT  = 0.8
KD_LAT  = 0.10
WALL_TARGET_DIST = 0.20  # m

# --- Forward speeds ---
VX_FOLLOW = 0.05   # m/s during wall-follow phases
VX_EXIT   = 0.06   # m/s during exit phase

# --- Turn parameters ---
TURN_WZ            =  0.15   # rad/s pivot speed (left)
ANTI_STRAFE_VY     = -0.05   # m/s during first 30° of turn
ANTI_STRAFE_THRESH =  0.523  # rad (30°)
TURN_MIN_DEG       =  80.0   # deg — minimum rotation before considering completion
TURN_MIN_RAD       =  math.radians(TURN_MIN_DEG)
TURN_PARALLEL_TOL  =  0.03   # m — |d_RF - d_RR| must be below this (wall-parallel)
TURN_WATCHDOG_SEC  =  6.0    # s — abort turn if it takes longer than this

# --- Hard velocity clamps ---
MAX_VX = 0.10
MAX_VY = 0.10
MAX_WZ = 0.20

# --- Noise filter ---
RANGE_MAX_VALID = 1.2   # m — rays beyond this are treated as open/scattered


class State:
    IDLE         = 0
    WALL_FOLLOW_1 = 1
    TURN_1       = 2
    WALL_FOLLOW_2 = 3
    TURN_2       = 4
    EXIT         = 5


class LidarDocker(Node):

    def __init__(self):
        super().__init__('lidar_docker')

        # use_sim_time support
        try:
            self.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        # --- FSM state ---
        self._state = State.IDLE

        # --- Odometry ---
        self._current_yaw   = 0.0
        self._current_odom_x = 0.0
        self._current_odom_y = 0.0
        self._has_odom      = False
        self._turn_start_yaw = 0.0

        # --- PD controller memory ---
        self._prev_e_y   = 0.0
        self._prev_e_yaw = 0.0
        self._last_pd_time = self.get_clock().now()

        # --- Confinement trigger state ---
        self._confinement_count = 0   # consecutive cycles below threshold
        self._cooldown_end_time = None  # ROS Time when cooldown expires (None = no cooldown)

        # --- Exit trigger state ---
        self._exit_open_count = 0   # consecutive cycles above exit threshold

        # --- Turn watchdog ---
        self._turn_start_time = None   # ROS Time when TURN state started

        # --- LiDAR sector cache ---
        self._sec_left  = RANGE_MAX_VALID
        self._sec_right = RANGE_MAX_VALID
        self._sec_rf    = RANGE_MAX_VALID
        self._sec_rr    = RANGE_MAX_VALID
        self._sec_front = RANGE_MAX_VALID
        self._has_scan  = False

        # --- Nav2 cancel service client ---
        self._nav2_cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal'
        )

        # --- ROS interfaces ---
        self.create_subscription(LaserScan, '/scan',      self._scan_callback, 10)
        self.create_subscription(Odometry,  '/rf2o/odom', self._odom_callback, 10)
        self.create_subscription(Odometry,  '/odom',      self._odom_callback, 10)

        # Publish exclusively to the twist_mux docking channel (priority 20).
        # In IDLE (State 0) we publish NOTHING — twist_mux sees a timeout on
        # /cmd_vel_dock and falls through to the Nav2 /cmd_vel_nav channel.
        self._cmd_vel_dock_pub = self.create_publisher(Twist, '/cmd_vel_dock', 10)

        # Control loop at 20 Hz
        self.create_timer(0.05, self._control_loop)

        # Logging throttle counter
        self._log_tick = 0

        self.get_logger().info(
            "\n=================================================================\n"
            "🤖 LIDAR_DOCKER V2: Diagnostic Telemetry Layer Activated\n"
            "Output channel: /cmd_vel_dock (twist_mux Priority 20)\n"
            f"confinement threshold: < {CONFINEMENT_THRESHOLD:.2f}m | exit threshold: > {EXIT_THRESHOLD:.2f}m\n"
            "================================================================="
        )

    # ======================================================================
    # LiDAR Sector Extraction
    # ======================================================================

    def _angle_to_index(self, angle_rad: float, msg: LaserScan) -> int:
        idx = int(round((angle_rad - msg.angle_min) / msg.angle_increment))
        return max(0, min(idx, len(msg.ranges) - 1))

    def _sector_distance(self, a_start: float, a_end: float, msg: LaserScan) -> float:
        """Return the median of valid rays in [a_start, a_end]; default RANGE_MAX_VALID."""
        i0 = self._angle_to_index(a_start, msg)
        i1 = self._angle_to_index(a_end, msg)
        lo, hi = min(i0, i1), max(i0, i1)
        raw = msg.ranges[lo : hi + 1]
        valid = [r for r in raw if not math.isnan(r) and not math.isinf(r) and r <= RANGE_MAX_VALID]
        return float(np.median(valid)) if valid else RANGE_MAX_VALID

    def _scan_callback(self, msg: LaserScan) -> None:
        # 1. Narrowed confinement sectors — [+80°..+100°] and [-100°..-80°]
        self._sec_left  = self._sector_distance(LEFT_CONF_START,  LEFT_CONF_END,  msg)
        self._sec_right = self._sector_distance(RIGHT_CONF_START, RIGHT_CONF_END, msg)
        # 2. Right-wall tracking sectors (unchanged)
        self._sec_rf    = self._sector_distance(RF_START, RF_END, msg)
        self._sec_rr    = self._sector_distance(RR_START, RR_END, msg)
        # 3. Collision-front sector
        self._sec_front = self._sector_distance(FRONT_START, FRONT_END, msg)
        self._has_scan  = True

    # ======================================================================
    # Odometry
    # ======================================================================

    def _odom_callback(self, msg: Odometry) -> None:
        self._current_odom_x = msg.pose.pose.position.x
        self._current_odom_y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
        self._current_yaw = math.atan2(siny, cosy)
        self._has_odom = True

    # ======================================================================
    # PD Controller (right-wall tracking)
    # ======================================================================

    def _compute_pd_control(self):
        """Return (vy, wz) to keep the robot 20 cm from and parallel to the right wall."""
        now = self.get_clock().now()
        dt = (now - self._last_pd_time).nanoseconds / 1e9
        if dt < 0.001:
            dt = 0.001
        self._last_pd_time = now

        d_rf = self._sec_rf
        d_rr = self._sec_rr

        # Yaw (heading) error: positive → nose pointing away from wall → rotate right (−wz)
        e_psi = d_rf - d_rr
        de_psi = (e_psi - self._prev_e_yaw) / dt
        self._prev_e_yaw = e_psi
        wz = KP_YAW * e_psi + KD_YAW * de_psi

        # Lateral error: positive → too far from wall → strafe right (−vy)
        avg_right = (d_rf + d_rr) / 2.0
        e_y = avg_right - WALL_TARGET_DIST
        de_y = (e_y - self._prev_e_y) / dt
        self._prev_e_y = e_y
        vy = KP_LAT * e_y + KD_LAT * de_y

        # Hard clamps
        vy = max(-MAX_VY, min(MAX_VY, vy))
        wz = max(-MAX_WZ, min(MAX_WZ, wz))
        return vy, wz

    # ======================================================================
    # Nav2 Handoff
    # ======================================================================

    def _cancel_nav2_goal(self) -> None:
        self._publish_twist(0.0, 0.0, 0.0)
        self.get_logger().info('Halting robot and cancelling Nav2 goal...')
        if self._nav2_cancel_client.service_is_ready():
            self._nav2_cancel_client.call_async(CancelGoal.Request())
            self.get_logger().info('Nav2 cancel sent.')
        else:
            self.get_logger().warn('Nav2 cancel service not ready — reactive node overrides velocity.')

    # ======================================================================
    # Helpers
    # ======================================================================

    def _normalize_angle(self, a: float) -> float:
        while a >  math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    def _publish_twist(self, vx: float, vy: float, wz: float) -> None:
        """Publish a clamped Twist to /cmd_vel_dock (twist_mux docking channel)."""
        msg = Twist()
        msg.linear.x  = float(max(-MAX_VX, min(MAX_VX, vx)))
        msg.linear.y  = float(max(-MAX_VY, min(MAX_VY, vy)))
        msg.angular.z = float(max(-MAX_WZ, min(MAX_WZ, wz)))
        self._cmd_vel_dock_pub.publish(msg)

    def _reset_pd(self) -> None:
        self._prev_e_y   = 0.0
        self._prev_e_yaw = 0.0
        self._last_pd_time = self.get_clock().now()

    def _elapsed_since(self, t) -> float:
        """Seconds elapsed since a ROS Time object `t`."""
        return (self.get_clock().now() - t).nanoseconds / 1e9

    # ======================================================================
    # Main Control Loop (20 Hz)
    # ======================================================================

    def _control_loop(self) -> None:
        if not self._has_scan:
            return

        now = self.get_clock().now()

        # ------------------------------------------------------------------
        # STATE 0: IDLE — publish NOTHING; twist_mux times out on /cmd_vel_dock
        # and falls through to the /cmd_vel_nav (Nav2) channel automatically.
        # ------------------------------------------------------------------
        if self._state == State.IDLE:
            # Check if still in cooldown from last EXIT
            if self._cooldown_end_time is not None:
                remaining = (self._cooldown_end_time - now).nanoseconds / 1e9
                if remaining > 0.0:
                    # Still cooling down — do not check confinement
                    return
                else:
                    self.get_logger().info('⏱️ Cooldown finished. Confinement trigger re-armed.')
                    self._cooldown_end_time = None

            # 15-cycle consecutive confinement guard
            if self._sec_left < CONFINEMENT_THRESHOLD and self._sec_right < CONFINEMENT_THRESHOLD:
                self._confinement_count += 1
                if self._confinement_count % 3 == 0:
                    self.get_logger().info(
                        f"🔍 Confinement warning: L={self._sec_left:.3f}m, R={self._sec_right:.3f}m | "
                        f"Confidence: {self._confinement_count}/{CONFINEMENT_CYCLES} cycles"
                    )
                if self._confinement_count >= CONFINEMENT_CYCLES:
                    self.get_logger().warn(
                        f"🚨 CONFINEMENT TRIGGERED! Swapping velocity authority to LiDAR Docking [Priority 20]"
                    )
                    self._cancel_nav2_goal()
                    self._state = State.WALL_FOLLOW_1
                    self._exit_open_count = 0
                    self._reset_pd()
            else:
                self._confinement_count = max(0, self._confinement_count - 1)
            # No publish in IDLE — silence lets twist_mux prefer Nav2

        # ------------------------------------------------------------------
        # STATE 1: WALL_FOLLOW_1 (entrance corridor)
        # ------------------------------------------------------------------
        elif self._state == State.WALL_FOLLOW_1:
            if self._sec_front < 0.25:
                self.get_logger().info('↩️ Corner detected. Initiating State 2: Slide-Pivot Turn 1.')
                self._state = State.TURN_1
                self._turn_start_yaw  = self._current_yaw
                self._turn_start_time = now
                self._publish_twist(0.0, 0.0, 0.0)
            else:
                vy, wz = self._compute_pd_control()
                self._publish_twist(VX_FOLLOW, vy, wz)
                
                self._log_tick += 1
                if self._log_tick % 10 == 0:
                    e_psi = self._sec_rf - self._sec_rr
                    e_y = ((self._sec_rf + self._sec_rr) / 2.0) - WALL_TARGET_DIST
                    self.get_logger().info(
                        f"🏎️ STATE 1 (Wall Follow) | YawErr: {e_psi:+.3f}m | "
                        f"LatErr: {e_y:+.3f}m | Cmd: vx={VX_FOLLOW:.3f}, vy={vy:+.3f}, wz={wz:+.3f}"
                    )

        # ------------------------------------------------------------------
        # STATE 2: TURN_1 — coordinated slide-pivot (first 90° turn)
        # ------------------------------------------------------------------
        elif self._state == State.TURN_1:
            delta_yaw = abs(self._normalize_angle(self._current_yaw - self._turn_start_yaw))

            # Anti-collision strafe for first 30°
            vy = ANTI_STRAFE_VY if delta_yaw < ANTI_STRAFE_THRESH else 0.0
            self._publish_twist(0.0, vy, TURN_WZ)

            self.get_logger().info(
                f"🔄 STATE 2 (Slide-Pivot 1) | Command: wz={TURN_WZ:.3f} (pivot left), vy={vy:.3f} (strafe out)",
                throttle_duration_sec=1.0
            )

            # Watchdog: abort if turn takes too long
            turn_elapsed = self._elapsed_since(self._turn_start_time)
            if turn_elapsed > TURN_WATCHDOG_SEC:
                self.get_logger().error(
                    'TURN_1 watchdog expired (%.1fs)! Forcing transition to WALL_FOLLOW_2.' % turn_elapsed
                )
                self._state = State.WALL_FOLLOW_2
                self._reset_pd()
                return

            # Closed-loop completion: rotation >= 80° AND wall-parallel
            squaring_err = abs(self._sec_rf - self._sec_rr)
            if delta_yaw >= TURN_MIN_RAD and squaring_err < TURN_PARALLEL_TOL:
                self.get_logger().info('✅ Turn 1 Complete. Parallel to back wall. Transitioning to State 3.')
                self._state = State.WALL_FOLLOW_2
                self._reset_pd()

        # ------------------------------------------------------------------
        # STATE 3: WALL_FOLLOW_2 (mid-section assembly tracking)
        # ------------------------------------------------------------------
        elif self._state == State.WALL_FOLLOW_2:
            if self._sec_front < 0.25:
                self.get_logger().info('↩️ Corner detected. Initiating State 4: Slide-Pivot Turn 2.')
                self._state = State.TURN_2
                self._turn_start_yaw  = self._current_yaw
                self._turn_start_time = now
                self._publish_twist(0.0, 0.0, 0.0)
            else:
                vy, wz = self._compute_pd_control()
                self._publish_twist(VX_FOLLOW, vy, wz)
                
                self._log_tick += 1
                if self._log_tick % 10 == 0:
                    e_psi = self._sec_rf - self._sec_rr
                    e_y = ((self._sec_rf + self._sec_rr) / 2.0) - WALL_TARGET_DIST
                    self.get_logger().info(
                        f"🏎️ STATE 3 (Wall Follow) | YawErr: {e_psi:+.3f}m | "
                        f"LatErr: {e_y:+.3f}m | Cmd: vx={VX_FOLLOW:.3f}, vy={vy:+.3f}, wz={wz:+.3f}"
                    )

        # ------------------------------------------------------------------
        # STATE 4: TURN_2 — coordinated slide-pivot (second 90° turn)
        # ------------------------------------------------------------------
        elif self._state == State.TURN_2:
            delta_yaw = abs(self._normalize_angle(self._current_yaw - self._turn_start_yaw))

            vy = ANTI_STRAFE_VY if delta_yaw < ANTI_STRAFE_THRESH else 0.0
            self._publish_twist(0.0, vy, TURN_WZ)

            self.get_logger().info(
                f"🔄 STATE 4 (Slide-Pivot 2) | Command: wz={TURN_WZ:.3f} (pivot left), vy={vy:.3f} (strafe out)",
                throttle_duration_sec=1.0
            )

            # Watchdog
            turn_elapsed = self._elapsed_since(self._turn_start_time)
            if turn_elapsed > TURN_WATCHDOG_SEC:
                self.get_logger().error(
                    'TURN_2 watchdog expired (%.1fs)! Forcing transition to EXIT.' % turn_elapsed
                )
                self._state = State.EXIT
                self._exit_open_count = 0
                self._reset_pd()
                return

            # Closed-loop completion
            squaring_err = abs(self._sec_rf - self._sec_rr)
            if delta_yaw >= TURN_MIN_RAD and squaring_err < TURN_PARALLEL_TOL:
                self.get_logger().info('✅ Turn 2 Complete. Parallel to exit corridor. Transitioning to State 5 (EXIT).')
                self._state = State.EXIT
                self._exit_open_count = 0
                self._reset_pd()

        # ------------------------------------------------------------------
        # STATE 5: EXIT (exit corridor — track right wall until open space)
        # ------------------------------------------------------------------
        elif self._state == State.EXIT:
            # 10-cycle consecutive open-space guard (Left & Right > 0.65m)
            if self._sec_left > EXIT_THRESHOLD and self._sec_right > EXIT_THRESHOLD:
                self._exit_open_count += 1
                if self._exit_open_count >= EXIT_CYCLES:
                    self.get_logger().warn(
                        f"🏁 EXIT CLEAN! Left={self._sec_left:.2f}m, Right={self._sec_right:.2f}m. "
                        "Halted docking commands. Handing back authority to Nav2 [Priority 10]. "
                        f"Cooldown armed for {COOLDOWN_SEC}s."
                    )
                    self._publish_twist(0.0, 0.0, 0.0)
                    self._state = State.IDLE
                    self._confinement_count = 0
                    # Start cooldown so we don't immediately re-trigger
                    from rclpy.duration import Duration
                    self._cooldown_end_time = now + Duration(seconds=COOLDOWN_SEC)
                else:
                    # Still accumulating open-space cycles — keep moving
                    vy, wz = self._compute_pd_control()
                    self._publish_twist(VX_EXIT, vy, wz)
            else:
                # Not yet open — reset the counter and keep tracking
                self._exit_open_count = 0
                vy, wz = self._compute_pd_control()
                self._publish_twist(VX_EXIT, vy, wz)
                
                self._log_tick += 1
                if self._log_tick % 10 == 0:
                    self.get_logger().info(f"🚪 STATE 5 (Exiting) | Clearing door... L={self._sec_left:.2f}m, R={self._sec_right:.2f}m")


# ===========================================================================
# Entry Point
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)
    node = LidarDocker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish a zero-velocity stop to /cmd_vel_dock so downstream hardware
        # sees a clean stop before the node vanishes and twist_mux times out.
        if rclpy.ok():
            node._publish_twist(0.0, 0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
