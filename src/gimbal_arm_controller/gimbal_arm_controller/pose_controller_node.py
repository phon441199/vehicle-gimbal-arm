"""Cartesian / named-pose controller for GimbalArm (analytic IK + quintic).

Reuses ArmControllerNode (send_joint_command clamp + /joint_commands pub +
/joint_states sub) and adds:

  inputs
    /goto_pose   std_msgs/String        -> named pose (standby/receive/slosh ...)
    /cup_target  geometry_msgs/PointStamped -> cup-center (x,y,z) [m], IK to it

  motion
    on each new goal, build a per-joint quintic from the CURRENT joints to the
    goal joints over move_time and stream setpoints at rate_hz to /joint_commands.

  fake_hw (default true)
    also publish /joint_states from the streamed setpoint so robot_state_publisher
    / RViz animate the arm with NO motors.  Set fake_hw:=false on real hardware
    (then /joint_states comes from the MCU instead).

Geometry comes from arm_geom.* params (keep in sync with
gimbalarm_description URDF + config/arm_params.yaml).  NOTE: the planar 2R IK
ignores the DH d2=54mm elbow lateral offset (modeled only in the URDF visual).
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .arm_controller_node import ArmControllerNode
from .joint_config import (
    JOINT_ACC_LIMITS,
    JOINT_LIMITS,
    JOINT_NAMES,
    JOINT_VEL_LIMITS,
    NUM_JOINTS,
)

# quintic (zero v,a boundary) peak factors: |v|max = 1.875*dq/T, |a|max = 5.7735*dq/T^2
_QV = 1.875
_QA = 5.7735
from .kinematics import ArmGeom, ik
from .trajectory import JointTrajectory


class PoseController(ArmControllerNode):
    def __init__(self):
        super().__init__(node_name='pose_controller')

        # ---- params ----
        self.declare_parameter('arm_geom.h0', 0.075)
        self.declare_parameter('arm_geom.l1', 0.362)   # upper arm (0.409 - 47mm cut)
        self.declare_parameter('arm_geom.l2', 0.313)   # forearm   (0.335 - 22mm cut)
        self.declare_parameter('arm_geom.l_tool', 0.103)
        self.declare_parameter('arm_geom.elbow', 'down')
        self.declare_parameter('move_time', 2.5)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('fake_hw', True)
        self.declare_parameter('cup_pitch', 0.0)
        self.declare_parameter('cup_roll', 0.0)
        self.declare_parameter('yaw_singularity_r', 0.03)  # [m] hold yaw below this
        # named poses [rad], JOINT_NAMES order
        self.declare_parameter('pose.standby', [0.0, 0.5, -0.7, 0.2, 0.0])
        self.declare_parameter('pose.receive', [0.3, 0.8, -0.6, -0.2, 0.0])
        self.declare_parameter('pose.slosh', [0.0, 0.6, -0.4, -0.2, 0.0])

        g = self.get_parameter
        self._geom = ArmGeom(
            h0=g('arm_geom.h0').value,
            l1=g('arm_geom.l1').value,
            l2=g('arm_geom.l2').value,
            l_tool=g('arm_geom.l_tool').value,
        )
        self._elbow = g('arm_geom.elbow').value
        self._move_time = float(g('move_time').value)
        self._rate = float(g('rate_hz').value)
        self._fake_hw = bool(g('fake_hw').value)
        self._cup_pitch = float(g('cup_pitch').value)
        self._cup_roll = float(g('cup_roll').value)
        self._yaw_sing_r = float(g('yaw_singularity_r').value)
        self._got_state = False  # set once a real /joint_states arrives (BUG1 gate)
        self._poses = {
            name: list(g(f'pose.{name}').value)
            for name in ('standby', 'receive', 'slosh')
        }

        # ---- fake-hw state echo ----
        if self._fake_hw:
            self._state_pub = self.create_publisher(JointState, '/joint_states', 10)
        # internal commanded pose (also seeds first trajectory start)
        self._cmd_pos = [0.0] * NUM_JOINTS

        # ---- goal inputs ----
        self.create_subscription(String, '/goto_pose', self._on_named_pose, 10)
        self.create_subscription(PointStamped, '/cup_target', self._on_cup_target, 10)

        # ---- streaming ----
        self._traj: JointTrajectory | None = None
        self._t = 0.0
        self._dt = 1.0 / self._rate
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'PoseController up (fake_hw={self._fake_hw}, geom={self._geom}, '
            f'move_time={self._move_time}s) — /goto_pose, /cup_target'
        )
        if self._fake_hw:
            self._publish_state(self._cmd_pos)  # show home immediately

    # ---- goal handlers ----
    def _within_limits(self, goal: list[float]) -> str | None:
        """Return name of first joint exceeding its limit, else None."""
        for q, name in zip(goal, JOINT_NAMES):
            lo, hi = JOINT_LIMITS[name]
            if q < lo - 1e-6 or q > hi + 1e-6:
                return f'{name}={q:.3f} not in [{lo:.3f}, {hi:.3f}]'
        return None

    def _start_traj(self, goal: list[float], tag: str):
        # BUG1 gate: on real HW never assume start=0.  Refuse until the MCU has
        # reported an actual pose, then interpolate FROM the measured joints.
        if not self._fake_hw and not self._got_state:
            self.get_logger().warn(
                f'rejected {tag}: no /joint_states from MCU yet (start unknown)')
            return
        # BUG2 gate: don't restart a quintic (assumes v=0) while one is running --
        # would step the velocity and jolt the arm.  Command only from rest.
        if self._traj is not None:
            self.get_logger().warn(f'rejected {tag}: busy (trajectory in progress)')
            return
        bad = self._within_limits(goal)
        if bad is not None:
            # REJECT (do not clamp): clamping distorts the pose / self-collides
            self.get_logger().warn(f'rejected {tag}: joint limit -- {bad}')
            return
        start = self._cmd_pos if self._fake_hw else self.current_positions
        # auto-stretch move_time so quintic peak vel/accel stay within per-joint
        # limits (clamp by slowing down, never by distorting the path).
        T = self._safe_duration(start, goal)
        try:
            self._traj = JointTrajectory(list(start), list(goal), T)
        except ValueError as e:
            self.get_logger().error(f'trajectory build failed ({tag}): {e}')
            return
        self._t = 0.0
        extra = f' (T stretched {self._move_time:.1f}->{T:.2f}s)' if T > self._move_time + 1e-3 else ''
        self.get_logger().info(f'-> {tag}: {[round(q, 3) for q in goal]}{extra}')

    def _safe_duration(self, start: list[float], goal: list[float]) -> float:
        """Smallest T >= move_time keeping quintic peak |v|,|a| within limits."""
        T = self._move_time
        for s, q, name in zip(start, goal, JOINT_NAMES):
            dq = abs(q - s)
            if dq < 1e-9:
                continue
            T = max(T, _QV * dq / JOINT_VEL_LIMITS[name],
                    math.sqrt(_QA * dq / JOINT_ACC_LIMITS[name]))
        return T

    def _on_named_pose(self, msg: String):
        name = msg.data.strip()
        if name not in self._poses:
            self.get_logger().warn(
                f'unknown pose "{name}"; have {list(self._poses)}')
            return
        self._start_traj(self._poses[name], f'pose:{name}')

    def _on_cup_target(self, msg: PointStamped):
        p = msg.point
        try:
            goal = ik(p.x, p.y, p.z, self._geom, elbow=self._elbow,
                      target='cup', cup_pitch=self._cup_pitch,
                      cup_roll=self._cup_roll)
        except ValueError as e:
            self.get_logger().warn(f'IK rejected target '
                                   f'({p.x:.3f},{p.y:.3f},{p.z:.3f}): {e}')
            return
        # BUG3: near the yaw axis atan2(y,x) is ill-conditioned -> base_yaw can
        # swing wildly.  Hold the previous yaw inside the singularity radius.
        if math.hypot(p.x, p.y) < self._yaw_sing_r:
            goal[0] = self._cmd_pos[0] if self._fake_hw else self.current_positions[0]
        self._start_traj(goal, f'cup:({p.x:.3f},{p.y:.3f},{p.z:.3f})')

    # ---- streaming loop ----
    def _tick(self):
        if self._traj is not None:
            pos = self._traj.sample(self._t)
            self.send_joint_command(pos)  # -> /joint_commands (clamped) to MCU
            self._cmd_pos = pos
            if self._traj.done(self._t):
                self._traj = None
            else:
                self._t += self._dt
        # keep /joint_states fresh so robot_state_publisher TF never goes stale
        # (RViz drops links on stale TF -> RobotModel error).  Idle = hold pose.
        if self._fake_hw:
            self._publish_state(self._cmd_pos)

    def _joint_state_callback(self, msg: JointState):
        super()._joint_state_callback(msg)
        self._got_state = True

    def _publish_state(self, pos: list[float]):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = list(pos)
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
