"""GimbalArm analytic kinematics (5-DOF).

Joints (JOINT_NAMES order): base_yaw, shoulder, elbow, wrist_pitch, wrist_roll.

  position (base_yaw + shoulder + elbow)  = articulated 3-DOF, CLOSED FORM.
  wrist    (wrist_pitch + wrist_roll)     = 2-DOF leveling (cup axis vertical).

No numeric IK, no MoveIt -- the geometry is simple enough to invert directly.

Home convention: arm straight UP (+Z).  shoulder/elbow measured from +Z, growing
toward the radial direction set by base_yaw.  Lengths from ArmGeom (keep in sync
with gimbalarm_description URDF + arm_params.yaml).

TODO(DH): verify lengths, joint axes, and sign conventions against the URDF in
RViz once the real arm is measured.  level_wrist() assumes wrist_pitch about Y
and wrist_roll about the tool axis -- confirm on hardware.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class ArmGeom:
    """Link geometry [m].  Defaults are PLACEHOLDERS (match URDF placeholders)."""
    h0: float = 0.075     # base_link -> shoulder axis height (DH a0+a1)
    l1: float = 0.362     # upper arm  (shoulder -> elbow)     (0.409 - 47mm cut)
    l2: float = 0.313     # forearm    (elbow -> wrist)        (0.335 - 22mm cut)
    l_tool: float = 0.103 # wrist -> cup center (DH a4), along leveled tool axis
    # NOTE: DH d2=54mm lateral elbow offset is NOT modeled (planar 2R). Verify in RViz.


def fk_wrist(q0: float, q1: float, q2: float, g: ArmGeom):
    """Forward kinematics -> wrist-center (x, y, z) [m].

    q0 base_yaw, q1 shoulder, q2 elbow [rad].
    """
    r = g.l1 * math.sin(q1) + g.l2 * math.sin(q1 + q2)
    z = g.h0 + g.l1 * math.cos(q1) + g.l2 * math.cos(q1 + q2)
    return (r * math.cos(q0), r * math.sin(q0), z)


def ik_position(x: float, y: float, z: float, g: ArmGeom, elbow: str = 'down'):
    """Inverse for a WRIST-CENTER target -> (base_yaw, shoulder, elbow) [rad].

    elbow: 'up' or 'down' branch.  Raises ValueError if target out of reach.

    Singularity: when the radial distance r = hypot(x, y) ~ 0 (target on the
    base_yaw axis) the yaw is unobservable and q0 is arbitrary; keep cup targets
    off-axis (r > a few cm).
    """
    q0 = math.atan2(y, x)
    r = math.hypot(x, y)
    zc = z - g.h0
    d = math.hypot(r, zc)
    reach = g.l1 + g.l2
    if d > reach + 1e-9 or d < abs(g.l1 - g.l2) - 1e-9:
        raise ValueError(
            f'target out of reach: d={d:.4f} not in '
            f'[{abs(g.l1 - g.l2):.4f}, {reach:.4f}]'
        )
    cos_q2 = (d * d - g.l1 * g.l1 - g.l2 * g.l2) / (2.0 * g.l1 * g.l2)
    cos_q2 = max(-1.0, min(1.0, cos_q2))
    q2 = math.acos(cos_q2)
    if elbow == 'down':
        q2 = -q2
    phi = math.atan2(r, zc)  # target direction from +Z
    alpha = math.atan2(g.l2 * math.sin(q2), g.l1 + g.l2 * math.cos(q2))
    q1 = phi - alpha
    return (q0, q1, q2)


def level_wrist(q1: float, q2: float, cup_pitch: float = 0.0, cup_roll: float = 0.0):
    """Wrist (pitch, roll) [rad] to hold the cup at cup_pitch / cup_roll vs vertical.

    The planar arm tilts the forearm by (q1 + q2) from +Z; wrist_pitch cancels it so
    the tool stays vertical (plus an optional desired cup_pitch).  roll passes through.
    """
    wrist_pitch = -(q1 + q2) + cup_pitch
    wrist_roll = cup_roll
    return (wrist_pitch, wrist_roll)


def ik(x: float, y: float, z: float, g: ArmGeom, elbow: str = 'down',
       target: str = 'cup', cup_pitch: float = 0.0, cup_roll: float = 0.0):
    """Full 5-joint IK -> [base_yaw, shoulder, elbow, wrist_pitch, wrist_roll].

    target='cup': (x,y,z) is the cup center; the leveled tool points +Z so the
      wrist center sits l_tool below it.  target='wrist': (x,y,z) is wrist center.
    """
    if target == 'cup':
        z = z - g.l_tool
    elif target != 'wrist':
        raise ValueError("target must be 'cup' or 'wrist'")
    q0, q1, q2 = ik_position(x, y, z, g, elbow=elbow)
    q3, q4 = level_wrist(q1, q2, cup_pitch, cup_roll)
    return [q0, q1, q2, q3, q4]
