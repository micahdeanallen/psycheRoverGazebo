"""Leg kinematics for the Psyche hexapod. Pure Python, no ROS.

Frames
------
Body frame (base_link): origin at body center, +x front, +y left, +z up.
Leg frame: origin on the hip (LX-16A) axis, +x pointing straight out from the
body, +z up. Left and right legs are mirror images, so the same math serves all six.

Joint conventions (match psyche_description/urdf/params.xacro):
    hip   +  : counter-clockwise seen from above
    femur +  : leg lifts
    tibia +  : foot swings outward
    all zero : standing pose (femur level, foot straight down)

The geometry numbers below must stay in sync with params.xacro.
"""
import math
from dataclasses import dataclass

LEGS = ('lf', 'lm', 'lr', 'rf', 'rm', 'rr')
JOINT_TYPES = ('hip', 'femur', 'tibia')
# Order of the 18 values in /leg_position_controller/commands
JOINT_ORDER = tuple(f'{leg}_{jt}_joint' for leg in LEGS for jt in JOINT_TYPES)

SIDE = {'lf': 1, 'lm': 1, 'lr': 1, 'rf': -1, 'rm': -1, 'rr': -1}
POSITION = {'lf': 1, 'lm': 0, 'lr': -1, 'rf': 1, 'rm': 0, 'rr': -1}  # front/middle/rear


@dataclass(frozen=True)
class Geometry:
    coxa: float = 0.075          # hip axis -> femur axis
    femur: float = 0.075         # femur axis -> tibia axis
    tibia: float = 0.100         # tibia axis -> foot center
    leg_spacing: float = 0.200
    leg_x_offset: float = -0.025
    hip_y: float = 0.1125        # body_width/2 + bracket_thickness
    hip_z: float = -0.0625       # -body_height/2 + 0.0125
    hip_limit: float = math.radians(120)
    femur_limit: float = math.radians(90)
    tibia_limit: float = math.radians(90)

    def hip_origin(self, leg):
        """Hip axis position in the body frame."""
        x = self.leg_x_offset + POSITION[leg] * self.leg_spacing
        return (x, SIDE[leg] * self.hip_y, self.hip_z)

    def limits(self):
        return (self.hip_limit, self.femur_limit, self.tibia_limit)


DEFAULT_GEOMETRY = Geometry()


# ---------- single leg, leg frame ----------

def leg_fk(q, geo=DEFAULT_GEOMETRY):
    """Joint angles (hip, femur, tibia) -> foot position in the leg frame."""
    q1, q2, q3 = q
    r = geo.coxa + geo.femur * math.cos(q2) + geo.tibia * math.sin(q2 + q3)
    z = geo.femur * math.sin(q2) - geo.tibia * math.cos(q2 + q3)
    return (r * math.cos(q1), r * math.sin(q1), z)


def leg_ik(p, geo=DEFAULT_GEOMETRY):
    """Foot position in the leg frame -> (angles, reachable).

    Closed form: the hip angle points the leg at the foot, then femur + tibia
    form a two-link arm in the vertical plane (law of cosines). Unreachable
    targets are clamped to the nearest reachable pose and joint limits are
    applied; `reachable` is False if either happened.
    """
    x, y, z = p
    reachable = True
    q1 = math.atan2(y, x)
    u = math.hypot(x, y) - geo.coxa          # horizontal distance from femur axis
    L2, L3 = geo.femur, geo.tibia
    d = (u * u + z * z - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    if d > 1.0 or d < -1.0:
        reachable = False
        d = max(-1.0, min(1.0, d))
    q3 = math.asin(d)                         # knee-down branch (standing pose is q3 = 0)
    gamma = q3 - math.pi / 2                  # tibia angle relative to femur
    q2 = math.atan2(z, u) - math.atan2(L3 * math.sin(gamma), L2 + L3 * math.cos(gamma))

    q = [q1, q2, q3]
    for i, lim in enumerate(geo.limits()):
        if abs(q[i]) > lim:
            reachable = False
            q[i] = max(-lim, min(lim, q[i]))
    return tuple(q), reachable


# ---------- body frame <-> leg frame ----------

def body_to_leg(leg, p, geo=DEFAULT_GEOMETRY):
    hx, hy, hz = geo.hip_origin(leg)
    dx, dy, dz = p[0] - hx, p[1] - hy, p[2] - hz
    # Leg frame is yawed by side * 90 deg: rotate the offset by the opposite.
    if SIDE[leg] > 0:
        return (dy, -dx, dz)
    return (-dy, dx, dz)


def leg_to_body(leg, p, geo=DEFAULT_GEOMETRY):
    hx, hy, hz = geo.hip_origin(leg)
    if SIDE[leg] > 0:
        dx, dy = -p[1], p[0]
    else:
        dx, dy = p[1], -p[0]
    return (hx + dx, hy + dy, hz + p[2])


def foot_ik(leg, p_body, geo=DEFAULT_GEOMETRY):
    """Foot position in the body frame -> (angles, reachable) for that leg."""
    return leg_ik(body_to_leg(leg, p_body, geo), geo)


def foot_fk(leg, q, geo=DEFAULT_GEOMETRY):
    return leg_to_body(leg, leg_fk(q, geo), geo)


def neutral_foot(leg, geo=DEFAULT_GEOMETRY, foot_z=None):
    """Standing-pose foot position in the body frame (all joints zero by default)."""
    hx, hy, hz = geo.hip_origin(leg)
    reach = geo.coxa + geo.femur
    z = hz - geo.tibia if foot_z is None else foot_z
    return (hx, hy + SIDE[leg] * reach, z)
