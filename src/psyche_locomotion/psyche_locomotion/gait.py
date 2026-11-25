"""Tripod gait for the Psyche hexapod. Pure Python, no ROS.

Two groups of three legs alternate:
    A = left-front, right-middle, left-rear
    B = right-front, left-middle, right-rear
While one group is planted and pushing (stance), the other lifts and moves
forward (swing), so three feet are always on the ground: a stable triangle.

Input:  body velocity (vx, vy, wz), the same thing /cmd_vel carries.
Output: 18 joint angles, in JOINT_ORDER.

Planted feet are moved backwards at exactly the body's velocity, so they don't
slide on the ground. Swinging feet travel to a touchdown point half a stride
ahead. When the command goes to zero, the gait keeps stepping until every foot
is back at its neutral spot, then stops with all six feet down.
"""
import math

from .kinematics import DEFAULT_GEOMETRY, LEGS, foot_ik, neutral_foot

GROUP_A = ('lf', 'rm', 'lr')
GROUP_B = ('rf', 'lm', 'rr')


def _clamp(v, lim):
    return max(-lim, min(lim, v))


def _approach(cur, target, max_step):
    return cur + _clamp(target - cur, max_step)


class TripodGait:

    def __init__(self, geo=DEFAULT_GEOMETRY, period=1.0, step_height=0.03,
                 foot_z=None, max_vx=0.10, max_vy=0.05, max_wz=0.40,
                 max_lin_accel=0.2, max_ang_accel=1.0):
        self.geo = geo
        self.period = period              # seconds per full cycle (two steps)
        self.step_height = step_height    # meters of foot lift
        self.max_v = (max_vx, max_vy, max_wz)
        self.max_accel = (max_lin_accel, max_lin_accel, max_ang_accel)
        self.neutral = {leg: neutral_foot(leg, geo, foot_z) for leg in LEGS}

        self.v = [0.0, 0.0, 0.0]          # smoothed body velocity actually being executed
        self.phase = 0.0                  # 0..1
        self.walking = False
        self.offset = {leg: [0.0, 0.0] for leg in LEGS}      # foot xy offset from neutral
        self.lift = {leg: 0.0 for leg in LEGS}
        self.in_swing = {leg: False for leg in LEGS}
        self.swing_start = {leg: (0.0, 0.0) for leg in LEGS}
        self.unreachable = False

    # ---- helpers ----

    def _local_phase(self, leg, phase):
        return (phase + (0.0 if leg in GROUP_A else 0.5)) % 1.0

    def _foot_velocity(self, leg):
        """Ground velocity of the neutral foot point, in the body frame."""
        vx, vy, wz = self.v
        px, py, _ = self.neutral[leg]
        return (vx - wz * py, vy + wz * px)

    def _feet_settled(self):
        return all(math.hypot(*self.offset[leg]) < 0.003 for leg in LEGS)

    # ---- main update ----

    def update(self, dt, vx, vy, wz):
        """Advance the gait by dt seconds toward the commanded velocity.
        Returns 18 joint angles in JOINT_ORDER."""
        target = [_clamp(vx, self.max_v[0]), _clamp(vy, self.max_v[1]), _clamp(wz, self.max_v[2])]
        for i in range(3):
            self.v[i] = _approach(self.v[i], target[i], self.max_accel[i] * dt)

        moving = any(abs(x) > 1e-4 for x in self.v)
        wants_to_step = moving or not self._feet_settled()

        if not self.walking and wants_to_step:
            self.walking = True

        if self.walking:
            new_phase = self.phase + dt / self.period
            # Only stop at a half-cycle boundary: that is when all six feet are down.
            if not wants_to_step:
                boundary = 0.5 if self.phase < 0.5 else 1.0
                if new_phase >= boundary:
                    new_phase = boundary
                    self.walking = False
            self.phase = new_phase % 1.0
            self._move_feet(dt)

        return self.joint_angles()

    def _move_feet(self, dt):
        half = self.period / 2.0
        for leg in LEGS:
            s = self._local_phase(leg, self.phase)
            fvx, fvy = self._foot_velocity(leg)
            if s < 0.5:
                # Stance: foot is on the ground, slides backwards relative to the body.
                self.in_swing[leg] = False
                self.offset[leg][0] -= fvx * dt
                self.offset[leg][1] -= fvy * dt
                self.lift[leg] = 0.0
            else:
                # Swing: lift and carry the foot to half a stride ahead of neutral.
                if not self.in_swing[leg]:
                    self.in_swing[leg] = True
                    self.swing_start[leg] = tuple(self.offset[leg])
                t = (s - 0.5) / 0.5
                blend = t * t * (3.0 - 2.0 * t)          # smooth start and stop
                tx, ty = fvx * half / 2.0, fvy * half / 2.0
                sx, sy = self.swing_start[leg]
                self.offset[leg][0] = sx + (tx - sx) * blend
                self.offset[leg][1] = sy + (ty - sy) * blend
                self.lift[leg] = self.step_height * math.sin(math.pi * t)

    def foot_positions(self):
        """Current foot targets in the body frame, keyed by leg."""
        out = {}
        for leg in LEGS:
            nx, ny, nz = self.neutral[leg]
            ox, oy = self.offset[leg]
            out[leg] = (nx + ox, ny + oy, nz + self.lift[leg])
        return out

    def joint_angles(self):
        angles = []
        self.unreachable = False
        for leg, p in self.foot_positions().items():
            q, ok = foot_ik(leg, p, self.geo)
            self.unreachable |= not ok
            angles.extend(q)
        return angles
