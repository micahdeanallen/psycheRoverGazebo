"""Run with: cd src/psyche_locomotion && python3 -m pytest test"""
import math
import random

from psyche_locomotion.kinematics import (LEGS, foot_fk, foot_ik, leg_fk, leg_ik,
                                          neutral_foot)
from psyche_locomotion.gait import TripodGait


def test_standing_pose_is_all_zeros():
    for leg in LEGS:
        q, ok = foot_ik(leg, neutral_foot(leg))
        assert ok
        assert all(abs(a) < 1e-9 for a in q), (leg, q)


def test_ik_inverts_fk():
    rng = random.Random(0)
    for _ in range(2000):
        q = (rng.uniform(-1.0, 1.0), rng.uniform(-0.8, 0.8), rng.uniform(-0.8, 0.8))
        q2, ok = leg_ik(leg_fk(q))
        assert ok
        assert all(abs(a - b) < 1e-7 for a, b in zip(q, q2)), (q, q2)


def test_body_frame_round_trip():
    rng = random.Random(1)
    for leg in LEGS:
        for _ in range(200):
            q = (rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6))
            q2, ok = foot_ik(leg, foot_fk(leg, q))
            assert ok and all(abs(a - b) < 1e-7 for a, b in zip(q, q2))


def test_unreachable_is_flagged():
    _, ok = leg_ik((1.0, 0.0, 0.0))
    assert not ok


def test_gait_keeps_three_feet_down_and_stops():
    gait = TripodGait()
    dt = 0.01
    for k in range(600):                      # 3 s forward, 3 s stopping
        cmd = (0.08, 0.0, 0.2) if k < 300 else (0.0, 0.0, 0.0)
        gait.update(dt, *cmd)
        down = sum(1 for leg in LEGS if gait.lift[leg] < 1e-6)
        assert down >= 3
        assert not gait.unreachable
    assert not gait.walking
    assert all(math.hypot(*gait.offset[leg]) < 0.003 for leg in LEGS)
