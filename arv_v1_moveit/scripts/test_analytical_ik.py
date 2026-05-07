#!/usr/bin/env python3
"""
验证 analytical_ik.hpp 的正确性:
1. 对随机关节角做 FK (Python 复现 URDF 链)
2. 对 FK 结果做 IK (Python 复现 analytical_ik.hpp 逻辑)
3. 检查 IK 输出 vs 原始关节角是否一致

用法: python3 test_analytical_ik.py
"""

import numpy as np
from numpy import cos, sin, pi, sqrt

# URDF geometry
D1 = 0.055
L2 = 0.440
L3 = 0.3505
D_TCP = 0.179

JOINT_LOWER = np.array([-1.2217, 0.49, -0.90, -2.975, -1.5708, -pi])
JOINT_UPPER = np.array([1.2217, 3.14, 0.70, 3.14, 1.5708, pi])

def Rz(a):
    return np.array([[cos(a), -sin(a), 0],
                     [sin(a),  cos(a), 0],
                     [0,       0,      1]])

def Rx(a):
    return np.array([[1, 0,      0],
                     [0, cos(a), -sin(a)],
                     [0, sin(a),  cos(a)]])

def T_from_Rp(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T

def fk(q):
    """Compute FK following exact URDF chain: base_link → tcp"""
    # J1: origin=(0,0,0), rpy=(0,0,0), axis=z
    T01 = T_from_Rp(Rz(q[0]), [0, 0, 0])

    # J2: origin=(0,0,0.055), rpy=(pi/2,0,0), axis=z
    T12 = T_from_Rp(Rx(pi/2) @ Rz(q[1]), [0, 0, D1])

    # J3: origin=(0.440,0,0), rpy=(0,0,0), axis=z
    T23 = T_from_Rp(Rz(q[2]), [L2, 0, 0])

    # J4: origin=(0,0,0), rpy=(pi/2,0,0), axis=z
    T34 = T_from_Rp(Rx(pi/2) @ Rz(q[3]), [0, 0, 0])

    # J5: origin=(0,0,0.3505), rpy=(-pi/2,0,0), axis=z
    T45 = T_from_Rp(Rx(-pi/2) @ Rz(q[4]), [0, 0, L3])

    # J6: origin=(0,0,0), rpy=(pi/2,0,0), axis=z
    T56 = T_from_Rp(Rx(pi/2) @ Rz(q[5]), [0, 0, 0])

    # TCP: origin=(0,0,0.179), fixed
    T6tcp = T_from_Rp(np.eye(3), [0, 0, D_TCP])

    T = T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T6tcp
    return T[:3, :3], T[:3, 3]


def compute_R03(q1, q2, q3):
    """R_03 = Rz(q1) * Rx(pi/2) * Rz(q2+q3)"""
    c1, s1 = cos(q1), sin(q1)
    c23, s23 = cos(q2 + q3), sin(q2 + q3)

    R03 = np.array([
        [c1*c23, -c1*s23, s1],
        [s1*c23, -s1*s23, -c1],
        [s23,     c23,     0.0]
    ])
    return R03


def analytical_ik(R_target, p_target, q_seed):
    """Python mirror of analytical_ik.hpp"""
    # Wrist center
    pw = p_target - D_TCP * R_target[:, 2]

    best_q = None
    best_cost = 1e18

    # J1 candidates
    q1_primary = np.arctan2(pw[1], pw[0])
    q1_candidates = []
    if JOINT_LOWER[0] <= q1_primary <= JOINT_UPPER[0]:
        q1_candidates.append(q1_primary)
    q1_flip = q1_primary + (-pi if q1_primary > 0 else pi)
    if JOINT_LOWER[0] <= q1_flip <= JOINT_UPPER[0]:
        q1_candidates.append(q1_flip)
    if not q1_candidates:
        q1_candidates.append(np.clip(q1_primary, JOINT_LOWER[0], JOINT_UPPER[0]))

    for q1 in q1_candidates:
        c1, s1 = cos(q1), sin(q1)
        r = c1 * pw[0] + s1 * pw[1]
        s = pw[2] - D1

        D2 = r*r + s*s
        cos_q3_raw = (D2 - L2*L2 - L3*L3) / (2.0 * L2 * L3)
        cos_q3_raw = np.clip(cos_q3_raw, -0.9999, 0.9999)
        if abs(cos_q3_raw) > 1.0:
            continue

        for elbow in [0, 1]:
            sin_q3 = sqrt(1.0 - cos_q3_raw**2)
            if elbow == 1:
                sin_q3 = -sin_q3

            q3_geom = np.arctan2(sin_q3, cos_q3_raw)
            alpha = np.arctan2(s, r)
            beta = np.arctan2(L3 * sin_q3, L2 + L3 * cos_q3_raw)
            q2 = alpha + beta
            q3 = pi/2 - q3_geom

            if not (JOINT_LOWER[1] - 1e-6 <= q2 <= JOINT_UPPER[1] + 1e-6):
                continue
            if not (JOINT_LOWER[2] - 1e-6 <= q3 <= JOINT_UPPER[2] + 1e-6):
                continue

            # Orientation
            R03 = compute_R03(q1, q2, q3)
            M = R03.T @ R_target

            # N = Rx(-pi/2) * M
            N = np.array([M[0], M[2], -M[1]])

            cos_q5 = N[2, 2]
            sin_q5_sq = N[0, 2]**2 + N[1, 2]**2
            sin_q5_abs = sqrt(sin_q5_sq)

            for wrist_sign in [0, 1]:
                sin_q5 = sin_q5_abs if wrist_sign == 0 else -sin_q5_abs

                if abs(sin_q5) < 0.05:
                    q5 = 0.05 if sin_q5 >= 0 else -0.05
                    sum_angle = np.arctan2(N[1, 0] - N[0, 1], N[0, 0] + N[1, 1])
                    if cos_q5 > 0:
                        q4 = q_seed[3]
                        q6 = sum_angle - q4
                    else:
                        q4 = q_seed[3]
                        q6 = q4 - sum_angle
                    q6 = (q6 + pi) % (2*pi) - pi
                else:
                    q5 = np.arctan2(sin_q5, cos_q5)
                    if sin_q5 > 0:
                        q4 = np.arctan2(N[1, 2], N[0, 2])
                        q6 = np.arctan2(N[2, 1], -N[2, 0])
                    else:
                        q4 = np.arctan2(-N[1, 2], -N[0, 2])
                        q6 = np.arctan2(-N[2, 1], N[2, 0])

                if not (JOINT_LOWER[3] - 1e-6 <= q4 <= JOINT_UPPER[3] + 1e-6):
                    continue
                if not (JOINT_LOWER[4] - 1e-6 <= q5 <= JOINT_UPPER[4] + 1e-6):
                    continue

                q_cand = np.array([q1, q2, q3, q4, q5, q6])
                w = np.array([2.0, 2.0, 2.0, 1.0, 1.5, 0.5])
                diff = q_cand - q_seed
                diff[5] = (diff[5] + pi) % (2*pi) - pi
                cost = np.sum(w * diff**2)

                if cost < best_cost:
                    best_cost = cost
                    best_q = q_cand.copy()

    return best_q


def main():
    np.random.seed(42)
    n_tests = 2000
    n_pass = 0
    n_fail = 0
    max_err = 0.0

    for i in range(n_tests):
        # Random joint angles within limits
        q_orig = np.array([
            np.random.uniform(JOINT_LOWER[0], JOINT_UPPER[0]),
            np.random.uniform(JOINT_LOWER[1], JOINT_UPPER[1]),
            np.random.uniform(JOINT_LOWER[2], JOINT_UPPER[2]),
            np.random.uniform(JOINT_LOWER[3], JOINT_UPPER[3]),
            np.random.uniform(JOINT_LOWER[4], JOINT_UPPER[4]),
            np.random.uniform(-pi, pi),
        ])

        # FK
        R, p = fk(q_orig)

        # IK with original as seed (best case)
        q_ik = analytical_ik(R, p, q_orig)
        if q_ik is None:
            n_fail += 1
            if n_fail <= 5:
                print(f"  FAIL [{i}]: no solution for q={np.round(q_orig, 3)}")
            continue

        # Verify: FK of IK result should match target
        R_check, p_check = fk(q_ik)
        pos_err = np.linalg.norm(p_check - p)
        rot_err = np.linalg.norm(R_check - R, 'fro')

        # Near wrist singularity (|q5|<0.05), clamp introduces intentional small error
        is_singular = abs(q_orig[4]) < 0.05
        tol_pos = 0.01 if is_singular else 1e-4
        tol_rot = 0.08 if is_singular else 1e-4

        if pos_err < tol_pos and rot_err < tol_rot:
            n_pass += 1
            max_err = max(max_err, pos_err)
        else:
            n_fail += 1
            if n_fail <= 5:
                print(f"  FAIL [{i}]: pos_err={pos_err:.6f} rot_err={rot_err:.6f} singular={is_singular}")
                print(f"    q_orig={np.round(q_orig, 4)}")
                print(f"    q_ik  ={np.round(q_ik, 4)}")

    print(f"\n{'='*60}")
    print(f"Analytical IK validation: {n_pass}/{n_tests} PASS, {n_fail} FAIL")
    print(f"Max position error: {max_err:.2e} m")
    print(f"{'='*60}")

    if n_fail > 0:
        print("\n⚠ Some tests failed — check coordinate mapping!")
    else:
        print("\n✓ All tests passed — IK is correct!")

    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    exit(main())
