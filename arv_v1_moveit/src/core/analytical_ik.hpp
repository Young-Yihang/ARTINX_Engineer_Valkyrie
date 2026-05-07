/// @file analytical_ik.hpp
/// @brief Closed-form 6-DOF IK for ARV_V1 (Pieper: yaw+pitch+pitch, no offset).
#ifndef ANALYTICAL_IK_HPP
#define ANALYTICAL_IK_HPP

#include <algorithm>
#include <array>
#include <cmath>

struct IKResult {
  std::array<double, 6> q{};
  bool valid = false;
};

class AnalyticalIK {
public:
  static constexpr double kD1 = 0.055;
  static constexpr double kL2 = 0.440;
  static constexpr double kL3 = 0.3505;
  static constexpr double kDtcp = 0.179;

  static constexpr double kJointLower[6] = {-1.2217, 0.49, -0.90, -2.975, -1.5708, -1e9};
  static constexpr double kJointUpper[6] = {1.2217, 3.14, 0.70, 3.14, 1.5708, 1e9};

  /// Solve IK given target rotation (row-major 3x3) and position, using seed for multi-solution.
  IKResult solve(const double R_target[9], const double p_target[3],
                 const double q_seed[6]) const {
    // Wrist center: pw = p - d_tcp * R[:,2]
    double pw[3] = {p_target[0] - kDtcp * R_target[2],
                    p_target[1] - kDtcp * R_target[5],
                    p_target[2] - kDtcp * R_target[8]};

    // Generate candidate solutions (up to 8: 2 q1 × 2 elbow × 2 wrist)
    IKResult best;
    double best_cost = 1e18;

    double q1_candidates[2];
    int n_q1 = generateQ1(pw, q_seed[0], q1_candidates);

    for (int iq1 = 0; iq1 < n_q1; iq1++) {
      double q1 = q1_candidates[iq1];

      for (int elbow = 0; elbow < 2; elbow++) {
        double q2, q3;
        if (!solvePosition(pw, q1, elbow, q2, q3)) continue;
        if (!inLimits(1, q2) || !inLimits(2, q3)) continue;

        for (int wrist_sign = 0; wrist_sign < 2; wrist_sign++) {
          double q4, q5, q6;
          if (!solveOrientation(R_target, q1, q2, q3, wrist_sign, q_seed, q4, q5, q6))
            continue;
          if (!inLimits(3, q4) || !inLimits(4, q5)) continue;

          std::array<double, 6> q_cand = {q1, q2, q3, q4, q5, q6};
          double cost = solutionCost(q_cand, q_seed);
          if (cost < best_cost) {
            best_cost = cost;
            best.q = q_cand;
            best.valid = true;
          }
        }
      }
    }
    return best;
  }

private:
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kSingularityThresh = 0.05;

  static bool inLimits(int idx, double val) {
    return val >= kJointLower[idx] - 1e-6 && val <= kJointUpper[idx] + 1e-6;
  }

  static double wrapPi(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
  }

  /// Wrap angle into joint limits (for revolute joints whose range spans ~2π like J4)
  static double wrapToLimits(int idx, double val) {
    double lo = kJointLower[idx], hi = kJointUpper[idx];
    while (val < lo) val += 2.0 * kPi;
    while (val > hi) val -= 2.0 * kPi;
    if (val >= lo && val <= hi) return val;
    // If still out, try one more wrap and pick closest
    double v_up = val + 2.0 * kPi;
    double v_dn = val - 2.0 * kPi;
    double d_cur = std::min(std::abs(val - lo), std::abs(val - hi));
    double d_up = (v_up >= lo && v_up <= hi) ? 0.0 : std::min(std::abs(v_up - lo), std::abs(v_up - hi));
    double d_dn = (v_dn >= lo && v_dn <= hi) ? 0.0 : std::min(std::abs(v_dn - lo), std::abs(v_dn - hi));
    if (v_up >= lo && v_up <= hi) return v_up;
    if (v_dn >= lo && v_dn <= hi) return v_dn;
    // All candidates out of limits — clamp to nearest boundary
    return (std::abs(val - lo) < std::abs(val - hi)) ? lo : hi;
  }

  static double solutionCost(const std::array<double, 6>& q, const double seed[6]) {
    double cost = 0.0;
    // Weighted: proximal joints matter more for continuity
    static constexpr double w[6] = {2.0, 2.0, 2.0, 1.0, 1.5, 0.5};
    for (int i = 0; i < 6; i++) {
      double d = (i == 5) ? wrapPi(q[i] - seed[i]) : (q[i] - seed[i]);
      cost += w[i] * d * d;
    }
    return cost;
  }

  int generateQ1(const double pw[3], double q1_seed, double q1_out[2]) const {
    double q1_primary = std::atan2(pw[1], pw[0]);
    int n = 0;

    if (inLimits(0, q1_primary)) q1_out[n++] = q1_primary;

    // Shoulder-flip candidate
    double q1_flip = q1_primary + ((q1_primary > 0) ? -kPi : kPi);
    if (inLimits(0, q1_flip)) q1_out[n++] = q1_flip;

    if (n == 0) {
      // Fallback: clamp primary to limits
      q1_out[0] = std::clamp(q1_primary, kJointLower[0], kJointUpper[0]);
      n = 1;
    }
    return n;
  }

  /// Solve J2, J3 for given q1 and elbow config (0=elbow-up, 1=elbow-down).
  /// URDF convention: q2 is angle in the pitch plane measured from link1's z-axis.
  bool solvePosition(const double pw[3], double q1, int elbow, double& q2, double& q3) const {
    double c1 = std::cos(q1), s1 = std::sin(q1);

    // Project wrist point into the J2 pitch plane
    // After J1 rotation + d1 offset: the pitch plane has:
    //   r = horizontal distance in q1 direction
    //   s = vertical distance above J2
    double r = c1 * pw[0] + s1 * pw[1];
    double s = pw[2] - kD1;

    double D2 = r * r + s * s;
    double cos_q3_raw = (D2 - kL2 * kL2 - kL3 * kL3) / (2.0 * kL2 * kL3);

    // Elbow singularity clamp
    if (cos_q3_raw > 0.9999) cos_q3_raw = 0.9999;
    if (cos_q3_raw < -0.9999) cos_q3_raw = -0.9999;
    if (std::abs(cos_q3_raw) > 1.0) return false;

    double sin_q3 = std::sqrt(1.0 - cos_q3_raw * cos_q3_raw);
    if (elbow == 1) sin_q3 = -sin_q3;

    // q3 in URDF frame: angle between link2 and link3 in the pitch plane
    // URDF q3 sign: positive = link3 goes "up" relative to link2
    double q3_geom = std::atan2(sin_q3, cos_q3_raw);

    double alpha = std::atan2(s, r);
    double beta = std::atan2(kL3 * sin_q3, kL2 + kL3 * cos_q3_raw);
    q2 = alpha + beta;

    // Mapping: q3_urdf = π/2 - q3_geom (verified empirically against URDF FK)
    q3 = kPi / 2.0 - q3_geom;

    return true;
  }

  /// Compute R_0^3 (rotation from base to frame3)
  void computeR03(double q1, double q2, double q3, double R03[9]) const {
    // T_01: Rz(q1)
    // T_12: Trans(0,0,d1) * Rx(π/2) * Rz(q2)
    // T_23: Trans(L2,0,0) * Rz(q3)
    //
    // R_01 = Rz(q1)
    // R_12 = Rx(π/2) * Rz(q2)
    // R_23 = Rz(q3)
    // R_03 = Rz(q1) * Rx(π/2) * Rz(q2) * Rz(q3) = Rz(q1) * Rx(π/2) * Rz(q2+q3)

    double c1 = std::cos(q1), s1 = std::sin(q1);
    double c23 = std::cos(q2 + q3), s23 = std::sin(q2 + q3);

    // Rz(q1) * Rx(π/2) * Rz(q2+q3):
    // Rx(π/2) = [[1,0,0],[0,0,-1],[0,1,0]]
    // Rz(a)   = [[ca,-sa,0],[sa,ca,0],[0,0,1]]
    //
    // Rx(π/2)*Rz(q23) = [[c23, -s23, 0], [0, 0, -1], [s23, c23, 0]]
    // Rz(q1) * above:
    // row0: [c1*c23, -c1*s23, -s1]  ... wait, let me be careful
    //
    // M = Rx(π/2)*Rz(q23):
    //   M[0][0]=c23, M[0][1]=-s23, M[0][2]=0
    //   M[1][0]=0,   M[1][1]=0,    M[1][2]=-1
    //   M[2][0]=s23, M[2][1]=c23,  M[2][2]=0
    //
    // R03 = Rz(q1) * M:
    //   [c1, -s1, 0]   [c23, -s23,  0]
    //   [s1,  c1, 0] × [ 0,    0,  -1]
    //   [ 0,   0, 1]   [s23,  c23,  0]

    R03[0] = c1 * c23;
    R03[1] = -c1 * s23;
    R03[2] = s1;  // was -s1*0 + ... let me redo

    // Actually:
    // Row 0 of Rz(q1)*M: c1*M[0] + (-s1)*M[1]
    //   = (c1*c23+0, -c1*s23+0, 0+s1) = (c1*c23, -c1*s23, s1)
    // Row 1 of Rz(q1)*M: s1*M[0] + c1*M[1]
    //   = (s1*c23+0, -s1*s23+0, 0-c1) = (s1*c23, -s1*s23, -c1)
    // Row 2 of Rz(q1)*M: M[2]
    //   = (s23, c23, 0)

    R03[0] = c1 * c23;   R03[1] = -c1 * s23;  R03[2] = s1;
    R03[3] = s1 * c23;   R03[4] = -s1 * s23;  R03[5] = -c1;
    R03[6] = s23;         R03[7] = c23;         R03[8] = 0.0;
  }

  /// Solve J4, J5, J6 from R_36 = R_03^T * R_target (ZYZ Euler angles)
  bool solveOrientation(const double R_target[9], double q1, double q2, double q3,
                        int wrist_sign, const double q_seed[6],
                        double& q4, double& q5, double& q6) const {
    double R03[9];
    computeR03(q1, q2, q3, R03);

    // R_36 = R_03^T * R_target (but we need to account for J4's fixed transform)
    // Chain after frame3: Rx(π/2)*Rz(q4) * Trans(0,0,L3) * Rx(-π/2)*Rz(q5) * Rx(π/2)*Rz(q6)
    //
    // The rotation from frame3 to frame6:
    // R_36 = Rx(π/2)*Rz(q4) * Rx(-π/2)*Rz(q5) * Rx(π/2)*Rz(q6)
    //
    // This is NOT a simple ZYZ. Let me work it out:
    // Let A = Rx(π/2)*Rz(q4)*Rx(-π/2) = rotation about Y by q4
    // Then R_34 = Rx(π/2)*Rz(q4) and the local z after is the new axis
    //
    // Actually the structure Rx(π/2)*Rz(q4)*Rx(-π/2)*Rz(q5)*Rx(π/2)*Rz(q6) is a standard
    // ZYZ Euler decomposition in the "rotated frame" basis.
    //
    // Let's define R_3_to_6:
    //   Frame3 → [Rx(π/2)] → axis=z for J4 → [Rz(q4)] →
    //   → [Trans(0,0,L3)*Rx(-π/2)] → axis=z for J5 → [Rz(q5)] →
    //   → [Rx(π/2)] → axis=z for J6 → [Rz(q6)]
    //
    // Rotational part only (translations don't affect rotation):
    //   R_36 = Rx(π/2) * Rz(q4) * Rx(-π/2) * Rz(q5) * Rx(π/2) * Rz(q6)
    //
    // Key identity: Rx(π/2)*Rz(θ)*Rx(-π/2) = Ry(θ)
    // And: Rx(-π/2)*Rz(θ)*Rx(π/2) = Ry(-θ)
    //
    // So: R_36 = Ry(q4) * Rz(q5) * Ry(-1)*... no, let me be more careful.
    //
    // R_36 = [Rx(π/2)*Rz(q4)] * [Rx(-π/2)*Rz(q5)] * [Rx(π/2)*Rz(q6)]
    //       = Rx(π/2)*Rz(q4)*Rx(-π/2) * Rz(q5) * Rx(π/2)*Rz(q6)
    //       = Ry(q4) * Rz(q5) * Rx(π/2)*Rz(q6)
    //
    // Hmm, this is getting complex. Let me use the direct approach:
    // Define M = R_03^T * R_target = R_36_with_fixed_transforms
    // But the "fixed transforms" around each joint add Rx(±π/2).
    //
    // The total rotation from frame3 to TCP (frame6 has no extra rotation to TCP):
    //   R_3tcp = Rx(π/2) * Rz(q4) * Rx(-π/2) * Rz(q5) * Rx(π/2) * Rz(q6)
    //
    // We want: R_3tcp = R_03^T * R_target
    // Let M = R_03^T * R_target
    //
    // Then: Rx(-π/2) * M = Rz(q4) * Rx(-π/2) * Rz(q5) * Rx(π/2) * Rz(q6)
    // Let N = Rx(-π/2) * M
    // N = Rz(q4) * [Rx(-π/2) * Rz(q5) * Rx(π/2)] * Rz(q6)
    //   = Rz(q4) * Ry(-q5) * Rz(q6)
    //
    // This is ZY'Z Euler angles! (with q5 negated in Y)
    // Actually Rx(-π/2)*Rz(q5)*Rx(π/2):
    //   Rx(-π/2) = [[1,0,0],[0,0,1],[0,-1,0]]
    //   Rx(π/2)  = [[1,0,0],[0,0,-1],[0,1,0]]
    //   Rx(-π/2)*Rz(q5)*Rx(π/2):
    //     Rz(q5)*Rx(π/2) = [[c5,0,-s5],[s5,0,c5],[0,-1,0]]... wait
    //     Rz(q5) = [[c5,-s5,0],[s5,c5,0],[0,0,1]]
    //     Rz(q5)*Rx(π/2) = [[c5, -s5*0-0, c5*0+(-s5)*(-1)+0],... this is wrong approach
    //
    // Let me just multiply directly:
    //   Rz(q5)*Rx(π/2) col-by-col:
    //     col0: Rz(q5)*[1,0,0]^T = [c5,s5,0]
    //     col1: Rz(q5)*[0,0,1]^T... no, Rx(π/2) = [[1,0,0],[0,0,-1],[0,1,0]]
    //     col0 of Rx(π/2): [1,0,0] → Rz*[1,0,0] = [c5,s5,0]
    //     col1 of Rx(π/2): [0,0,1] → Rz*[0,0,1] = [0,0,1]
    //     col2 of Rx(π/2): [0,-1,0] → Rz*[0,-1,0] = [s5,-c5,0]
    //   So Rz(q5)*Rx(π/2) = [[c5,0,s5],[s5,0,-c5],[0,1,0]]
    //
    //   Rx(-π/2)*above: Rx(-π/2) = [[1,0,0],[0,0,1],[0,-1,0]]
    //     row0: [c5, 0, s5]
    //     row1: [0, 1, 0]
    //     row2: [-s5, 0, c5]
    //   So Rx(-π/2)*Rz(q5)*Rx(π/2) = [[c5,0,s5],[0,1,0],[-s5,0,c5]] = Ry(q5)
    //
    // So: N = Rz(q4) * Ry(q5) * Rz(q6) — standard ZYZ Euler angles!

    // Compute M = R_03^T * R_target
    double M[9];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        M[i * 3 + j] = 0.0;
        for (int k = 0; k < 3; k++) {
          M[i * 3 + j] += R03[k * 3 + i] * R_target[k * 3 + j];  // R03^T * R_target
        }
      }
    }

    // N = Rx(-π/2) * M
    // Rx(-π/2) = [[1,0,0],[0,0,1],[0,-1,0]]
    double N[9];
    N[0] = M[0]; N[1] = M[1]; N[2] = M[2];           // row0 = M row0
    N[3] = M[6]; N[4] = M[7]; N[5] = M[8];           // row1 = M row2
    N[6] = -M[3]; N[7] = -M[4]; N[8] = -M[5];        // row2 = -M row1

    // ZYZ decomposition of N = Rz(q4)*Ry(q5)*Rz(q6)
    // N[2][2] = cos(q5)
    // N[0][2] = cos(q4)*sin(q5)
    // N[1][2] = sin(q4)*sin(q5)
    // N[2][0] = sin(q5)*cos(q6)   ... wait, let me verify
    //
    // Rz(a)*Ry(b)*Rz(c):
    //   [[ca*cb*cc - sa*sc, -ca*cb*sc - sa*cc, ca*sb],
    //    [sa*cb*cc + ca*sc, -sa*cb*sc + ca*cc, sa*sb],
    //    [-sb*cc,            sb*sc,             cb   ]]
    //
    // So: N[8] = cos(q5)  → N[2][2]
    //     N[2] = cos(q4)*sin(q5)  → N[0][2]
    //     N[5] = sin(q4)*sin(q5)  → N[1][2]
    //     N[6] = -sin(q5)*cos(q6) → N[2][0]
    //     N[7] = sin(q5)*sin(q6)  → N[2][1]

    double cos_q5 = N[8];
    double sin_q5_sq = N[2] * N[2] + N[5] * N[5];  // (ca*sb)^2 + (sa*sb)^2 = sb^2
    double sin_q5 = std::sqrt(sin_q5_sq);

    if (wrist_sign == 1) sin_q5 = -sin_q5;

    // Wrist singularity handling
    if (std::abs(sin_q5) < kSingularityThresh) {
      // q4 + q6 is determined, individual values are not
      // Lock q5 to clamp value, distribute q4+q6 by seed proximity
      q5 = (sin_q5 >= 0) ? kSingularityThresh : -kSingularityThresh;
      // When sin(q5)≈0, cos(q5)≈±1: N ≈ Rz(q4+q6) or Rz(q4-q6)
      double sum_angle = std::atan2(N[3] - N[1], N[0] + N[4]);  // atan2(sa*cb-(-sa), ca*cb+ca) simplified
      // For cos_q5≈1: sum = q4+q6; for cos_q5≈-1: sum = q4-q6
      if (cos_q5 > 0) {
        // q4 + q6 = sum_angle
        q4 = q_seed[3];  // keep q4 at seed
        q6 = wrapPi(sum_angle - q4);
      } else {
        // q4 - q6 = sum_angle
        q4 = q_seed[3];
        q6 = wrapPi(q4 - sum_angle);
      }
    } else {
      q5 = std::atan2(sin_q5, cos_q5);
      if (sin_q5 > 0) {
        q4 = std::atan2(N[5], N[2]);         // atan2(sa*sb, ca*sb)
        q6 = std::atan2(N[7], -N[6]);        // atan2(sb*sc, sb*cc)
      } else {
        q4 = std::atan2(-N[5], -N[2]);       // flip sign for negative sin_q5
        q6 = std::atan2(-N[7], N[6]);
      }
    }

    // J4: revolute with ~2π range, atan2 output may land in the ~10° dead zone
    q4 = wrapToLimits(3, q4);
    // J6: continuous joint, normalize to [-π, π]
    q6 = wrapPi(q6);

    return true;
  }
};

#endif // ANALYTICAL_IK_HPP
