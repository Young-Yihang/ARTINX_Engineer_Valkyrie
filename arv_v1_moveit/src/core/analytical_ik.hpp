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

  /// Solve IK without joint limit filtering (for diagnostics — reports what joints exceed limits).
  IKResult solveUnclamped(const double R_target[9], const double p_target[3],
                          const double q_seed[6]) const {
    double pw[3] = {p_target[0] - kDtcp * R_target[2], p_target[1] - kDtcp * R_target[5],
                    p_target[2] - kDtcp * R_target[8]};
    IKResult best;
    double best_cost = kInfCost;
    double q1_candidates[2];
    int n_q1 = generateQ1(pw, q_seed[0], q1_candidates);
    for (int iq1 = 0; iq1 < n_q1; iq1++) {
      double q1 = q1_candidates[iq1];
      for (int elbow = 0; elbow < 2; elbow++) {
        double q2, q3;
        if (!solvePosition(pw, q1, elbow, q2, q3)) continue;
        for (int wrist_sign = 0; wrist_sign < 2; wrist_sign++) {
          double q4, q5, q6;
          if (!solveOrientation(R_target, q1, q2, q3, wrist_sign, q_seed, q4, q5, q6)) continue;
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

  /// Solve IK given target rotation (row-major 3x3) and position, using seed for multi-solution.
  IKResult solve(const double R_target[9], const double p_target[3], const double q_seed[6]) const {
    // Wrist center: pw = p - d_tcp * R[:,2]
    double pw[3] = {p_target[0] - kDtcp * R_target[2], p_target[1] - kDtcp * R_target[5],
                    p_target[2] - kDtcp * R_target[8]};

    // Generate candidate solutions (up to 8: 2 q1 × 2 elbow × 2 wrist)
    IKResult best;
    double best_cost = kInfCost;

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
          if (!solveOrientation(R_target, q1, q2, q3, wrist_sign, q_seed, q4, q5, q6)) continue;
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
  static constexpr double kElbowClamp = 0.9999;  // avoid sqrt(0) at full extension/fold
  static constexpr double kInfCost = 1e18;

  static bool inLimits(int idx, double val) {
    return val >= kJointLower[idx] - 1e-6 && val <= kJointUpper[idx] + 1e-6;
  }

  static double wrapPi(double a) { return std::remainder(a, 2.0 * kPi); }

  /// Wrap angle into joint limits (for revolute joints whose range spans ~2π like J4)
  static double wrapToLimits(int idx, double val) {
    double lo = kJointLower[idx], hi = kJointUpper[idx];
    while (val < lo) val += 2.0 * kPi;
    while (val > hi) val -= 2.0 * kPi;
    if (val >= lo && val <= hi) return val;
    double v_up = val + 2.0 * kPi;
    double v_dn = val - 2.0 * kPi;
    if (v_up >= lo && v_up <= hi) return v_up;
    if (v_dn >= lo && v_dn <= hi) return v_dn;
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

    if (cos_q3_raw > kElbowClamp) cos_q3_raw = kElbowClamp;
    if (cos_q3_raw < -kElbowClamp) cos_q3_raw = -kElbowClamp;
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

  /// Compute R_0^3: R03 = Rz(q1) * Rx(π/2) * Rz(q2+q3)
  void computeR03(double q1, double q2, double q3, double R03[9]) const {
    double c1 = std::cos(q1), s1 = std::sin(q1);
    double c23 = std::cos(q2 + q3), s23 = std::sin(q2 + q3);

    R03[0] = c1 * c23;
    R03[1] = -c1 * s23;
    R03[2] = s1;
    R03[3] = s1 * c23;
    R03[4] = -s1 * s23;
    R03[5] = -c1;
    R03[6] = s23;
    R03[7] = c23;
    R03[8] = 0.0;
  }

  /// Solve J4, J5, J6 via ZYZ decomposition: N = Rz(q4)*Ry(q5)*Rz(q6)
  /// where N = Rx(-π/2) * R_03^T * R_target (accounting for fixed Rx(±π/2) between joints)
  bool solveOrientation(const double R_target[9], double q1, double q2, double q3, int wrist_sign,
                        const double q_seed[6], double& q4, double& q5, double& q6) const {
    double R03[9];
    computeR03(q1, q2, q3, R03);

    // M = R_03^T * R_target
    double M[9];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        M[i * 3 + j] = 0.0;
        for (int k = 0; k < 3; k++) {
          M[i * 3 + j] += R03[k * 3 + i] * R_target[k * 3 + j];  // R03^T * R_target
        }
      }
    }

    // N = Rx(-π/2) * M  →  row0=M[0], row1=M[2], row2=-M[1]
    double N[9];
    N[0] = M[0];
    N[1] = M[1];
    N[2] = M[2];
    N[3] = M[6];
    N[4] = M[7];
    N[5] = M[8];
    N[6] = -M[3];
    N[7] = -M[4];
    N[8] = -M[5];

    // ZYZ: N[8]=cb, N[2]=ca*sb, N[5]=sa*sb, N[6]=-sb*cc, N[7]=sb*sc
    double cos_q5 = N[8];
    double sin_q5_sq = N[2] * N[2] + N[5] * N[5];
    double sin_q5 = std::sqrt(sin_q5_sq);

    if (wrist_sign == 1) sin_q5 = -sin_q5;

    // Wrist singularity: q4+q6 determined but individual values degenerate
    if (std::abs(sin_q5) < kSingularityThresh) {
      q5 = (sin_q5 >= 0) ? kSingularityThresh : -kSingularityThresh;
      double sum_angle = std::atan2(N[3] - N[1], N[0] + N[4]);
      if (cos_q5 > 0) {
        q4 = q_seed[3];
        q6 = wrapPi(sum_angle - q4);
      } else {
        q4 = q_seed[3];
        q6 = wrapPi(q4 - sum_angle);
      }
    } else {
      q5 = std::atan2(sin_q5, cos_q5);
      if (sin_q5 > 0) {
        q4 = std::atan2(N[5], N[2]);
        q6 = std::atan2(N[7], -N[6]);
      } else {
        q4 = std::atan2(-N[5], -N[2]);
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

#endif  // ANALYTICAL_IK_HPP
