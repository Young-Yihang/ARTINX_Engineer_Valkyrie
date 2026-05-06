/// @file impedance_controller.hpp
/// @brief Joint impedance control: tau = K*(q_ref - q) - D*qdot, with D-priority anti-saturation
#ifndef IMPEDANCE_CONTROLLER_HPP
#define IMPEDANCE_CONTROLLER_HPP

#include <algorithm>
#include <cmath>
#include <vector>

struct ImpedanceGains
{
    double K = 0.0;         // Stiffness (Nm/rad)
    double D = 0.0;         // Damping (Nm*s/rad)
    double tau_limit = 0.0; // Torque saturation (Nm)
};

class JointImpedance
{
  public:
    JointImpedance() = default;
    explicit JointImpedance(const ImpedanceGains& gains) : gains_(gains) {}

    /// @return feedback torque (Nm), excludes dynamics feedforward
    double compute(double pos_ref, double pos_fdb, double vel_fdb)
    {
        pos_error_ = pos_ref - pos_fdb;
        tau_d_ = -gains_.D * vel_fdb;

        // D-priority anti-saturation: D always gets full budget, K uses remainder
        double d_abs = std::abs(tau_d_);
        double k_budget = (d_abs < gains_.tau_limit) ? (gains_.tau_limit - d_abs) : 0.0;
        tau_k_ = std::clamp(gains_.K * pos_error_, -k_budget, k_budget);

        double tau_total = tau_k_ + tau_d_;
        return std::clamp(tau_total, -gains_.tau_limit, gains_.tau_limit);
    }

    void reset()
    {
        pos_error_ = 0.0;
        tau_k_ = 0.0;
        tau_d_ = 0.0;
    }

    void setGains(const ImpedanceGains& gains) { gains_ = gains; }
    double getPositionError() const { return pos_error_; }
    double getTauK() const { return tau_k_; }
    double getTauD() const { return tau_d_; }

  private:
    ImpedanceGains gains_;
    double pos_error_ = 0.0;
    double tau_k_ = 0.0;
    double tau_d_ = 0.0;
};

/// Multi-joint impedance controller
class MultiJointImpedance
{
  public:
    explicit MultiJointImpedance(size_t num_joints) : controllers_(num_joints) {}

    void setJointGains(size_t idx, const ImpedanceGains& gains)
    {
        if (idx < controllers_.size())
            controllers_[idx].setGains(gains);
    }

    void compute(const std::vector<double>& pos_ref, const std::vector<double>& pos_fdb,
                 const std::vector<double>& vel_fdb, std::vector<double>& torque_out)
    {
        torque_out.resize(controllers_.size());
        for (size_t i = 0; i < controllers_.size(); ++i)
        {
            torque_out[i] = controllers_[i].compute(pos_ref[i], pos_fdb[i], vel_fdb[i]);
        }
    }

    void resetAll()
    {
        for (auto& c : controllers_)
            c.reset();
    }

    JointImpedance& getJointController(size_t idx) { return controllers_[idx]; }

  private:
    std::vector<JointImpedance> controllers_;
};

#endif // IMPEDANCE_CONTROLLER_HPP
