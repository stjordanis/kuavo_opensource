#pragma once

#include <iostream>
#include <vector>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/inverse_kinematics/com_position_constraint.h"
#include "drake/multibody/optimization/centroidal_momentum_constraint.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/solve.h"
#include "drake/common/autodiff.h"

namespace drake
{
  class CoMIK
  {
  public:
    CoMIK(multibody::MultibodyPlant<double> *plant, std::vector<std::string> frames_name, double tol = 1.0e-8, double solver_tol = 1.0e-8);

    bool solve(const std::vector<std::vector<Eigen::Vector3d>> &pose, const Eigen::VectorXd &q0, Eigen::VectorXd &q_sol);

  private:
    solvers::Binding<solvers::Constraint> AddCoMPositionConstraint(multibody::InverseKinematics &ik, const Eigen::Vector3d &r_des);

    multibody::MultibodyPlant<double> *plant_;
    std::unique_ptr<systems::Context<double>> plant_context_;
    std::vector<std::string> frames_name_;
    double tol_;
    double solver_tol_;
    Eigen::VectorXd prev_q_sol;
  };

  class CMIK
  {
  public:
    CMIK(multibody::MultibodyPlant<double> *plant, std::vector<std::string> frames_name, double tol = 1.0e-8, double solver_tol = 1.0e-8);

    bool solve(const std::vector<std::vector<Eigen::Vector3d>> &pose, const Eigen::VectorXd &q0, const Eigen::VectorXd &v0, double dt,
               Eigen::VectorXd &q_sol, Eigen::VectorXd &v_sol);

  private:
    solvers::Binding<solvers::Constraint> AddCoMPositionConstraint(multibody::InverseKinematics &ik, const Eigen::Vector3d &r_des);
    solvers::Binding<solvers::Constraint> AddCMConstraint(multibody::InverseKinematics &ik, solvers::VectorXDecisionVariable &v,
                                                          const Eigen::Vector3d &k_WC_des, const Eigen::VectorXd &q0, const Eigen::VectorXd &v0, double dt);

    multibody::MultibodyPlant<double> *plant_;
    std::unique_ptr<systems::Context<double>> plant_context_;
    std::unique_ptr<multibody::MultibodyPlant<AutoDiffXd>> plant_autodiff_;
    std::unique_ptr<systems::Context<AutoDiffXd>> plant_context_autodiff_;
    uint32_t nq_;
    uint32_t nv_;
    double robot_total_mass_;
    std::vector<std::string> frames_name_;
    double tol_;
    double solver_tol_;
  };

  class CoMVIK
  {
  public:
    CoMVIK(multibody::MultibodyPlant<double> *plant, std::vector<std::string> frames_name, double tol = 1.0e-8, double solver_tol = 1.0e-7);

    bool solve(std::vector<std::vector<Eigen::Vector3d>> &pose, Eigen::VectorXd &q0, Eigen::VectorXd &v0, double dt,
               Eigen::VectorXd &q_sol, Eigen::VectorXd &v_sol);

  private:
    solvers::Binding<solvers::Constraint> AddCMConstraint(multibody::InverseKinematics &ik, solvers::VectorXDecisionVariable &v,
                                                          Eigen::VectorXd &k_WC_des, Eigen::VectorXd &q0, Eigen::VectorXd &v0, double dt);
    solvers::Binding<solvers::Constraint> AddVelocityConstraint(multibody::InverseKinematics &ik, solvers::VectorXDecisionVariable &v,
                                                                Eigen::VectorXd &q0, Eigen::VectorXd &v0, std::string frame_name,
                                                                const Eigen::Matrix<double, 6, 1> &sv_lower,
                                                                const Eigen::Matrix<double, 6, 1> &sv_upper);

    multibody::MultibodyPlant<double> *plant_;
    std::unique_ptr<systems::Context<double>> plant_context_;
    std::unique_ptr<multibody::MultibodyPlant<AutoDiffXd>> plant_autodiff_;
    std::unique_ptr<systems::Context<AutoDiffXd>> plant_context_autodiff_;
    uint32_t nq_;
    uint32_t nv_;
    double robot_total_mass_;
    std::vector<std::string> frames_name_;
    double tol_;
    double solver_tol_;
  };

  class AnalyticalIK
  {
  public:
    AnalyticalIK(multibody::MultibodyPlant<double> *plant, std::vector<std::string> frames_name, double tol = 1.0e-8, double solver_tol = 1.0e-7);

    void AnalyticalFK(Eigen::VectorXd &pose_err, Eigen::VectorXd &qv);
    bool solve(const std::vector<Eigen::VectorXd> &pose, const Eigen::VectorXd &q0, const Eigen::VectorXd &v0,
               Eigen::VectorXd &q_sol, Eigen::VectorXd &qd_sol, Eigen::VectorXd &qdd_sol);

  private:
    multibody::MultibodyPlant<double> *plant_;
    std::unique_ptr<systems::Context<double>> plant_context_;
    std::unique_ptr<multibody::MultibodyPlant<AutoDiffXd>> plant_autodiff_;
    std::unique_ptr<systems::Context<AutoDiffXd>> plant_context_autodiff_;
    uint32_t nq_;
    uint32_t nv_;
    double robot_total_mass_;
    std::vector<std::string> frames_name_;
    double alpha_;
    double tol_;
    double solver_tol_;
  };
} // namespace drake
