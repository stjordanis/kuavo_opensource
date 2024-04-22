#ifndef VELOCITY_SMOOTH_INTERPOLATOR_H
#define VELOCITY_SMOOTH_INTERPOLATOR_H
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/lcm/drake_lcm.h"
#include <drake/systems/lcm/lcm_interface_system.h>
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "utils.h"
#include <Eigen/Dense>
// #include <trajectories/piecewise_polynomial.h>
#include <iostream>
class SmoothInterpolator
{
public:
    SmoothInterpolator(const Eigen::VectorXd &start_pos,
                       const Eigen::VectorXd &start_vel,
                       const Eigen::VectorXd &target_pos,
                       double acceleration = 0.05,
                       double dt = 0.001);

    // void get(double t_step, Eigen::VectorXd& pos, Eigen::VectorXd& vel);
    void get(double t_step, Eigen::VectorXd *pos = nullptr, Eigen::VectorXd *vel = nullptr, Eigen::VectorXd *acc = nullptr);

    // 按顺序获取pos, vel, acc
    void get(Eigen::VectorXd *pos = nullptr, Eigen::VectorXd *vel = nullptr, Eigen::VectorXd *acc = nullptr);

    // 更新初始速度和目标位置
    void update(const Eigen::VectorXd &start_pos,
                const Eigen::VectorXd &start_vel,
                const std::vector<Eigen::VectorXd> &target_pos,
                double acceleration = 0.05);
    void update(const Eigen::VectorXd &start_pos,
                const Eigen::VectorXd &start_vel,
                const Eigen::VectorXd &target_pos,
                double acceleration = 0.05);
    void update(const Eigen::VectorXd &target_pos,
                double acceleration = 0.05);

private:
    Eigen::VectorXd start_pos_;
    Eigen::VectorXd start_vel_;
    std::vector<Eigen::VectorXd> target_pos_;
    double dt_;
    double step_;
    double current_traj_time_;
    double acceleration_;
    drake::trajectories::PiecewisePolynomial<double> pos_traj_;
    drake::trajectories::PiecewisePolynomial<double> vel_traj_;
    drake::trajectories::PiecewisePolynomial<double> acc_traj_;

    void updateTrajectory();
};

#endif // VELOCITY_SMOOTH_INTERPOLATOR_H
