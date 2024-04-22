#include <Trajectory.h>
DECLARE_double(powerscale);


namespace HighlyDynamic
{
  void Trajectory::planArm(RobotState_t &state_des, RobotState_t &state_est)
  {
    if (state_des.phase == P_walk)
      {
        Eigen::VectorXd desire_arm_q(NUM_ARM_JOINT);
        mtx_walk_pose.lock();
        desire_arm_q << walk_arm_pose * TO_RADIAN;
        mtx_walk_pose.unlock();
        if (state_des.sub_phase == walking && swing_arm)
          {
            double amplitude = walk_swing_arm_degree * TO_RADIAN; // 振幅
            double angle_p = amplitude * std::sin(M_PI / StepDuration * state_des.phase_time);
            if (state_des.walk_contact == L_Contact)
              {
                angle_p = -angle_p;
              }
            desire_arm_q[0] += angle_p;
            desire_arm_q[NUM_ARM_JOINT / 2] -= angle_p;
            // Eigen::VectorXd desire_arm_v(NUM_ARM_JOINT);
            // desire_arm_v = (desire_arm_q - state_des.arm_q) / dt_;
            // state_des.arm_v = desire_arm_v;
            // state_des.arm_q = desire_arm_q;
            if (step_num < 1)
              {
                arm_interpolator->update(state_des.arm_q, state_des.arm_v, desire_arm_q, 1);
              }
            else
              {
                arm_interpolator->update(state_des.arm_q, state_des.arm_v, desire_arm_q, 500);
              }
            Eigen::VectorXd pos(desire_arm_q.size()), vel(desire_arm_q.size()), acc(desire_arm_q.size());

            arm_interpolator->get(&pos, &vel, &acc);

            state_des.arm_v = vel;
            state_des.arm_q = pos;
          }
        else
          {
            // state_des.arm_v = (desire_arm_q - state_des.arm_q) * 3;
            // state_des.arm_q += state_des.arm_v * dt_;
            if (state_des.phase_time <= FLAGS_dt)
              arm_interpolator->update(state_des.arm_q, state_des.arm_v, desire_arm_q, 5);

            Eigen::VectorXd pos(state_des.arm_q.size()), vel(state_des.arm_q.size()), acc(state_des.arm_q.size());
            arm_interpolator->get(&pos, &vel, &acc);
            state_des.arm_v = vel;
            state_des.arm_q = pos;
            state_des.arm_vd = acc;
          }
        // state_des.arm_v  = (desire_arm_q - state_des.arm_q) * 10;
        // state_des.arm_q += state_des.arm_v * dt_;
      }
    else
      {
        if (state_des.phase == P_stand)
          {
            Eigen::VectorXd desire_arm_q(NUM_ARM_JOINT);
            bool is_update = false;
            mtx_pose.lock();
            desire_arm_q << current_arm_pose_degree;
            is_update = is_arm_pose_updated;
            is_arm_pose_updated = false;
            mtx_pose.unlock();
            desire_arm_q *= TO_RADIAN;

            if (ifOpenRosArm)
              {
                state_des.arm_v = (desire_arm_q - state_des.arm_q) * 3;
                state_des.arm_q += state_des.arm_v * dt_;
              }
            else
              {
                if (multi_arm_poses.size() > 0)
                  {
                    auto multi_desire_arm_q = multi_arm_poses;
                    for (auto &aq : multi_desire_arm_q)
                      aq *= TO_RADIAN;
                    mtx_pose.lock();
                    multi_arm_poses.clear();
                    mtx_pose.unlock();
                    arm_interpolator->update(state_des.arm_q, state_des.arm_v, multi_desire_arm_q, 2);

                    std::cout << "set multi arm pose\n";
                  }
                else if (is_update || state_des.phase_time <= FLAGS_dt)
                  arm_interpolator->update(state_des.arm_q, state_des.arm_v, desire_arm_q, 2);

                Eigen::VectorXd pos(state_des.arm_q.size()), vel(state_des.arm_q.size()), acc(state_des.arm_q.size());
                arm_interpolator->get(&pos, &vel, &acc);
                state_des.arm_v = vel;
                state_des.arm_q = pos;
                state_des.arm_vd = acc;
              }
          }
        else
          {
            Eigen::VectorXd desire_arm_q(NUM_ARM_JOINT);
            desire_arm_q.setZero();
            desire_arm_q *= TO_RADIAN;
            if (state_des.phase_time <= FLAGS_dt)
              arm_interpolator->update(state_des.arm_q, state_des.arm_v, desire_arm_q, 5);
            Eigen::VectorXd pos(state_des.arm_q.size()), vel(state_des.arm_q.size()), acc(state_des.arm_q.size());
            arm_interpolator->get(&pos, &vel, &acc);
            state_des.arm_v = vel;
            state_des.arm_q = pos;
            state_des.arm_vd = acc;
          }
      }
  }
}
