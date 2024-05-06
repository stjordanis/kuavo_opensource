#include <HighlyDynamicRobot.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

#include <dynamic_biped/walkCommand.h>
#include <dynamic_biped/srvChangePhases.h>
#include <dynamic_biped/srvClearPositionCMD.h>
#include <dynamic_biped/srvchangeCtlMode.h>
#include <dynamic_biped/changeArmCtrlMode.h>
#include <dynamic_biped/changeAMBACCtrlMode.h>
#include <dynamic_biped/controlEndHand.h>
#include <dynamic_biped/srvArmIK.h>

#include <sensor_msgs/JointState.h>
#include <dynamic_biped/robotQVTau.h>
#include <dynamic_biped/robotTorsoState.h>
#include <dynamic_biped/robotPhase.h>
#include <dynamic_biped/robotArmQVVD.h>
#include <dynamic_biped/robotHandPosition.h>
#include <dynamic_biped/robotHeadMotionData.h>
#include <dynamic_biped/armTargetPoses.h>
#include <dynamic_biped/robotArmPose.h>

#define TO_DEGREE (180.0 / M_PI)

HighlyDynamic::HighlyDynamicRobot *robot_ptr;
RobotState_t HDrobotState;
std::map<std::string, mainPhase_t> phase_map = {
    {"P_stand", P_stand},
    {"P_walk", P_walk},
    {"P_jump", P_jump},
    {"P_squat", P_squat},
    {"P_ERROR", P_ERROR},
    {"P_None", P_None}};
std::map<std::string, subPhase_t> sub_phase_map = {

    // walk
    {"walk_pre", walk_pre},
    {"walking", walking},
    {"walk_stop", walk_stop},

    // jump
    {"jump_pre", jump_pre},
    {"jump_take_off", jump_take_off},
    {"jump_flight", jump_flight},
    {"jump_touch_down", jump_touch_down},
    {"jump_to_ready", jump_to_ready},

    {"sub_phase_none", sub_phase_none},

};

std::map<int, std::string> sub_phase_name_map = {
    {walk_pre, "walk_pre"},
    {walking, "walking"},
    {walk_stop, "walk_stop"},
    {jump_pre, "jump_pre"},
    {jump_take_off, "jump_take_off"},
    {jump_flight, "jump_flight"},
    {jump_touch_down, "jump_touch_down"},
    {jump_to_ready, "jump_to_ready"},
    {squat_normal, "squat_normal"},
    {squat_quick, "squat_quick"},
    {sub_phase_none, "sub_phase_none"}};

std::map<int, std::string> main_phase_name_map = {
    {P_stand, "P_stand"},
    {P_walk, "P_walk"},
    {P_jump, "P_jump"},
    {P_squat, "P_squat"},
    {P_ERROR, "P_ERROR"},
    {P_None, "P_None"}};

static void updateState()
{
    robot_ptr->getRobotState(HDrobotState);
}

class HDrobot_node
{
public:
    ros::Publisher robot_q_v_tau_pub;
    ros::Publisher robot_arm_q_v_vd_pub;
    ros::Publisher robot_torso_state_pub;
    ros::Publisher robot_phase_pub;
    ros::Publisher robot_hand_position_pub;
    ros::Publisher robot_arm_pose_pub;

    HDrobot_node(ros::NodeHandle &nh) : nh_(nh)
    {
        CMD_sub = nh_.subscribe("/walkCommand", 10, &HDrobot_node::walkCommandCallback);

        // Publish timestamp topic at 100hz
        // robot_q_v_tau_pub = nh_.advertise<ros::Time>("/robot_q_v_tau", 100);
        robot_q_v_tau_pub = nh_.advertise<dynamic_biped::robotQVTau>("/robot_q_v_tau", 10);

        robot_arm_q_v_vd_pub = nh_.advertise<dynamic_biped::robotArmQVVD>("/robot_arm_q_v_tau", 10);
        // robot_q_v_tau_pub = nh_.advertise<std_msgs::Time>("/robot_q_v_tau", 100);

        robot_torso_state_pub = nh_.advertise<dynamic_biped::robotTorsoState>("robot_torso_state", 10);

        robot_phase_pub = nh_.advertise<dynamic_biped::robotPhase>("leju_robot_phase", 10);

        robot_hand_position_pub = nh_.advertise<dynamic_biped::robotHandPosition>("robot_hand_position", 10);

        robot_arm_pose_pub = nh_.advertise<dynamic_biped::robotArmPose>("robot_arm_pose", 10);
        // Subscribe to /etherCATJoint/motordata topic

        // Subscribe to /kuavo_arm_traj topic
        joint2_command_desired_sub = nh_.subscribe("/kuavo_arm_traj", 10, &HDrobot_node::joint2CommandDesiredCallback);
        arm_target_poses_sub = nh_.subscribe("/kuavo_arm_target_poses", 10, &HDrobot_node::armTargetPoseCallback);
        robot_head_motion_data_sub = nh_.subscribe("/robot_head_motion_data", 10, &HDrobot_node::robotHeadMotionDataCallback);
        // Create services and bind callback functions
        static ros::ServiceServer change_phases_service = nh.advertiseService("setPhase", changePhasesCallback);
        static ros::ServiceServer clear_position_cmd_service = nh.advertiseService("clear_position_cmd", clearPositionCMDCallback);
        static ros::ServiceServer change_ctl_mode_service = nh.advertiseService("change_ctl_mode", changeCtlModeCallback);
        static ros::ServiceServer change_arm_ctrl_mode_service = nh.advertiseService("change_arm_ctrl_mode", changeArmCtlModeCallback);
        static ros::ServiceServer change_AMBAC_ctrl_mode_service = nh.advertiseService("change_AMBAC_ctrl_mode", changeAMBACCtlModeCallback);
        static ros::ServiceServer control_end_hand_service = nh.advertiseService("control_end_hand", controlEndHandCallback);
        static ros::ServiceServer arm_IK_service = nh.advertiseService("arm_IK", armIKCallback);
    }   
    //
    static bool changeArmCtlModeCallback(dynamic_biped::changeArmCtrlMode::Request &req,
                                         dynamic_biped::changeArmCtrlMode::Response &res)
    {
        bool control_mode = req.control_mode;
        robot_ptr->switchArmCtrlMode(control_mode);
        res.result = true;
        return true;
    }

    // AMBAC系统（Active Mass Balance Auto-Control）：用于机动战士在太空中的移动和平衡控制。
    static bool changeAMBACCtlModeCallback(dynamic_biped::changeAMBACCtrlMode::Request &req,
                                           dynamic_biped::changeAMBACCtrlMode::Response &res)
    {
        bool control_mode = req.control_mode;
        robot_ptr->setAMBACReady(control_mode);
        res.result = true;
        return true;
    }

    static bool changeCtlModeCallback(dynamic_biped::srvchangeCtlMode::Request &req,
                                      dynamic_biped::srvchangeCtlMode::Response &res)
    {
        // TODO masterID
        robot_ptr->changeCtlMode((controlMode_t)req.control_mode);
        return true;
    }

    static bool clearPositionCMDCallback(dynamic_biped::srvClearPositionCMD::Request &req,
                                         dynamic_biped::srvClearPositionCMD::Response &res)
    {
        robot_ptr->clearPositionCMD();
        return true;
    }

    static bool changePhasesCallback(dynamic_biped::srvChangePhases::Request &req,
                                     dynamic_biped::srvChangePhases::Response &res)
    {
        // TODO: SET masterID
        std::string newphase_str = (!req.stateReq.empty() && phase_map.count(req.stateReq)) ? req.stateReq : "";
        std::string subState_str = (!req.subState.empty() && sub_phase_map.count(req.subState)) ? req.subState : "";

        if (!subState_str.empty() && !newphase_str.empty())
        {
            robot_ptr->changePhases(phase_map[newphase_str], sub_phase_map[subState_str]);
            return true;
        }
        else if (!subState_str.empty())
        {
            robot_ptr->changePhases(P_None, sub_phase_map[subState_str]);
            return true;
        }
        else if (!newphase_str.empty())
        {
            robot_ptr->changePhases(phase_map[newphase_str]);
            return true;
        }
        else
        {
            std::string warn_str = "UNKNOWN phase! req:" + newphase_str + ",subreq:" + subState_str;
            ROS_WARN("%s", warn_str.c_str());
            return false;
        }
    }

    static void walkCommandCallback(const dynamic_biped::walkCommand::ConstPtr &msg)
    {
        updateState();
        if (HDrobotState.phase != P_walk && msg->mode != 2)
        {
            ROS_WARN("NOT IN P_walk STATUS, ignore P_walk Command");
            return;
        }
        int controlmode = (int)robot_ptr->getCtlMode();
        if (controlmode != msg->mode)
        {
            ROS_WARN("Control mode does not match, msg.mode=%d | controll mode=%d", msg->mode, controlmode);
            return;
        }
        if (msg->mode == 0) // 0: position control
        {
            ROS_INFO("Received position control command: [%f, %f, %f]",
                     msg->values[0], msg->values[1], msg->values[2]);
            robot_ptr->positionCommand({msg->values[0], msg->values[1], msg->values[2]});
        }
        else if (msg->mode == 1) // 1: velocity control
        {
            ROS_INFO("Received velocity control command: [%f, %f, %f]",
                     msg->values[0], msg->values[1], msg->values[2]);
            robot_ptr->velocityCommand({msg->values[0], msg->values[1], msg->values[2]});
        }
        else if (msg->mode == 2) // 2: torque control
        {
            if (msg->values.size() != 4)
            {
                ROS_INFO("Received invalid step control command!");
                return;
            }
            else
            {
                ROS_INFO("Received step control command: [%f, %f, %f, %f]",
                         msg->values[0], msg->values[1], msg->values[2], msg->values[3]);
                robot_ptr->stepCommand(msg->values[0], {msg->values[1], msg->values[2], msg->values[3]});
            }
        }
    }

    static void armTargetPoseCallback(const dynamic_biped::armTargetPoses::ConstPtr &msg)
    {
        std::cout << "Received arm target poses" << std::endl;

        if (msg->values.empty() || msg->times.empty() || msg->values.size() != msg->times.size() * HighlyDynamic::NUM_ARM_JOINT)
        {
            ROS_WARN("Invalid armTargetPoses data. Empty values or different sizes.");
            return;
        }

        std::vector<double> times;
        std::vector<Eigen::VectorXd> target_poses;
        for (int i = 0; i < msg->times.size(); i++)
        {
            Eigen::VectorXd pose(HighlyDynamic::NUM_ARM_JOINT);
            for (int j = 0; j < HighlyDynamic::NUM_ARM_JOINT; j++)
            {
                pose[j] = msg->values[i * HighlyDynamic::NUM_ARM_JOINT + j];
            }
            times.push_back(msg->times[i]);
            target_poses.push_back(pose);
        }
        robot_ptr->changeArmPoses(times, target_poses);
        return;
    }

    // 回调函数处理接收到的 /etherCATJoint/motordata 消息
    static void joint2CommandDesiredCallback(const sensor_msgs::JointState::ConstPtr &msg)
    {
        // 处理收到的消息
        // ROS_INFO("Received joint_command_desired: [%f, %f, %f, %f, %f, %f,...]", msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5]);

        // 长度为 6
        // 检查 msg 的维度是否符合要求
        if (msg->position.size() == 14)
        {
            // 将 std::vector<double> 转换为 Eigen::VectorXd
            Eigen::VectorXd targetRosPosition(14);

            for (int i = 0; i < msg->position.size(); i++)
            {
                targetRosPosition[i] = msg->position[i];
            }

            // // 左手
            // targetRosPosition[0] = msg->position[0] ;
            // targetRosPosition[1] = msg->position[1] ;   // moveit电机输出轴方向和机器人Kuavo输出轴方向相反
            // targetRosPosition[2] = msg->position[2] ;   // moveit电机输出轴方向和机器人Kuavo输出轴方向相反

            // // 右手
            // targetRosPosition[3] = msg->position[3] ;
            // targetRosPosition[4] = msg->position[4] ;
            // targetRosPosition[5] = msg->position[5] ;

            // 调用 setROSArmPose
            robot_ptr->setROSArmPose(targetRosPosition);
        }
        else
        {
            ROS_WARN("Invalid arm_command_desired data. Expected 6 elements, but received %lu elements.", msg->position.size());
        }
    }

    static void robotHeadMotionDataCallback(const dynamic_biped::robotHeadMotionData::ConstPtr &msg)
    {
        // Check if the message has the correct number of elements
        if (msg->joint_data.size() == 2)
        {
            if (msg->joint_data[0] < -30 || msg->joint_data[0] > 30 || msg->joint_data[1] < -25 || msg->joint_data[1] > 25)
            {
                ROS_WARN("Invalid robot head motion data. Joint data must be in the range [-30, 30] and [-25, 25].");
                return;
            }
            ROS_INFO("Received robot head motion data joint_data: [%f, %f]", msg->joint_data[0], msg->joint_data[1]);

            robot_ptr->setHeadJointData(msg->joint_data);
        }
        else
        {
            ROS_WARN("Invalid robot head motion data. Expected 2 elements, but received %lu elements.", msg->joint_data.size());
        }
    }

    static bool controlEndHandCallback(dynamic_biped::controlEndHand::Request &req,
                                       dynamic_biped::controlEndHand::Response &res)
    {
        if (req.left_hand_position.size() != 6 || req.right_hand_position.size() != 6)
        {
            ROS_ERROR("Invalid hand positions size. Both left and right hand positions must have size 6.");
            res.result = false;
            return false;
        }

        auto isInRange = [](const std::vector<uint8_t> &positions)
        {
            return std::all_of(positions.begin(), positions.end(), [](uint8_t pos)
                               { return pos >= 0 && pos <= 100; });
        };

        if (!isInRange(req.left_hand_position) || !isInRange(req.right_hand_position))
        {
            ROS_ERROR("Invalid hand positions value. All positions value must be in the range [0, 100].");
            res.result = false;
            return false;
        }

        Eigen::VectorXd left_right_pos(12);
        for (int i = 0; i < 6; i++)
        {
            left_right_pos[i] = req.left_hand_position[i];
            left_right_pos[i + 6] = req.right_hand_position[i];
        }

        robot_ptr->setEndhand(left_right_pos);

        res.result = true;
        return true;
    }

    static bool armIKCallback(dynamic_biped::srvArmIK::Request &req,
                              dynamic_biped::srvArmIK::Response &res)
    {
        if (req.left_arm_pose.size() != 7 || req.right_arm_pose.size() != 7)
        {
            ROS_ERROR("Invalid arm pose size. Both left and right arm pose must have size 7.");
            return false;
        }

        int joint_state_size = 14;
        int single_arm_pose_size = 7;
        Eigen::VectorXd arm_xyzrpy(single_arm_pose_size * 2);
        Eigen::VectorXd arm_joint_state(joint_state_size);
        for (int i = 0; i < single_arm_pose_size; i++)
        {
            arm_xyzrpy[i] = req.left_arm_pose[i];
            arm_xyzrpy[i + single_arm_pose_size] = req.right_arm_pose[i];
        }

        robot_ptr->armCoMIK(arm_xyzrpy, arm_joint_state);
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.position.resize(joint_state_size);
        for (int i = 0; i < joint_state_size; i++)
        {
            joint_state.position[i] = arm_joint_state[i] * TO_DEGREE;
        }
        res.joint_state = joint_state;
        return true;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber CMD_sub;
    ros::Subscriber joint2_command_desired_sub;
    ros::Subscriber arm_target_poses_sub;
    ros::Subscriber robot_head_motion_data_sub;
};

void resizeAndCopyVector(const Eigen::VectorXd &source, std::vector<double> &destination)
{
    destination.resize(source.size());
    for (int i = 0; i < source.size(); i++)
    {
        destination[i] = source[i];
    }
}

std::string getPhaseName(const int phase, const std::map<int, std::string> &phase_name_map)
{
    std::string est_main_phase_name = "Unknown";
    auto phase_name = phase_name_map.find(phase);
    if (phase_name != phase_name_map.end())
    {
        est_main_phase_name = phase_name->second;
    }
    return est_main_phase_name;
}

void ros_publish_robot_arm_q_v_vd(const RobotState_t &state_est, const ros::Publisher &robot_ros_publisher)
{
    dynamic_biped::robotArmQVVD msg;
    resizeAndCopyVector(state_est.arm_q, msg.q);
    resizeAndCopyVector(state_est.arm_v, msg.v);
    resizeAndCopyVector(state_est.arm_vd, msg.vd);

    robot_ros_publisher.publish(msg);
}

void ros_publish_robot_q_v_tau(const RobotState_t &state_est, const ros::Publisher &robot_ros_publisher)
{
    dynamic_biped::robotQVTau msg;

    resizeAndCopyVector(state_est.q, msg.q);
    resizeAndCopyVector(state_est.v, msg.v);
    resizeAndCopyVector(state_est.tau, msg.tau);

    robot_ros_publisher.publish(msg);
}

void publishRobotTorsoState(const RobotState_t &state_est, const ros::Publisher &publisher)
{
    dynamic_biped::robotTorsoState robot_torsor_state_msg;

    robot_torsor_state_msg.torsoR.x = state_est.torsoR.x();
    robot_torsor_state_msg.torsoR.y = state_est.torsoR.y();
    robot_torsor_state_msg.torsoR.z = state_est.torsoR.z();

    robot_torsor_state_msg.torsoRd.x = state_est.torsoRd.x();
    robot_torsor_state_msg.torsoRd.y = state_est.torsoRd.y();
    robot_torsor_state_msg.torsoRd.z = state_est.torsoRd.z();

    robot_torsor_state_msg.torsoRdd.x = state_est.torsoRdd.x();
    robot_torsor_state_msg.torsoRdd.y = state_est.torsoRdd.y();
    robot_torsor_state_msg.torsoRdd.z = state_est.torsoRdd.z();

    robot_torsor_state_msg.r.x = state_est.r.x();
    robot_torsor_state_msg.r.y = state_est.r.y();
    robot_torsor_state_msg.r.z = state_est.r.z();

    robot_torsor_state_msg.rd.x = state_est.rd.x();
    robot_torsor_state_msg.rd.y = state_est.rd.y();
    robot_torsor_state_msg.rd.z = state_est.rd.z();

    robot_torsor_state_msg.rdd.x = state_est.rdd.x();
    robot_torsor_state_msg.rdd.y = state_est.rdd.y();
    robot_torsor_state_msg.rdd.z = state_est.rdd.z();

    publisher.publish(robot_torsor_state_msg);
}

void publishRobotPhase(const RobotState_t &state_des, ros::Publisher &robot_phase_pub)
{
    dynamic_biped::robotPhase robot_phase_msg;
    robot_phase_msg.mainPhase = state_des.phase;
    robot_phase_msg.subPhase = state_des.sub_phase;

    robot_phase_pub.publish(robot_phase_msg);
}

void publishRobotHandPosition(ros::Publisher &robot_hand_position_pub)
{
    dynamic_biped::robotHandPosition robot_hand_position_msg;
    robot_hand_position_msg.left_hand_position.resize(6);
    robot_hand_position_msg.right_hand_position.resize(6);

    auto left_right_pos = robot_ptr->getEndhand();

    for (int i = 0; i < 6; i++)
    {
        robot_hand_position_msg.left_hand_position[i] = left_right_pos[i];
        robot_hand_position_msg.right_hand_position[i] = left_right_pos[i + 6];
    }

    robot_hand_position_pub.publish(robot_hand_position_msg);
}

void publishRobotArmPose(ros::Publisher &robot_arm_pose_pub)
{
    dynamic_biped::robotArmPose robot_arm_pose_msg;
    robot_arm_pose_msg.left_arm_pose.resize(7);
    robot_arm_pose_msg.right_arm_pose.resize(7);

    auto arm_xyzabc = robot_ptr->armFK();
    for (int i = 0; i < 7; i++)
    {
        robot_arm_pose_msg.left_arm_pose[i] = arm_xyzabc[i];
        robot_arm_pose_msg.right_arm_pose[i] = arm_xyzabc[i + 7];
    }

    robot_arm_pose_pub.publish(robot_arm_pose_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "HDrobot_node");
    ros::NodeHandle nh;
    HighlyDynamic::HighlyDynamicRobot robot;
    robot_ptr = &robot;
    // robot.traj_ptr->setRosArmTrue();
    // robot.listening_keyboard = false;
    HDrobot_node node(nh);
    int ret = 0;
    robot.doMainAsync(argc, argv);
    // robot.switchArmCtrlMode(true);
    if (ret != 0)
    {
        std::cout << "robot start failed!\n";
        return -1;
    }
    ros::Rate rate(100);

    uint64_t low_rate_count = 0;
    while (ros::ok())
    {
        if (low_rate_count++ % 100 == 0)
        {
            RobotData robotData = robot.queryNewestRobotStates();
            RobotState_t state_des = robotData.state_des;
            RobotState_t state_est = robotData.state_est;

            ros_publish_robot_q_v_tau(state_est, node.robot_q_v_tau_pub);

            ros_publish_robot_arm_q_v_vd(state_est, node.robot_arm_q_v_vd_pub);

            publishRobotTorsoState(state_est, node.robot_torso_state_pub);

            publishRobotPhase(state_des, node.robot_phase_pub);

            publishRobotHandPosition(node.robot_hand_position_pub);

            publishRobotArmPose(node.robot_arm_pose_pub);
        }

        ros::spinOnce();
        rate.sleep();
    }
}
