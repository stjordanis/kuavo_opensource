#include <HighlyDynamicRobot.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <dynamic_biped/walkCommand.h>
#include <dynamic_biped/srvChangePhases.h>
#include <dynamic_biped/srvClearPositionCMD.h>
#include <dynamic_biped/srvchangeCtlMode.h>
#include <dynamic_biped/changeArmCtrlMode.h>
#include <dynamic_biped/changeAMBACCtrlMode.h>
#include <dynamic_biped/srvChangeJoller.h>

#include <sensor_msgs/JointState.h>
#include <dynamic_biped/robotQVTau.h>
#include <dynamic_biped/robotTorsoState.h>
#include <dynamic_biped/robotPhase.h>
#include <dynamic_biped/robotArmQVVD.h>

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
        // Subscribe to /etherCATJoint/motordata topic

        // Subscribe to /kuavo_arm_traj topic
        joint2_command_desired_sub = nh_.subscribe("/kuavo_arm_traj", 10, &HDrobot_node::joint2CommandDesiredCallback);
        // Create services and bind callback functions
        static ros::ServiceServer change_phases_service = nh.advertiseService("setPhase", changePhasesCallback);
        static ros::ServiceServer clear_position_cmd_service = nh.advertiseService("clear_position_cmd", clearPositionCMDCallback);
        static ros::ServiceServer change_ctl_mode_service = nh.advertiseService("change_ctl_mode", changeCtlModeCallback);
        static ros::ServiceServer change_arm_ctrl_mode_service = nh.advertiseService("change_arm_ctrl_mode", changeArmCtlModeCallback);
        static ros::ServiceServer change_AMBAC_ctrl_mode_service = nh.advertiseService("change_AMBAC_ctrl_mode", changeAMBACCtlModeCallback);
        static ros::ServiceServer change_Joller_pos_service = nh.advertiseService("change_joller_position", srvChangeJollerCallback);
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
        if (HDrobotState.phase != P_walk)
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
        if (msg->mode == 1) // 1: velocity control
        {
            ROS_INFO("Received velocity control command: [%f, %f, %f]",
                     msg->values[0], msg->values[1], msg->values[2]);
            robot_ptr->velocityCommand({msg->values[0], msg->values[1], msg->values[2]});
        }
        else // 0: position control
        {
            ROS_INFO("Received position control command: [%f, %f, %f]",
                     msg->values[0], msg->values[1], msg->values[2]);
            robot_ptr->positionCommand({msg->values[0], msg->values[1], msg->values[2]});
        }
    }

    // 服务函数
    static bool srvChangeJollerCallback(dynamic_biped::srvChangeJoller::Request &req, dynamic_biped::srvChangeJoller::Response &res)
    {
        // safe check
        if (req.l_pos < 0 || req.l_pos > 255 || req.r_pos < 0 || req.r_pos > 255)
        {
            ROS_ERROR("Invalid positions. Positions must be in the range [0, 255].");
            res.result = false;
            return false;
        }

        Eigen::Vector2d ros_left_right_pos = {req.l_pos, req.r_pos};

        robot_ptr->SetsetEndEffectors(ros_left_right_pos);

        res.result = true;
        return true;
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
            robot_ptr->rosSetMoveitMotr(targetRosPosition);
        }
        else
        {
            ROS_WARN("Invalid arm_command_desired data. Expected 6 elements, but received %lu elements.", msg->position.size());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber CMD_sub;
    ros::Subscriber joint2_command_desired_sub;
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
        }

        ros::spinOnce();
        rate.sleep();
    }
}
