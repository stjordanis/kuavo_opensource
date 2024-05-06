#include "config.h"
#include <cstdint>
namespace HighlyDynamic
{
#if ROBOT_VERSION_INT >= 40
    std::string robot_config_path = "src/biped_v2/config/kuavo_v4.0/";
#elif ROBOT_VERSION_INT >= 34
    std::string robot_config_path = "src/biped_v2/config/kuavo_v3.4/";
#elif ROBOT_VERSION_INT >= 33
    std::string robot_config_path = "src/biped_v2/config/kuavo_v3.3/";
#elif ROBOT_VERSION_INT >= 32
    std::string robot_config_path = "src/biped_v2/config/kuavo_v3.2/";
#elif ROBOT_VERSION_INT >= 30
    std::string robot_config_path = "src/biped_v2/config/kuavo_v3/";
#elif ROBOT_VERSION_INT >= 23
    std::string robot_config_path = "src/biped_v2/config/kuavo_v2.3/";
#elif ROBOT_VERSION_INT >= 20
    std::string robot_config_path = "src/biped_v2/config/kuavo_v2/";
#elif ROBOT_VERSION_INT >= 11
    std::string robot_config_path = "src/biped_v2/config/kuavo_mt-1.1/";
#elif ROBOT_VERSION_INT >= 10
    std::string robot_config_path = "src/biped_v2/config/kuavo_mt-1.0/";
#else

#error "Invalid version selected"
#endif
    std::filesystem::path file_path = __FILE__;
    std::filesystem::path CURRENT_SOURCE_DIR = file_path.parent_path().parent_path().parent_path().parent_path();
    std::string robot_config_file = robot_config_path + "kuavo.json";
    float ROBOT_VERSION = ROBOT_VERSION_INT / 10.0;
    uint16_t nq_f = 7, nv_f = 6;
    uint8_t NUM_ARM_JOINT, NUM_JOINT;

    MotorInfo motor_info;

    struct motor_config
    {
        uint32_t encoder_range;
        double max_current;
        double c2t_coeff;
        MotorDriveType driver;
    };

    JSONConfigReader::JSONConfigReader(const std::string &filename) : filename_(filename)
    {
        std::cout << "CURRENT_SOURCE_DIR: " << CURRENT_SOURCE_DIR << std::endl;
        std::cout << "CONFIG_PATH: " << filename << std::endl;
        std::cout << "ROBOT_VERSION is: " << ROBOT_VERSION << std::endl;

        load(filename);
        configHardware();
    }

    JSONConfigReader::JSONConfigReader()
    {
    }
    void JSONConfigReader::reload()
    {
        load(filename_);
    }
    void JSONConfigReader::load(const std::string &filename)
    {
        std::ifstream file(GetAbsolutePath(filename));
        if (file.is_open())
        {
            file >> data;
        }
        else
        {
            std::cerr << "Failed to open config file: " << filename << std::endl;
        }
    }
    void JSONConfigReader::configHardware()
    {
        NUM_ARM_JOINT = getValue<uint8_t>("NUM_ARM_JOINT");
        NUM_JOINT = getValue<uint8_t>("NUM_JOINT");
        motor_info.resize(NUM_JOINT);
        std::map<std::string, motor_config>
            motor_name_map = {
                {"PA100", {BIT_17_10, PA100_MC, PA100_C2T, EC_MASTER}},
                {"PA81", {BIT_17_10, PA81_MC, PA81_C2T, EC_MASTER}},
                {"PA72", {BIT_17_36, PA72_MC, PA72_C2T, EC_MASTER}},
                {"PA50", {BIT_17_36, PA50_MC, PA50_C2T, EC_MASTER}},
                {"AK10_9", {BIT_17_9, AK10_9_MC, AK10_9_C2T, EC_MASTER}},
                {"CK", {BIT_17_36, CK_MC, CK_C2T, EC_MASTER}},
                {"dynamixel", {BIT_17_36, CK_MC, CK_C2T, DYNAMIXEL}},
                {"realman", {BIT_17_36, CK_MC, CK_C2T, REALMAN}},
                {"ruiwo_elmo", {BIT_17_36, CK_MC, CK_C2T, EC_MASTER}},
                {"ruiwo", {BIT_17_36, CK_MC, CK_C2T, RUIWO}},
                {"PA100_20", {BIT_17_20, PA100_MC, PA100_20_C2T, EC_MASTER}}};
        std::cout << "NUM_JOINT: " << static_cast<int>(NUM_JOINT) << std::endl;
        std::vector<std::string> MOTORS_TYPE = getValue<std::vector<std::string>>("MOTORS_TYPE");
        std::vector<double> min_limits = getValue<std::vector<double>>("min_joint_position_limits");
        std::vector<double> max_limits = getValue<std::vector<double>>("max_joint_position_limits");
        for (uint8_t i = 0; i < NUM_JOINT; i++)
        {
            motor_config motor = motor_name_map[MOTORS_TYPE[i]];
            motor_info.joint_ids[i] = i + 1;
            motor_info.motor_type[i] = MOTORS_TYPE[i];
            motor_info.driver[i] = motor.driver;
            motor_info.encoder_range[i] = motor.encoder_range;
            motor_info.max_current[i] = motor.max_current;
            motor_info.c2t_coeff[i] = motor.c2t_coeff;
            motor_info.min_joint_position_limits[i] = min_limits[i];
            motor_info.max_joint_position_limits[i] = max_limits[i];
        }

        std::vector<std::string> end_effector_type = RobotConfig.getValue<std::vector<std::string>>("EndEffectorType");
        std::map<std::string, EndEffectorType> end_effector_type_map = {{"none", EndEffectorType::none},
                                                                        {"jodell", EndEffectorType::jodell},
                                                                        {"qiangnao", EndEffectorType::qiangnao}};
        for (auto &name : end_effector_type)
        {
            // std::cout << "EndEffectorType: " << name << std::endl;
            motor_info.end_effector_type.push_back(end_effector_type_map[name]);
        }
    }

    Eigen::VectorXd JSONConfigReader::getEigenVector(const std::string &key)
    {
        if (data.contains(key))
        {
            std::vector<double> std_vector = data[key].get<std::vector<double>>();
            Eigen::Map<Eigen::VectorXd> eigen_vector(std_vector.data(), std_vector.size());
            return eigen_vector;
        }
        else
        {
            std::cerr << "Key not found: " << key << std::endl;
            return {};
        }
    }

    JSONConfigReader RobotConfig(robot_config_file);
    std::vector<std::string> end_frames_name = RobotConfig.getValue<std::vector<std::string>>("end_frames_name");
    std::vector<std::string> contact_frames_name = RobotConfig.getValue<std::vector<std::string>>("contact_frames_name");
}
std::string GetAbsolutePath(const std::string &path)
{
    static std::string ROOT_DIR = HighlyDynamic::CURRENT_SOURCE_DIR;
    // if (ROOT_DIR.empty())
    // {
    //     ROOT_DIR = FindRootDir();
    // }
    std::string abs_path = path;
    std::string project_root = ROOT_DIR;
    if (path[0] != '/')
    {
        // 拼接配置文件相对路径
        std::filesystem::path relative_path = path;

        // 生成配置文件绝对路径
        abs_path = (project_root + "/" + relative_path.string());
    }
    return abs_path;
}
