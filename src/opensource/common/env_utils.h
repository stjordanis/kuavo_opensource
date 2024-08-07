#ifndef KUAVO_ENV_UTILS_H_
#define KUAVO_ENV_UTILS_H_
#include <stdint.h>
#include <string>

namespace HighlyDynamic {
namespace env_utils {

/// @brief 获取版本号， 
/// @return 比如 4.0
float GetRobotVersion();

/// @brief 获取当前代码仓库的根目录路径
/// @note  注意该值在编译时已经确定，且运行时是不会改变
std::string GetSourceRootPath();

/// @brief 获取配置文件路径
/// @note  请确保在 Environment::init() 之后使用，才能获取到正确的值
/// @return 比如 $HOME/.config/lejuconfig/config/kuavo_v4.0/kuavo.json
std::string GetConfigFilePath();

/// @brief 获取配置文件目录路径
/// @note  请确保在 Environment::init() 之后使用，才能获取到正确的值
/// @return 比如 $HOME/.config/lejuconfig/config/kuavo_v4.0
std::string GetConfigFileParentPath();

/// @brief 获取配置文件根目录路径
/// @note  请确保在 Environment::init() 之后使用，才能获取到正确的值
/// @return $HOME/.config/lejuconfig
std::string GetConfigRootPath();

class Environment {
 public:
    Environment()=default;
    bool Init();

 private:
    bool CheckConfigRootPath();
    bool CheckConfigFilePath();
    bool CheckModelFilePath();
    bool CheckRevoConfigFilePath();
};

} // namespace env_utils
} // namespace HighlyDynamic

#endif