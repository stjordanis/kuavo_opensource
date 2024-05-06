# 基础环境准备

1. **安装 ROS1 的 noetic 版本** - 这个版本在机器人的环境上应该已经是默认安装的。请确认 `/opt/ros/noetic/setup.bash` 文件是否存在。
2. **新建 ROS1 的 workspace 目录** - 例如 `~/catkin_ws`，然后在 `~/catkin_ws/src` 下克隆 kuavo 的仓库。kuavo 的仓库地址为: `https://gitee.com/leju-robot/kuavo_opensource.git`。克隆的命令为: 
   ```
   cd ~/catkin_ws && mkdir -p src && git clone https://gitee.com/leju-robot/kuavo_opensource.git kuavo
   ```
3. **编译工程** - 切换到 `~/catkin_ws`，加载 ROS1 的环境 `source ~/catkin_ws/devel/setup.bash`，使用 `catkin_make` 来编译工程。如果编译出错，请检查对应的错误信息。
4. **启动机器人的主节点** - 如果编译成功，使用 `source ~/catkin_ws/devel/setup.bash` 加载对应的环境。然后使用 `roslaunch dynamic_biped highly_dynamic_robot.launch` 来启动机器人的主节点。

## 依赖的库

1. geometry_msgs

  1.1 执行命令 `rospack find geometry_msgs`。如果没有返回结果，说明缺少该库，请进行安装。

  1.2 安装命令如下：

  sudo apt-get update
  sudo apt-get install ros-noetic-geometry-msgs


# Topics - 机器人话题及数据类型定义如下

## robot_q_v_tau
数据格式
* robotQVTau.msg
```
float64[] q
float64[] v
float64[] tau
```

说明：除了 q 的前 4 个值代表躯干的四元数位置外，v 和 tau 分别对应机器人 URDF 文件中的关键位置的速度和力矩。

## robot_torso_state

数据格式
* robotTorsoState.msg
```
geometry_msgs/Vector3 torsoR
geometry_msgs/Vector3 torsoRd
geometry_msgs/Vector3 torsoRdd
geometry_msgs/Vector3 r
geometry_msgs/Vector3 rd
geometry_msgs/Vector3 rdd
```

- 说明：torsoR 躯干旋转角度，torsoRd 躯干旋转速度，torsoRdd 躯干旋转加速度。
- 说明：r 质心旋转角度， rd 质心旋转速度， rdd 质心旋转加速度

## robot_arm_q_v_tau

说明：当前手臂每个关节的位置，速度，加速度。以下是关节和消息之间的位置关系。

左手 = [ 1,  2,  3,  4,  5,  6,  7]
右手 = [ 8,  9, 10, 11, 12, 13, 14]



## kuavo_arm_traj

数据格式： https://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html

```
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
```

说明：控制每个手臂的关节，可以控制手臂关节的位置，速度和力矩。请注意：目前我们只使用了位置信息，所以在 topic 里面的速度和力矩参数会被忽略掉。后续如果有支持速度和力矩我们会同步更新文档。

补充说明：kuavo_arm_traj 里面消息和关节的对应关系。

左手 = [ 1,  2,  3,  4,  5,  6,  7]
右手 = [ 8,  9, 10, 11, 12, 13, 14]

## walkCommand

数据格式
* walkCommand.msg
```
# mode: 0->PositionCommand | 1->VelocityCommand | 2->stepCommand
uint8 mode
float64[] values
```

说明：
- mode 控制模式，0 为位置控制，1 为速度控制，2 为步态控制。目前建议使用速度控制。

  - 在速度控制里面 values 一共 3 个值，分别为 x,y 轴的行进速度，以及 yaw 角的旋转速度。如果想让机器人停下来，可以直接把机器人的主模式切换到 P_stand，或者将三个速度设置为 0
  - 在位置控制模式下, 发送目标位置指令
  - 在单步控制模式下, 发送的 values 是四维的数组, 分别为 [num_step,x,y,yaw] , 机器人会自动进入走路状态, 以 x,y,yaw 的速度指令走 num_step 步, 走完之前没有下发递增的num_step, 将会停下来.

## leju_robot_phase
数据格式
* robotPhase.msg
```
uint8 mainPhase
uint8 subPhase
```

- 说明：mainPhase 机器人主状态， subPhase 机器人子状态
- 具体机器人状态序号说明可参考下面服务/setPhase的状态图

---
# Services - 机器人服务及类型定义如下

## setPhase

数据格式
* srvChangePhases.srv
```
uint8 masterID
string stateReq
string subState
---
int16 stateRes

```

说明：以上 masterID 可以固定为 0，steteReq 是主状态，subState 是子状态

主状态及子状态的信息 

```
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

```


## change_arm_ctrl_mode

数据格式
* changeArmCtrlMode.srv
```
bool control_mode
---
bool result
```

说明：在发送手臂数据给手臂之前需要使能手臂的控制模式，设置 control_mode 为 true，反之为 false. 请注意，请勿在机器人移动的时候使能手臂控制模式，目前机器人的反馈控制还不能处理行走过程中手臂的不规则运动，会影响平衡。所以请在机器人站立稳定的时候才使能手臂控制模式。

## change_joller_position

控制夹爪的位置

数据格式
* srvChangeJoller.srv
```
int32 l_pos
int32 r_pos
---
bool result
```
- l_pos 为左手机械爪的开合位置，有效数据范围为[0, 255]
- r_pos 为右手机械爪的开合位置，有效数据范围为[0, 255]
- 若pos的输入大于255或者小于0，此时机械爪/change_joller_position服务会提示超有效范围

---
## > Assistant: 内容检查和简化后如下：

# 基础环境准备

1. **安装 ROS1 的 noetic 版本** - 确认 `/opt/ros/noetic/setup.bash` 文件是否存在，以验证是否已安装。
2. **新建 ROS1 的 workspace** - 在 `~/catkin_ws/src` 下克隆 kuavo 仓库：
   ```
   cd ~/catkin_ws && mkdir -p src && git clone https://gitee.com/leju-robot/kuavo_opensource.git kuavo
   ```
3. **编译工程** - 在 `~/catkin_ws`，执行 `source ~/catkin_ws/devel/setup.bash` 并使用 `catkin_make` 编译。编译错误时请参考错误信息。
4. **启动机器人主节点** - 成功编译后，运行 `source ~/catkin_ws/devel/setup.bash` 并使用 `roslaunch dynamic_biped highly_dynamic_robot.launch` 启动。

## 依赖的库

1. geometry_msgs

  1.1 检查库是否安装：`rospack find geometry_msgs`。
  
  1.2 安装库：
  ```
  sudo apt-get update
  sudo apt-get install ros-noetic-geometry-msgs
  ```

# Topics

## robot_q_v_tau

- **数据格式**
  ```
  float64[] q
  float64[] v
  float64[] tau
  ```
- **说明**：`q` 的前 4 个值为躯干四元数位置，`v` 和 `tau` 分别表示速度和力矩。

## robot_torso_state

- **数据格式**
  ```
  geometry_msgs/Vector3 torsoR
  geometry_msgs/Vector3 torsoRd
  geometry_msgs/Vector3 torsoRdd
  geometry_msgs/Vector3 r
  geometry_msgs/Vector3 rd
  geometry_msgs/Vector3 rdd
  ```
- **说明**：torsoR 躯干旋转角度，torsoRd 躯干旋转速度，torsoRdd 躯干旋转加速度。
- **说明**：r 质心旋转角度， rd 质心旋转速度， rdd 质心旋转加速度

## kuavo_arm_traj

- **数据格式**：参考 [sensor_msgs/JointState](https://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html)。
- **说明**：控制手臂关节位置。目前仅使用位置信息，速度和力矩被忽略。

## walkCommand

- **数据格式**
  ```
  # mode: 0->PositionCommand | 1->VelocityCommand
  uint8 mode
  float64[] values
  ```
- **说明**：建议使用速度控制。`values` 包含 x,y 轴速度和 yaw 角速度。

## leju_robot_phase
数据格式
* robotPhase.msg
```
uint8 mainPhase
uint8 subPhase
```

- **说明**：mainPhase 机器人主状态， subPhase 机器人子状态

# Services

## setPhase

- **数据格式**
  ```
  uint8 masterID
  string stateReq
  string subState
  ---
  int16 stateRes
  ```
- **说明**：`masterID` 通常为 0，`stateReq` 和 `subState` 分别为主状态和子状态。

## change_arm_ctrl_mode

- **数据格式**
  ```
  bool control_mode
  ---
  bool result
  ```
- **说明**：在机器人稳定站立时，通过设置 `control_mode` 为 true 来使能手臂控制模式。

## change_joller_position
数据格式
* srvChangeJoller.srv
```
int32 l_pos
int32 r_pos
---
bool result
```
- **说明** l_pos 为左手机械爪的开合位置，有效数据范围为[0, 255] . r_pos 为右手机械爪的开合位置，有效数据范围为[0, 255]
- **说明** 若pos位置的输入大于255或者小于0，此时机械爪/change_joller_position服务会提示超有效范围
