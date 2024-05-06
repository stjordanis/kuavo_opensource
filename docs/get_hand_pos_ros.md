# ROS节点获取手掌位置信息

### 编译
* 创建ROS工作空间
```bash
mkdir -p ～/catkin_ws/src
cd ～/catkin_ws
```

* 克隆相关仓库到`src`目录下
```bash
cd src/
git clone https://gitee.com/leju-robot/kuavo_opensource.git 
```

* 编译
```bash
cd ~/catkin_ws
catkin_make
```

### 运行
* 启动ROS master
```bash
source /opt/ros/noetic/setup.bash
roscore
```

* 启动机器人ROS节点
```bash
sudo su
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun dynamic_biped highlyDynamicRobot_node --real
```

* 检查话题是否存在
```bash
rosnode info /HDrobot_node 

Output:
--------------------------------------------------------------------------------
Node [/HDrobot_node]
Publications: 
 * /leju_robot_phase [dynamic_biped/robotPhase]
 * /robot_arm_q_v_tau [dynamic_biped/robotArmQVVD]
 * /robot_hand_position [dynamic_biped/robotHandPosition]
 * /robot_q_v_tau [dynamic_biped/robotQVTau]
 * /robot_torso_state [dynamic_biped/robotTorsoState]
 * /rosout [rosgraph_msgs/Log]
```

如果存在`/robot_hand_position`话题，则说明机器人节点已经启动成功。
```bash
rostopic info /robot_hand_position

Output:
Type: dynamic_biped/robotHandPosition

Publishers: 
 * /HDrobot_node (http://rongman-ubuntu:46459/)

Subscribers: None
```

* 运行示例，能够收到机器人手掌位置信息
```bash
source devel/setup.bash
python3 src/kuavo/ros_package/src/get_end_hand_demo.py 

Output:
[INFO] [1712989416.423139]: left hand position: [80, 0, 0, 0, 0, 0]
[INFO] [1712989416.424122]: right hand position: [100, 0, 0, 0, 0, 0]
[INFO] [1712989417.423166]: left hand position: [80, 0, 0, 0, 0, 0]
[INFO] [1712989417.424141]: right hand position: [100, 0, 0, 0, 0, 0]
[INFO] [1712989418.423142]: left hand position: [80, 0, 0, 0, 0, 0]
[INFO] [1712989418.424029]: right hand position: [100, 0, 0, 0, 0, 0]
```

* 使用ROS命令行工具
```bash
rostopic echo /robot_hand_position

Output:
left_hand_position: [0, 0, 0, 0, 0, 0]
right_hand_position: [0, 0, 0, 0, 0, 0]
---
left_hand_position: [0, 0, 0, 0, 0, 0]
right_hand_position: [0, 0, 0, 0, 0, 0]
```

### ROS话题详情
* 话题名称：`/robot_hand_position`
* MSG类型：`dynamic_biped/robotHandPosition`
* MSG请求：`uint8[] left_hand_position, uint8[] right_hand_position`
* MSG说明： 
    * `left_hand_position`：左手掌位置，数组长度为6，对应拇指外展肌，大拇指关节，食指关节, 中指关节，无名指关节，小指关节
    * `right_hand_position`：右手掌位置，数组长度为6，对应拇指外展肌，大拇指关节，食指关节, 中指关节，无名指关节，小指关节
    * 位置范围：0-100，0为打开，100为关闭


### 注意事项
* 请确已经`source devel/setup.bash`。
* 请确保节点运行在正确的 ROS 工作空间下。
* 请确保相关依赖已经安装。