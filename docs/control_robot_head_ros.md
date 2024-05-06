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

* 检查节点是否订阅相关话题
```bash
rosnode info /HDrobot_node 

Output:
--------------------------------------------------------------------------------
Node [/HDrobot_node]
Subscriptions: 
 * /kuavo_arm_traj [unknown type]
 * /robot_head_motion_data [dynamic_biped/robotHeadMotionData]
 * /walkCommand [unknown type]

```

如果存在订阅者中存在`/robot_head_motion_data`话题，则说明机器人节点已经启动成功。
```bash
rostopic info /robot_hand_position

Output:
Type: dynamic_biped/robotHandPosition

Publishers: 
 * /HDrobot_node (http://rongman-ubuntu:46459/)

Subscribers: None
```

* 运行示例，能够控制机器人头部运动, 机器人头部会先水平转动，再垂直转动
```bash
source devel/setup.bash
python3 src/kuavo/ros_package/scripts/head_motion_control_demo.py 
```
* 自行修改`src/kuavo/ros_package/scripts/head_motion_control_demo.py`中的`msg.joint_data`数据，可以控制机器人头部的运动方向，修改`time.sleep()`的时间间隔，可以控制机器人头部运动的速度。

* 主程序会收到头部关节数据
```bash
[ INFO] [1712991040.763163738]: Received robot head motion data joint_data: [-20.000000, 0.000000]
[ INFO] [1712991040.863136754]: Received robot head motion data joint_data: [-17.000000, 0.000000]
[ INFO] [1712991040.963065995]: Received robot head motion data joint_data: [-14.000000, 0.000000]
[ INFO] [1712991041.063127669]: Received robot head motion data joint_data: [-11.000000, 0.000000]
[ INFO] [1712991041.163072952]: Received robot head motion data joint_data: [-8.000000, 0.000000]
[ INFO] [1712991041.263052537]: Received robot head motion data joint_data: [-5.000000, 0.000000]
```

### ROS话题详情
* 话题名称：`/robot_head_motion_data`
* MSG类型：`dynamic_biped/robotHeadMotionData`
* MSG请求：`float64[] joint_data`
* MSG说明： 
    * `joint_data`：头部关节数据，数组长度为2
    * `joint_data[0]`: yaw关节角度，范围为-30到30
    * `joint_data[1]`: pitch关节角度，范围为-25到25

### 注意事项
* 请确已经`source devel/setup.bash`。
* 请确保节点运行在正确的 ROS 工作空间下。
* 请确保相关依赖已经安装。
