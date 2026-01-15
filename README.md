# FSM EGO-Planner Controller V2

这是一个基于 **有限状态机 (FSM)** 的 PX4 无人机轨迹跟踪控制器，专为配合 **EGO-Planner** 自主规划算法设计。

相比于 V1 版本，V2 版本引入了 **PID 控制**、**速度前馈 (Feedforward)** 以及 **绝对坐标系** 逻辑，显著提升了轨迹跟踪的精度、响应速度和工程安全性。

## ✨ 主要特性 (Key Features)

1. **PID + 前馈控制 (PID + Feedforward)**
* 摒弃了简单的 P 控制，采用完整的 PID 控制器消除稳态误差。
* 引入 EGO-Planner 的速度前馈，实现“零延迟”响应，飞机不再是被动追赶目标，而是主动同步。


2. **绝对坐标系 (Absolute Coordinate System)**
* **去除了“去皮/归零”逻辑**。直接信任 PX4 飞控的 Local Odom 坐标。
* 防止因程序重启、中途断电等操作导致的坐标系错位炸机风险。


3. **RC 人工接管保护 (Human-in-the-loop)**
* 实时监听遥控器通道 5 (Channel 5)。
* 在任何状态下，拨动开关 (值 > 1600) 可强制中断任务并切入自动降落。


4. **超时安全保护 (Timeout Protection)**
* 若 EGO-Planner 算法崩溃或通讯中断超过 0.5秒，飞机将**主动悬停**，防止沿最后速度矢量撞墙。


5. **状态机架构 (FSM Architecture)**
* 清晰的状态流转：`READY` -> `TAKEOFF` -> `EXPLORE` -> `AUTO_LAND` -> `END`。



## 📂 文件结构 (File Structure)

确保你的 ROS 包目录结构如下所示：

```text
px4_gazebo_fuel/
└── control/
    ├── CMakeLists.txt       <-- 需添加 Eigen 和 include 路径
    ├── package.xml
    ├── include/             <-- 头文件存放处
    │   └── PID_Controller.h
    └── src/
        └── cxr_egoctrl_v2.cpp

```

## 🛠️ 编译配置 (Build Setup)

由于引入了 `Eigen` 库和自定义头文件，请确保 `CMakeLists.txt` 包含以下配置：

```cmake
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(cxr_egoctrl_v2 src/cxr_egoctrl_v2.cpp)
target_link_libraries(cxr_egoctrl_v2
  ${catkin_LIBRARIES}
)

```

## 🤖 节点说明 (Node Details)

### 订阅话题 (Subscribed Topics)

| 话题名称 | 消息类型 | 说明 |
| --- | --- | --- |
| `/mavros/state` | `mavros_msgs::State` | 飞控连接状态、模式、解锁状态 |
| `/mavros/local_position/odom` | `nav_msgs::Odometry` | 飞控输出的位姿 (位置+速度) |
| `/planning/pos_cmd` | `quadrotor_msgs::PositionCommand` | EGO-Planner 规划的轨迹 (位置+速度+偏航) |
| `/mavros/rc/in` | `mavros_msgs::RCIn` | 遥控器通道数据 (用于人工接管) |

### 发布话题 (Published Topics)

| 话题名称 | 消息类型 | 说明 |
| --- | --- | --- |
| `/mavros/setpoint_raw/local` | `mavros_msgs::PositionTarget` | 发送给飞控的控制指令 (速度+偏航) |

### 状态机逻辑 (State Machine)

1. **READY**: 等待飞控连接。自动切入 `OFFBOARD` 模式并 **Arm (解锁)**。
2. **TAKEOFF**: 起飞至指定高度 (默认 0.4m)。使用 PID 垂直控制。
3. **EXPLORE**: 核心状态。
* 收到轨迹 -> 执行 **PID位置修正 + EGO速度前馈**。
* 轨迹中断 (>0.5s) -> **主动悬停**。
* 遥控器开关触发 -> 跳转 `AUTO_LAND`。


4. **AUTO_LAND**: 调用 PX4 的 `AUTO.LAND` 模式自动降落。
5. **MANUAL_LAND**: 落地检测。当高度 < 0.2m 且已落地，强制 **Disarm (上锁)**。

## 🎮 操作流程 (Operation Guide)

### 1. 准备工作

* 连接飞机电池，等待 GPS/VIO 初始化完成（红灯闪烁变为绿灯或蓝灯）。
* 打开遥控器，确认通道 5 开关处于 **低位** (默认值 < 1500)。
* 启动 Mavros 和 EGO-Planner 节点。

### 2. 启动控制

运行节点：

```bash
rosrun control cxr_egoctrl_v2

```

* 程序启动后，飞机会自动：
1. 切入 OFFBOARD。
2. 解锁电机。
3. 起飞至 0.4 米高度并悬停。



### 3. 开始探索

* 在 Rviz 中使用 `2D Nav Goal` 给定目标点。
* 飞机将跟随 EGO-Planner 的轨迹飞行。

### 4. 紧急/正常结束

* **正常结束**：任务完成后，拨动遥控器通道 5 开关至 **高位**。
* **紧急情况**：任何时候觉得不对劲，立刻拨动通道 5 开关。
* 飞机将切入自动降落，落地后自动上锁。

## ⚠️ 安全警告 (Safety Warning)

* **初次飞行**：建议使用拴绳测试 (Tethered Flight)。
* **坐标系**：本程序信任飞控的上电位置为原点。**严禁**在未重启飞控的情况下，随意搬动飞机后直接重启本程序（虽然代码有防护，但物理环境变了 EGO 地图可能对不上）。
* **PID 参数**：当前的 PID 参数可能需要根据你的机型重量进行微调 (在 `PID_Controller.h` 中调整)。