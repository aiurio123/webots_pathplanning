# webots_pathplanning

> 基于 Webots 的无人机自主路径规划与控制仿真系统

---

## 📌 项目简介（Introduction）

本项目实现了一套 **轻量级的无人机路径规划与控制仿真方案**，在 Webots 仿真环境中完成无人机的路径规划、路径跟踪与飞行控制，
支持从 **手动控制模式** 切换至 **自动导航模式**，无人机能够从当前位置自主规划路径并飞行至目标点。

针对当前主流无人机 **三维路径规划仿真环境配置复杂、上手门槛高** 的问题，
本项目对相关功能进行了模块化封装，充分利用 Webots 仿真环境配置简单、可视化友好的优势，
使用者 **仅需修改航点或路径生成函数（`pathplanner.c`）**，即可快速验证并可视化路径规划算法效果。

项目以 **D* Lite 路径规划算法** 为核心，并融合 **人工势场法（APF）** 作为局部规划与补充策略，
适用于 **未知或部分动态环境**。当全局规划不可用或失效时，系统可退化为 APF 进行局部导航，
适合 **路径规划与无人机导航初学者** 快速入门相关算法与系统结构。

---

## 🧰 技术栈（Tech Stack）

- 仿真平台：**Webots**
- 编程语言：**C**
- 控制方法：PID
- 路径规划算法：D* Lite / APF

---

## ✨ 项目功能（Features）

- [x] 无人机手动 / 自动模式切换  
- [x] 基于 **D* Lite** 的全局路径规划  
- [x] 基于 **APF** 的局部路径规划  
- [x] 路径跟踪与 **PID 控制**  
- [x] 多文件、模块化工程结构  
- [ ] 动态障碍物在线重规划（计划中）  
- [ ] 三维路径平滑优化（计划中）  

---

## 🧠 算法与方法（Method）

### 1️⃣ 全局路径规划（D* Lite）

本项目采用 **D* Lite（Dynamic A\* Lite）算法** 进行全局路径规划，其特点包括：

- 从目标点反向搜索至起点
- 在环境或代价发生变化时，仅对受影响区域进行 **增量式重规划**
- 相较于 A\* 算法，更适用于未知或变化环境

### 2️⃣ 局部路径规划（APF）

人工势场法（APF）用于局部路径生成与应急补充：

- 在全局路径可用时，对轨迹进行局部引导
- 当全局路径规划失败或不可达时，系统可退化为 APF 模式完成导航任务

### 3️⃣ 系统整体流程

1. 通过 GPS / IMU 获取无人机当前状态  
2. 接收目标点信息  
3. 执行全局路径规划（D* Lite）  
4. 若全局路径不可用，则退化为 APF 局部规划  
5. 路径点输入控制模块  
6. PID 控制器输出速度与姿态控制指令  

---

## 🗂️ 项目结构（Project Structure）

```text
webots_pathplanning/
├── .vscode/                   # VSCode 配置
├── build/                     # 编译输出
├── src/
│   ├── main.c                 # 主控制逻辑
│   ├── obstacles.c            # 障碍物信息处理
│   ├── pathplanner.c          # 路径规划核心实现
│   └── model/
│       ├── pid_controller.c   # PID 控制器
│       ├── actuators.c        # 电机接口
│       ├── control.c          # 控制接口
│       ├── devices.c          # Webots 设备封装
│       ├── nav.c              # 导航逻辑
│       ├── planner.c          # 高层规划接口封装
│       ├── sensors.c          # 传感器接口
│       ├── teleop.c           # 键盘遥控
│       └── utils.c            # 数学与工具函数
├── include/
│   ├── obstacles.h
│   ├── path_planner.h
│   ├── pid_controller.h
│   ├── actuator.h
│   ├── control.h
│   ├── devices.h
│   ├── nav.h
│   ├── planner.h
│   ├── sensors.h
│   ├── teleop.h
│   ├── utils.h
│   └── keyboard_controller.h
├── README.md
└── CMakeLists.txt
```
## 🚀 快速开始（Getting Started）

### 1️⃣ 环境要求（Requirements）

在运行本项目之前，请确保已正确安装以下环境：

- 操作系统：Linux / macOS /windows 
- Webots（推荐 R2023 及以上版本）
- CMake（>= 3.10）
- GCC / Clang（支持 C11）

---

### 2️⃣ 获取代码（Clone Repository）

```bash
git clone https://github.com/your_username/webots_pathplanning.git
cd webots_pathplanning
```

### 3️⃣ 编译项目（Build）
```bash
mkdir build
cd build
cmake ..
make
```

### 4️⃣ 运行仿真（Run）

1.打开 Webots

2.加载项目中对应的 .wbt 世界文件

3.确保控制器选择为本项目生成的可执行文件（extern模式）

4.启动仿真即可运行
```bash
./webots_pathplanning
```

### 5️⃣ 模式说明（Mode Switch）

- 手动模式
使用键盘控制无人机运动（具体按键定义见 teleop.c）

- 自动模式
无人机根据路径规划算法（D* Lite / APF）自主飞行至目标点

模式切换逻辑在程序内部实现，可根据需求自行扩展。

## 📝 备注（Notes）
本节用于说明路径规划模块与控制模块之间的**关键数据结构、调用链与数据流**，
以便读者快速理解系统内部的工作机制。

---

### 🔧 关键数据结构

#### 全局航点数组

- **变量名**：`waypoints`
- **类型**：`double waypoints[MAX_WAYPOINTS][4]`
- **航点格式**：`[x, y, z, yaw]`
- **航点数量**：`int waypoint_count`
- **声明位置**：`path_planner.h`

```c
extern double waypoints[MAX_WAYPOINTS][4];
extern int waypoint_count;
````

---

### 🔁 调用链与数据流（简洁步骤）

#### 1️⃣ 外部设置目标并启动规划

在 `main.c` 中定义目标点并启动规划流程：

```c
double goal[4] = {x, y, z, yaw};
planner_start(dev.gps, dev.imu, goal, segments);
```

---

#### 2️⃣ Planner → Nav → Path Planner（生成航点）

路径规划模块的调用关系如下：

* `planner_start()`
  → `nav_start()`
  → `init_global_path()`

对应文件：

* `planner.c`
* `nav.c`
* `path_planner.c`

---

#### 3️⃣ 全局路径生成（Path Planner）

`init_global_path()` 是全局路径生成函数，主要职责包括：

* 读取 GPS / IMU 数据作为起点状态
* 使用 **D* Lite** 生成全局粗路径
* 使用 **APF** 对路径进行局部细化
* 将生成的航点写入全局 `waypoints` 数组
* 设置航点数量 `waypoint_count`

---

### 🧭 Nav 模块中的航点跟踪逻辑

`nav_update()`（`nav.c`）根据当前航点索引 `current_wp` 读取目标航点：

```c
target_x = waypoints[current_wp][0];
target_y = waypoints[current_wp][1];
target_z = waypoints[current_wp][2];
```

随后执行：

* 计算当前位置与目标航点之间的位置误差
* 生成期望速度与高度指令：

  * `desired_state->vx`
  * `desired_state->vy`
  * `desired_state->altitude`

当满足以下到达条件时，自动切换至下一个航点：

```c
if (fabs(ex) < 0.10 && fabs(ey) < 0.10 && fabs(ez) < 0.10)
    current_wp++;
```

---

### 🔄 当前目标点接口

`nav_get_current_target(&tx, &ty, &tz)` 用于向上层模块提供当前目标点：

* 返回值为 `1`：当前存在有效目标
* 返回值为 `0`：当前无可用目标

该接口用于实现导航模块与规划模块之间的解耦。

---

### 🧠 Planner 模块中的航向控制

在 `planner_step()`（`planner.c`）中：

* 调用 `nav_get_current_target()` 获取当前目标坐标
* 根据目标点计算期望偏航角或偏航速率
* 将结果写入：

```c
desired_state->yaw_rate
```

用于后续姿态与航向控制。

---

### ⚠️ 重要实现细节与行为说明

* **回退策略**
  当 D* Lite 无法找到可行路径（如起点或目标被障碍物包围）时，
  `init_global_path()` 将退化为仅使用 **APF** 生成航点，
  生成结果仍会写入 `waypoints` 与 `waypoint_count`。

* **航点格式约定**

  * `waypoints[i][0]`：x 坐标
  * `waypoints[i][1]`：y 坐标
  * `waypoints[i][2]`：z 坐标
  * `waypoints[i][3]`：yaw（在写入前会检查其是否为有限值）

* **缓冲与限制机制**

  * 航点数量受 `MAX_WAYPOINTS` 限制
  * APF 设置最大迭代步数，防止阻塞仿真主循环

---

### 💡 总结

系统中的航点数据流向如下：

```text
外部目标
   ↓
init_global_path()
   ↓
waypoints / waypoint_count
   ↓
nav 跟踪当前航点（current_wp）
   ↓
planner 获取当前目标并计算航向
```

该设计通过全局航点数组实现了 **路径规划模块与控制模块之间的共享与解耦**，
结构清晰，便于后续算法替换与功能扩展。

- 本项目为仿真系统，不涉及真实无人机飞行
- 路径规划算法可在 `pathplanner.c` 中自由替换
- 默认环境为静态障碍物场景


## 📚 参考资料（References）
1. 无人机模型以及pid控制逻辑来自webots开源项目crazyfile
2. 本项目基于crazyfile实现，为webots示例项目，可通过webots直接运行，无需任何环境配置
