# 导弹制导算法仿真系统

## 项目概述

本项目是一个基于C++和Python的导弹制导算法仿真系统，旨在提供一个高效、可扩展的平台，用于开发、测试和评估各种导弹制导算法。系统实现了比例导引律(PNG)、卡尔曼滤波、导弹动力学模型、目标运动模型和3D向量运算等核心功能，支持多种目标机动类型和仿真场景。

## 系统架构

系统采用分层架构设计，具有良好的模块化和可扩展性：

1. **基础层**：提供3D向量运算库
2. **滤波层**：实现卡尔曼滤波器用于目标状态估计
3. **模型层**：包含导弹动力学模型和目标运动模型
4. **制导层**：实现各种制导算法，如比例导引律
5. **仿真层**：协调各模块进行完整的导弹拦截仿真
6. **应用层**：提供主程序入口和结果分析功能

## 核心模块说明

### 1. 3D向量运算库 (core/vector3.hpp)

提供高效的三维向量运算，包括：
- 基本运算（加减乘除）
- 点积和叉积
- 向量归一化
- 长度计算
- 角度计算

### 2. 卡尔曼滤波器 (filter/kalman.hpp)

实现了通用的卡尔曼滤波模板类，用于目标状态估计：
- 支持任意状态维度和测量维度
- 实现了预测和更新步骤
- 使用LLT/SVD分解确保数值稳定性
- 支持设置过程噪声和测量噪声

### 3. 比例导引律 (guidance/png.hpp)

实现了经典的比例导引律算法：
- 支持设置导航比(N)
- 支持最大加速度限制
- 实现了视线角速率计算
- 支持参数配置和重置

### 4. 导弹动力学模型 (models/missile.hpp)

模拟导弹的运动特性：
- 考虑空气阻力
- 支持推力模型
- 实现速度和位置更新
- 可配置质量、阻力系数等参数

### 5. 目标运动模型 (models/target.hpp)

支持多种目标机动类型：
- 匀速直线运动
- 正弦机动
- 协调转弯
- 随机机动

### 6. 仿真引擎 (simulation/engine.hpp)

协调各模块进行完整的仿真：
- 支持设置初始条件
- 实现主仿真循环
- 支持噪声模拟
- 记录仿真结果
- 计算脱靶量

## 快速开始

### 环境要求

- C++17编译器
- CMake 3.15+
- Eigen3库
- Python 3.7+ (用于测试)

### 编译和运行

```bash
# 创建构建目录
mkdir build
cd build

# 配置项目
cmake ..

# 编译项目
make

# 运行仿真
./guidance_simulation
```

### Python测试

```bash
# 运行Python测试脚本
python test_guidance.py
```

## 用法示例

### C++示例

```cpp
#include "simulation/engine.hpp"
#include "guidance/png.hpp"
#include "filter/kalman.hpp"

int main() {
    // 创建仿真引擎
    guidance::simulation::SimulationEngine engine;
    
    // 设置导弹初始状态
    guidance::models::MissileState missile_init_state;
    missile_init_state.position = guidance::math::Vector3(0, 0, 0);
    missile_init_state.velocity = guidance::math::Vector3(300, 0, 0);
    engine.set_missile_initial_state(missile_init_state);
    
    // 设置目标配置
    guidance::models::TargetConfig target_config;
    target_config.initial_state.position = guidance::math::Vector3(5000, 1000, 500);
    target_config.initial_state.velocity = guidance::math::Vector3(200, 0, 0);
    target_config.maneuver_type = guidance::models::TargetManeuver::SINUSOIDAL;
    engine.set_target_config(target_config);
    
    // 创建并配置比例导引律
    guidance::PNGConfig png_config;
    png_config.navigation_ratio = 4.0;
    png_config.max_acceleration = 40.0;
    auto guidance_law = std::make_shared<guidance::ProportionalNavigation>(png_config);
    
    // 运行仿真
    engine.run(20.0, 0.01, guidance_law);
    
    return 0;
}
```

### Python示例

```python
from test_guidance import Vector3, ProportionalNavigation

# 创建比例导引律实例
png = ProportionalNavigation(navigation_ratio=4.0)

# 设置初始状态
missile_pos = Vector3(0, 0, 0)
missile_vel = Vector3(300, 0, 0)
target_pos = Vector3(5000, 1000, 500)
target_vel = Vector3(200, 0, 0)

# 计算制导指令
cmd = png.calculate_command(missile_pos, missile_vel, target_pos, target_vel, time=0.0)
```

## 测试说明

项目提供了Python测试脚本，用于验证核心功能：

1. **向量运算测试**：验证基本向量运算的正确性
2. **比例导引律测试**：验证制导指令计算的正确性
3. **导弹拦截测试**：模拟完整的导弹拦截场景

```bash
python test_guidance.py
```

## 项目结构

```
project/
├── core/            # 核心功能模块
│   └── vector3.hpp  # 3D向量运算库
├── filter/          # 滤波算法
│   └── kalman.hpp   # 卡尔曼滤波器
├── guidance/        # 制导算法
│   ├── guidance_interface.hpp  # 导引律接口
│   └── png.hpp     # 比例导引律
├── models/          # 动力学模型
│   ├── missile.hpp  # 导弹模型
│   └── target.hpp  # 目标模型
├── simulation/      # 仿真引擎
│   └── engine.hpp  # 仿真引擎
├── main.cpp         # 主程序入口
├── test_guidance.py # Python测试脚本
└── CMakeLists.txt   # CMake配置文件
```

## 技术栈

### C++部分
- **语言标准**：C++17
- **数值计算库**：Eigen3
- **构建工具**：CMake

### Python部分
- **版本**：Python 3.7+
- **依赖**：NumPy

## 扩展指南

### 添加新的制导算法

1. 继承`IGuidanceLaw`接口
2. 实现`calculate_command`方法
3. 实现`configure`和`reset`方法
4. 在`guidance_interface.hpp`中添加配置类型

### 添加新的目标机动类型

1. 在`TargetManeuver`枚举中添加新类型
2. 在`TargetModel::calculate_maneuver_acceleration`方法中添加处理逻辑
3. 添加相应的计算方法

### 添加新的滤波算法

1. 在`filter`目录下创建新的滤波类
2. 实现预测和更新方法
3. 在`simulation/engine.hpp`中添加支持

## 许可证

MIT License

## 联系方式

如有问题或建议，请通过GitHub Issues提交。
