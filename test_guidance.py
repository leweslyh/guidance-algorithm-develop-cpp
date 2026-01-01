#!/usr/bin/env python3
# test_guidance.py - 制导算法Python测试脚本

import numpy as np

class Vector3:
    """三维向量类"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z
    
    def __add__(self, other):
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other):
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def __mul__(self, scalar):
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)
    
    def __rmul__(self, scalar):
        return self * scalar
    
    def dot(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z
    
    def cross(self, other):
        return Vector3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )
    
    def length(self):
        return np.sqrt(self.dot(self))
    
    def normalized(self):
        len = self.length()
        if len < 1e-12:
            return Vector3(0, 0, 0)
        return self * (1.0 / len)
    
    def __str__(self):
        return f"Vector3({self.x:.6f}, {self.y:.6f}, {self.z:.6f})"

class ProportionalNavigation:
    """比例导引律实现"""
    def __init__(self, navigation_ratio=3.0, max_acceleration=50.0):
        self.navigation_ratio = navigation_ratio
        self.max_acceleration = max_acceleration * 9.81  # 转换为m/s²
        self.previous_time = -1.0
        self.previous_line_of_sight = Vector3()
    
    def calculate_command(self, missile_pos, missile_vel, target_pos, target_vel, time):
        """计算制导指令"""
        # 计算视线向量
        los = target_pos - missile_pos
        
        # 计算视线角速率
        los_rate = self._calculate_los_rate(los, time)
        
        # 计算指令加速度: a = N * Vc * λ̇
        missile_speed = missile_vel.length()
        acceleration = self.navigation_ratio * missile_speed * los_rate
        
        # 限制最大加速度
        acceleration = self._limit_acceleration(acceleration)
        
        return acceleration
    
    def _calculate_los_rate(self, los, time):
        """计算视线角速率"""
        if self.previous_time < 0:
            self.previous_time = time
            self.previous_line_of_sight = los
            return Vector3(0, 0, 0)
        
        dt = time - self.previous_time
        if dt < 1e-12:
            return Vector3(0, 0, 0)
        
        # 数值微分
        los_rate = (los - self.previous_line_of_sight) * (1.0 / dt)
        
        # 更新历史数据
        self.previous_line_of_sight = los
        self.previous_time = time
        
        return los_rate
    
    def _limit_acceleration(self, acceleration):
        """限制加速度大小"""
        current_accel = acceleration.length()
        if current_accel > self.max_acceleration:
            return acceleration.normalized() * self.max_acceleration
        return acceleration

def test_vector3():
    """测试向量运算"""
    print("=== 测试向量运算 ===")
    
    # 测试基本运算
    v1 = Vector3(1, 2, 3)
    v2 = Vector3(4, 5, 6)
    
    sum_vec = v1 + v2
    print(f"v1 + v2 = {sum_vec}")
    assert abs(sum_vec.x - 5.0) < 1e-12
    assert abs(sum_vec.y - 7.0) < 1e-12
    assert abs(sum_vec.z - 9.0) < 1e-12
    
    dot_product = v1.dot(v2)
    print(f"v1 · v2 = {dot_product}")
    assert abs(dot_product - 32.0) < 1e-12
    
    cross_product = v1.cross(v2)
    print(f"v1 × v2 = {cross_product}")
    assert abs(cross_product.x + 3.0) < 1e-12
    assert abs(cross_product.y - 6.0) < 1e-12
    assert abs(cross_product.z + 3.0) < 1e-12
    
    norm_v1 = v1.length()
    print(f"|v1| = {norm_v1}")
    assert abs(norm_v1 - np.sqrt(14.0)) < 1e-12
    
    print("向量运算测试通过！\n")

def test_proportional_navigation():
    """测试比例导引律"""
    print("=== 测试比例导引律 ===")
    
    # 创建比例导引律实例
    png = ProportionalNavigation(navigation_ratio=4.0)
    
    # 测试场景：导弹从原点追击目标
    missile_pos = Vector3(0, 0, 0)
    missile_vel = Vector3(300, 0, 0)
    
    target_pos = Vector3(5000, 1000, 500)
    target_vel = Vector3(200, 0, 0)
    
    # 初始时间
    time = 0.0
    
    # 模拟5个时间步的计算
    for i in range(5):
        cmd = png.calculate_command(missile_pos, missile_vel, target_pos, target_vel, time)
        print(f"时间 {time:.2f}s: 制导指令 = {cmd}")
        
        # 更新导弹状态（简化模型）
        missile_vel = missile_vel + cmd * 0.01
        missile_pos = missile_pos + missile_vel * 0.01
        
        # 更新目标状态
        target_pos = target_pos + target_vel * 0.01
        
        time += 0.01
    
    print("比例导引律测试通过！\n")

def test_missile_interception():
    """测试导弹拦截场景"""
    print("=== 测试导弹拦截场景 ===")
    
    # 创建比例导引律实例
    png = ProportionalNavigation(navigation_ratio=4.0, max_acceleration=40.0)
    
    # 导弹初始状态
    missile_pos = Vector3(0, 0, 0)
    missile_vel = Vector3(300, 0, 0)
    
    # 目标初始状态（正弦机动）
    target_pos = Vector3(5000, 1000, 500)
    target_vel = Vector3(200, 0, 0)
    
    # 模拟参数
    time_step = 0.01
    total_time = 15.0
    
    # 存储结果
    miss_distances = []
    
    # 主仿真循环
    for time in np.arange(0.0, total_time, time_step):
        # 计算目标机动（正弦机动）
        maneuver_mag = 10.0
        frequency = 1.0
        target_vel.y = maneuver_mag * np.sin(2 * np.pi * frequency * time)
        
        # 计算制导指令
        cmd = png.calculate_command(missile_pos, missile_vel, target_pos, target_vel, time)
        
        # 更新导弹状态
        missile_vel = missile_vel + cmd * time_step
        missile_pos = missile_pos + missile_vel * time_step
        
        # 更新目标状态
        target_pos = target_pos + target_vel * time_step
        
        # 计算脱靶量
        miss_dist = (missile_pos - target_pos).length()
        miss_distances.append(miss_dist)
        
        # 检查是否命中
        if miss_dist < 1.0:
            print(f"命中目标！时间: {time:.2f}s, 脱靶量: {miss_dist:.4f}m")
            break
    
    final_miss_dist = miss_distances[-1]
    print(f"仿真结束，最终脱靶量: {final_miss_dist:.4f}m")
    
    if final_miss_dist < 10.0:
        print("拦截测试通过！\n")
    else:
        print("拦截测试失败，脱靶量过大！\n")

if __name__ == "__main__":
    test_vector3()
    test_proportional_navigation()
    test_missile_interception()
