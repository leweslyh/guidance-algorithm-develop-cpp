// missile.hpp - 导弹动力学模型
#ifndef GUIDANCE_MISSILE_HPP
#define GUIDANCE_MISSILE_HPP

#include "../core/vector3.hpp"

namespace guidance {
namespace models {

// 导弹状态
struct MissileState {
    math::Vector3 position;
    math::Vector3 velocity;
    math::Vector3 acceleration;
    double mass = 100.0;           // 导弹质量 (kg)
    double inertia = 100.0;        // 转动惯量 (kg·m²)
    double drag_coefficient = 0.5; // 阻力系数
    double reference_area = 0.1;   // 参考面积 (m²)
    double thrust = 5000.0;        // 推力 (N)
};

// 导弹模型类
class MissileModel {
public:
    explicit MissileModel(const MissileState& initial_state = {})
        : state_(initial_state) {}
    
    // 更新导弹状态
    void update(double time_step, const math::Vector3& guidance_accel) {
        // 计算空气阻力 (简化模型)
        double air_density = 1.225; // 海平面空气密度 (kg/m³)
        double speed = state_.velocity.length();
        math::Vector3 drag_force = -0.5 * air_density * speed * speed * 
                                  state_.drag_coefficient * state_.reference_area * 
                                  state_.velocity.normalized();
        
        // 计算总加速度 (包括制导指令和阻力)
        math::Vector3 total_accel = guidance_accel + (drag_force / state_.mass);
        
        // 更新速度和位置
        state_.velocity = state_.velocity + total_accel * time_step;
        state_.position = state_.position + state_.velocity * time_step;
        state_.acceleration = total_accel;
    }
    
    // 获取当前状态
    const MissileState& get_state() const {
        return state_;
    }
    
    // 重置导弹状态
    void reset(const MissileState& initial_state) {
        state_ = initial_state;
    }
    
private:
    MissileState state_;
};

} // namespace models
} // namespace guidance

#endif // GUIDANCE_MISSILE_HPP
