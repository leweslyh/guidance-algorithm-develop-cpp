// target.hpp - 目标运动模型
#ifndef GUIDANCE_TARGET_HPP
#define GUIDANCE_TARGET_HPP

#include "../core/vector3.hpp"

namespace guidance {
namespace models {

// 目标机动类型
enum class TargetManeuver {
    CONSTANT_VELOCITY, // 匀速直线运动
    SINUSOIDAL,        // 正弦机动
    COORDINATED_TURN,  // 协调转弯
    RANDOM             // 随机机动
};

// 目标状态
struct TargetState {
    math::Vector3 position;
    math::Vector3 velocity;
    math::Vector3 acceleration;
};

// 目标配置
struct TargetConfig {
    TargetState initial_state;
    TargetManeuver maneuver_type = TargetManeuver::CONSTANT_VELOCITY;
    double maneuver_magnitude = 5.0;  // 机动加速度大小 (m/s²)
    double maneuver_frequency = 1.0;  // 机动频率 (Hz) - 仅用于正弦机动
    double turn_rate = 0.1;           // 转弯速率 (rad/s) - 仅用于协调转弯
};

// 目标模型类
class TargetModel {
public:
    explicit TargetModel(const TargetConfig& config = {})
        : config_(config), state_(config.initial_state), time_(0.0) {}
    
    // 更新目标状态
    void update(double time_step) {
        time_ += time_step;
        
        // 根据机动类型计算加速度
        math::Vector3 maneuver_accel = calculate_maneuver_acceleration();
        
        // 更新速度和位置
        state_.velocity = state_.velocity + maneuver_accel * time_step;
        state_.position = state_.position + state_.velocity * time_step;
        state_.acceleration = maneuver_accel;
    }
    
    // 获取当前状态
    const TargetState& get_state() const {
        return state_;
    }
    
    // 重置目标状态
    void reset(const TargetConfig& config) {
        config_ = config;
        state_ = config.initial_state;
        time_ = 0.0;
    }
    
private:
    // 计算机动加速度
    math::Vector3 calculate_maneuver_acceleration() {
        switch (config_.maneuver_type) {
            case TargetManeuver::CONSTANT_VELOCITY:
                return math::Vector3(0.0, 0.0, 0.0);
                
            case TargetManeuver::SINUSOIDAL:
                // 正弦机动 - 在垂直于速度方向产生加速度
                return calculate_sinusoidal_maneuver();
                
            case TargetManeuver::COORDINATED_TURN:
                // 协调转弯 - 垂直于速度方向的向心加速度
                return calculate_coordinated_turn();
                
            case TargetManeuver::RANDOM:
                // 随机机动 - 简单的随机加速度
                return calculate_random_maneuver();
                
            default:
                return math::Vector3(0.0, 0.0, 0.0);
        }
    }
    
    // 计算正弦机动
    math::Vector3 calculate_sinusoidal_maneuver() {
        // 创建垂直于速度的单位向量
        math::Vector3 up(0.0, 0.0, 1.0);
        math::Vector3 velocity_dir = state_.velocity.normalized();
        math::Vector3 perp_dir = velocity_dir.cross(up).normalized();
        
        // 计算正弦加速度
        double accel_magnitude = config_.maneuver_magnitude * 
                               std::sin(2.0 * M_PI * config_.maneuver_frequency * time_);
        
        return perp_dir * accel_magnitude;
    }
    
    // 计算协调转弯
    math::Vector3 calculate_coordinated_turn() {
        // 向心加速度 = v × ω
        math::Vector3 omega(0.0, 0.0, config_.turn_rate); // 绕z轴转弯
        return state_.velocity.cross(omega);
    }
    
    // 计算随机机动
    math::Vector3 calculate_random_maneuver() {
        // 简单随机机动 - 每次调用产生不同的加速度
        // 注意：实际项目中应该使用更合理的随机过程模型
        double x = config_.maneuver_magnitude * (2.0 * (rand() / (double)RAND_MAX) - 1.0);
        double y = config_.maneuver_magnitude * (2.0 * (rand() / (double)RAND_MAX) - 1.0);
        double z = config_.maneuver_magnitude * (2.0 * (rand() / (double)RAND_MAX) - 1.0);
        
        return math::Vector3(x, y, z);
    }
    
    TargetConfig config_;
    TargetState state_;
    double time_; // 当前仿真时间
};

} // namespace models
} // namespace guidance

#endif // GUIDANCE_TARGET_HPP
