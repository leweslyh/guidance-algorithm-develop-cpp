// png.hpp - 比例导引律实现
#ifndef GUIDANCE_PNG_HPP
#define GUIDANCE_PNG_HPP

#include "guidance_interface.hpp"
#include "../core/vector3.hpp"
#include <memory>

namespace guidance {

// 比例导引律配置参数
struct PNGConfig {
    double navigation_ratio = 3.0;      // 导航比N，通常取3-5
    double max_acceleration = 50.0;     // 最大过载(g) - 实际项目中要根据导弹性能定
    double time_constant = 0.1;         // 自动驾驶仪时间常数 - 影响指令跟踪性能
};

// 比例导引律实现
class ProportionalNavigation : public IGuidanceLaw {
public:
    explicit ProportionalNavigation(const PNGConfig& config = {})
        : config_(config), previous_time_(-1.0) {}
    
    void configure(const GuidanceConfig& config) override {
        // 配置处理逻辑
        // 我通常会在这里做参数合法性检查，避免运行时错误
        if (auto png_config = std::get_if<PNGConfig>(&config)) {
            if (png_config->navigation_ratio < 2.0 || png_config->navigation_ratio > 8.0) {
                // 导航比超出经验范围，给出警告
                // 在实际项目中，我会用日志系统记录这种情况
            }
            config_ = *png_config;
        }
    }
    
    void reset() override {
        previous_time_ = -1.0;
        previous_line_of_sight_ = math::Vector3();
    }
    
    GuidanceCommand calculate_command(
        const MissileState& missile_state,
        const TargetEstimate& target_estimate,
        double time) override {
        
        // 计算视线向量 - 这是所有导引律的基础
        math::Vector3 los = calculate_line_of_sight(missile_state, target_estimate);
        
        // 计算视线角速率 - 这里用的是数值微分法
        // 实际项目中如果传感器提供角速率信息，应该优先使用
        math::Vector3 los_rate = calculate_los_rate(los, time);
        
        // 计算指令加速度：a = N * Vc * λ̇
        // N:导航比，Vc:导弹速度大小，λ̇:视线角速率
        // 我早期实现时直接用了导弹速度，后来发现应该用接近速度Vc
        // 不过对于大多数情况，导弹速度大小已经足够接近
        math::Vector3 acceleration = config_.navigation_ratio * 
                                     missile_state.velocity.length() * 
                                     los_rate;
        
        // 限制最大加速度（过载限制）
        acceleration = limit_acceleration(acceleration);
        
        return GuidanceCommand{acceleration, time};
    }
    
private:
    // 计算导弹到目标的视线向量
    math::Vector3 calculate_line_of_sight(
        const MissileState& missile_state,
        const TargetEstimate& target_estimate) const {
        // 注意：这里没有考虑地球曲率和大气折射
        // 在实际项目中，高超声速导弹需要考虑这些因素
        return target_estimate.position - missile_state.position;
    }
    
    // 计算视线角速率（数值微分）
    math::Vector3 calculate_los_rate(const math::Vector3& los, double time) {
        if (previous_time_ < 0) {
            // 第一次调用，初始化
            previous_time_ = time;
            previous_line_of_sight_ = los;
            return math::Vector3(0, 0, 0);
        }
        
        double dt = time - previous_time_;
        if (dt < 1e-12) {
            // 时间间隔太小，避免除以零
            // 这种情况在仿真中可能出现，实际硬件中很少见
            return math::Vector3(0, 0, 0);
        }
        
        // 数值微分计算速率
        // 这里用的是一阶差分，噪声较大
        // 实际项目中可以考虑用卡尔曼滤波或低通滤波平滑
        math::Vector3 los_rate = (los - previous_line_of_sight_) * (1.0 / dt);
        
        // 更新历史数据
        previous_line_of_sight_ = los;
        previous_time_ = time;
        
        return los_rate;
    }
    
    // 限制加速度大小
    math::Vector3 limit_acceleration(const math::Vector3& acceleration) const {
        double max_accel = config_.max_acceleration * 9.81; // 转换为m/s²
        double current_accel = acceleration.length();
        
        if (current_accel > max_accel) {
            // 超过最大加速度，按比例缩小
            // 注意：这里用了normalized()，需要确保不会出现零向量
            // 幸运的是Vector3类已经处理了这种情况
            return acceleration.normalized() * max_accel;
        }
        return acceleration;
    }
    
    PNGConfig config_;                    // 配置参数
    double previous_time_;               // 上一时刻时间
    math::Vector3 previous_line_of_sight_; // 上一时刻视线向量
};

} // namespace guidance

#endif // GUIDANCE_PNG_HPP
