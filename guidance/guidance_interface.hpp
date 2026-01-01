// guidance_interface.hpp - 导引律抽象接口
#ifndef GUIDANCE_GUIDANCE_INTERFACE_HPP
#define GUIDANCE_GUIDANCE_INTERFACE_HPP

#include "../core/vector3.hpp"
#include <variant>

namespace guidance {

// 导弹状态
struct MissileState {
    math::Vector3 position;
    math::Vector3 velocity;
};

// 目标估计状态
struct TargetEstimate {
    math::Vector3 position;
    math::Vector3 velocity;
};

// 制导指令
struct GuidanceCommand {
    math::Vector3 acceleration;
    double time;
};

// 通用制导配置类型
using GuidanceConfig = std::variant<struct PNGConfig>;

// 导引律抽象接口
class IGuidanceLaw {
public:
    virtual ~IGuidanceLaw() = default;
    
    // 计算制导指令 - 核心功能，必须实现
    virtual GuidanceCommand calculate_command(
        const MissileState& missile_state,
        const TargetEstimate& target_estimate,
        double time) = 0;
    
    // 配置参数 - 灵活配置不同导引律的参数
    virtual void configure(const GuidanceConfig& config) = 0;
    
    // 重置状态 - 用于多次仿真或场景切换
    virtual void reset() = 0;
};

} // namespace guidance

#endif // GUIDANCE_GUIDANCE_INTERFACE_HPP
