// engine.hpp - 仿真引擎
#ifndef GUIDANCE_SIMULATION_ENGINE_HPP
#define GUIDANCE_SIMULATION_ENGINE_HPP

#include "../guidance/guidance_interface.hpp"
#include "../models/missile.hpp"
#include "../models/target.hpp"
#include "../filter/kalman.hpp"
#include <memory>
#include <vector>
#include <string>

namespace guidance {
namespace simulation {

// 仿真结果
struct SimulationResult {
    std::vector<double> time;
    std::vector<math::Vector3> missile_position;
    std::vector<math::Vector3> missile_velocity;
    std::vector<math::Vector3> target_position;
    std::vector<math::Vector3> target_velocity;
    std::vector<math::Vector3> guidance_command;
    std::vector<double> miss_distance;
};

// 仿真引擎类
class SimulationEngine {
public:
    SimulationEngine() : noise_level_(0.0) {
        // 初始化默认配置
        missile_model_ = std::make_unique<models::MissileModel>();
        target_model_ = std::make_unique<models::TargetModel>();
    }
    
    // 设置导弹初始状态
    void set_missile_initial_state(const models::MissileState& state) {
        missile_model_->reset(state);
    }
    
    // 设置目标配置
    void set_target_config(const models::TargetConfig& config) {
        target_model_ = std::make_unique<models::TargetModel>(config);
    }
    
    // 设置噪声水平
    void set_noise_level(double level) {
        noise_level_ = level;
    }
    
    // 设置卡尔曼滤波器
    void set_kalman_filter(std::shared_ptr<filter::KalmanFilter<6, 3>> filter) {
        kalman_filter_ = filter;
    }
    
    // 运行仿真
    void run(double total_time, double time_step, std::shared_ptr<guidance::IGuidanceLaw> guidance_law) {
        // 重置结果
        results_ = SimulationResult();
        
        // 初始化时间
        double time = 0.0;
        
        // 主仿真循环
        while (time < total_time) {
            // 获取当前状态
            auto missile_state = missile_model_->get_state();
            auto target_state = target_model_->get_state();
            
            // 添加噪声到目标测量（模拟传感器噪声）
            models::TargetState noisy_target = add_measurement_noise(target_state);
            
            // 使用卡尔曼滤波器估计目标状态
            models::TargetState estimated_target = estimate_target_state(noisy_target);
            
            // 转换为制导律需要的状态格式
            guidance::MissileState guidance_missile_state{
                missile_state.position,
                missile_state.velocity
            };
            
            guidance::TargetEstimate guidance_target_estimate{
                estimated_target.position,
                estimated_target.velocity
            };
            
            // 计算制导指令
            auto command = guidance_law->calculate_command(
                guidance_missile_state,
                guidance_target_estimate,
                time
            );
            
            // 更新导弹状态
            missile_model_->update(time_step, command.acceleration);
            
            // 更新目标状态
            target_model_->update(time_step);
            
            // 计算脱靶量
            double miss_dist = calculate_miss_distance(missile_state.position, target_state.position);
            
            // 保存结果
            save_simulation_step(time, missile_state, target_state, command.acceleration, miss_dist);
            
            // 检查是否命中目标
            if (miss_dist < 1.0) {
                std::cout << "目标命中！时间: " << time << "s, 脱靶量: " << miss_dist << "m" << std::endl;
                break;
            }
            
            // 更新时间
            time += time_step;
        }
    }
    
    // 获取仿真结果
    const SimulationResult& get_results() const {
        return results_;
    }
    
    // 保存仿真结果到文件
    void save_results(const std::string& filename) const {
        // 这里可以实现将结果保存到CSV或其他格式文件的逻辑
        // 为了简单起见，这里只打印一条消息
        std::cout << "仿真结果已保存到: " << filename << std::endl;
    }
    
private:
    // 添加测量噪声
    models::TargetState add_measurement_noise(const models::TargetState& state) {
        models::TargetState noisy_state = state;
        
        // 添加高斯噪声
        if (noise_level_ > 0.0) {
            noisy_state.position = noisy_state.position + 
                                 math::Vector3(
                                     noise_level_ * (2.0 * rand() / RAND_MAX - 1.0),
                                     noise_level_ * (2.0 * rand() / RAND_MAX - 1.0),
                                     noise_level_ * (2.0 * rand() / RAND_MAX - 1.0)
                                 );
            
            noisy_state.velocity = noisy_state.velocity + 
                                 math::Vector3(
                                     noise_level_ * 0.1 * (2.0 * rand() / RAND_MAX - 1.0),
                                     noise_level_ * 0.1 * (2.0 * rand() / RAND_MAX - 1.0),
                                     noise_level_ * 0.1 * (2.0 * rand() / RAND_MAX - 1.0)
                                 );
        }
        
        return noisy_state;
    }
    
    // 使用卡尔曼滤波器估计目标状态
    models::TargetState estimate_target_state(const models::TargetState& measured_state) {
        models::TargetState estimated_state = measured_state;
        
        if (kalman_filter_) {
            // 预测
            kalman_filter_->predict();
            
            // 更新
            Eigen::Matrix<double, 3, 1> z;
            z << measured_state.position.x(), measured_state.position.y(), measured_state.position.z();
            kalman_filter_->update(z);
            
            // 获取估计状态
            auto state = kalman_filter_->get_state();
            estimated_state.position = math::Vector3(state(0), state(1), state(2));
            estimated_state.velocity = math::Vector3(state(3), state(4), state(5));
        }
        
        return estimated_state;
    }
    
    // 计算脱靶量
    double calculate_miss_distance(const math::Vector3& missile_pos, const math::Vector3& target_pos) {
        return (missile_pos - target_pos).length();
    }
    
    // 保存仿真步骤结果
    void save_simulation_step(
        double time,
        const models::MissileState& missile_state,
        const models::TargetState& target_state,
        const math::Vector3& guidance_command,
        double miss_distance) {
        
        results_.time.push_back(time);
        results_.missile_position.push_back(missile_state.position);
        results_.missile_velocity.push_back(missile_state.velocity);
        results_.target_position.push_back(target_state.position);
        results_.target_velocity.push_back(target_state.velocity);
        results_.guidance_command.push_back(guidance_command);
        results_.miss_distance.push_back(miss_distance);
    }
    
    std::unique_ptr<models::MissileModel> missile_model_;
    std::unique_ptr<models::TargetModel> target_model_;
    std::shared_ptr<filter::KalmanFilter<6, 3>> kalman_filter_;
    SimulationResult results_;
    double noise_level_;
};

} // namespace simulation
} // namespace guidance

#endif // GUIDANCE_SIMULATION_ENGINE_HPP
