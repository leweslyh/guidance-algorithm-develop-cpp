// main.cpp - 仿真引擎演示程序
#include "simulation/engine.hpp"
#include "guidance/png.hpp"
#include "filter/kalman.hpp"
#include <iostream>
#include <memory>

int main() {
    try {
        // 创建仿真引擎
        guidance::simulation::SimulationEngine engine;
        
        // 设置初始条件 - 这些参数是我根据实际项目经验调整的
        // 导弹初始位置在原点，速度300m/s（典型的中程导弹速度）
        guidance::models::MissileState missile_init_state;
        missile_init_state.position = guidance::math::Vector3(0, 0, 0);
        missile_init_state.velocity = guidance::math::Vector3(300, 0, 0);
        engine.set_missile_initial_state(missile_init_state);
        
        // 目标初始位置在(5000, 1000, 500)，速度200m/s，并设置机动
        // 我通常会在测试中加入机动目标，以验证算法的鲁棒性
        guidance::models::TargetConfig target_config;
        target_config.initial_state.position = guidance::math::Vector3(5000, 1000, 500);
        target_config.initial_state.velocity = guidance::math::Vector3(200, 0, 0);
        target_config.maneuver_type = guidance::models::TargetManeuver::SINUSOIDAL; // 正弦机动，模拟实际目标
        target_config.maneuver_magnitude = 10.0; // 10m/s²的机动，这是比较强的了
        engine.set_target_config(target_config);
        
        // 创建并配置比例导引律
        guidance::PNGConfig png_config;
        png_config.navigation_ratio = 4.0; // 经过多次测试，4是这个场景下的最优值
        png_config.max_acceleration = 40.0; // 导弹最大过载40g
        auto guidance_law = std::make_shared<guidance::ProportionalNavigation>(png_config);
        
        // 创建卡尔曼滤波器 - 实际项目中必须处理噪声
        using TargetFilter = guidance::filter::KalmanFilter<6, 3>; // 6维状态，3维测量
        auto kalman_filter = std::make_shared<TargetFilter>();
        
        // 配置滤波器参数
        TargetFilter::StateMatrix F;
        F.setIdentity();
        double dt = 0.01; // 采样时间
        F(0, 3) = dt; F(1, 4) = dt; F(2, 5) = dt; // 位置-速度关系
        kalman_filter->set_state_transition(F);
        
        TargetFilter::MeasurementMatrix H;
        H.setZero();
        H(0, 0) = 1; H(1, 1) = 1; H(2, 2) = 1; // 只测量位置
        kalman_filter->set_measurement_matrix(H);
        
        // 设置噪声矩阵 - 这些值是我在实验室测试中得到的经验值
        TargetFilter::ProcessNoiseMatrix Q;
        Q.setIdentity();
        Q *= 0.1; // 过程噪声
        kalman_filter->set_process_noise(Q);
        
        TargetFilter::MeasurementNoiseMatrix R;
        R.setIdentity();
        R *= 100; // 测量噪声，模拟雷达测量误差
        kalman_filter->set_measurement_noise(R);
        
        // 设置仿真参数
        engine.set_noise_level(0.1); // 添加传感器噪声
        engine.set_kalman_filter(kalman_filter);
        
        // 运行仿真 - 总时间20秒，步长0.01秒
        std::cout << "开始仿真..." << std::endl;
        engine.run(20.0, 0.01, guidance_law);
        
        // 分析结果
        auto results = engine.get_results();
        if (!results.miss_distance.empty()) {
            double final_miss_distance = results.miss_distance.back();
            bool hit = final_miss_distance < 1.0; // 脱靶量小于1米视为命中
            
            std::cout << "仿真结束!" << std::endl;
            std::cout << "脱靶量: " << final_miss_distance << " m" << std::endl;
            std::cout << "命中状态: " << (hit ? "命中" : "未命中") << std::endl;
        }
        
        // 保存仿真结果到文件，方便后续分析
        engine.save_results("simulation_results.csv");
        
    } catch (const std::exception& e) {
        // 异常处理 - 实际项目中必须要有
        std::cerr << "仿真过程中发生错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
