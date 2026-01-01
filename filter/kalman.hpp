// kalman.hpp - 卡尔曼滤波器模板类
#ifndef GUIDANCE_KALMAN_HPP
#define GUIDANCE_KALMAN_HPP

#include <Eigen/Dense>

namespace guidance {
namespace filter {

// 卡尔曼滤波器模板类
template <int StateDim, int MeasurementDim>
class KalmanFilter {
public:
    using StateVector = Eigen::Matrix<double, StateDim, 1>;
    using MeasurementVector = Eigen::Matrix<double, MeasurementDim, 1>;
    using StateMatrix = Eigen::Matrix<double, StateDim, StateDim>;
    using MeasurementMatrix = Eigen::Matrix<double, MeasurementDim, StateDim>;
    using MeasurementNoiseMatrix = Eigen::Matrix<double, MeasurementDim, MeasurementDim>;
    using ProcessNoiseMatrix = Eigen::Matrix<double, StateDim, StateDim>;
    
    KalmanFilter() {
        // 初始化状态和协方差
        x_.setZero();
        P_.setIdentity();
        
        // 默认噪声矩阵 - 实际项目中一定要根据传感器特性调整
        Q_.setIdentity();
        R_.setIdentity();
        
        // 默认状态转移矩阵和测量矩阵
        F_.setIdentity();
        H_.setZero();
        if (StateDim >= MeasurementDim) {
            for (int i = 0; i < MeasurementDim; ++i) {
                H_(i, i) = 1.0;
            }
        }
    }
    
    // 设置状态转移矩阵
    void set_state_transition(const StateMatrix& F) {
        F_ = F;
    }
    
    // 设置测量矩阵
    void set_measurement_matrix(const MeasurementMatrix& H) {
        H_ = H;
    }
    
    // 设置过程噪声协方差
    void set_process_noise(const ProcessNoiseMatrix& Q) {
        Q_ = Q;
    }
    
    // 设置测量噪声协方差
    void set_measurement_noise(const MeasurementNoiseMatrix& R) {
        R_ = R;
    }
    
    // 预测步骤
    void predict() {
        // 预测状态: x(k|k-1) = F * x(k-1|k-1)
        x_ = F_ * x_;
        
        // 预测协方差: P(k|k-1) = F * P(k-1|k-1) * F^T + Q
        // 注意：这里可能会出现数值不稳定的情况
        // 实际项目中，我会在这里检查P_是否为正定矩阵
        P_ = F_ * P_ * F_.transpose() + Q_;
        
        // 确保协方差矩阵对称
        P_ = 0.5 * (P_ + P_.transpose());
    }
    
    // 更新步骤
    void update(const MeasurementVector& z) {
        // 计算卡尔曼增益: K(k) = P(k|k-1) * H^T * (H * P(k|k-1) * H^T + R)^-1
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        
        // 计算S的逆 - 直接求逆可能不稳定，建议使用分解法
        // 我早期项目中直接用inverse()，在某些情况下会导致崩溃
        Eigen::MatrixXd K;
        try {
            // 使用LLT分解，假设S是正定矩阵
            K = P_ * H_.transpose() * S.llt().solve(Eigen::MatrixXd::Identity(MeasurementDim, MeasurementDim));
        } catch (const std::exception& e) {
            // 分解失败，使用伪逆
            K = P_ * H_.transpose() * S.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(MeasurementDim, MeasurementDim));
        }
        
        // 更新状态: x(k|k) = x(k|k-1) + K(k) * (z(k) - H * x(k|k-1))
        x_ = x_ + K * (z - H_ * x_);
        
        // 更新协方差: P(k|k) = (I - K(k) * H) * P(k|k-1)
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(StateDim, StateDim);
        P_ = (I - K * H_) * P_;
        
        // 再次确保协方差矩阵对称和正定
        P_ = 0.5 * (P_ + P_.transpose());
    }
    
    // 获取当前状态估计
    const StateVector& get_state() const {
        return x_;
    }
    
    // 获取当前协方差
    const StateMatrix& get_covariance() const {
        return P_;
    }
    
    // 设置初始状态
    void set_initial_state(const StateVector& x0, const StateMatrix& P0) {
        x_ = x0;
        // 确保初始协方差矩阵对称
        P_ = 0.5 * (P0 + P0.transpose());
    }
    
private:
    StateVector x_;              // 状态估计
    StateMatrix P_;              // 状态协方差
    StateMatrix F_;              // 状态转移矩阵
    MeasurementMatrix H_;        // 测量矩阵
    ProcessNoiseMatrix Q_;       // 过程噪声协方差
    MeasurementNoiseMatrix R_;   // 测量噪声协方差
};

} // namespace filter
} // namespace guidance

#endif // GUIDANCE_KALMAN_HPP
