// vector3.hpp - 三维向量类实现
#ifndef GUIDANCE_VECTOR3_HPP
#define GUIDANCE_VECTOR3_HPP

#include <cmath>
#include <array>
#include <iostream>

namespace guidance {
namespace math {

class Vector3 {
public:
    // 构造函数
    Vector3() : data_{0, 0, 0} {}  // 默认构造零向量
    Vector3(double x, double y, double z) : data_{x, y, z} {}  // 带坐标的构造
    
    // 访问器 - 我喜欢用这种方式访问坐标，代码更清晰
    double x() const { return data_[0]; }
    double y() const { return data_[1]; }
    double z() const { return data_[2]; }
    
    // 基本运算
    Vector3 operator+(const Vector3& other) const {
        return Vector3(x() + other.x(), y() + other.y(), z() + other.z());
    }
    
    Vector3 operator-(const Vector3& other) const {
        return Vector3(x() - other.x(), y() - other.y(), z() - other.z());
    }
    
    Vector3 operator*(double scalar) const {
        return Vector3(x() * scalar, y() * scalar, z() * scalar);
    }
    
    Vector3 operator/(double scalar) const {
        if (std::abs(scalar) < 1e-12) {
            return *this;
        }
        return Vector3(x() / scalar, y() / scalar, z() / scalar);
    }
    
    // 点积运算 - 制导算法中经常用到，比如计算视线角
    double dot(const Vector3& other) const {
        return x() * other.x() + y() * other.y() + z() * other.z();
    }
    
    // 叉积运算 - 计算垂直向量，用于视线角速率计算
    Vector3 cross(const Vector3& other) const {
        return Vector3(
            y() * other.z() - z() * other.y(),
            z() * other.x() - x() * other.z(),
            x() * other.y() - y() * other.x()
        );
    }
    
    // 归一化 - 这个函数非常重要，计算单位向量时经常用到
    Vector3 normalized() const {
        double len = length();
        if (len < 1e-12) return Vector3(0, 0, 0);  // 避免除以零
        return Vector3(x() / len, y() / len, z() / len);
    }
    
    // 向量长度
    double length() const {
        return std::sqrt(dot(*this));
    }
    
    // 向量长度的平方
    double length_squared() const {
        return dot(*this);
    }
    
    // 转换为数组（兼容性）
    const std::array<double, 3>& to_array() const { return data_; }
    
private:
    std::array<double, 3> data_;  // 内部数据存储
};

// 计算两个向量之间的夹角
double angle_between(const Vector3& a, const Vector3& b) {
    double dot_product = a.dot(b);
    double len_product = a.length() * b.length();
    if (len_product < 1e-12) return 0.0;  // 避免除以零
    return std::acos(std::max(-1.0, std::min(1.0, dot_product / len_product)));
}

// 标量乘法的友元函数
inline Vector3 operator*(double scalar, const Vector3& vec) {
    return vec * scalar;
}

} // namespace math
} // namespace guidance

#endif // GUIDANCE_VECTOR3_HPP
