//
// Created by xiang on 2022/2/15.
//

#pragma once

#include "common/eigen_types.h"
#include "common/s2.hpp"

#include <glog/logging.h>
#include <iomanip>

namespace lightning {
/**
 * 重构之后的状态变量
 * 显式写出各维度状态
 *
 * 虽然我觉得有些地方还是啰嗦了点。。
 *
 * 3*7+2 = 23维
 * S2写成矢量的话24维
 */
struct NavState {
    constexpr static int dim = 23;       //  状态变量维度（实际自由度）
    constexpr static int full_dim = 24;  // 误差变量维度

    using VectState = Eigen::Matrix<double, dim, 1>;           // 矢量形式
    using FullVectState = Eigen::Matrix<double, full_dim, 1>;  // 全状态矢量形式

    NavState() = default;

    bool operator<(const NavState& other) { return timestamp_ < other.timestamp_; }

    FullVectState ToState() {
        FullVectState ret;
        ret.block<3, 1>(0, 0) = pos_;
        ret.block<3, 1>(3, 0) = rot_.log();
        ret.block<3, 1>(6, 0) = offset_R_lidar_.log();
        ret.block<3, 1>(9, 0) = offset_t_lidar_;
        ret.block<3, 1>(12, 0) = vel_;
        ret.block<3, 1>(15, 0) = bg_;
        ret.block<3, 1>(18, 0) = ba_;
        ret.block<3, 1>(21, 0) = grav_.vec_;
        return ret;
    }

    void FromVectState(const FullVectState& state) {
        pos_ = state.block<3, 1>(0, 0);
        rot_ = SO3::exp(state.block<3, 1>(3, 0));
        offset_R_lidar_ = SO3::exp(state.block<3, 1>(6, 0));
        offset_t_lidar_ = state.block<3, 1>(9, 0);
        vel_ = state.block<3, 1>(12, 0);
        bg_ = state.block<3, 1>(15, 0);
        ba_ = state.block<3, 1>(18, 0);
        grav_.vec_ = state.block<3, 1>(21, 0);
    }

    // 运动学模型，基于imu计算状态对时间的导数
    inline FullVectState get_f(const Vec3d& gyro, const Vec3d& acce) const {
        FullVectState res = FullVectState::Zero();
        // 减零偏
        Vec3d omega = gyro - bg_;
        Vec3d a_inertial = rot_ * (acce - ba_);  // 加计读数-ba 并转到 世界系下

        for (int i = 0; i < 3; i++) {
            res(i) = vel_[i]; // 位置导数
            res(i + 3) = omega[i]; // 旋转导数
            res(i + 12) = a_inertial[i] + grav_[i]; // 速度导数
        }
        return res;
    }

    /// 运动方程(8个运动方程，每个变量3个维度)对状态量（23）的雅可比
    inline Eigen::Matrix<double, full_dim, dim> df_dx(const Vec3d& acce) const {
        Eigen::Matrix<double, full_dim, dim> cov = Eigen::Matrix<double, full_dim, dim>::Zero();
        cov.block<3, 3>(0, 12) = Mat3d::Identity(); // 位置变化率(dp/dt)对速度(v)的偏导数是单位矩阵
        Vec3d acc = acce - ba_;
        // Vec3d omega = gyro - bg_;
        cov.block<3, 3>(12, 3) = -rot_.matrix() * SO3::hat(acc); // 速度变化率对旋转的偏导
        cov.block<3, 3>(12, 18) = -rot_.matrix(); // 速度变化率对加计零偏的偏导

        Vec2d vec = Vec2d::Zero();
        Eigen::Matrix<double, 3, 2> grav_matrix = grav_.S2_Mx(vec);

        cov.block<3, 2>(12, 21) = grav_matrix; // 速度变化率(dv/dt)对重力(g)的偏导数通过S2球面的切空间映射获得
        cov.block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity(); // 旋转变化率(dR/dt)对陀螺仪零偏的偏导

        // 速度影响位置
        // 旋转影响速度
        // 加计偏置影响速度
        // 重力影响速度
        // 陀螺仪偏置影响旋转
        return cov;
    }

    /// 运动方程对噪声的雅可比
    inline Eigen::Matrix<double, 24, 12> df_dw() const {
        // 24 维状态导数对 12 维噪声变量的偏导

        // 加计零偏噪声--速度--加计噪声
        // 陀螺仪零偏噪声--旋转--陀螺仪噪声
        Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
        cov.block<3, 3>(12, 3) = -rot_.matrix(); // 速度变化率(dv/dt)对加速度计噪声的偏导
        cov.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity(); // 旋转变化率(dR/dt)对陀螺仪噪声的偏导
        cov.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity(); // 陀螺仪零偏变化率对自身随机游走噪声的偏导
        cov.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity(); // 加速度计零偏变化率对自身随机游走噪声的偏导
        return cov;
    }

    /// 状态量的名义值递推
    void oplus(const FullVectState& vec, double dt) {
        timestamp_ += dt;
        pos_ += vec.middleRows(0, 3) * dt;
        rot_ = rot_ * SO3::exp(vec.middleRows(3, 3) * dt);
        offset_R_lidar_ = offset_R_lidar_ * SO3::exp(vec.middleRows(6, 3) * dt);
        offset_t_lidar_ = offset_t_lidar_ + vec.middleRows(9, 3) * dt;
        vel_ += vec.middleRows(12, 3) * dt;
        bg_ += vec.middleRows(15, 3) * dt;
        ba_ += vec.middleRows(18, 3) * dt;
        grav_.oplus(vec.middleRows(21, 3) * dt);
    }

    /**
     * 广义减法, this - other
     * @param result 减法结果
     * @param other 另一个状态变量
     */
    VectState boxminus(const NavState& other) {
        VectState result;
        result.block<3, 1>(0, 0) = pos_ - other.pos_;
        result.block<3, 1>(3, 0) = (other.rot_.inverse() * rot_).log();
        result.block<3, 1>(6, 0) = (other.offset_R_lidar_.inverse() * offset_R_lidar_).log();
        result.block<3, 1>(9, 0) = offset_t_lidar_ - other.offset_t_lidar_;
        result.block<3, 1>(12, 0) = vel_ - other.vel_;
        result.block<3, 1>(15, 0) = bg_ - other.bg_;
        result.block<3, 1>(18, 0) = ba_ - other.ba_;

        Vec2d dg = grav_.boxminus(other.grav_);
        result.block<2, 1>(21, 0) = dg;
        return result;
    }

    /**
     * 广义加法 this = this+dx 将一个误差状态量加到名义状态量中，更新了名义状态量
     * @param dx 增量
     */
    NavState boxplus(const VectState& dx) {
        NavState ret;
        ret.timestamp_ = timestamp_;
        ret.pos_ = pos_ + dx.middleRows(0, 3);
        ret.rot_ = rot_ * SO3::exp(dx.middleRows(3, 3));
        ret.offset_R_lidar_ = offset_R_lidar_ * SO3::exp(dx.middleRows(6, 3));
        ret.offset_t_lidar_ = offset_t_lidar_ + dx.middleRows(9, 3);
        ret.vel_ = vel_ + dx.middleRows(12, 3);
        ret.bg_ = bg_ + dx.middleRows(15, 3);
        ret.ba_ = ba_ + dx.middleRows(18, 3);
        ret.grav_ = grav_;
        ret.grav_.boxplus(dx.middleRows(21, 2));

        return ret;
    }

    /// 各个子变量所在维度信息
    struct MetaInfo {
        MetaInfo(int idx, int vdim, int dof) : idx_(idx), dim_(vdim), dof_(dof) {}
        int idx_ = 0;  // 变量所在索引
        int dim_ = 0;  // 变量维度
        int dof_ = 0;  // 自由度
    };

    static const std::vector<MetaInfo> vect_states_;  // 矢量变量的维度
    static const std::vector<MetaInfo> SO3_states_;   // SO3 变量的维度
    static const std::vector<MetaInfo> S2_states_;    // S2 变量维度

    friend inline std::ostream& operator<<(std::ostream& os, const NavState& s) {
        os << std::setprecision(18) << s.pos_.transpose() << " " << s.rot_.unit_quaternion().coeffs().transpose() << " "
           << s.offset_R_lidar_.unit_quaternion().coeffs().transpose() << " " << s.offset_t_lidar_.transpose() << " "
           << s.vel_.transpose() << " " << s.bg_.transpose() << " " << s.ba_.transpose() << " "
           << s.grav_.vec_.transpose();
        return os;
    }

    inline SE3 GetPose() const { return SE3(rot_, pos_); }
    inline SO3 GetRot() const { return rot_; }
    inline void SetPose(const SE3& pose) {
        rot_ = pose.so3();
        pos_ = pose.translation();
    }

    inline Vec3d Getba() const { return ba_; }
    inline Vec3d Getbg() const { return bg_; }
    inline Vec3d GetVel() const { return vel_; }
    void SetVel(const Vec3d& v) { vel_ = v; }

    double timestamp_ = 0.0;           // 时间戳
    double confidence_ = 0.0;          // 定位置信度
    bool pose_is_ok_ = true;           // 定位是否有效
    bool lidar_odom_reliable_ = true;  // lio是否有效
    bool is_parking_ = false;          // 是否在停车

    Vec3d pos_ = Vec3d::Zero();             // 位置
    SO3 rot_;                               // 旋转
    SO3 offset_R_lidar_;                    // 外参R
    Vec3d offset_t_lidar_ = Vec3d::Zero();  // 外参t
    Vec3d vel_ = Vec3d::Zero();             // 速度
    Vec3d bg_ = Vec3d::Zero();              // 陀螺零偏
    Vec3d ba_ = Vec3d::Zero();              // 加计零偏
    S2 grav_;                               // 重力
};

}  // namespace lightning
