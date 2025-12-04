//
// Created by xiang on 25-3-24.
//

#ifndef LIGHTNING_ROS_UTILS_H
#define LIGHTNING_ROS_UTILS_H

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include "common/point_def.h"

namespace lightning {

/// @brief 将ROS时间转换为秒级双精度浮点数，在需要浮点数时间的场合使用
/// @param time 
/// @return 
inline double ToSec(const builtin_interfaces::msg::Time &time) {
    // 内联避免频繁的调用开销
    return double(time.sec) + 1e-9 * time.nanosec; }

/// @brief 将ROS时间转为纳秒级整数
/// @param time 
/// @return 
inline uint64_t ToNanoSec(const builtin_interfaces::msg::Time &time) { return time.sec * 1e9 + time.nanosec; }

}  // namespace lightning

#endif  // LIGHTNING_ROS_UTILS_H
