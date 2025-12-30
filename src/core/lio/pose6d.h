//
// Created by xiang on 25-4-14.
//

#ifndef FASTER_LIO_POSE6D_H
#define FASTER_LIO_POSE6D_H

#include "common/eigen_types.h"

namespace lightning {

struct Pose6D {
    Pose6D() = default;

    Pose6D(const double t, const Vec3d &a, const Vec3d &g, const Vec3d &v, const Vec3d &p, const Mat3d &R) {
        offset_time = t;
        acc = a;
        gyr = g;
        vel = v;
        pos = p;
        rot = R;
    };

    double offset_time = 0; // 时间偏移量
    Vec3d acc = Vec3d::Zero(); // 加速度
    Vec3d gyr = Vec3d::Zero(); // 角速度
    Vec3d vel = Vec3d::Zero(); // 线速度
    Vec3d pos = Vec3d::Zero(); // 位置
    Mat3d rot = Mat3d::Identity(); // 旋转
};

}  // namespace lightning

#endif  // FASTER_LIO_POSE6D_H
