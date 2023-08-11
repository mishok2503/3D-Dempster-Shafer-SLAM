#pragma once

#include "mutil/mutil.h"

inline mutil::Matrix3 RotationMatrixFromEuler(const mutil::Vector3 &euler) {
    const auto [a, b, c] = euler.vec;
    return mutil::Matrix3{
            cos(a) * cos(c) - sin(a) * cos(b) * sin(c), -cos(a) * sin(c) - sin(a) * cos(b) * cos(c), sin(a) * sin(b),
            sin(a) * cos(c) + cos(a) * cos(b) * sin(c), -sin(a) * sin(c) + cos(a) * cos(b) * cos(c), -cos(a) * sin(b),
            sin(b) * sin(c), sin(b) * cos(c), cos(b)
    };
}