#pragma once

#include "mutil/mutil.h"

struct TLidarPoint {
    enum Type {
        UNKNOWN,
        POINT,
        MAX // out-of-sight measurement
    };

    mutil::Vector3 coordinates;
    float quality;
    Type type = UNKNOWN;
};