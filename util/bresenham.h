#pragma once

#include <cmath>
#include <vector>
#include <array>

#include "mutil/mutil.h"

// https://www.geeksforgeeks.org/bresenhams-algorithm-for-3-d-line-drawing

template<typename F>
inline void Bresenham3D(mutil::IntVector3 begin, mutil::IntVector3 end, unsigned holeSize, F&& f) {
    auto [x1, y1, z1] = begin.vec;
    auto [x2, y2, z2] = end.vec;
    f({ x1, y1, z1 });
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int dz = abs(z2 - z1);
    if (std::max(dx, std::max(dy, dz)) <= holeSize) {
        return;
    }
    int xs;
    int ys;
    int zs;
    if (x2 > x1)
        xs = 1;
    else
        xs = -1;
    if (y2 > y1)
        ys = 1;
    else
        ys = -1;
    if (z2 > z1)
        zs = 1;
    else
        zs = -1;

// Driving axis is X-axis"
    if (dx >= dy && dx >= dz) {
        int p1 = 2 * dy - dx;
        int p2 = 2 * dz - dx;
        while (x1 != x2 - xs * holeSize) {
            x1 += xs;
            if (p1 >= 0) {
                y1 += ys;
                p1 -= 2 * dx;
            }
            if (p2 >= 0) {
                z1 += zs;
                p2 -= 2 * dx;
            }
            p1 += 2 * dy;
            p2 += 2 * dz;
            f({ x1, y1, z1 });
        }

        // Driving axis is Y-axis"
    }
    else if (dy >= dx && dy >= dz) {
        int p1 = 2 * dx - dy;
        int p2 = 2 * dz - dy;
        while (y1 != y2 - ys * holeSize) {
            y1 += ys;
            if (p1 >= 0) {
                x1 += xs;
                p1 -= 2 * dy;
            }
            if (p2 >= 0) {
                z1 += zs;
                p2 -= 2 * dy;
            }
            p1 += 2 * dx;
            p2 += 2 * dz;
            f({ x1, y1, z1 });
        }

        // Driving axis is Z-axis"
    }
    else {
        int p1 = 2 * dy - dz;
        int p2 = 2 * dx - dz;
        while (z1 != z2 - zs * holeSize) {
            z1 += zs;
            if (p1 >= 0) {
                y1 += ys;
                p1 -= 2 * dz;
            }
            if (p2 >= 0) {
                x1 += xs;
                p2 -= 2 * dz;
            }
            p1 += 2 * dy;
            p2 += 2 * dx;
            f({ x1, y1, z1 });
        }
    }
}