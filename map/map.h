#pragma once

#include <vector>

#include "mutil/mutil.h"
#include "robot/robot.h"
#include "util/bresenham.h"

#include <iostream>

template<typename TCell, unsigned SizeX, unsigned SizeY, unsigned SizeZ>
class TMap {
private:

    using TPlane = std::vector<std::vector<TCell>>;
    std::vector<TPlane> Data;

    float CellSize;
    unsigned HoleSize;

    float GetPointScore(const mutil::Vector3 &point) const {
        mutil::IntVector3 i(point / CellSize);
        return Data[i.x][i.y][i.z].GetOccupancy();
    }

    void BeamUpdate(mutil::IntVector3 begin, mutil::IntVector3 end) {
        using std::max, std::abs;
        Bresenham3D(begin, end, HoleSize, [this](mutil::IntVector3 i) {
            std::cout << i.x << ' ' << i.y << ' ' << i.z << '\n';
            Data[i.x][i.y][i.z].Update(0);
        });
        for (int x = -HoleSize; x <= HoleSize; ++x) {
            for (int y = -HoleSize; y <= HoleSize; ++y) {
                for (int z = -HoleSize; z <= HoleSize; ++z) {
                    Data[end.x + x][end.y + y][end.z + z].Update(
                            (0.5 + 0.5 / (1 + max(abs(x), max(abs(y), abs(z))))) * 0.94 // TODO
                    );
                }
            }
        }
    }

public:

    explicit TMap(float cellSize, unsigned holeSize = 1)
            : Data(SizeX, TPlane(SizeY, std::vector<TCell>(SizeZ))), CellSize(cellSize), HoleSize(holeSize) {}

    float GetScore(const TRobot &robot, const std::vector<mutil::Vector3> &data) const {
        float result = 0;
        for (const auto &point: data) {
            result += GetPointScore(robot.LidarToWorld(point));
        }
        return result;
    }

    [[nodiscard]] mutil::Vector3 GetCenter() const {
        return mutil::Vector3{SizeX, SizeY, SizeZ} * CellSize / 2;
    }

    void Update(const TRobot &robot, const std::vector<mutil::Vector3> &data) {
        for (const auto &point: data) {
            BeamUpdate(mutil::IntVector3{robot.GetPosition()}, mutil::IntVector3{point});
        }
    }
};