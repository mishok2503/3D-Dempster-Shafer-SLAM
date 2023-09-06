#pragma once

#include <vector>

#include "mutil/mutil.h"
#include "robot/robot.h"
#include "util/bresenham.h"
#include "types/lidar_point.h"

#include <iostream>

template<typename TCell, unsigned SizeX, unsigned SizeY, unsigned SizeZ>
class TMap {
private:

    using TPlane = std::vector<std::vector<TCell>>;
    std::vector<TPlane> Data;

    float CellSize;
    unsigned HoleSize;

    [[nodiscard]] float GetPointScore(const mutil::Vector3 &point, const float quality) const {
        auto p = WorldToMap(point);
        if (p.x >= SizeX || p.y >= SizeY || p.z >= SizeZ) {
            return -1e9;
        }
        return GetCellOccupancy(p) * quality;
    }

    void BeamUpdate(mutil::IntVector3 begin, mutil::IntVector3 end, const float quality) {
        using std::max, std::abs;
        if (end.x >= SizeX || end.y >= SizeY || end.z >= SizeZ) {
            std::cerr << "Out of bounds: " <<  end.x << ' ' << end.y << ' ' << end.z << '\n';
            return;
        }
        Bresenham3D(begin, end, HoleSize + 1, [this, quality](mutil::IntVector3 i) {
            Data[i.x][i.y][i.z].Update(0, quality);
        });
        for (int x = -HoleSize; x <= HoleSize; ++x) {
            for (int y = -HoleSize; y <= HoleSize; ++y) {
                for (int z = -HoleSize; z <= HoleSize; ++z) {
                    Data[end.x + x][end.y + y][end.z + z].Update(
                            0.5 + 0.5 / (1 + max(abs(x), max(abs(y), abs(z)))),
                            quality
                    );
                }
            }
        }
    }

public:

    explicit TMap(float cellSize, unsigned holeSize = 1)
            : Data(SizeX, TPlane(SizeY, std::vector<TCell>(SizeZ))), CellSize(cellSize), HoleSize(holeSize) {}

    [[nodiscard]] float GetScore(const TRobot &robot, const std::vector<TLidarPoint> &data) const {
        float result = 0;
        for (const auto &point: data) {
            result += GetPointScore(robot.LidarToWorld(point.coordinates), point.quality);
            if (result < 0) {
                break;
            }
        }
        return result;
    }

    [[nodiscard]] mutil::Vector3 GetCenter() const {
        return mutil::Vector3{SizeX, SizeY, SizeZ} * CellSize / 2;
    }

    void Update(const TRobot &robot, const std::vector<TLidarPoint> &data) {
        for (const auto &point: data) {
            BeamUpdate(
                    WorldToMap(robot.GetPosition()),
                    WorldToMap(robot.LidarToWorld(point.coordinates)),
                    point.quality
            );
        }
    }

    [[nodiscard]] float GetCellOccupancy(unsigned x, unsigned y, unsigned z) const {
        return Data[x][y][z].GetOccupancy();
    }

    [[nodiscard]] float GetCellOccupancy(const mutil::IntVector3 i) const {
        return GetCellOccupancy(i.x, i.y, i.z);
    }

    [[nodiscard]] mutil::IntVector3 WorldToMap(const mutil::Vector3 &v) const {
        return mutil::IntVector3{v / CellSize};
    }

    [[nodiscard]] constexpr std::tuple<unsigned, unsigned, unsigned> GetSize() const {
        return {SizeX, SizeY, SizeZ};
    }

    void Draw(std::ostream& os) {
        for (unsigned i=0; i < SizeX; ++i) {
            for (unsigned j=0; j < SizeY; ++j) {
                for (unsigned k=0; k < SizeZ; ++k) {
                    if (Data[i][j][k].GetOccupancy() > 0.7) {
                        os << CellSize * i << ' ' << CellSize *  j << ' ' << CellSize * k << '\n';
                    }
                }
            }
        }
    }
};