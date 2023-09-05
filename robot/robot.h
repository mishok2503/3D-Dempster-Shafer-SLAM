#pragma once

#include <vector>
#include <random>

#include "mutil/mutil.h"
#include "util/rotation.h"
#include "types/lidar_point.h"

class TRobot {
private:

    mutil::Vector3 Position;
    mutil::Vector3 Orientation;
    mutil::Matrix3 RotationMatrix;

    static constexpr int SEED = 239;
    mutable std::mt19937 RandomGenerator{SEED};
public:

    TRobot(mutil::Vector3 position, mutil::Vector3 orientation = mutil::Vector3{0})
            : Position(position), Orientation(orientation),
              RotationMatrix(RotationMatrixFromEuler(Orientation)) {}


    mutil::Vector3 LidarToWorld(const mutil::Vector3 &point) const {
        return RotationMatrix * point + Position;
    }

    template<class TMap>
    void ErrorCorrection(const TMap &map, const std::vector<TLidarPoint> &data, unsigned samples,
                         float stddev_position, float stddev_orientation) {
        std::normal_distribution<float> PositionDistribution{0, stddev_position};
        std::normal_distribution<float> OrientationDistribution{0, stddev_orientation};

        TRobot bestRobot = *this;
        float score = map.GetScore(bestRobot, data);

        for (int i = 0; i < samples; ++i) {
            mutil::Vector3 deltaPosition = {
                    PositionDistribution(RandomGenerator),
                    PositionDistribution(RandomGenerator),
                    0 //PositionDistribution(RandomGenerator)
            };
            mutil::Vector3 deltaOrientation = {
                    0, // OrientationDistribution(RandomGenerator),
                    0, // OrientationDistribution(RandomGenerator),
                    OrientationDistribution(RandomGenerator)
            };
            TRobot sample{Position + deltaPosition, Orientation + deltaOrientation};
            float s = map.GetScore(sample, data);
            if (s > score) {
                score = s;
                bestRobot = sample;
            }
        }

        *this = bestRobot;
    }

    const mutil::Vector3& GetPosition() const {
        return Position;
    }

    const mutil::Vector3 &GetOrientation() const {
        return Orientation;
    }

    void ApplyOdometry(const mutil::Vector3& deltaPos, const mutil::Vector3& deltaOrientation) {
        if (deltaOrientation != mutil::Vector3{}) {
            Orientation += deltaOrientation;
            RotationMatrix = RotationMatrixFromEuler(Orientation);
        }
        Position += RotationMatrix * deltaPos;
    }
};

