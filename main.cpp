#include <fstream>
#include <iostream>

#include "mutil/mutil.h"
#include "nlohmann/json.hpp"
#include "cell/ds.h"
#include "cell/counting.h"
#include "map/map.h"
#include "types/lidar_point.h"

namespace {
    std::ostream &operator<<(std::ostream &os, const mutil::Vector3 &v) {
        return os << v.x << ' ' << v.y << ' ' << v.z;
    }

    mutil::Vector3 JsonToVector(const auto& json) {
        return {json[0], json[1], json[2]};
    }
}

int main() {

    unsigned mapBuildSteps = 1;
    constexpr unsigned sizeX = 2000, sizeY = 2000, sizeZ = 220;
    constexpr float cellSize = 0.1;
    constexpr unsigned samplesCount = 300;

    // change cell type here
    TMap<TDSCell, sizeX, sizeY, sizeZ> map(cellSize, 1);
    TRobot robot(map.GetCenter());

    // result.json - input data
    nlohmann::json data = nlohmann::json::parse(std::ifstream("result.json"))["data"];

    size_t reserveSize = data["measurements"][0]["lidar_data"].size();

    mutil::Vector3 odomPos{};
    mutil::Vector3 odomOrient{};

    int step = 0;
    for (const auto& measurement : data["measurements"]) {
        std::cerr << ++step << '\n';

        std::vector<TLidarPoint> lidarData;
        lidarData.reserve(reserveSize);
        for (const auto& point : measurement["lidar_data"]) {
            if (point["type"] == "point") {
                TLidarPoint lidarPoint{
                    JsonToVector(point["coordinates"]),
                    point["quality"],
                    TLidarPoint::POINT
                };
                if (lidarPoint.coordinates.length() < 50) { // TODO: magic number
                    lidarData.push_back(std::move(lidarPoint));
                }
            }
        }

        if (mapBuildSteps) {
            --mapBuildSteps;
        } else {
            robot.ApplyOdometry(odomPos, odomOrient);
            robot.ErrorCorrection(map, lidarData, samplesCount, 0.1, 0.05);
        }

        const auto& odom = measurement["odometry"];
        odomPos = JsonToVector(odom["position"]);
        odomOrient = JsonToVector(odom["euler_angles"]);

        std::cout << robot.GetPosition() << ' ' << robot.GetOrientation() << '\n';
        map.Update(robot, lidarData); // update map with new lidar scan
    }

    std::ofstream mapFile("result.txt");
    map.Draw(mapFile); // write occupied cells coordinates

    return 0;
}
