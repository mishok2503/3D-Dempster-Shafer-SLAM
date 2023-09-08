#include <fstream>
#include <iostream>

#include "mutil/mutil.h"
#include "nlohmann/json.hpp"
#include "cell/DS.h"
#include "cell/Counting.h"
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

int main(int argc, char* argv[]) {

    unsigned mapBuildSteps = 1;
    constexpr unsigned sizeX = 1000, sizeY = 1000, sizeZ = 220;
    constexpr float cellSize = 0.1;
    constexpr unsigned samplesCount = 300;
    using cellType = TDSCell; // or TCountingCell
    constexpr float cellDrawThreshold = 0.7; // in [0; 1]

    std::string outputFileName = "map.txt";
    if (argc >= 3) {
        outputFileName = argv[2];
    }

    if (argc < 2) {
        std::cerr << "Error: provide input file\n";
        return 1;
    }

    std::ifstream inputFile(argv[1]);
    if (!inputFile.is_open()) {
        std::cerr << "Error: Can't open file \"" << argv[1] << "\"\n";
        return 2;
    }
    nlohmann::json data = nlohmann::json::parse(inputFile)["data"];
    inputFile.close();

    TMap<cellType, sizeX, sizeY, sizeZ> map(cellSize, 1);
    TRobot robot(map.GetCenter());

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

    std::ofstream mapFile(outputFileName);
    map.Draw(mapFile, cellDrawThreshold); // write occupied cells coordinates

    return 0;
}
