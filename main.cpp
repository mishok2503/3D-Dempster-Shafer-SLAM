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

//    freopen("/Users/mishok/PycharmProjects/BagConverter/tracks/gt.txt", "w", stdout);

    unsigned mapBuildSteps = 1;
    constexpr unsigned sizeX = 2000, sizeY = 2000, sizeZ = 220;
    constexpr float cellSize = 0.1;

    TMap<TDSCell, sizeX, sizeY, sizeZ> map(cellSize, 1);
    TRobot robot(map.GetCenter());

    nlohmann::json data = nlohmann::json::parse(std::ifstream("result.json"))["data"];

    size_t reserveSize = data["measurements"][0]["lidar_data"].size();

    mutil::Vector3 odomPos{};
    mutil::Vector3 odomOrient{};

    int step = 0;
    for (const auto& measurement : data["measurements"]) {

        std::cout << ++step << '\n';
//        if (step > 500) {
//            break;
//        }
//        if (step % 30 == 0) {
//            break;
//            std::ofstream mapFile("/Users/mishok/PycharmProjects/BagConverter/pc/res.txt");
//            map.Draw(mapFile);
//        }
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
//            robot.ErrorCorrection(map, lidarData, 300, 0.1, 0.05);
        }

        const auto& odom = measurement["odometry"];
        odomPos = JsonToVector(odom["position"]);
        odomOrient = JsonToVector(odom["euler_angles"]);

        std::cout << robot.GetPosition() << ' ' << robot.GetOrientation() << '\n';
        map.Update(robot, lidarData);
    }

    std::ofstream mapFile("/Users/mishok/PycharmProjects/BagConverter/pc/res.txt");
    map.Draw(mapFile);

    return 0;
}
