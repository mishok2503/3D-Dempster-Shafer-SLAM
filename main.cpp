#include <fstream>

#include "mutil/mutil.h"
#include "nlohmann/json.hpp"
#include "cell/cell.h"
#include "map/map.h"

#include <iostream>


std::ostream& operator<<(std::ostream& os, const mutil::Vector3& v) {
    return os << '[' << v.x << ", " << v.y << ", " << v.z << ']';
}

int main() {
    unsigned mapBuildSteps = 5;

    TMap<TDSCell, 800, 800, 90> map(0.1);
    TRobot robot(map.GetCenter());

    nlohmann::json data = nlohmann::json::parse(std::ifstream("res_new3d.json"))["data"];

    size_t reserveSize = data["measurements"][0]["lidar_data"].size();

    int step = 0;
    for (const auto& measurement : data["measurements"]) {
        std::cout << ++step << '\n';
        std::vector<mutil::Vector3> lidarData;
        lidarData.reserve(reserveSize);
        for (const auto& point : measurement["lidar_data"]) {
            if (point["type"] == "point") {
                const auto& p = point["coordinates"];
                lidarData.emplace_back(p[0], p[1], p[2]);
            }
        }

        if (mapBuildSteps) {
            --mapBuildSteps;
        } else {
            robot.ApplyOdometry(map, lidarData, 100, 0.85, 0.2);
        }

        std::cout << robot.GetPosition() << ' ' << robot.GetOrientation() << '\n';
        map.Update(robot, lidarData);
    }

    return 0;
}
