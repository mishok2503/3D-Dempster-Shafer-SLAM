#pragma once

#include <array>
#include <algorithm>

class TDSCell {
private:
    static constexpr float DEFAULT_CONFLICT = 0.1;

    // data[0] - empty
    // data[1] - occupied
    // data[2] - conflict
    std::array<float, 3> data;

public:
    explicit TDSCell(float p = 0.5, float conflict = DEFAULT_CONFLICT) : data({
        (1 - p) * (1 - conflict), p * (1 - conflict), conflict
    }) {}

    [[nodiscard]] float GetOccupancy() const {
        return data[1] + data[2] / 2;
    }

    [[nodiscard]] float GetConflict() const {
        return data[2];
    }

    void Update(float p, const float quality) {
        float conflict = std::min(1 - 1e-4f, DEFAULT_CONFLICT / quality);
        TDSCell t{p, conflict};
        data[0] = data[0] * t.data[0] + data[0] * t.data[2] + data[2] * t.data[0];
        data[1] = data[1] * t.data[1] + data[1] * t.data[2] + data[2] * t.data[1];
        data[2] = data[2] * t.data[2];
        float sum = data[0] + data[1] + data[2];
        data[0] /= sum;
        data[1] /= sum;
        data[2] /= sum;
    }
};