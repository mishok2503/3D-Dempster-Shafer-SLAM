#pragma once

#include <array>

class TDSCell {
private:
    static constexpr float DEFAULT_CONFLICT = 0.1; // TODO: somewhere magic constant

    std::array<float, 3> data;

public:
    explicit TDSCell(float p = 0.5) : data({
        1 - DEFAULT_CONFLICT / 2 - p,
        p - DEFAULT_CONFLICT / 2,
        DEFAULT_CONFLICT
    }) {}

    [[nodiscard]] float GetOccupancy() const {
        return data[1] + data[2] / 2;
    }

    void Update(float p) {
        TDSCell t{p};
        data[0] = data[0] * t.data[0] + data[0] * t.data[2] + data[2] * t.data[0];
        data[1] = data[1] * t.data[1] + data[1] * t.data[2] + data[2] * t.data[1];
        data[2] = data[2] * t.data[2];
        float sum = data[0] + data[1] + data[2];
        data[0] /= sum;
        data[1] /= sum;
        data[2] /= sum;
    }
};