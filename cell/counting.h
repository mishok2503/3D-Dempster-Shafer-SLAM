#pragma once

class TCountingCell {
private:
    float Sum;
    float N;

public:
    explicit TCountingCell(float p = 0.5) : Sum(p), N(1) {}

    [[nodiscard]] float GetOccupancy() const {
        return Sum / N;
    }

    void Update(float p, float quality) {
        quality = 1;
        Sum += p * quality;
        N += quality;
    }
};
