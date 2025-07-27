#pragma once
#include <vector>
#include <stdexcept>

inline double linear_interp(double x, const std::vector<double>& xp, const std::vector<double>& fp) {
    if (xp.size() != fp.size()) throw std::invalid_argument("xp and fp size mismatch");
    if (xp.empty()) throw std::invalid_argument("xp is empty");

    if (x <= xp.front()) return fp.front();
    if (x >= xp.back()) return fp.back();

    for (size_t i = 0; i < xp.size() - 1; ++i) {
        if (x >= xp[i] && x <= xp[i + 1]) {
            double t = (x - xp[i]) / (xp[i + 1] - xp[i]);
            return fp[i] * (1 - t) + fp[i + 1] * t;
        }
    }

    return fp.back();  // fallback
}
