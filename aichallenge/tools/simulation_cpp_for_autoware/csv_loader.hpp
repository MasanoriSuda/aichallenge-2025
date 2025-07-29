#pragma once

#include <string>
#include <vector>
#include "reference_path.hpp"  // ← Waypoint構造体の定義

class CSVLoader {
private:
    std::vector<Waypoint> waypoints_;

public:
    bool load(const std::string & filename);
    const std::vector<Waypoint> & getWaypoints() const { return waypoints_; }
    std::vector<Waypoint> extract_subpath(double current_s, double range) const;
};
