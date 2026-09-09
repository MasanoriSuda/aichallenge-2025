// Offline replay of the production motion-filter kernel; no publisher or authority.
#include "multi_purpose_mpc_ros/v2x_overtake_core.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <cmath>
namespace core = multi_purpose_mpc_ros::v2x_overtake_core;
struct State {
  bool sample{}, motion{};
  double t{}, x{}, y{}, vx{}, vy{}, ax{}, ay{};
};
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  const auto input = YAML::LoadFile(argv[1]);
  std::map<std::string, State> states;
  YAML::Node output(YAML::NodeType::Sequence);
  for (const auto & row : input) {
    if (row["topic"].as<std::string>() != "/v2x/vehicle_positions") continue;
    for (const auto & vehicle : row["vehicles"]) {
      const auto id = vehicle["id"].as<std::string>();
      auto & s = states[id];
      const double t=vehicle["stamp"].as<double>();
      const double x=vehicle["x"].as<double>(), y=vehicle["y"].as<double>();
      const double dt=t-s.t;
      if (s.sample && dt == 0.0 && x == s.x && y == s.y) continue;
      YAML::Node result;
      result["id"]=id; result["source_sec"]=t;
      result["array_sec"]=row["stamp"]; result["receipt_sec"]=row["receipt"];
      result["x"]=x; result["y"]=y; result["dt"]=dt;
      if (s.sample && dt > 0.0) {
        const double vx=(x-s.x)/dt, vy=(y-s.y)/dt;
        const auto filtered=core::update_opponent_motion_filter(
          core::OpponentMotionFilterRequest{s.motion,s.vx,s.vy,s.ax,s.ay,vx,vy,dt,0.35,0.25,3.0});
        if (!filtered.valid) return 3;
        s.vx=filtered.velocity_x_mps; s.vy=filtered.velocity_y_mps;
        s.ax=filtered.acceleration_x_mps2; s.ay=filtered.acceleration_y_mps2;
        s.motion=true;
        result["raw_chord_vx"]=vx; result["raw_chord_vy"]=vy;
      }
      result["motion_valid"]=s.motion;
      result["vx"]=s.vx; result["vy"]=s.vy;
      result["ax"]=s.ax; result["ay"]=s.ay;
      output.push_back(result);
      s.sample=true; s.t=t; s.x=x; s.y=y;
    }
  }
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
  emitter << output;
  std::ofstream(argv[2]) << emitter.c_str() << '\n';
  std::cout << "Replayed " << output.size() << " source updates\n";
}
