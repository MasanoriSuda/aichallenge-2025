#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main
#include "stop_recursion.hpp"

int main(int argc, char **argv)
{
  if (argc != 3) return 2;
  const std::filesystem::path path = argv[1];
  const auto root = YAML::LoadFile(path.string());
  const auto evidence = root["revalidation_evidence"];
  auto captured = mpcc_observation::read_request(evidence["request"], path.parent_path());
  captured.plan = read_plan(evidence["certified_plan_evidence"], path);
  const auto out = sample_stop_recursion(captured, path.string());
  YAML::Emitter emitter; emitter.SetDoublePrecision(17); emitter << out;
  std::ofstream(argv[2]) << emitter.c_str() << '\n';
  std::cout << emitter.c_str() << '\n';
}
