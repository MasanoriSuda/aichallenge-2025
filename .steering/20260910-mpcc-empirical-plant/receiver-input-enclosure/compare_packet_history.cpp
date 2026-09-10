#include "multi_purpose_mpc_ros/mpcc_vehicle_prediction.hpp"
#include <iostream>

namespace model = multi_purpose_mpc_ros::mpcc_vehicle_model;
int main()
{
  std::vector<model::PublishedCommand> history{{0, -3, 0}};
  if (!model::record_serialized_publication(history, {1.01, -3, -.125}, 1.01, 2) ||
    !model::record_serialized_publication(history, {1.01, 1, .125}, 1.01, 2)) return 2;
  std::cout << "publication_count=" << history.size()
            << " required=3 (both equal-epoch packets plus recorded predecessor)\n";
  if (history.size() != 3 || history[1].wire_acceleration_mps2 != -3 ||
    history[2].wire_acceleration_mps2 != 1) return 1;
  const model::ObservationProvenance observation{
    {1.02, {0, 0, 0, 2, 0, 0, 0, 0}}, 1.02, 1.02, 1.02, 1.03, 1.16, 0, .1, history};
  if (!model::valid(observation)) return 3;
  std::cout << "lossless publication history and observation contract pass\n";
}
