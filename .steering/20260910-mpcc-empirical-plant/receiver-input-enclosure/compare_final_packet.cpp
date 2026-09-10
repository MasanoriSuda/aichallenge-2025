#include "multi_purpose_mpc_ros/mpcc_vehicle_prediction.hpp"
#include <iostream>
int main()
{
  namespace vehicle = multi_purpose_mpc_ros::mpcc_vehicle_model;
  constexpr double physical = -.271520636, gain = 1.435;
  const double actual_wire = static_cast<float>(static_cast<double>(static_cast<float>(physical)) * gain);
  vehicle::ProspectivePublicationPrediction prediction;
  prediction.observation.now_sec = 1;
  prediction.proposed_packet = {1, -3, actual_wire};
  const bool actual = vehicle::publication_packet_matches(prediction, 1, -3, physical, gain);
  prediction.proposed_packet.wire_steering_rad = static_cast<float>(physical * gain);
  const bool single_round = vehicle::publication_packet_matches(prediction, 1, -3, physical, gain);
  std::cout << "actual two-stage float packet=" << actual << " incorrect single-round packet=" << single_round << '\n';
  return actual && !single_round ? 0 : 1;
}
