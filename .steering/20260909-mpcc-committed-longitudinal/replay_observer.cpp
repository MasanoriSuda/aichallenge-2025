// Native observer replay in recorded receive order; no control authority.
#include "multi_purpose_mpc_ros/mpc_longitudinal_prediction.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  namespace p=multi_purpose_mpc_ros::mpc_state_prediction;
  p::LongitudinalResponseObserver observer(.9,.5);
  std::optional<p::LongitudinalResponseObservation> observation;
  std::ifstream input(argv[1]);std::ofstream output(argv[2]);
  output<<std::setprecision(17);
  char kind;double stamp,value;
  while(input>>kind>>stamp>>value) {
    if(kind=='O') observation=observer.observe(stamp,std::abs(value));
    else if(kind=='C') {
      const auto intervals=observer.prediction_intervals(stamp,.13);
      output<<stamp<<' '<<bool(intervals)<<' ';
      if (intervals && observation) {
        double speed=observation->speed_mps;
        for(const auto & interval:*intervals)
          speed=std::max(0.,speed+interval.acceleration_mps2*interval.duration_sec);
        output<<speed<<' '<<observation->filtered_measured_acceleration_mps2<<' '
          <<observation->filtered_command_acceleration_mps2<<' '<<observation->observed_sec;
      } else output<<"nan nan nan nan";
      output<<'\n';
      if(!observer.record_published_command(stamp,stamp+.13,value)) return 3;
    } else return 4;
  }
}
