#define main preserved_native_driver_main
#include "../native_vehicle_model.cpp"
#undef main
#include "interval_model.hpp"

#include <random>

int main()
{
  const auto p = read_parameters();
  const model::State initial{0, 0, 0, 3.738, .06, .565, .2, .18};
  auto range = enclosure::point(initial);
  std::vector<enclosure::Box> population{range};
  std::vector<model::State> samples(100, initial);
  std::mt19937 random(583951); std::uniform_real_distribution<double> unit(0, 1);
  std::size_t scalars = 0;
  // alpha in[0,1], gain/grip positive: each tire update lies between its
  // previous angle and the clipped demand, even while the slew limit acts.
  // This independent global invariant is narrower than a growing interval.
  const double lower = std::min(initial.tire_steering_rad, .078 * p.steering_wire_gain * p.tire_grip);
  const double upper = std::max(initial.tire_steering_rad, .223 * p.steering_wire_gain * p.tire_grip);
  for (std::size_t k = 0; k < 200; ++k) {
    const enclosure::I desired = k < 50 ? enclosure::I(.078, .223) : enclosure::I(.086);
    const enclosure::I wire = k < 50 ? enclosure::I(-3, 1.37) : enclosure::I(-3);
    for (auto & box : population) box[6] = desired;
    population = enclosure::advance_partitioned(population, wire, p, .005, nullptr);
    range = enclosure::joined(population);
    if (range[7].lo < lower - 1e-12 || range[7].hi > upper + 1e-12) {
      std::cerr << "tire enclosure violates independent convex-hull bound at step=" << k
                << " tire=[" << range[7].lo << ',' << range[7].hi << "] bound=["
                << lower << ',' << upper << "]\n";
      return 1;
    }
    for (std::size_t j = 0; j < samples.size(); ++j) {
      auto & sample = samples[j];
      const double fraction = j == 0 ? 0 : (j == 1 ? 1 : unit(random));
      sample.desired_steering_rad = desired.lo + (desired.hi - desired.lo) * fraction;
      const double input = wire.lo + (wire.hi - wire.lo) * unit(random);
      const auto next = model::advance(sample, {input, 0}, p, .005);
      require(next.has_value(), "native reference invalid"); sample = next->state;
      const auto values = enclosure::values(sample);
      for (std::size_t i = 0; i < values.size(); ++i) {
        require(values[i] >= range[i].lo && values[i] <= range[i].hi, "native outside refined range");
        ++scalars;
      }
    }
  }
  std::cout << "tire convex-hull bound and " << scalars << " native scalar values pass\n";
}
