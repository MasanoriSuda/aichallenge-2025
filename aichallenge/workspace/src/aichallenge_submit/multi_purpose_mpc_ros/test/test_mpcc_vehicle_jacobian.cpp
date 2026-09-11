#include "mpcc_vehicle_model_fixture.hpp"
#include "multi_purpose_mpc_ros/detail/mpcc_vehicle_enclosure.hpp"

#include <boost/multiprecision/cpp_dec_float.hpp>
#include <gtest/gtest.h>
#include <random>

// Differentiate the original shared kernel, independently of the production
// coefficient algebra, with 100 decimal digits. This is a sampled regression,
// not a replacement for the outward-interval argument or physical proof.
namespace hp_oracle {
using Real = boost::multiprecision::cpp_dec_float_100;
struct D {
  Real v{};
  std::array<Real, 7> d{};
  D() = default;
  D(double x) : v(x) {}
  D(const Real &x) : v(x) {}
};
D operator+(const D &a, const D &b) {
  D r(Real(a.v + b.v));
  for (size_t i = 0; i < 7; ++i)
    r.d[i] = a.d[i] + b.d[i];
  return r;
}
D operator-(const D &a, const D &b) {
  D r(Real(a.v - b.v));
  for (size_t i = 0; i < 7; ++i)
    r.d[i] = a.d[i] - b.d[i];
  return r;
}
D operator-(const D &a) {
  D r(Real(-a.v));
  for (size_t i = 0; i < 7; ++i)
    r.d[i] = -a.d[i];
  return r;
}
D operator*(const D &a, const D &b) {
  D r(Real(a.v * b.v));
  for (size_t i = 0; i < 7; ++i)
    r.d[i] = a.d[i] * b.v + a.v * b.d[i];
  return r;
}
D operator/(const D &a, double b) {
  D r(Real(a.v / Real(b)));
  for (size_t i = 0; i < 7; ++i)
    r.d[i] = a.d[i] / Real(b);
  return r;
}
struct A {
  static D sin(const D &a) {
    D r(Real(boost::multiprecision::sin(a.v)));
    const Real c = boost::multiprecision::cos(a.v);
    for (size_t i = 0; i < 7; ++i)
      r.d[i] = c * a.d[i];
    return r;
  }
  static D cos(const D &a) {
    D r(Real(boost::multiprecision::cos(a.v)));
    const Real s = -boost::multiprecision::sin(a.v);
    for (size_t i = 0; i < 7; ++i)
      r.d[i] = s * a.d[i];
    return r;
  }
  static D clamp(const D &a, double lo, double hi) {
    if (a.v <= Real(lo))
      return D(lo);
    if (a.v >= Real(hi))
      return D(hi);
    return a;
  }
  static D reverse_correction(const D &u, const D &a, double interval) {
    if (u.v < 0) {
      const auto restore = -u / interval;
      return a.v >= restore.v ? a : restore;
    }
    return a;
  }
  static D rolling(const D &u, double interval, double limit) {
    return clamp(-u / interval, -limit, limit);
  }
};
using S = std::array<D, 8>;
S map(S s, const D &wire,
      const multi_purpose_mpc_ros::mpcc_vehicle_model::Parameters &p, double dt,
      bool rest) {
  namespace k = multi_purpose_mpc_ros::mpcc_vehicle_model::kernel;
  const A a;
  k::update_tire(s, p, dt, a);
  if (rest) {
    s[3] = s[4] = s[5] = D(0);
    return s;
  }
  const auto d1 = k::derivative(s, wire, p, dt, a);
  const auto mid = k::body_increment(s, d1, .5 * dt);
  const auto rates = k::derivative(mid, wire, p, dt, a);
  return k::body_increment(s, rates, dt);
}
} // namespace hp_oracle

namespace vehicle = multi_purpose_mpc_ros::mpcc_vehicle_model;
namespace nd = vehicle::numerical;
namespace hp = hp_oracle;

TEST(MpccVehicleJacobian,
     SharedKernelHighPrecisionValuesAndDerivativesAreEnclosed) {
  const auto baseline = multi_purpose_mpc_ros::test::vehicle_model();
  std::mt19937_64 rng(20260911);
  std::uniform_real_distribution<double> unit(0, 1);
  size_t values = 0, derivatives = 0, discontinuous_boxes = 0, points = 0;
  for (size_t box = 0; box < 400; ++box) {
    SCOPED_TRACE(box);
    auto p = baseline;
    if (box % 4) {
      p.mass_kg *= .8 + .4 * unit(rng);
      p.yaw_inertia_kgm2 *= .8 + .4 * unit(rng);
      for (auto &wheel : p.wheels) {
        wheel.com_forward_m *= .8 + .4 * unit(rng);
        wheel.com_left_m *= .8 + .4 * unit(rng);
        wheel.cornering_per_sec *= .8 + .4 * unit(rng);
      }
    }
    ASSERT_TRUE(vehicle::valid(p));
    const bool rest = box % 10 == 0;
    const double dt = p.maximum_step_sec * (.2 + .8 * unit(rng));
    nd::Box ranges{};
    ranges[2] = nd::I(-3 + 6 * unit(rng));
    const double velocity = rest           ? 0
                            : box % 5 == 0 ? -.05 + .1 * unit(rng)
                                           : .1 + 9 * unit(rng);
    ranges[3] = {velocity - (rest ? .01 : .04 * unit(rng)),
                 velocity + (rest ? .01 : .04 * unit(rng))};
    for (size_t i = 4; i < 8; ++i) {
      const double center = i == 5 ? -1 + 2 * unit(rng) : -.4 + .8 * unit(rng);
      const double radius = .025 * unit(rng);
      ranges[i] = {center - radius, center + radius};
    }
    const double accel = rest ? -2 : box % 7 == 0 ? 0 : -4 + 7 * unit(rng);
    const nd::I acceleration(accel - .1 * unit(rng), accel + .1 * unit(rng));
    nd::JS input;
    for (size_t i = 2; i < 8; ++i) {
      input[i] = nd::J(ranges[i]);
      input[i].d[i - 2] = nd::I(1);
    }
    nd::J a(acceleration);
    a.d[6] = nd::I(1);
    bool discontinuous = false;
    const auto enclosure = nd::map(input, a, p, dt, rest, &discontinuous);
    discontinuous_boxes += discontinuous;
    const auto contains = [](const hp::Real &x, nd::I range) {
      return x >= hp::Real(range.lo) && x <= hp::Real(range.hi);
    };
    for (size_t sample = 0; sample < 12; ++sample) {
      SCOPED_TRACE(sample);
      hp::S point;
      for (size_t i = 2; i < 8; ++i) {
        const double x =
            sample == 0 ? ranges[i].lo
            : sample == 1
                ? ranges[i].hi
                : ranges[i].lo + (ranges[i].hi - ranges[i].lo) * unit(rng);
        point[i] = hp::D(x);
        point[i].d[i - 2] = 1;
      }
      const double aw =
          sample == 0   ? acceleration.lo
          : sample == 1 ? acceleration.hi
                        : acceleration.lo +
                              (acceleration.hi - acceleration.lo) * unit(rng);
      hp::D command(aw);
      command.d[6] = 1;
      const auto reference = hp::map(point, command, p, dt, rest);
      ++points;
      for (size_t i = 0; i < 8; ++i) {
        ASSERT_TRUE(contains(reference[i].v, enclosure[i].v))
            << "state=" << i << " value=" << reference[i].v
            << " interval=" << enclosure[i].v.lo << "," << enclosure[i].v.hi;
        ++values;
        // At a discontinuity the production map uses the full value enclosure,
        // not the mean-value Jacobian. Never assert a derivative across a jump.
        if (!discontinuous) {
          for (size_t j = 0; j < 7; ++j) {
            ASSERT_TRUE(contains(reference[i].d[j], enclosure[i].d[j]))
                << "state=" << i << " input=" << j
                << " derivative=" << reference[i].d[j]
                << " interval=" << enclosure[i].d[j].lo << ","
                << enclosure[i].d[j].hi;
            ++derivatives;
          }
        }
      }
    }
  }
  EXPECT_EQ(points, 4800U);
  EXPECT_EQ(values, 38400U);
  EXPECT_GT(derivatives, 250000U);
  EXPECT_GT(discontinuous_boxes, 0U);
}

TEST(MpccVehicleJacobian, ForceCoefficientsRequireExactTireAndModelContext) {
  const auto p = multi_purpose_mpc_ros::test::vehicle_model();
  const auto other_model = p;
  nd::JS state{};
  state[vehicle::kernel::Tire] = nd::J(nd::I(.1, .2));
  const auto coefficients =
      nd::force_coefficients(state[vehicle::kernel::Tire].v, p);
  const nd::Arithmetic<nd::J> arithmetic;
  const nd::J command(0);
  EXPECT_NO_THROW(
      nd::derivative_map(state, command, p, .005, arithmetic, &coefficients));
  EXPECT_THROW(nd::derivative_map(state, command, p, .005, arithmetic, nullptr),
               std::runtime_error);
  EXPECT_THROW(nd::derivative_map(state, command, other_model, .005, arithmetic,
                                  &coefficients),
               std::runtime_error);
  state[vehicle::kernel::Tire].v.hi = nd::up(.2);
  EXPECT_THROW(
      nd::derivative_map(state, command, p, .005, arithmetic, &coefficients),
      std::runtime_error);
}
