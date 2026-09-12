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

TEST(MpccVehicleJacobian,
     CompiledParentMapsEncloseIndependentCurrentBodyAndCorners) {
  namespace v = vehicle;
  namespace n = nd;
  const auto original = multi_purpose_mpc_ros::test::vehicle_model();
  std::mt19937_64 rng(20260912);
  std::uniform_real_distribution<double> unit(0, 1);
  size_t points = 0, body_checks = 0, corner_checks = 0;
  for (size_t c = 0; c < 80; ++c) {
    auto p = original;
    if (c % 4) {
      p.mass_kg *= .8 + .4 * unit(rng);
      p.yaw_inertia_kgm2 *= .8 + .4 * unit(rng);
      for (auto &w : p.wheels)
        w.cornering_per_sec *= .8 + .4 * unit(rng);
    }
    n::Box parent{};
    parent[0] = {-.02, .02};
    parent[1] = {-.03, .03};
    for (size_t i = 2; i < 8; ++i) {
      const double center = i == 3 ? (c % 4 == 0 ? .005 : -.05 + 4 * unit(rng))
                                   : -.35 + .7 * unit(rng);
      const double radius = .005 + .08 * unit(rng);
      parent[i] = {center - radius, center + radius};
    }
    const n::I acceleration =
        c % 5 == 0 ? n::I(-.2, .2) : n::I(-3 + unit(rng), 1 + unit(rng));
    const double dt = p.maximum_step_sec * (.2 + .8 * unit(rng));
    n::CornerProbe probe;
    probe.origin_yaw = -2 + 4 * unit(rng);
    probe.offsets = {n::I(1.1), n::I(.4), n::I(1.1), n::I(-.4),
                     n::I(-.8), n::I(.4), n::I(-.8), n::I(-.4)};
    const auto corners = [&](const n::Box &b) {
      n::Box result;
      const auto co = n::cosine(n::I(probe.origin_yaw)),
                 so = n::sine(n::I(probe.origin_yaw)),
                 ca = n::cosine(n::I(probe.origin_yaw) + b[2]),
                 sa = n::sine(n::I(probe.origin_yaw) + b[2]);
      for (size_t i = 0; i < 8; i += 2) {
        result[i] = co * b[0] - so * b[1] + ca * probe.offsets[i] -
                    sa * probe.offsets[i + 1];
        result[i + 1] = so * b[0] + co * b[1] + sa * probe.offsets[i] +
                        ca * probe.offsets[i + 1];
      }
      return result;
    };
    n::CompiledStepMapRecorder recorder{p};
    const auto parent_corners = corners(parent);
    std::vector<n::CornerImage> images;
    n::step_parts(parent, acceleration, p, dt, &probe, &parent_corners, &images,
                  nullptr, &recorder);
    const auto maps = n::CompiledInputMapAccess::finish(std::move(recorder), 1);
    ASSERT_TRUE(maps);
    const auto data = n::CompiledInputMapAccess::get(maps.get(), p);
    ASSERT_TRUE(data);
    const auto view = n::CompiledInputMapAccess::row(data, 0, p);
    ASSERT_GT(view.count, 0U);
    auto child = parent;
    for (size_t i = 0; i < 8; ++i) {
      const double w = parent[i].hi - parent[i].lo;
      child[i] = {parent[i].lo + .15 * w, parent[i].hi - .25 * w};
    }
    const double aw = acceleration.hi - acceleration.lo;
    const n::I current_a(acceleration.lo + .1 * aw, acceleration.hi - .2 * aw);
    const auto previous = corners(child);
    images.clear();
    const auto branches = n::step_parts(child, current_a, p, dt, &probe,
                                        &previous, &images, &view);
    const auto bound = n::joined(branches);
    n::Box corner_bound = images.front().endpoint;
    for (const auto &im : images)
      for (size_t i = 0; i < 8; ++i)
        corner_bound[i] = n::hull(corner_bound[i], im.endpoint[i]);
    for (size_t point = 0; point < 160; ++point) {
      hp::S state;
      std::array<double, 8> x;
      for (size_t i = 0; i < 8; ++i) {
        const double f =
            point < 128 ? double((point >> (i % 7)) & 1) : unit(rng);
        x[i] = child[i].lo + f * (child[i].hi - child[i].lo);
        x[i] = std::clamp(x[i], child[i].lo, child[i].hi);
        state[i] = hp::D(x[i]);
      }
      const double f = point < 128 ? double((point >> 6) & 1) : unit(rng);
      const double wire =
          std::clamp(current_a.lo + f * (current_a.hi - current_a.lo),
                     current_a.lo, current_a.hi);
      const auto native = v::advance(
          {x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7]}, {wire, 0}, p, dt);
      ASSERT_TRUE(native);
      const auto endpoint =
          hp::map(state, hp::D(wire), p, dt, native->nominal_rest_applied);
      for (size_t i = 0; i < 8; ++i) {
        ASSERT_TRUE(hp::Real(bound[i].lo) <= endpoint[i].v &&
                    hp::Real(bound[i].hi) >= endpoint[i].v)
            << "compiled body lost native point";
        ++body_checks;
      }
      const hp::Real yaw = hp::Real(probe.origin_yaw), co = cos(yaw),
                     so = sin(yaw), ca = cos(yaw + endpoint[2].v),
                     sa = sin(yaw + endpoint[2].v);
      for (size_t i = 0; i < 8; i += 2) {
        const hp::Real cx = co * endpoint[0].v - so * endpoint[1].v +
                            ca * hp::Real(probe.offsets[i].lo) -
                            sa * hp::Real(probe.offsets[i + 1].lo);
        const hp::Real cy = so * endpoint[0].v + co * endpoint[1].v +
                            sa * hp::Real(probe.offsets[i].lo) +
                            ca * hp::Real(probe.offsets[i + 1].lo);
        ASSERT_TRUE(hp::Real(corner_bound[i].lo) <= cx &&
                    hp::Real(corner_bound[i].hi) >= cx &&
                    hp::Real(corner_bound[i + 1].lo) <= cy &&
                    hp::Real(corner_bound[i + 1].hi) >= cy)
            << "compiled corner lost native point";
        corner_checks += 2;
      }
      ++points;
    }
  }
  EXPECT_EQ(points, 12800U);
  EXPECT_EQ(body_checks, 102400U);
  EXPECT_EQ(corner_checks, 102400U);
}
