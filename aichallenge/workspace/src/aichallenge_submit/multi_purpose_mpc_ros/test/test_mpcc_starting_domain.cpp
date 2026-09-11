#include "mpcc_vehicle_model_fixture.hpp"
#include "multi_purpose_mpc_ros/mpcc_starting_domain.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <limits>

namespace v = multi_purpose_mpc_ros::mpcc_vehicle_model;
using multi_purpose_mpc_ros::test::vehicle_model;
namespace {
v::ObservationProvenance observation() {
  return {{1.4, {0, 0, .2, .1, .001, .002, 0, 0}},
          1.39,
          1.38,
          1.39,
          1.45,
          1.58,
          0,
          .1,
          {{0, 1, 0}, {1.1, 1, 0}, {1.2, 1, 0}, {1.3, 1, 0}, {1.4, 1, 0}}};
}
v::InputApplicationProfile profile() {
  return {"test-starting-domain", .25, .25, .1};
}
v::FootprintRanges offsets() {
  return {{{1, 1},
           {.5, .5},
           {1, 1},
           {-.5, -.5},
           {-1, -1},
           {.5, .5},
           {-1, -1},
           {-.5, -.5}}};
}
v::StartingDomainRequest request() {
  v::StartingDomainRequest r;
  r.source_observation = observation();
  r.profile = profile();
  r.program = {.025, {{1.475, 1, 0}, {1.5, -3, 0}}, true, .025};
  r.program.nanosecond_clock =
      v::publication_nanosecond_clock(1.475, .025, .025);
  r.coordinate_origin = {0, 0, .2, 0, 0, 0, 0, 0};
  r.body = {{{-.01, .01},
             {-.02, .02},
             {-.01, .01},
             {.05, .1},
             {-.001, .001},
             {-.001, .001},
             {0, 0},
             {-.01, .01}}};
  r.footprint_offsets = offsets();
  r.starting_sec = {1.475, 1.5};
  return r;
}
std::array<double, 8> values(const v::State &s) {
  return {s.x_m,
          s.y_m,
          s.yaw_rad,
          s.forward_velocity_mps,
          s.lateral_velocity_mps,
          s.yaw_rate_radps,
          s.desired_steering_rad,
          s.tire_steering_rad};
}
} // namespace

TEST(MpccStartingDomain,
     ShortPrefixExactlyMatchesCompletePendingBodyAndCorners) {
  const auto p = vehicle_model();
  for (double duration : {0.0, .001, .010, .037, .05}) {
    auto o = observation();
    o.now_sec = o.initial.source_sec + duration;
    o.control_origin_sec = o.now_sec + .13;
    v::PublishedInputProgram program{.025, {{o.now_sec, -3, 0}}, true, .025};
    v::AppliedFootprintValidation footprint{
        offsets(),
        [](const auto &, const auto &, double, double) { return true; }};
    const auto full = v::predict_pending_inputs_to_rest(o, program, profile(),
                                                        p, {}, &footprint);
    const auto short_result =
        v::predict_pending_input_prefix(o, program, profile(), p, offsets());
    ASSERT_TRUE(full.tube);
    ASSERT_TRUE(short_result.prefix);
    ASSERT_TRUE(full.tube->numerical.publication_footprint);
    for (std::size_t i = 0; i < 8; ++i) {
      EXPECT_DOUBLE_EQ(short_result.prefix->body[i].lower,
                       full.tube->numerical.publication_body[i].lower);
      EXPECT_DOUBLE_EQ(short_result.prefix->body[i].upper,
                       full.tube->numerical.publication_body[i].upper);
      EXPECT_DOUBLE_EQ(short_result.prefix->footprint[i].lower,
                       (*full.tube->numerical.publication_footprint)[i].lower);
      EXPECT_DOUBLE_EQ(short_result.prefix->footprint[i].upper,
                       (*full.tube->numerical.publication_footprint)[i].upper);
    }
    EXPECT_NE(short_result.prefix->context_fingerprint,
              full.tube->numerical.context_fingerprint);
  }
}

TEST(MpccStartingDomain,
     EverySampledStartingTimeAndNativeInputArmStaysEnclosedThroughRest) {
  const auto r = request();
  const auto p = vehicle_model();
  const auto result = v::predict_starting_domain_to_rest(r, p);
  ASSERT_TRUE(result.tube);
  const auto &tube = *result.tube;
  ASSERT_FALSE(tube.source_to_rest.empty());
  EXPECT_GT(r.starting_sec.lower + tube.source_to_rest.back().relative_end_sec,
            1.75);
  for (std::size_t i : {3U, 4U, 5U}) {
    EXPECT_DOUBLE_EQ(tube.source_to_rest.back().endpoint_body[i].lower, 0);
    EXPECT_DOUBLE_EQ(tube.source_to_rest.back().endpoint_body[i].upper, 0);
  }
  for (double start :
       {r.starting_sec.lower, (r.starting_sec.lower + r.starting_sec.upper) / 2,
        r.starting_sec.upper}) {
    for (int arm = 0; arm < 4; ++arm) {
      std::array<double, 8> seed{};
      for (std::size_t i = 0; i < 8; ++i)
        seed[i] = (arm & 1) ? r.body[i].upper : r.body[i].lower;
      v::State state{seed[0], seed[1], seed[2], seed[3],
                     seed[4], seed[5], seed[6], seed[7]};
      for (std::size_t step = 0; step < tube.source_to_rest.size(); ++step) {
        const double begin = start + step * p.maximum_step_sec,
                     end = start + (step + 1) * p.maximum_step_sec;
        const auto inputs = v::applied_input_bounds(
            r.source_observation.commands, r.program, r.profile, begin, end);
        ASSERT_TRUE(inputs);
        const auto &sample = tube.source_to_rest[step];
        EXPECT_LE(sample.absolute_begin_sec, begin);
        EXPECT_GE(sample.absolute_end_sec, end);
        const double acceleration = (arm & 2) ? inputs->acceleration_mps2.upper
                                              : inputs->acceleration_mps2.lower;
        state.desired_steering_rad =
            ((arm & 1) ? inputs->wire_steering_rad.upper
                       : inputs->wire_steering_rad.lower) /
            p.steering_wire_gain;
        const auto next =
            v::advance(state, {acceleration, 0}, p, p.maximum_step_sec);
        ASSERT_TRUE(next);
        state = next->state;
        const auto point = values(state);
        for (std::size_t i = 0; i < 8; ++i) {
          EXPECT_LE(sample.endpoint_body[i].lower, point[i]);
          EXPECT_GE(sample.endpoint_body[i].upper, point[i]);
        }
        const double co = std::cos(r.coordinate_origin.yaw_rad),
                     so = std::sin(r.coordinate_origin.yaw_rad);
        const double ca = std::cos(r.coordinate_origin.yaw_rad + state.yaw_rad),
                     sa = std::sin(r.coordinate_origin.yaw_rad + state.yaw_rad);
        for (std::size_t i = 0; i < 8; i += 2) {
          const double x = r.footprint_offsets[i].lower,
                       y = r.footprint_offsets[i + 1].lower;
          const double cx = co * state.x_m - so * state.y_m + ca * x - sa * y,
                       cy = so * state.x_m + co * state.y_m + sa * x + ca * y;
          EXPECT_LE(sample.endpoint_footprint[i].lower, cx);
          EXPECT_GE(sample.endpoint_footprint[i].upper, cx);
          EXPECT_LE(sample.endpoint_footprint[i + 1].lower, cy);
          EXPECT_GE(sample.endpoint_footprint[i + 1].upper, cy);
        }
      }
      EXPECT_DOUBLE_EQ(state.forward_velocity_mps, 0);
      EXPECT_DOUBLE_EQ(state.lateral_velocity_mps, 0);
      EXPECT_DOUBLE_EQ(state.yaw_rate_radps, 0);
    }
  }
}

TEST(MpccStartingDomain,
     IdentityBindsDomainTimeFrameFootprintModelAndOriginalInputs) {
  const auto r = request();
  const auto p = vehicle_model();
  const auto id = v::starting_domain_context_fingerprint(r, p);
  ASSERT_NE(id, 0U);
  for (std::size_t i = 0; i < 8; ++i) {
    auto changed = r;
    changed.body[i].upper += .001;
    EXPECT_NE(v::starting_domain_context_fingerprint(changed, p), id);
    changed = r;
    changed.footprint_offsets[i].upper += .001;
    EXPECT_NE(v::starting_domain_context_fingerprint(changed, p), id);
  }
  auto changed = r;
  changed.starting_sec.lower += .001;
  EXPECT_NE(v::starting_domain_context_fingerprint(changed, p), id);
  changed = r;
  changed.coordinate_origin.yaw_rad += .001;
  EXPECT_NE(v::starting_domain_context_fingerprint(changed, p), id);
  changed = r;
  changed.source_observation.commands[1].wire_acceleration_mps2 = -3;
  EXPECT_NE(v::starting_domain_context_fingerprint(changed, p), id);
  changed = r;
  changed.program.commands[0].wire_acceleration_mps2 = .5;
  EXPECT_NE(v::starting_domain_context_fingerprint(changed, p), id);
  changed = r;
  changed.profile.profile_id += "-different";
  EXPECT_NE(v::starting_domain_context_fingerprint(changed, p), id);
  auto model = p;
  model.drag_per_sec += .01;
  EXPECT_NE(v::starting_domain_context_fingerprint(r, model), id);
}

TEST(MpccStartingDomain, WholePrefixOutsideAnyStateTimeOrFootprintIsRejected) {
  auto r = request();
  v::CurrentInputPrefix prefix;
  prefix.coordinate_origin = r.coordinate_origin;
  prefix.body = r.body;
  prefix.footprint_offsets = r.footprint_offsets;
  prefix.observation.now_sec = r.starting_sec.lower;
  EXPECT_TRUE(v::starting_domain_contains_prefix(r, prefix));
  prefix.observation.now_sec = r.starting_sec.upper;
  EXPECT_TRUE(v::starting_domain_contains_prefix(r, prefix));
  for (std::size_t i = 0; i < 8; ++i) {
    auto bad = prefix;
    bad.body[i].upper = std::nextafter(r.body[i].upper, INFINITY);
    EXPECT_FALSE(v::starting_domain_contains_prefix(r, bad));
    bad = prefix;
    bad.body[i].lower = std::nextafter(r.body[i].lower, -INFINITY);
    EXPECT_FALSE(v::starting_domain_contains_prefix(r, bad));
    bad = prefix;
    bad.body[i].lower = NAN;
    EXPECT_FALSE(v::starting_domain_contains_prefix(r, bad));
    bad = prefix;
    bad.footprint_offsets[i].upper += .001;
    EXPECT_FALSE(v::starting_domain_contains_prefix(r, bad));
  }
  for (double stamp : {std::nextafter(r.starting_sec.lower, -INFINITY),
                       std::nextafter(r.starting_sec.upper, INFINITY),
                       std::numeric_limits<double>::quiet_NaN()}) {
    prefix.observation.now_sec = stamp;
    EXPECT_FALSE(v::starting_domain_contains_prefix(r, prefix));
  }
}

TEST(MpccStartingDomain, DirectFrameCompositionRetainsThinLateralRange) {
  auto r = request();
  r.coordinate_origin = {10000, 20000, .8, 0, 0, 0, 0, 0};
  r.body[0] = {-.011, .011};
  r.body[1] = {-1e-6, 1e-6};
  r.body[2] = {-1e-6, 1e-6};
  v::CurrentInputPrefix prefix;
  prefix.coordinate_origin = r.coordinate_origin;
  prefix.coordinate_origin.x_m += 1e-10;
  prefix.body = r.body;
  prefix.body[0] = {-.01, .01};
  prefix.body[1] = {-1e-7, 1e-7};
  prefix.body[2] = {0, 0};
  prefix.footprint_offsets = r.footprint_offsets;
  prefix.observation.now_sec = r.starting_sec.lower;
  EXPECT_TRUE(v::starting_domain_contains_prefix(r, prefix));
  prefix.coordinate_origin.y_m += .1;
  EXPECT_FALSE(v::starting_domain_contains_prefix(r, prefix));
}

TEST(MpccStartingDomain, InvalidAndUncoveredDomainsReturnNoPartialTube) {
  const auto r = request();
  const auto p = vehicle_model();
  auto bad = r;
  bad.starting_sec.lower -= 1e-9;
  EXPECT_FALSE(v::predict_starting_domain_to_rest(bad, p).tube);
  bad = r;
  bad.starting_sec.upper += 1e-9;
  EXPECT_FALSE(v::predict_starting_domain_to_rest(bad, p).tube);
  bad = r;
  bad.body[3] = {1, -1};
  EXPECT_FALSE(v::predict_starting_domain_to_rest(bad, p).tube);
  bad = r;
  bad.coordinate_origin.yaw_rad = NAN;
  EXPECT_FALSE(v::predict_starting_domain_to_rest(bad, p).tube);
  bad = r;
  bad.program.repeat_last_until_rest = false;
  EXPECT_FALSE(v::predict_starting_domain_to_rest(bad, p).tube);
  bad = r;
  bad.program.commands.back().wire_acceleration_mps2 = 1;
  EXPECT_FALSE(v::predict_starting_domain_to_rest(bad, p).tube);
  bad = r;
  bad.source_observation.commands = {{0, 1, 0}, {1.45, 1, 0}};
  const auto missing = v::predict_starting_domain_to_rest(bad, p);
  EXPECT_EQ(missing.reason, v::AppliedInputRejectReason::HistoryUnavailable);
  EXPECT_FALSE(missing.tube);
  auto o = observation();
  o.commands = {{0, 1, 0}, {1.45, 1, 0}};
  EXPECT_FALSE(v::predict_pending_input_prefix(o, r.program, r.profile, p,
                                               r.footprint_offsets)
                   .prefix);
}

TEST(MpccProgrammeStartingDomain, LateStartingTimesRetainAllOriginalDelayedPackets)
{
  auto r = request();
  r.starting_sec = {1.575, 1.85};
  const auto p = vehicle_model();
  const v::ProgrammeStartingDomainRequest query{r, 1.85};
  EXPECT_FALSE(v::predict_starting_domain_to_rest(r, p).tube);
  const auto prediction = v::predict_programme_starting_domain_to_rest(query, p);
  ASSERT_TRUE(prediction.tube);
  const auto &tube = prediction.tube->numerical;
  ASSERT_FALSE(tube.source_to_rest.empty());
  // The positive original first packet is already nominally in the past, but
  // its receiver memory remains possible at early members of this time set.
  EXPECT_GT(tube.source_to_rest.front().inputs.acceleration_mps2.upper, 0);
  EXPECT_GT(r.starting_sec.lower + tube.source_to_rest.back().relative_end_sec, 1.75);
  for (double start : {1.575, 1.675, 1.85}) {
    for (unsigned arm = 0; arm < 256; ++arm) {
      std::array<double, 8> seed;
      for (std::size_t i = 0; i < 8; ++i)
        seed[i] = (arm & (1U << i)) ? r.body[i].upper : r.body[i].lower;
      v::State state{seed[0],seed[1],seed[2],seed[3],seed[4],seed[5],seed[6],seed[7]};
      for (std::size_t step = 0; step < tube.source_to_rest.size(); ++step) {
        const auto inputs = v::applied_input_bounds(r.source_observation.commands,
          r.program,r.profile,start+step*p.maximum_step_sec,start+(step+1)*p.maximum_step_sec);
        ASSERT_TRUE(inputs);
        const double acceleration = (arm & 1) ? inputs->acceleration_mps2.upper : inputs->acceleration_mps2.lower;
        state.desired_steering_rad = ((arm & 2) ? inputs->wire_steering_rad.upper : inputs->wire_steering_rad.lower) / p.steering_wire_gain;
        const auto next = v::advance(state,{acceleration,0},p,p.maximum_step_sec);
        ASSERT_TRUE(next); state = next->state;
        const auto &sample = tube.source_to_rest[step]; const auto point = values(state);
        for (std::size_t i = 0; i < 8; ++i) {
          ASSERT_LE(sample.endpoint_body[i].lower,point[i]);
          ASSERT_GE(sample.endpoint_body[i].upper,point[i]);
        }
        const double co=std::cos(r.coordinate_origin.yaw_rad),so=std::sin(r.coordinate_origin.yaw_rad);
        const double ca=std::cos(r.coordinate_origin.yaw_rad+state.yaw_rad),sa=std::sin(r.coordinate_origin.yaw_rad+state.yaw_rad);
        for (std::size_t i = 0; i < 8; i += 2) {
          const double x=r.footprint_offsets[i].lower,y=r.footprint_offsets[i+1].lower;
          const double cx=co*state.x_m-so*state.y_m+ca*x-sa*y;
          const double cy=so*state.x_m+co*state.y_m+sa*x+ca*y;
          ASSERT_LE(sample.endpoint_footprint[i].lower,cx); ASSERT_GE(sample.endpoint_footprint[i].upper,cx);
          ASSERT_LE(sample.endpoint_footprint[i+1].lower,cy); ASSERT_GE(sample.endpoint_footprint[i+1].upper,cy);
        }
      }
      EXPECT_DOUBLE_EQ(state.forward_velocity_mps,0);
      EXPECT_DOUBLE_EQ(state.lateral_velocity_mps,0);
      EXPECT_DOUBLE_EQ(state.yaw_rate_radps,0);
    }
  }
}

TEST(MpccProgrammeStartingDomain, OriginalHorizonAndWholeIdentityAreBoundWithoutWideningFirstWindowApi)
{
  auto r = request(); const auto p = vehicle_model();
  const v::ProgrammeStartingDomainRequest original{r,1.85};
  const auto hash = v::programme_starting_domain_context_fingerprint(original,p);
  ASSERT_NE(hash,0U);
  EXPECT_NE(hash,v::starting_domain_context_fingerprint(r,p));
  for (int variant=0;variant<5;++variant) {
    auto changed=original;
    if (variant==0) changed.original_rest_sec+=.1;
    if (variant==1) changed.domain.starting_sec.upper+=.1;
    if (variant==2) changed.domain.body[0].lower-=.1;
    if (variant==3) changed.domain.source_observation.commands.back().wire_acceleration_mps2+=.1;
    if (variant==4) changed.domain.footprint_offsets[0].lower-=.1;
    EXPECT_NE(v::programme_starting_domain_context_fingerprint(changed,p),hash);
  }
  for (int variant=0;variant<6;++variant) {
    auto bad=original;
    if (variant==0) bad.original_rest_sec=std::numeric_limits<double>::infinity();
    if (variant==1) bad.domain.starting_sec.upper=1.851;
    if (variant==2) bad.domain.starting_sec.lower=1.474;
    if (variant==3) bad.domain.starting_sec.lower=1.501;
    if (variant==4) bad.domain.body[2].upper=std::numeric_limits<double>::quiet_NaN();
    if (variant==5) bad.domain.program.repeat_last_until_rest=false;
    EXPECT_EQ(v::programme_starting_domain_context_fingerprint(bad,p),0U);
    EXPECT_FALSE(v::predict_programme_starting_domain_to_rest(bad,p).tube);
  }
}
