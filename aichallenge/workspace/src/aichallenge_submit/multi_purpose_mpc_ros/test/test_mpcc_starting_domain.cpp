#include "mpcc_vehicle_model_fixture.hpp"
#include "multi_purpose_mpc_ros/mpcc_starting_domain.hpp"
#include "multi_purpose_mpc_ros/detail/mpcc_vehicle_enclosure.hpp"
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

TEST(MpccRelativeStartingDomain, WholeBodyBoxComposesNativeEndpointsAndSweptCorners)
{
  namespace vehicle = v;
  namespace n = vehicle::numerical;
  const auto p = vehicle_model();
  const auto vertex_ranges = ::offsets();
  n::Box offsets;
  for(size_t k=0;k<8;++k)offsets[k]=n::I(vertex_ranges[k].lower,vertex_ranges[k].upper);
  size_t checked = 0;
  bool rest_seen=false, launch_seen=false, reverse_seen=false;
  for (double frame : {-2.9, -.3, 0., 1.7, 3.1}) for (double initial_u : {-.02, 0., .05, 2.}) {
    vehicle::State relative{0,0,0,initial_u,.002,-.003,0,.04};
    auto body=n::point(relative);
    body[3]=n::I(initial_u,initial_u+.003);body[4]=n::I(-.001,.004);
    body[5]=n::I(-.004,.002);body[6]=n::I(-.03,.03);body[7]=n::I(.035,.045);
    std::vector<n::Box> population{body};
    n::CornerPopulation corners{{offsets,0},{offsets},offsets};
    const n::I px(-.004,.03),py(-.012,.007),heading(-.02,.03);
    auto proposed = request();
    for(size_t k=0;k<8;++k) {
      proposed.body[k]={body[k].lo,body[k].hi};
      proposed.footprint_offsets[k]={offsets[k].lo,offsets[k].hi};
    }
    const auto certified=v::predict_relative_programme_domain_to_rest({proposed,1.8},p);
    ASSERT_TRUE(certified.tube);
    v::CurrentInputPrefix prefix;
    prefix.observation.now_sec=proposed.starting_sec.lower;
    prefix.coordinate_origin.yaw_rad=frame;
    prefix.footprint_offsets=proposed.footprint_offsets;
    prefix.body=proposed.body;
    prefix.body[0]={px.lo,px.hi};prefix.body[1]={py.lo,py.hi};prefix.body[2]={heading.lo,heading.hi};
    const auto transform=v::RelativeDomainTransform::build(*certified.tube,prefix);
    ASSERT_TRUE(transform);
    const auto ranges=[](const n::Box &b){v::BodyRanges r;for(size_t k=0;k<8;++k)r[k]={b[k].lo,b[k].hi};return r;};
    const auto box=[](const v::BodyRanges &r){n::Box b;for(size_t k=0;k<8;++k)b[k]=n::I(r[k].lower,r[k].upper);return b;};
    std::vector<vehicle::State> scalar;
    for(size_t mask=0;mask<64;++mask){
      auto choose=[&](const n::I &r,size_t bit){return mask&(1U<<bit)?r.hi:r.lo;};
      scalar.push_back({choose(px,0),choose(py,1),choose(heading,2),choose(body[3],0),choose(body[4],1),choose(body[5],2),choose(body[6],3),choose(body[7],4)});
    }
    for(size_t step=0;step<30;++step){
      const std::vector<n::I> arms=step<12?std::vector<n::I>{{-3,-.0001},{0},{.0001,1.37}}:std::vector<n::I>{{-3}};
      const n::I desired(-.07,.08);
      for(auto &b:population)b[6]=desired;
      n::Box swept;population=n::advance_partitioned_inputs(std::move(population),arms,p,.005,&swept,&corners);
      const auto endpoint=n::joined(population);
      const auto corner_endpoint=n::joined(corners.states);
      auto compose_body=[&](const n::Box &b){return box(transform->body(ranges(b)));};
      auto compose_corner=[&](const n::Box &b){return box(transform->footprint(ranges(b)));};
      const auto body_end=compose_body(endpoint),corners_end=compose_corner(corner_endpoint),corners_sweep=compose_corner(corners.swept);
      for(size_t mask=0;mask<scalar.size();++mask){
        auto &state=scalar[mask];const auto before=state;const auto &a=arms[mask%arms.size()];
        state.desired_steering_rad=mask&8?desired.hi:desired.lo;
        const auto next=vehicle::advance(state,{mask&16?a.hi:a.lo,0},p,.005);ASSERT_TRUE(next);state=next->state;
        rest_seen|=state.forward_velocity_mps==0;launch_seen|=before.forward_velocity_mps==0&&state.forward_velocity_mps>0;reverse_seen|=state.forward_velocity_mps<0;
        const auto q=n::point(state);
        for(size_t k=0;k<8;++k){ASSERT_GE(q[k].lo,body_end[k].lo)<<"component="<<k;ASSERT_LE(q[k].hi,body_end[k].hi)<<"component="<<k;}
        for(size_t part=0;part<=8;++part){const double f=part/8.;
          const double x=before.x_m+f*(state.x_m-before.x_m),y=before.y_m+f*(state.y_m-before.y_m);
          const double yaw=frame+before.yaw_rad+f*(state.yaw_rad-before.yaw_rad);
          for(size_t k=0;k<8;k+=2){
            const double a=(offsets[k].lo+offsets[k].hi)/2,b=(offsets[k+1].lo+offsets[k+1].hi)/2;
            const double X=std::cos(frame)*x-std::sin(frame)*y+std::cos(yaw)*a-std::sin(yaw)*b;
            const double Y=std::sin(frame)*x+std::cos(frame)*y+std::sin(yaw)*a+std::cos(yaw)*b;
            ASSERT_GE(X,corners_sweep[k].lo);ASSERT_LE(X,corners_sweep[k].hi);
            ASSERT_GE(Y,corners_sweep[k+1].lo);ASSERT_LE(Y,corners_sweep[k+1].hi);
            if(part==8){ASSERT_GE(X,corners_end[k].lo);ASSERT_LE(X,corners_end[k].hi);ASSERT_GE(Y,corners_end[k+1].lo);ASSERT_LE(Y,corners_end[k+1].hi);}
            // World-coordinate addition is outward and does not alter the body kernel.
            for(double origin:{-1e9,89631.,1e9}){
              const auto bx=n::I(origin)+corners_sweep[k],by=n::I(-origin)+corners_sweep[k+1];
              ASSERT_GE(origin+X,bx.lo);ASSERT_LE(origin+X,bx.hi);ASSERT_GE(-origin+Y,by.lo);ASSERT_LE(-origin+Y,by.hi);
            }
            checked+=2;
          }
        }
      }
    }
  }
  EXPECT_EQ(checked,2764800U);EXPECT_TRUE(rest_seen);EXPECT_TRUE(launch_seen);EXPECT_TRUE(reverse_seen);
}

TEST(MpccRelativeStartingDomain, DistinctIdentityAndStrictBodyTimeFootprintJoin) {
  const auto p=vehicle_model();auto r=request();
  const auto relative=v::predict_relative_programme_domain_to_rest({r,1.8},p);
  ASSERT_TRUE(relative.tube);
  const auto &d=relative.tube->normalized.numerical;
  const auto normalized=v::predict_programme_starting_domain_to_rest({d.request,1.8},p);
  ASSERT_TRUE(normalized.tube);
  EXPECT_NE(d.context_fingerprint,normalized.tube->numerical.context_fingerprint);
  EXPECT_EQ(d.request.program.commands.size(),r.program.commands.size());
  EXPECT_EQ(d.request.source_observation.commands.size(),r.source_observation.commands.size());
  EXPECT_EQ(d.request.profile.acceleration_age_sec,r.profile.acceleration_age_sec);
  EXPECT_EQ(d.request.starting_sec.lower,r.starting_sec.lower);
  EXPECT_EQ(d.request.starting_sec.upper,r.starting_sec.upper);
  v::CurrentInputPrefix prefix;prefix.observation.now_sec=r.starting_sec.lower;
  prefix.body=r.body;prefix.footprint_offsets=r.footprint_offsets;
  prefix.coordinate_origin={89631,43128,2.1,0,0,0,0,0};
  ASSERT_TRUE(v::RelativeDomainTransform::build(*relative.tube,prefix));
  EXPECT_FALSE(v::starting_domain_contains_prefix(d.request,prefix));
  for(int variant=0;variant<9;++variant) {
    auto bad=prefix;
    if(variant<5)bad.body[variant+3].upper=std::nextafter(r.body[variant+3].upper,INFINITY);
    if(variant==5)bad.observation.now_sec=std::nextafter(r.starting_sec.lower,-INFINITY);
    if(variant==6)bad.observation.now_sec=std::nextafter(r.starting_sec.upper,INFINITY);
    if(variant==7)bad.footprint_offsets[0].upper=std::nextafter(r.footprint_offsets[0].upper,INFINITY);
    if(variant==8)bad.coordinate_origin.yaw_rad=NAN;
    EXPECT_FALSE(v::RelativeDomainTransform::build(*relative.tube,bad));
  }
  auto malformed=*relative.tube;malformed.normalized.numerical.request.body[0]={0,.1};
  EXPECT_FALSE(v::RelativeDomainTransform::build(malformed,prefix));
  r.coordinate_origin.x_m=NAN;
  EXPECT_FALSE(v::predict_relative_programme_domain_to_rest({r,1.8},p).tube);
}
