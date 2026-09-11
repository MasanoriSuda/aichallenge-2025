#include "multi_purpose_mpc_ros/mpcc_publication_ledger.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace v = multi_purpose_mpc_ros::mpcc_vehicle_model;
namespace {
v::PublishedProgramSource source(std::size_t index = 0) { return {11, 21, 31, 41, index}; }

v::PublishedInputProgram program(bool integer_clock = true) {
  v::PublishedInputProgram result{.025, {{1.025, 1, .125}, {1.025 + .025, -3, .25}}, true, .025};
  if (integer_clock) {
    result.nanosecond_clock = v::publication_nanosecond_clock(1.025, .025, .025);
    for (std::size_t i = 0; i < result.commands.size(); ++i)
      result.commands[i].published_sec = *v::publication_epoch(result, i);
  }
  return result;
}

void same_history(const std::vector<v::PublishedCommand> &a,
                  const std::vector<v::PublishedCommand> &b) {
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].published_sec, b[i].published_sec);
    EXPECT_EQ(std::memcmp(&a[i].wire_acceleration_mps2, &b[i].wire_acceleration_mps2, sizeof(double)), 0);
    EXPECT_EQ(std::memcmp(&a[i].wire_steering_rad, &b[i].wire_steering_rad, sizeof(double)), 0);
  }
}
} // namespace

TEST(MpccPublicationLedger, HistoryRetainsLegacyValuesOrderPruningAndClockFloor) {
  v::PublishedInputLedger ledger(256);
  std::vector<v::PublishedCommand> legacy;
  ASSERT_FALSE(ledger.snapshot());
  const auto append = [&](const v::PublishedCommand &packet, double before, double after) {
    ASSERT_TRUE(v::record_serialized_publication(legacy, packet, after, .1));
    ASSERT_TRUE(ledger.record(packet, before, after, .1, source()));
    same_history(legacy, ledger.history());
    const auto *event = ledger.latest_transaction();
    ASSERT_NE(event, nullptr);
    EXPECT_EQ(event->before_clock_sec, before); EXPECT_EQ(event->after_clock_sec, after);
    EXPECT_EQ(event->nominal.published_sec, packet.published_sec);
    EXPECT_EQ(event->published.published_sec, std::max(packet.published_sec, after));
  };
  append({1, 1, .125}, .99, .995);
  append({1, -3, .25}, 1, 1.01);
  append({1.01, 0, -.125}, 1.01, 1.01);
  append({1.01, -0.0, -.25}, 1.01, 1.01);
  ASSERT_EQ(ledger.history().size(), 4U);
  for (std::size_t i = 0; i < 40; ++i) {
    const double now = 1.025 + i * .025;
    append({now, -3, .125}, now, now);
  }
  EXPECT_LT(ledger.history().size(), 10U);
  const auto old = ledger.snapshot(); ASSERT_TRUE(old);
  append({.1, -3, .125}, .1, .11);
  EXPECT_FALSE(ledger.since(*old));
  EXPECT_FALSE(ledger.snapshot_ready());
  ledger.reset(); legacy.clear();
  append({.12, -3, .125}, .12, .12);
  EXPECT_TRUE(ledger.snapshot_ready());
  EXPECT_FALSE(ledger.since(*old));
}

TEST(MpccPublicationLedger, SnapshotSurvivesHistoryPruningButNeverLostTransactions) {
  v::PublishedInputLedger ledger(3);
  ASSERT_TRUE(ledger.record({0, 0, 0}, 0, 0, 1));
  ASSERT_TRUE(ledger.record({.5, 1, 0}, .5, .5, 1));
  ASSERT_TRUE(ledger.record({1, -3, 0}, 1, 1, 1));
  const auto snapshot = ledger.snapshot(); ASSERT_TRUE(snapshot);
  const auto original = snapshot->history();
  for (std::size_t i = 1; i <= 3; ++i) {
    const double now = 1 + i;
    ASSERT_TRUE(ledger.record({now, -3, 0}, now, now, .01));
    const auto events = ledger.since(*snapshot); ASSERT_TRUE(events);
    EXPECT_EQ(events->size(), i);
    EXPECT_EQ(events->front().sequence, snapshot->sequence() + 1);
    same_history(snapshot->history(), original);
  }
  EXPECT_EQ(ledger.history().size(), 2U);
  ASSERT_TRUE(ledger.record({5, -3, 0}, 5, 5, .01));
  EXPECT_EQ(ledger.transaction_count(), 3U);
  EXPECT_FALSE(ledger.since(*snapshot));
  const auto current = ledger.snapshot(); ASSERT_TRUE(current);
  const auto empty = ledger.since(*current); ASSERT_TRUE(empty); EXPECT_TRUE(empty->empty());
  v::PublishedInputLedger other(3);
  ASSERT_TRUE(other.record({5, -3, 0}, 5, 5, .01));
  EXPECT_FALSE(other.since(*current));
  ledger.reset();
  ASSERT_TRUE(ledger.record({5, -3, 0}, 5, 5, .01));
  EXPECT_FALSE(ledger.since(*current));
}

TEST(MpccPublicationLedger, RawClockResetIsNotConcealedByTheNominalFloor) {
  for (const bool within_publication : {false, true}) {
    v::PublishedInputLedger ledger(4);
    ASSERT_TRUE(ledger.record({1, 1, 0}, 1, 1, .5));
    const auto snapshot = ledger.snapshot(); ASSERT_TRUE(snapshot);
    const double before = within_publication ? 1.01 : .94;
    ASSERT_TRUE(ledger.record({1.05, -3, 0}, before, .95, .5));
    EXPECT_EQ(ledger.history().back().published_sec, 1.05);
    EXPECT_FALSE(ledger.snapshot()); EXPECT_FALSE(ledger.since(*snapshot));
    ASSERT_TRUE(ledger.record({1.05, -3, 0}, .96, .97, .5));
    EXPECT_FALSE(ledger.snapshot());
    ledger.reset();
    ASSERT_TRUE(ledger.record({.98, -3, 0}, .98, .98, .5));
    EXPECT_TRUE(ledger.snapshot()); EXPECT_FALSE(ledger.since(*snapshot));
  }
}

TEST(MpccPublicationLedger, FailedPostSendRecordInvalidatesPendingSnapshots) {
  for (std::size_t arm = 0; arm < 5; ++arm) {
    v::PublishedInputLedger ledger(4);
    ASSERT_TRUE(ledger.record({1, 1, 0}, 1, 1, .5));
    const auto snapshot = ledger.snapshot(); ASSERT_TRUE(snapshot);
    auto packet = v::PublishedCommand{1.025, -3, .125};
    double before = 1.025, after = 1.025;
    auto identity = source();
    if (arm == 0) packet.wire_acceleration_mps2 = NAN;
    if (arm == 1) before = NAN;
    if (arm == 2) after = INFINITY;
    if (arm == 3) packet.wire_steering_rad = .1; // not an actual serialized float
    if (arm == 4) identity.problem_fingerprint = 0;
    EXPECT_FALSE(ledger.record(packet, before, after, .5, identity));
    same_history(ledger.history(), snapshot->history());
    EXPECT_FALSE(ledger.snapshot()); EXPECT_FALSE(ledger.since(*snapshot));
  }
}

TEST(MpccPublicationLedger, IndependentClosedWindowsAndRepeatedTailMatchInOrder) {
  for (const bool integer_clock : {false, true}) {
    const auto scheduled = program(integer_clock);
    for (std::size_t arm = 0; arm < 18; ++arm) {
      v::PublishedInputLedger ledger(8);
      ASSERT_TRUE(ledger.record({1, 0, 0}, 1, 1, .5));
      const auto snapshot = ledger.snapshot(); ASSERT_TRUE(snapshot);
      std::vector<std::optional<v::PublishedProgramSource>> expected;
      double previous = 1;
      for (std::size_t i = 0; i < 5; ++i) {
        const double nominal = *v::publication_epoch(scheduled, i);
        const double latest = *v::publication_epoch(scheduled, i, true);
        const double phase = double((i + arm) % 3) / 2;
        double after = nominal + phase * scheduled.maximum_publication_delay_sec;
        if (integer_clock) {
          const auto &c = *scheduled.nanosecond_clock;
          after = static_cast<double>(c.first_ns + static_cast<std::int64_t>(i) * c.interval_ns +
            static_cast<std::int64_t>(phase * c.maximum_delay_ns)) / 1e9;
        }
        const double before = std::max(previous, arm < 9 ? nominal : after);
        after = std::min(latest, std::max(before, after));
        ASSERT_LE(before, after); ASSERT_LE(after, latest);
        auto packet = scheduled.commands[std::min(i, scheduled.commands.size() - 1)];
        packet.published_sec = nominal;
        // A composite planned prefix can refer to later indices of an old
        // certificate; its own new programme indices still begin at zero.
        expected.emplace_back(source(7 + i));
        ASSERT_TRUE(ledger.record(packet, before, after, .5, expected.back()));
        EXPECT_TRUE(ledger.matches_prefix(*snapshot, scheduled, expected)) << arm << '/' << i;
        previous = after;
      }
    }
  }
}

TEST(MpccPublicationLedger, MissingExtraReorderedOrDifferentPacketsCannotMatch) {
  const auto scheduled = program();
  for (std::size_t arm = 0; arm < 4; ++arm) {
    v::PublishedInputLedger ledger(8);
    ASSERT_TRUE(ledger.record({1, 0, 0}, 1, 1, .5));
    const auto snapshot = ledger.snapshot(); ASSERT_TRUE(snapshot);
    EXPECT_FALSE(ledger.matches_prefix(*snapshot, scheduled, {source()}));
    auto first = scheduled.commands[arm == 2 ? 1 : 0];
    if (arm == 3) first.wire_steering_rad = -.125;
    ASSERT_TRUE(ledger.record(first, first.published_sec, first.published_sec, .5, source()));
    if (arm == 0) {
      EXPECT_TRUE(ledger.matches_prefix(*snapshot, scheduled, {source()}));
      EXPECT_FALSE(ledger.matches_prefix(*snapshot, scheduled, {source(), source(1)}));
    } else if (arm == 1) {
      auto extra = first; extra.wire_acceleration_mps2 = -3;
      ASSERT_TRUE(ledger.record(extra, first.published_sec, first.published_sec, .5, source(1)));
      EXPECT_FALSE(ledger.matches_prefix(*snapshot, scheduled, {source()}));
      EXPECT_FALSE(ledger.matches_prefix(*snapshot, scheduled, {source(), source(1)}));
    } else {
      EXPECT_FALSE(ledger.matches_prefix(*snapshot, scheduled, {source()}));
    }
  }
}

TEST(MpccPublicationLedger, SourceAndSignedSerializedValueRemainBound) {
  const auto scheduled = program();
  v::PublishedInputLedger ledger(4);
  ASSERT_TRUE(ledger.record({1, 0, 0}, 1, 1, .5));
  const auto snapshot = ledger.snapshot(); ASSERT_TRUE(snapshot);
  ASSERT_TRUE(ledger.record(scheduled.commands.front(), 1.025, 1.025, .5, source()));
  EXPECT_FALSE(ledger.matches_prefix(*snapshot, scheduled, {std::nullopt}));
  for (std::size_t arm = 0; arm < 5; ++arm) {
    auto changed = source();
    if (arm == 0) ++changed.decision_id;
    if (arm == 1) ++changed.solution_id;
    if (arm == 2) ++changed.problem_fingerprint;
    if (arm == 3) ++changed.input_context_fingerprint;
    if (arm == 4) ++changed.packet_index;
    EXPECT_FALSE(ledger.matches_prefix(*snapshot, scheduled, {changed}));
  }
  v::PublishedInputLedger unbound(4);
  ASSERT_TRUE(unbound.record({1, 0, 0}, 1, 1, .5));
  const auto origin = unbound.snapshot(); ASSERT_TRUE(origin);
  auto stop = scheduled; stop.commands.front().wire_acceleration_mps2 = -0.0;
  ASSERT_TRUE(unbound.record(stop.commands.front(), 1.025, 1.025, .5));
  EXPECT_TRUE(unbound.matches_prefix(*origin, stop, {std::nullopt}));
  EXPECT_FALSE(unbound.matches_prefix(*origin, stop, {source()}));
  stop.commands.front().wire_acceleration_mps2 = 0.0;
  EXPECT_FALSE(unbound.matches_prefix(*origin, stop, {std::nullopt}));
}

TEST(MpccPublicationLedger, EarlyLateResetAndNonGridPublicationAreRejected) {
  const auto scheduled = program();
  for (const auto &clocks : std::vector<std::pair<double, double>>{
      {1.024999999, 1.025}, {1.05, 1.050000001}, {1.03, 1.025},
      {std::nextafter(1.025, INFINITY), 1.03}}) {
    v::PublishedInputLedger ledger(4);
    ASSERT_TRUE(ledger.record({1, 0, 0}, 1, 1, .5));
    const auto snapshot = ledger.snapshot(); ASSERT_TRUE(snapshot);
    ASSERT_TRUE(ledger.record(scheduled.commands.front(), clocks.first, clocks.second, .5, source()));
    EXPECT_FALSE(ledger.matches_prefix(*snapshot, scheduled, {source()}));
  }
  auto finite_word = scheduled; finite_word.repeat_last_until_rest = false;
  EXPECT_FALSE(v::scheduled_publication_bracket_admitted(finite_word, 2, 1, 1.075, 1.075));
  EXPECT_TRUE(v::scheduled_publication_bracket_admitted(scheduled, 2, 1, 1.075, 1.1));
  EXPECT_FALSE(v::scheduled_publication_bracket_admitted(scheduled, 2, 1, 1.075, 1.100000001));
}

TEST(MpccPublicationLedger, CompositePrefixKeepsEveryOldPacketClockAndSourceIndex) {
  const auto prior = program();
  auto successor = prior;
  successor.nanosecond_clock->first_ns = 1125000000;
  successor.commands[0] = {1.125, 1, -.125};
  successor.commands[1] = {1.15, -3, -.125};
  const auto joined = v::prepend_publication_prefix(prior, 1, 3, successor);
  ASSERT_TRUE(joined); ASSERT_EQ(joined->commands.size(), 5U);
  EXPECT_EQ(joined->nanosecond_clock->first_ns, 1050000000);
  EXPECT_EQ(joined->publication_interval_sec, prior.publication_interval_sec);
  EXPECT_EQ(joined->maximum_publication_delay_sec, prior.maximum_publication_delay_sec);
  EXPECT_TRUE(joined->repeat_last_until_rest);
  v::PublishedInputLedger ledger(8);
  ASSERT_TRUE(ledger.record({1, 0, 0}, 1, 1, .5));
  const auto cursor = ledger.snapshot(); ASSERT_TRUE(cursor);
  std::vector<std::optional<v::PublishedProgramSource>> sources;
  for (std::size_t i = 0; i < joined->commands.size(); ++i) {
    const auto &packet = joined->commands[i];
    const auto &original = i < 3 ? prior.commands.back() : successor.commands[i - 3];
    EXPECT_EQ(std::memcmp(&packet.wire_acceleration_mps2, &original.wire_acceleration_mps2, sizeof(double)), 0);
    EXPECT_EQ(std::memcmp(&packet.wire_steering_rad, &original.wire_steering_rad, sizeof(double)), 0);
    EXPECT_EQ(packet.published_sec, i < 3 ? *v::publication_epoch(prior, i + 1) : original.published_sec);
    sources.push_back(i < 3 ? source(i + 1) : v::PublishedProgramSource{12, 22, 32, 42, i - 3});
    const double latest = *v::publication_epoch(*joined, i, true);
    ASSERT_TRUE(ledger.record(packet, packet.published_sec, latest, .5, sources.back()));
    EXPECT_TRUE(ledger.matches_prefix(*cursor, *joined, sources));
  }
  // Identical wire packets from a different authority cannot conceal a change
  // in which certificate owned the committed waiting prefix.
  sources[1] = v::PublishedProgramSource{12, 22, 32, 42, 0};
  EXPECT_FALSE(ledger.matches_prefix(*cursor, *joined, sources));
}

TEST(MpccPublicationLedger, CompositionRejectsClockChangesGapsAndUndeclaredTail) {
  const auto prior = program();
  auto successor = prior;
  successor.nanosecond_clock->first_ns = 1075000000;
  successor.commands[0].published_sec = 1.075;
  successor.commands[1].published_sec = 1.1;
  ASSERT_TRUE(v::prepend_publication_prefix(prior, 1, 1, successor));
  for (std::size_t arm = 0; arm < 6; ++arm) {
    auto changed = successor;
    if (arm == 0) {
      ++changed.nanosecond_clock->first_ns;
      for (std::size_t i = 0; i < changed.commands.size(); ++i)
        changed.commands[i].published_sec = *v::publication_epoch(changed, i);
    }
    if (arm == 1) changed.nanosecond_clock.reset();
    if (arm == 2) {
      changed.maximum_publication_delay_sec = .02;
      changed.nanosecond_clock->maximum_delay_ns = 20000000;
    }
    if (arm == 3) {
      changed.publication_interval_sec = .02;
      changed.maximum_publication_delay_sec = .02;
      changed.nanosecond_clock->interval_ns = 20000000;
      changed.nanosecond_clock->maximum_delay_ns = 20000000;
      changed.commands[1].published_sec = 1.095;
    }
    if (arm == 4) changed.commands[1].wire_acceleration_mps2 = NAN;
    if (arm == 5) changed.commands.clear();
    EXPECT_FALSE(v::prepend_publication_prefix(prior, 1, 1, changed)) << arm;
  }
  auto finite = prior; finite.repeat_last_until_rest = false;
  EXPECT_FALSE(v::prepend_publication_prefix(finite, 1, 2, successor));
  EXPECT_FALSE(v::prepend_publication_prefix(finite, 3, 0, successor));
  EXPECT_FALSE(v::prepend_publication_prefix(prior, std::numeric_limits<std::size_t>::max(), 1, successor));
  EXPECT_FALSE(v::prepend_publication_prefix(prior, 1, std::numeric_limits<std::size_t>::max(), successor));
  EXPECT_FALSE(v::prepend_publication_prefix(prior, 1, 9999, successor));
  // A zero-length prefix still owns the handoff phase; it cannot move a word.
  const auto zero = v::prepend_publication_prefix(finite, 2, 0, successor);
  ASSERT_TRUE(zero); same_history(zero->commands, successor.commands);
  EXPECT_FALSE(v::prepend_publication_prefix(finite, 1, 0, successor));
}

TEST(MpccPublicationLedger, ContinuousCompositionKeepsExactEpochsAndSignedWireBits) {
  const v::PublishedInputProgram prior{.125, {{1, 1, .125}, {1.125, -3, -0.0}}, true, .125};
  const v::PublishedInputProgram successor{.125, {{1.5, 1, .25}, {1.625, -3, .25}}, true, .125};
  const auto joined = v::prepend_publication_prefix(prior, 1, 3, successor);
  ASSERT_TRUE(joined); EXPECT_FALSE(joined->nanosecond_clock);
  EXPECT_EQ(joined->commands.front().published_sec, 1.125);
  EXPECT_EQ(joined->commands[2].published_sec, 1.375);
  EXPECT_TRUE(std::signbit(joined->commands[2].wire_steering_rad));
  auto shifted = successor;
  shifted.commands[0].published_sec = std::nextafter(1.5, INFINITY);
  shifted.commands[1].published_sec = shifted.commands[0].published_sec + .125;
  EXPECT_FALSE(v::prepend_publication_prefix(prior, 1, 3, shifted));
}
