#include <gtest/gtest.h>

#include <multi_purpose_mpc_ros/latest_only_worker.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace
{

using namespace std::chrono_literals;

TEST(LatestOnlyWorkerInterval, UsesBaseIntervalUntilComputeCostIsAvailable)
{
  EXPECT_DOUBLE_EQ(
    multi_purpose_mpc_ros::resolve_latest_only_worker_interval(
      {true, 0.20, 0.30, 0.35, 0.0}),
    0.20);
}

TEST(LatestOnlyWorkerInterval, PreservesBaseIntervalWhenLoadSheddingIsDisabled)
{
  EXPECT_DOUBLE_EQ(
    multi_purpose_mpc_ros::resolve_latest_only_worker_interval(
      {false, 0.20, 0.30, 0.35, 150.0}),
    0.20);
}

TEST(LatestOnlyWorkerInterval, ExtendsIntervalToRespectComputeBudget)
{
  EXPECT_NEAR(
    multi_purpose_mpc_ros::resolve_latest_only_worker_interval(
      {true, 0.20, 0.30, 0.35, 84.0}),
    0.24, 1.0e-12);
}

TEST(LatestOnlyWorkerInterval, BoundsIntervalAtConfiguredMaximum)
{
  EXPECT_DOUBLE_EQ(
    multi_purpose_mpc_ros::resolve_latest_only_worker_interval(
      {true, 0.20, 0.30, 0.35, 150.0}),
    0.30);
}

TEST(LatestOnlyWorkerResultPublication, PublishesCompletedResultWithNewerJobQueued)
{
  EXPECT_TRUE(
    multi_purpose_mpc_ros::should_publish_latest_only_result(
      {7U, 7U, 10U, 11U, 9U}));
}

TEST(LatestOnlyWorkerResultPublication, RejectsOldContextAndSequenceRollback)
{
  EXPECT_FALSE(
    multi_purpose_mpc_ros::should_publish_latest_only_result(
      {6U, 7U, 10U, 11U, 9U}));
  EXPECT_FALSE(
    multi_purpose_mpc_ros::should_publish_latest_only_result(
      {7U, 7U, 9U, 11U, 9U}));
}

TEST(LatestOnlyWorkerResultPublication, RejectsResultThatWasNeverSubmitted)
{
  EXPECT_FALSE(
    multi_purpose_mpc_ros::should_publish_latest_only_result(
      {7U, 7U, 12U, 11U, 9U}));
}

TEST(LatestOnlyWorkerOwnership, DefersOrdinaryLiveTacticalGeneration)
{
  EXPECT_TRUE(
    multi_purpose_mpc_ros::defer_live_tactical_generation(true, false));
}

TEST(LatestOnlyWorkerOwnership, KeepsCompleteGenerationInsideWorkerSnapshot)
{
  EXPECT_FALSE(
    multi_purpose_mpc_ros::defer_live_tactical_generation(true, true));
}

TEST(LatestOnlyWorkerOwnership, KeepsSynchronousFallbackWhenWorkerIsDisabled)
{
  EXPECT_FALSE(
    multi_purpose_mpc_ros::defer_live_tactical_generation(false, false));
}

TEST(LatestOnlyWorkerOwnership, DefersStartGridTacticalGenerationToo)
{
  EXPECT_TRUE(
    multi_purpose_mpc_ros::defer_live_tactical_generation(true, false));
}

TEST(LatestOnlyWorker, ReplacesPendingJobWithoutWaitingForRunningJob)
{
  multi_purpose_mpc_ros::LatestOnlyWorker worker;
  std::mutex mutex;
  std::condition_variable condition;
  bool first_started = false;
  bool release_first = false;
  std::atomic<int> last_value{0};

  ASSERT_TRUE(worker.submit_latest([&]() {
    std::unique_lock<std::mutex> lock(mutex);
    first_started = true;
    condition.notify_all();
    condition.wait(lock, [&]() {return release_first;});
    last_value.store(1);
  }).accepted);

  bool observed_first_start = false;
  {
    std::unique_lock<std::mutex> lock(mutex);
    observed_first_start = condition.wait_for(
      lock, 5s, [&]() {return first_started;});
    // A failed startup assertion must not leave the non-cancelable test job
    // blocked while the worker destructor joins its thread.
    if (!observed_first_start) {
      release_first = true;
    }
  }
  condition.notify_all();
  ASSERT_TRUE(observed_first_start);
  ASSERT_TRUE(worker.stats().running);

  ASSERT_TRUE(worker.submit_latest([&]() {last_value.store(2);}).accepted);
  const auto replacement = worker.submit_latest([&]() {last_value.store(3);});
  EXPECT_TRUE(replacement.accepted);
  EXPECT_TRUE(replacement.replaced_pending);

  {
    std::lock_guard<std::mutex> lock(mutex);
    release_first = true;
  }
  condition.notify_one();

  for (int attempt = 0; attempt < 200 && worker.stats().completed < 2U; ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  const auto stats = worker.stats();
  EXPECT_EQ(stats.submitted, 3U);
  EXPECT_EQ(stats.replaced, 1U);
  EXPECT_EQ(stats.started, 2U);
  EXPECT_EQ(stats.completed, 2U);
  EXPECT_EQ(last_value.load(), 3);
}

TEST(LatestOnlyWorker, ContainsJobExceptionAndContinues)
{
  multi_purpose_mpc_ros::LatestOnlyWorker worker;
  std::atomic<bool> second_ran{false};
  ASSERT_TRUE(worker.submit_latest([]() {throw std::runtime_error("test");}).accepted);
  for (int attempt = 0; attempt < 200 && worker.stats().completed < 1U; ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(worker.submit_latest([&]() {second_ran.store(true);}).accepted);
  for (int attempt = 0; attempt < 200 && !second_ran.load(); ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_TRUE(second_ran.load());
  EXPECT_EQ(worker.stats().exceptions, 1U);
}

TEST(LatestOnlyWorker, CancelableJobObservesNewerAcceptedGeneration)
{
  multi_purpose_mpc_ros::LatestOnlyWorker worker;
  std::atomic<bool> first_started{false};
  std::atomic<bool> first_superseded{false};
  std::atomic<bool> second_ran{false};

  ASSERT_TRUE(worker.submit_latest_cancelable(
    [&](const multi_purpose_mpc_ros::LatestOnlyWorker::SupersessionToken & token) {
      first_started.store(true);
      while (!token.superseded()) {
        std::this_thread::yield();
      }
      first_superseded.store(true);
    }).accepted);
  for (int attempt = 0; attempt < 200 && !first_started.load(); ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(first_started.load());

  ASSERT_TRUE(worker.submit_latest_cancelable(
    [&](const multi_purpose_mpc_ros::LatestOnlyWorker::SupersessionToken & token) {
      EXPECT_FALSE(token.superseded());
      second_ran.store(true);
    }).accepted);
  for (int attempt = 0; attempt < 200 && !second_ran.load(); ++attempt) {
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_TRUE(first_superseded.load());
  EXPECT_TRUE(second_ran.load());
  const auto stats = worker.stats();
  EXPECT_EQ(stats.submitted, 2U);
  EXPECT_EQ(stats.started, 2U);
  EXPECT_EQ(stats.completed, 2U);
}

TEST(LatestOnlyWorker, LifecycleInvalidationSupersedesRunningWithoutNewJob)
{
  multi_purpose_mpc_ros::LatestOnlyWorker worker;
  std::atomic<bool> started{false};
  std::atomic<bool> superseded{false};
  ASSERT_TRUE(worker.submit_latest_cancelable(
    [&](const multi_purpose_mpc_ros::LatestOnlyWorker::SupersessionToken & token) {
      started.store(true);
      while (!token.superseded()) {
        std::this_thread::yield();
      }
      superseded.store(true);
    }).accepted);
  for (int attempt = 0; attempt < 200 && !started.load(); ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(started.load());

  const auto invalidation = worker.invalidate_pending_and_running();
  EXPECT_TRUE(invalidation.invalidated_running);
  EXPECT_FALSE(invalidation.discarded_pending);
  for (int attempt = 0; attempt < 200 && !superseded.load(); ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_TRUE(superseded.load());
  EXPECT_EQ(worker.stats().invalidated_running, 1U);
}

TEST(LatestOnlyWorker, LifecycleInvalidationDiscardsPendingJob)
{
  multi_purpose_mpc_ros::LatestOnlyWorker worker;
  std::mutex mutex;
  std::condition_variable condition;
  bool release_running = false;
  std::atomic<bool> pending_ran{false};
  ASSERT_TRUE(worker.submit_latest([&]() {
    std::unique_lock<std::mutex> lock(mutex);
    condition.wait(lock, [&]() {return release_running;});
  }).accepted);
  for (int attempt = 0; attempt < 200 && !worker.stats().running; ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(worker.stats().running);
  ASSERT_TRUE(worker.submit_latest([&]() {pending_ran.store(true);}).accepted);

  const auto invalidation = worker.invalidate_pending_and_running();
  EXPECT_FALSE(invalidation.invalidated_running);
  EXPECT_TRUE(invalidation.discarded_pending);
  {
    std::lock_guard<std::mutex> lock(mutex);
    release_running = true;
  }
  condition.notify_one();
  for (int attempt = 0; attempt < 200 && worker.stats().running; ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_FALSE(pending_ran.load());
  EXPECT_EQ(worker.stats().discarded_pending, 1U);
  EXPECT_EQ(worker.stats().completed, 1U);
}

}  // namespace
