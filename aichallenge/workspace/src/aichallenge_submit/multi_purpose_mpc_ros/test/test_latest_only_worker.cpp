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

TEST(LatestOnlyWorkerOwnership, DefersOrdinaryLiveTacticalGeneration)
{
  EXPECT_TRUE(
    multi_purpose_mpc_ros::defer_live_tactical_generation(true, false, false));
}

TEST(LatestOnlyWorkerOwnership, KeepsCompleteGenerationInsideWorkerSnapshot)
{
  EXPECT_FALSE(
    multi_purpose_mpc_ros::defer_live_tactical_generation(true, true, false));
}

TEST(LatestOnlyWorkerOwnership, KeepsSynchronousFallbackWhenWorkerIsDisabled)
{
  EXPECT_FALSE(
    multi_purpose_mpc_ros::defer_live_tactical_generation(false, false, false));
}

TEST(LatestOnlyWorkerOwnership, KeepsDedicatedStartGridAssessmentLive)
{
  EXPECT_FALSE(
    multi_purpose_mpc_ros::defer_live_tactical_generation(true, false, true));
}

TEST(LatestOnlyWorker, ReplacesPendingJobWithoutWaitingForRunningJob)
{
  multi_purpose_mpc_ros::LatestOnlyWorker worker;
  std::mutex mutex;
  std::condition_variable condition;
  bool release_first = false;
  std::atomic<int> last_value{0};

  ASSERT_TRUE(worker.submit_latest([&]() {
    std::unique_lock<std::mutex> lock(mutex);
    condition.wait(lock, [&]() {return release_first;});
    last_value.store(1);
  }).accepted);

  for (int attempt = 0; attempt < 100 && !worker.stats().running; ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
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

}  // namespace
