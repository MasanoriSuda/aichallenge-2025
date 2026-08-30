#include "multi_purpose_mpc_ros/bounded_single_job_executor.hpp"

#include <gtest/gtest.h>

#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace
{

using Executor = multi_purpose_mpc_ros::BoundedSingleJobExecutor;

TEST(BoundedSingleJobExecutor, ReusesOnePersistentWorkerThread)
{
  Executor executor;
  std::thread::id first_thread;
  std::thread::id second_thread;

  const auto first = executor.submit([&]() {
      first_thread = std::this_thread::get_id();
    });
  ASSERT_TRUE(first.accepted());
  EXPECT_EQ(executor.wait(first.ticket), Executor::WaitReason::Completed);

  const auto second = executor.submit([&]() {
      second_thread = std::this_thread::get_id();
    });
  ASSERT_TRUE(second.accepted());
  EXPECT_EQ(executor.wait(second.ticket), Executor::WaitReason::Completed);
  EXPECT_NE(first_thread, std::this_thread::get_id());
  EXPECT_EQ(first_thread, second_thread);
  const auto stats = executor.stats();
  EXPECT_EQ(stats.submitted, 2U);
  EXPECT_EQ(stats.completed, 2U);
  EXPECT_EQ(stats.failed, 0U);
}

TEST(BoundedSingleJobExecutor, RejectsConcurrentSubmissionWithoutQueueing)
{
  Executor executor;
  std::mutex mutex;
  std::condition_variable condition;
  bool release = false;
  const auto first = executor.submit([&]() {
      std::unique_lock<std::mutex> lock(mutex);
      condition.wait(lock, [&]() {return release;});
    });
  ASSERT_TRUE(first.accepted());

  const auto second = executor.submit([]() {});
  EXPECT_EQ(second.reason, Executor::SubmitReason::Busy);
  EXPECT_FALSE(second.ticket.valid());
  {
    std::lock_guard<std::mutex> lock(mutex);
    release = true;
  }
  condition.notify_one();
  EXPECT_EQ(executor.wait(first.ticket), Executor::WaitReason::Completed);
  EXPECT_EQ(executor.stats().busy_rejected, 1U);
}

TEST(BoundedSingleJobExecutor, ReportsJobFailureAndInvalidTicket)
{
  Executor executor;
  EXPECT_EQ(executor.wait({}), Executor::WaitReason::InvalidTicket);
  const auto submission = executor.submit([]() {
      throw std::runtime_error("test failure");
    });
  ASSERT_TRUE(submission.accepted());
  EXPECT_EQ(executor.wait(submission.ticket), Executor::WaitReason::JobFailed);
  EXPECT_EQ(executor.stats().failed, 1U);
}

TEST(BoundedSingleJobExecutor, RejectsSubmissionAfterStop)
{
  Executor executor;
  executor.stop();
  const auto submission = executor.submit([]() {});
  EXPECT_EQ(submission.reason, Executor::SubmitReason::Stopped);
  EXPECT_TRUE(executor.stats().stopped);
}

}  // namespace
