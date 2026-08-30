#pragma once

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

namespace multi_purpose_mpc_ros
{

/// Persistent one-running-job executor. It deliberately has no pending queue:
/// a caller either owns the one exact ticket or receives Busy. Waiting belongs
/// to a background producer and must never be performed by the control callback.
class BoundedSingleJobExecutor
{
public:
  using Job = std::function<void()>;

  enum class SubmitReason
  {
    Accepted,
    InvalidJob,
    Busy,
    Stopped,
  };

  enum class WaitReason
  {
    Completed,
    JobFailed,
    InvalidTicket,
    Stopped,
  };

  struct Ticket
  {
    std::uint64_t value{};

    bool valid() const noexcept {return value > 0U;}
  };

  struct SubmitResult
  {
    SubmitReason reason{SubmitReason::InvalidJob};
    Ticket ticket;

    bool accepted() const noexcept
    {
      return reason == SubmitReason::Accepted && ticket.valid();
    }
  };

  struct Stats
  {
    std::uint64_t submitted{};
    std::uint64_t completed{};
    std::uint64_t failed{};
    std::uint64_t busy_rejected{};
    bool running{false};
    bool pending{false};
    bool stopped{false};
  };

  BoundedSingleJobExecutor();
  ~BoundedSingleJobExecutor();

  BoundedSingleJobExecutor(const BoundedSingleJobExecutor &) = delete;
  BoundedSingleJobExecutor & operator=(const BoundedSingleJobExecutor &) = delete;

  SubmitResult submit(Job job);
  WaitReason wait(Ticket ticket);
  Stats stats() const;
  void stop() noexcept;

private:
  struct PendingJob
  {
    Ticket ticket;
    Job job;
  };

  void run() noexcept;

  mutable std::mutex mutex_;
  std::condition_variable work_condition_;
  std::condition_variable completion_condition_;
  std::optional<PendingJob> pending_job_;
  std::thread thread_;
  std::uint64_t next_ticket_{1U};
  std::uint64_t active_ticket_{};
  std::uint64_t last_completed_ticket_{};
  bool last_completed_failed_{false};
  bool stop_requested_{false};
  Stats stats_;
};

const char * to_string(BoundedSingleJobExecutor::SubmitReason reason) noexcept;
const char * to_string(BoundedSingleJobExecutor::WaitReason reason) noexcept;

}  // namespace multi_purpose_mpc_ros
