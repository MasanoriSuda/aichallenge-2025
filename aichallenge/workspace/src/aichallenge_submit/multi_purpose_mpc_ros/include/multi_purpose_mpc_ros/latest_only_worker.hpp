#pragma once

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

namespace multi_purpose_mpc_ros
{

struct LatestOnlyWorkerIntervalRequest
{
  bool load_shedding_enabled{false};
  double base_interval_sec{0.20};
  double maximum_interval_sec{0.30};
  double target_worker_utilization{0.35};
  double last_compute_ms{0.0};
};

/// Resolve a bounded submission interval from the most recent worker cost.
/// This does not delay the control callback; it only controls when the next
/// latest-only snapshot may be queued.
double resolve_latest_only_worker_interval(
  const LatestOnlyWorkerIntervalRequest & request);

/// One-running/one-pending worker for receding-horizon planning. Submitting a
/// newer job replaces only the pending job and never waits for the running job.
class LatestOnlyWorker
{
public:
  using Job = std::function<void()>;

  struct SubmitResult
  {
    bool accepted{false};
    bool replaced_pending{false};
  };

  struct Stats
  {
    std::uint64_t submitted{0U};
    std::uint64_t replaced{0U};
    std::uint64_t started{0U};
    std::uint64_t completed{0U};
    std::uint64_t exceptions{0U};
    bool running{false};
    bool pending{false};
  };

  LatestOnlyWorker();
  ~LatestOnlyWorker();

  LatestOnlyWorker(const LatestOnlyWorker &) = delete;
  LatestOnlyWorker & operator=(const LatestOnlyWorker &) = delete;

  SubmitResult submit_latest(Job job);
  Stats stats() const;
  void stop() noexcept;

private:
  void run() noexcept;

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<Job> pending_job_;
  Stats stats_;
  bool stop_requested_{false};
  std::thread thread_;
};

}  // namespace multi_purpose_mpc_ros
