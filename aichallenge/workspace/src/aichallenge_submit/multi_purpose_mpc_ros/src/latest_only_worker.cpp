#include <multi_purpose_mpc_ros/latest_only_worker.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace multi_purpose_mpc_ros
{

double resolve_latest_only_worker_interval(
  const LatestOnlyWorkerIntervalRequest & request)
{
  constexpr double kMinimumIntervalSec = 1.0e-3;
  constexpr double kMinimumUtilization = 1.0e-3;
  const double base_interval_sec =
    std::isfinite(request.base_interval_sec) ?
    std::max(kMinimumIntervalSec, request.base_interval_sec) : 0.20;
  const double maximum_interval_sec =
    std::isfinite(request.maximum_interval_sec) ?
    std::max(base_interval_sec, request.maximum_interval_sec) : base_interval_sec;
  if (
    !request.load_shedding_enabled || !std::isfinite(request.last_compute_ms) ||
    request.last_compute_ms <= 0.0)
  {
    return base_interval_sec;
  }
  const double target_worker_utilization =
    std::isfinite(request.target_worker_utilization) ?
    std::clamp(request.target_worker_utilization, kMinimumUtilization, 1.0) : 1.0;
  const double compute_budget_interval_sec =
    request.last_compute_ms * 1.0e-3 / target_worker_utilization;
  return std::clamp(
    std::max(base_interval_sec, compute_budget_interval_sec),
    base_interval_sec, maximum_interval_sec);
}

bool defer_live_tactical_generation(
  const bool async_worker_enabled, const bool worker_context,
  const bool start_grid_breakout_attempt)
{
  return async_worker_enabled && !worker_context &&
         !start_grid_breakout_attempt;
}

LatestOnlyWorker::LatestOnlyWorker()
: thread_([this]() {run();})
{
}

LatestOnlyWorker::~LatestOnlyWorker()
{
  stop();
}

LatestOnlyWorker::SubmitResult LatestOnlyWorker::submit_latest(Job job)
{
  if (!job) {
    return {};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (stop_requested_) {
    return {};
  }
  const bool replaced = pending_job_.has_value();
  pending_job_ = std::move(job);
  ++stats_.submitted;
  if (replaced) {
    ++stats_.replaced;
  }
  stats_.pending = true;
  condition_.notify_one();
  return {true, replaced};
}

LatestOnlyWorker::Stats LatestOnlyWorker::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void LatestOnlyWorker::stop() noexcept
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_requested_) {
      return;
    }
    stop_requested_ = true;
    pending_job_.reset();
    stats_.pending = false;
  }
  condition_.notify_one();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void LatestOnlyWorker::run() noexcept
{
  while (true) {
    Job job;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this]() {
        return stop_requested_ || pending_job_.has_value();
      });
      if (stop_requested_) {
        return;
      }
      job = std::move(pending_job_.value());
      pending_job_.reset();
      stats_.pending = false;
      stats_.running = true;
      ++stats_.started;
    }

    bool failed = false;
    try {
      job();
    } catch (...) {
      failed = true;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.running = false;
      ++stats_.completed;
      if (failed) {
        ++stats_.exceptions;
      }
    }
  }
}

}  // namespace multi_purpose_mpc_ros
