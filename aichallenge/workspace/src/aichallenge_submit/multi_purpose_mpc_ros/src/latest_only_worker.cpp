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

bool should_publish_latest_only_result(
  const LatestOnlyResultPublicationRequest & request) noexcept
{
  return
    request.result_context_epoch == request.active_context_epoch &&
    request.result_sequence > request.latest_published_sequence &&
    request.result_sequence <= request.latest_submitted_sequence;
}

bool defer_live_tactical_generation(
  const bool async_worker_enabled, const bool worker_context)
{
  return async_worker_enabled && !worker_context;
}

LatestOnlyWorker::LatestOnlyWorker()
: latest_generation_(std::make_shared<std::atomic<std::uint64_t>>(0U)),
  thread_([this]() {run();})
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
  return submit_latest_cancelable(
    [job = std::move(job)](const SupersessionToken &) mutable {job();});
}

LatestOnlyWorker::SupersessionToken::SupersessionToken(
  std::shared_ptr<const std::atomic<std::uint64_t>> latest_generation,
  const std::uint64_t generation) noexcept
: latest_generation_(std::move(latest_generation)), generation_(generation)
{
}

bool LatestOnlyWorker::SupersessionToken::superseded() const noexcept
{
  return
    latest_generation_ == nullptr ||
    latest_generation_->load(std::memory_order_acquire) != generation_;
}

LatestOnlyWorker::SubmitResult LatestOnlyWorker::submit_latest_cancelable(
  CancelableJob job)
{
  if (!job) {
    return {};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (stop_requested_) {
    return {};
  }
  const bool replaced = pending_job_.has_value();
  const auto generation =
    latest_generation_->fetch_add(1U, std::memory_order_acq_rel) + 1U;
  pending_job_ = PendingJob{std::move(job), generation};
  ++stats_.submitted;
  if (replaced) {
    ++stats_.replaced;
  }
  stats_.pending = true;
  condition_.notify_one();
  return {true, replaced};
}

LatestOnlyWorker::InvalidateResult
LatestOnlyWorker::invalidate_pending_and_running() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (stop_requested_) {
    return {};
  }
  const auto latest_generation =
    latest_generation_->load(std::memory_order_acquire);
  const bool invalidate_running =
    stats_.running && running_generation_ == latest_generation;
  const bool discard_pending = pending_job_.has_value();
  if (!invalidate_running && !discard_pending) {
    return {};
  }
  if (invalidate_running) {
    static_cast<void>(
      latest_generation_->fetch_add(1U, std::memory_order_acq_rel));
    ++stats_.invalidated_running;
  }
  if (discard_pending) {
    pending_job_.reset();
    stats_.pending = false;
    ++stats_.discarded_pending;
  }
  return {invalidate_running, discard_pending};
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
    static_cast<void>(
      latest_generation_->fetch_add(1U, std::memory_order_acq_rel));
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
    PendingJob job;
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
      running_generation_ = job.generation;
      ++stats_.started;
    }

    bool failed = false;
    try {
      job.function(SupersessionToken{latest_generation_, job.generation});
    } catch (...) {
      failed = true;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.running = false;
      running_generation_ = 0U;
      ++stats_.completed;
      if (failed) {
        ++stats_.exceptions;
      }
    }
  }
}

}  // namespace multi_purpose_mpc_ros
