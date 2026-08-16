#include <multi_purpose_mpc_ros/latest_only_worker.hpp>

#include <utility>

namespace multi_purpose_mpc_ros
{

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
