#include <multi_purpose_mpc_ros/bounded_single_job_executor.hpp>

#include <utility>

namespace multi_purpose_mpc_ros
{

const char * to_string(
  const BoundedSingleJobExecutor::SubmitReason reason) noexcept
{
  switch (reason) {
    case BoundedSingleJobExecutor::SubmitReason::Accepted: return "accepted";
    case BoundedSingleJobExecutor::SubmitReason::InvalidJob: return "invalid-job";
    case BoundedSingleJobExecutor::SubmitReason::Busy: return "busy";
    case BoundedSingleJobExecutor::SubmitReason::Stopped: return "stopped";
  }
  return "unknown";
}

const char * to_string(
  const BoundedSingleJobExecutor::WaitReason reason) noexcept
{
  switch (reason) {
    case BoundedSingleJobExecutor::WaitReason::Completed: return "completed";
    case BoundedSingleJobExecutor::WaitReason::JobFailed: return "job-failed";
    case BoundedSingleJobExecutor::WaitReason::InvalidTicket: return "invalid-ticket";
    case BoundedSingleJobExecutor::WaitReason::Stopped: return "stopped";
  }
  return "unknown";
}

BoundedSingleJobExecutor::BoundedSingleJobExecutor()
: thread_([this]() {run();})
{
}

BoundedSingleJobExecutor::~BoundedSingleJobExecutor()
{
  stop();
}

BoundedSingleJobExecutor::SubmitResult BoundedSingleJobExecutor::submit(Job job)
{
  if (!job) {
    return {SubmitReason::InvalidJob, {}};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (stop_requested_) {
    return {SubmitReason::Stopped, {}};
  }
  if (pending_job_.has_value() || active_ticket_ != 0U) {
    ++stats_.busy_rejected;
    return {SubmitReason::Busy, {}};
  }
  const Ticket ticket{next_ticket_++};
  pending_job_ = PendingJob{ticket, std::move(job)};
  ++stats_.submitted;
  stats_.pending = true;
  work_condition_.notify_one();
  return {SubmitReason::Accepted, ticket};
}

BoundedSingleJobExecutor::WaitReason BoundedSingleJobExecutor::wait(
  const Ticket ticket)
{
  if (!ticket.valid()) {
    return WaitReason::InvalidTicket;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  if (ticket.value >= next_ticket_) {
    return WaitReason::InvalidTicket;
  }
  completion_condition_.wait(lock, [this, ticket]() {
    return last_completed_ticket_ >= ticket.value ||
           (stop_requested_ && active_ticket_ == 0U && !pending_job_.has_value());
  });
  if (last_completed_ticket_ != ticket.value) {
    return WaitReason::Stopped;
  }
  return last_completed_failed_ ? WaitReason::JobFailed : WaitReason::Completed;
}

BoundedSingleJobExecutor::Stats BoundedSingleJobExecutor::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void BoundedSingleJobExecutor::stop() noexcept
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_requested_) {
      return;
    }
    stop_requested_ = true;
    pending_job_.reset();
    stats_.pending = false;
    stats_.stopped = true;
  }
  work_condition_.notify_one();
  completion_condition_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void BoundedSingleJobExecutor::run() noexcept
{
  while (true) {
    PendingJob pending;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      work_condition_.wait(lock, [this]() {
        return stop_requested_ || pending_job_.has_value();
      });
      if (stop_requested_) {
        return;
      }
      pending = std::move(pending_job_.value());
      pending_job_.reset();
      stats_.pending = false;
      stats_.running = true;
      active_ticket_ = pending.ticket.value;
    }

    bool failed = false;
    try {
      pending.job();
    } catch (...) {
      failed = true;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_completed_ticket_ = pending.ticket.value;
      last_completed_failed_ = failed;
      active_ticket_ = 0U;
      stats_.running = false;
      ++stats_.completed;
      if (failed) {
        ++stats_.failed;
      }
    }
    completion_condition_.notify_all();
  }
}

}  // namespace multi_purpose_mpc_ros
