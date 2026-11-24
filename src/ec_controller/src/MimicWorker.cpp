#include "ec_controller/MimicWorker.h"

namespace ec::mimic
{
void Worker::start()
{
  stop();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = false;
    state_ = State::Idle;
    error_ = nullptr;
  }
  thread_ = std::thread(&Worker::run, this);
}

void Worker::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  wakeup_.notify_one();
  if (thread_.joinable()) {thread_.join();}
}

bool Worker::submit(const PolicyRequest & request)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock() || stopping_ || state_ != State::Idle) {return false;}
  request_ = request;
  state_ = State::Pending;
  lock.unlock();
  wakeup_.notify_one();
  return true;
}

bool Worker::takeResult(PolicyResult & result)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock() || stopping_) {return false;}
  if (state_ == State::Failed) {std::rethrow_exception(error_);}
  if (state_ != State::Ready) {return false;}
  result = result_;
  state_ = State::Idle;
  return true;
}

void Worker::run()
{
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    wakeup_.wait(lock, [this]() {return stopping_ || state_ == State::Pending;});
    if (stopping_) {return;}
    const auto request = request_;
    state_ = State::Running;
    lock.unlock();
    PolicyResult result;
    std::exception_ptr error;
    try {
      policy_.setReference(request.horizon);
      if (request.reset) {policy_.reset(request.sample);}
      result.action = policy_.infer(request.sample, 0);
      result.target = policy_.target();
      result.frame = request.frame;
      result.phase = request.phase;
      result.generation = request.generation;
      result.reference_elapsed = request.reference_elapsed;
      result.input = request.sample;
      result.reference = request.horizon.front();
      result.input_time_ns = request.input_time_ns;
    } catch (...) {
      error = std::current_exception();
    }
    lock.lock();
    result_ = result;
    error_ = error;
    state_ = error ? State::Failed : State::Ready;
  }
}
}  // namespace ec::mimic
