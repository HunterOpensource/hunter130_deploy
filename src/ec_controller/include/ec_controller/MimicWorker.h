#ifndef EC_CONTROLLER_MIMIC_WORKER_H_
#define EC_CONTROLLER_MIMIC_WORKER_H_

#include "ec_controller/MimicPolicy.h"

#include <condition_variable>
#include <cstdint>
#include <exception>
#include <mutex>
#include <thread>

namespace ec::mimic
{
enum class Phase {StandingBefore, Preparing, Tracking, Paused, Returning, StandingAfter, Fault};

struct PolicyRequest
{
  Sample sample;
  int64_t input_time_ns {0};
  size_t frame {0};
  bool reset {false};
  Phase phase {Phase::StandingBefore};
  uint64_t generation {0};
  double reference_elapsed {0};
  std::array<Frame, kFuture> horizon;
};

struct PolicyResult
{
  Sample input;
  Frame reference;
  int64_t input_time_ns {0};
  Action action {};
  Joints target {Joints::Zero()};
  size_t frame {0};
  Phase phase {Phase::StandingBefore};
  uint64_t generation {0};
  double reference_elapsed {0};
};

// A single outstanding inference, never a queue. submit/takeResult only try the
// mutex; start/stop join at lifecycle boundaries. Policy must outlive Worker and
// may only be mutated by this worker while started. Metadata/frames stay immutable.
class Worker
{
public:
  explicit Worker(Policy & policy)
  : policy_(policy)
  {
  }
  ~Worker()
  {
    stop();
  }
  Worker(const Worker &) = delete;
  Worker & operator=(const Worker &) = delete;

  void start();
  void stop();
  bool submit(const PolicyRequest & request);
  bool takeResult(PolicyResult & result);

private:
  enum class State {Idle, Pending, Running, Ready, Failed};
  void run();
  Policy & policy_;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable wakeup_;
  State state_ {State::Idle};
  PolicyRequest request_;
  PolicyResult result_;
  std::exception_ptr error_;
  bool stopping_ {true};
};
}  // namespace ec::mimic
#endif
