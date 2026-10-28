// Copyright 2026 mujoco-ros2-manipulation contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef MANIPULATION_CONTROL_LOOP_SHUTDOWN_HPP_
#define MANIPULATION_CONTROL_LOOP_SHUTDOWN_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstddef>
#include <thread>
#include <stdexcept>
#include <utility>

namespace manipulation::control_loop_shutdown
{

// This is a shutdown request, not proof that executor callbacks are quiescent.
// The signal handler only sets this flag. It must never run a ROS callback, take a
// mutex, log, or wait for the control thread. Static initialization also avoids
// initialization of a function-local object from within a signal handler.
class StopRequest
{
public:
  constexpr StopRequest() noexcept = default;
  void request() noexcept { requested_.store(true, std::memory_order_relaxed); }
  bool requested() const noexcept { return requested_.load(std::memory_order_relaxed); }

private:
  static_assert(std::atomic<bool>::is_always_lock_free, "signal stop flag must be lock-free");
  std::atomic<bool> requested_{false};
};

inline StopRequest process_stop;
inline constexpr auto poll_interval = std::chrono::milliseconds(1);

enum class ClockWaitResult { ready, stopped, backwards, clock_error };

// Service groups may appear after robot_description arrives. Absence is not
// callback completion: the RT owner must still wait for executor spin_finished.
enum class ManagementGroupState { not_ready, ready };
inline ManagementGroupState management_group_state(std::size_t matching_services, bool mutually_exclusive)
{
  if (matching_services == 0) { return ManagementGroupState::not_ready; }
  if (matching_services != 1) {
    throw std::runtime_error("Expected exactly one switch_controller service instance");
  }
  if (!mutually_exclusive) {
    throw std::runtime_error("switch_controller requires a MutuallyExclusive callback group");
  }
  return ManagementGroupState::ready;
}

// now_ns returns signed nanoseconds; pause accepts a nanoseconds duration. The
// injected operations make missing/paused/backwards clocks testable without ROS.
// minimum_ns also detects a rewind between target calculation and the first poll.
// A ROS clock can stop forever, so it must not own the blocking wait. Only the
// bounded wall pause may block, and stop is checked before clock access and again
// before success. No control cycle is repeated while this wait is paused.
template<class NowNs, class ShouldStop, class Pause>
ClockWaitResult wait_for_ros_time(
  NowNs && now_ns, std::int64_t target_ns, ShouldStop && should_stop, Pause && pause,
  std::int64_t minimum_ns = 0)
{
  if (minimum_ns < 0 || target_ns < minimum_ns) {
    return ClockWaitResult::clock_error;
  }
  bool have_previous = false;
  std::int64_t previous = 0;
  for (;;) {
    if (should_stop()) {
      return ClockWaitResult::stopped;
    }
    std::int64_t current;
    try {
      current = now_ns();
    } catch (...) {
      return ClockWaitResult::clock_error;
    }
    if (should_stop()) {
      return ClockWaitResult::stopped;
    }
    if (current < 0) {
      return ClockWaitResult::clock_error;
    }
    if (current < minimum_ns || (have_previous && current < previous)) {
      return ClockWaitResult::backwards;
    }
    if (current >= target_ns) {
      return ClockWaitResult::ready;
    }
    previous = current;
    have_previous = true;
    try {
      pause(std::chrono::duration_cast<std::chrono::nanoseconds>(poll_interval));
    } catch (...) {
      return ClockWaitResult::clock_error;
    }
  }
}

// An in-flight read/update may finish after stop is requested, but no following
// phase is entered. In particular, stop during update skips write. Lifetime is
// protected by joining the owner thread, not by assuming this flag interrupts a
// controller callback that has already begun.
template<class ShouldStop, class Read, class Update, class Write>
bool run_control_cycle(ShouldStop && should_stop, Read && read, Update && update, Write && write)
{
  if (should_stop()) { return false; }
  read();
  if (should_stop()) { return false; }
  update();
  if (should_stop()) { return false; }
  write();
  return true;
}

// cancel() stops future dispatch but cannot interrupt a callback already inside
// ControllerManager. Such a callback can need another RT list acknowledgement.
// Keep the owner alive until spin has joined every executor worker. Only an
// observed in-flight management callback permits housekeeping read/update; this
// helper has no write operation. No ROS time is invented while the clock is
// paused. All waits use bounded wall slices, including the active cycle period.
template<class Finished, class Cancelled, class Busy, class Read, class Update, class Pause>
std::uint64_t drain_until_executor_finished(
  Finished && finished, Cancelled && cancellation_observed, Busy && callback_busy,
  Read && read, Update && update, Pause && pause, std::chrono::nanoseconds cycle_period)
{
  if (cycle_period <= std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument("shutdown drain requires a positive wall period");
  }
  const auto wall_slice = std::chrono::duration_cast<std::chrono::nanoseconds>(poll_interval);
  std::uint64_t cycles = 0;
  while (!finished()) {
    if (!cancellation_observed() || !callback_busy()) {
      pause(wall_slice);
      continue;
    }
    if (finished()) { break; }
    read();
    if (finished()) { break; }
    update();
    ++cycles;
    auto remaining = cycle_period;
    while (remaining > std::chrono::nanoseconds::zero() && !finished()) {
      const auto interval = remaining < wall_slice ? remaining : wall_slice;
      pause(interval);
      remaining -= interval;
    }
  }
  return cycles;
}

// Context::shutdown runs ControllerManager pre-shutdown callbacks synchronously.
// The owner must have joined its RT thread before any such callback is allowed.
// This helper is deliberately not callable from the signal handler or GUI thread.
template<class Thread, class OnJoined, class Shutdown>
void join_then_shutdown(Thread & thread, OnJoined && on_joined, Shutdown && shutdown)
{
  process_stop.request();
  if (thread.joinable()) {
    thread.join();
  }
  std::forward<OnJoined>(on_joined)();
  std::forward<Shutdown>(shutdown)();
}

}  // namespace manipulation::control_loop_shutdown
#endif  // MANIPULATION_CONTROL_LOOP_SHUTDOWN_HPP_
