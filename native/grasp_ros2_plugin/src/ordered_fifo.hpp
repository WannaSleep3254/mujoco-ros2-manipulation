// SPDX-License-Identifier: Apache-2.0
#ifndef MANIPULATION_GRASP_ORDERED_FIFO_HPP
#define MANIPULATION_GRASP_ORDERED_FIFO_HPP

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace manipulation_grasp
{
enum class FifoWrite {accepted, full, invalid, closed};

// One producer and one consumer. Storage is allocated with the owner, never in
// a control update. A consumer owns its front slot until its callback returns;
// close only closes admission and never resets/reuses any in-flight slot.
template<class Frame, std::size_t Capacity>
class OrderedFifo
{
  static_assert(Capacity > 0);
  static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
public:
  template<class Fill>
  FifoWrite try_write(Fill && fill)
  {
    if (closed_.load(std::memory_order_acquire)) return FifoWrite::closed;
    const auto head = head_.load(std::memory_order_relaxed);
    if (head - tail_.load(std::memory_order_acquire) >= Capacity) return FifoWrite::full;
    if (!std::forward<Fill>(fill)(frames_[head % Capacity])) return FifoWrite::invalid;
    if (closed_.load(std::memory_order_acquire)) return FifoWrite::closed;
    head_.store(head + 1, std::memory_order_release);
    return FifoWrite::accepted;
  }

  template<class Consume>
  bool try_read(Consume && consume)
  {
    if (closed_.load(std::memory_order_acquire)) return false;
    const auto tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) return false;
    struct Release {
      std::atomic<std::uint64_t> & tail;
      std::uint64_t next;
      ~Release() {tail.store(next, std::memory_order_release);}
    } release{tail_, tail + 1};
    std::forward<Consume>(consume)(static_cast<const Frame &>(frames_[tail % Capacity]));
    return true;
  }

  void close() {closed_.store(true, std::memory_order_release);}

private:
  std::array<Frame, Capacity> frames_{};
  alignas(64) std::atomic<std::uint64_t> head_{0};
  alignas(64) std::atomic<std::uint64_t> tail_{0};
  std::atomic<bool> closed_{false};
};

inline bool capture_age_valid(std::uint64_t captured_ns, std::uint64_t now_ns,
                             std::uint64_t maximum_age_ns)
{
  return captured_ns > 0 && now_ns >= captured_ns && now_ns - captured_ns <= maximum_age_ns;
}
}  // namespace manipulation_grasp
#endif
