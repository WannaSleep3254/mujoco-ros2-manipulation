// Test the actual action-server expiration timer against a ROS time update.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <link.h>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <rcl/time.h>
#include <rcl/timer.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_server.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

namespace {
using Action = control_msgs::action::FollowJointTrajectory;
using AddCallback = decltype(&rcl_clock_add_jump_callback);
std::mutex gate_mutex;
std::mutex output_mutex;
std::condition_variable gate_changed;
bool entered = false, released = false, attempted = false;
std::atomic<bool> armed{false};
rcl_clock_t * target_clock = nullptr;
rcl_timer_t * target_timer = nullptr;

void event(const char * name, const char * extra = "") {
  std::lock_guard<std::mutex> output_lock(output_mutex);
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  std::printf("CLOCK_RACE {\"event\":\"%s\",\"pid\":%ld,\"monotonic_ns\":%lld%s}\n",
    name, static_cast<long>(getpid()), static_cast<long long>(ns), extra);
  std::fflush(stdout);
}
[[noreturn]] void fail(const char * reason) {
  std::fprintf(stderr, "CLOCK_RACE_FAILURE %s\n", reason);
  std::_Exit(2);
}
void require(bool value, const char * reason) { if (!value) { fail(reason); } }
void library(const char * name, const char * key) {
  void * handle = dlopen(name, RTLD_NOW | RTLD_NOLOAD);
  link_map * mapping = nullptr;
  require(handle && !dlinfo(handle, RTLD_DI_LINKMAP, &mapping), "mapped library missing");
  char details[1024];
  std::snprintf(details, sizeof(details), ",\"key\":\"%s\",\"path\":\"%s\"", key, mapping->l_name);
  event("library", details);
  dlclose(handle);
}
}  // namespace

extern "C" rcl_ret_t rcl_clock_add_jump_callback(
  rcl_clock_t * clock, rcl_jump_threshold_t threshold, rcl_jump_callback_t callback,
  void * user_data)
{
  static auto actual = reinterpret_cast<AddCallback>(dlsym(RTLD_NEXT, "rcl_clock_add_jump_callback"));
  require(actual != nullptr, "real jump callback registration is missing");
  const auto result = actual(clock, threshold, callback, user_data);
  Dl_info caller{};
  const bool timer_call = dladdr(__builtin_return_address(0), &caller) && caller.dli_sname
    && std::strcmp(caller.dli_sname, "rcl_timer_init") == 0;
  if (result == RCL_RET_OK && clock == target_clock && timer_call && armed.exchange(false)) {
    target_timer = static_cast<rcl_timer_t *>(user_data);
    require(target_timer && target_timer->impl == nullptr, "real timer was already initialized at registration");
    std::unique_lock<std::mutex> lock(gate_mutex);
    entered = true;
    event("registered_timer_impl_null");
    gate_changed.notify_all();
    require(gate_changed.wait_for(lock, std::chrono::seconds(2), [] { return released; }),
      "registration gate timed out");
    event("registration_released");
  }
  return result;
}

int main(int argc, char ** argv) {
  require(argc == 2, "select legacy or fixed");
  const bool fixed = std::strcmp(argv[1], "fixed") == 0;
  require(fixed || std::strcmp(argv[1], "legacy") == 0, "unknown variant");
  int ros_argc = 1;
  rclcpp::init(ros_argc, argv);
  auto node = std::make_shared<rclcpp::Node>("clock_action_startup_race",
    rclcpp::NodeOptions().use_clock_thread(false));
  auto clock = node->get_clock();
  target_clock = clock->get_clock_handle();
  require(target_clock->type == RCL_ROS_TIME, "action node does not use a ROS clock");
  require(rcl_enable_ros_time_override(target_clock) == RCL_RET_OK, "ROS time override activation failed");
  library("librclcpp_action.so", "rclcpp_action");
  library("librcl.so", "rcl");
  library("librclcpp.so", "rclcpp");
  rclcpp_action::Server<Action>::SharedPtr server;
  armed.store(true);
  std::thread constructor([&] {
    // Both variants call exactly the same installed API without an external lock.
    server = rclcpp_action::create_server<Action>(node, "probe_follow_joint_trajectory",
      [](const auto &, auto) { return rclcpp_action::GoalResponse::REJECT; },
      [](auto) { return rclcpp_action::CancelResponse::REJECT; }, [](auto) {});
    event("action_server_constructed");
  });
  {
    std::unique_lock<std::mutex> lock(gate_mutex);
    require(gate_changed.wait_for(lock, std::chrono::seconds(5), [] { return entered; }),
      "actual expiration-timer boundary was not reached");
  }
  std::thread updater([&] {
    std::unique_lock<std::mutex> lock(clock->get_clock_mutex(), std::defer_lock);
    const bool acquired = lock.try_lock();
    event(acquired ? "clock_mutex_acquired" : "clock_mutex_blocked");
    require(acquired != fixed, "actual library does not have the selected locking behavior");
    {
      std::lock_guard<std::mutex> gate_lock(gate_mutex);
      attempted = true;
      gate_changed.notify_all();
    }
    if (!acquired) { lock.lock(); }
    require((target_timer->impl != nullptr) == fixed, "unexpected timer state at actual ROS time update");
    event(fixed ? "update_timer_impl_ready" : "update_timer_impl_null");
    // This is the real API called by TimeSource while holding this same mutex.
    require(rcl_set_ros_time_override(target_clock, 1000000000) == RCL_RET_OK,
      "actual ROS time update failed");
    event("time_update_returned");
  });
  if (fixed) {
    std::unique_lock<std::mutex> lock(gate_mutex);
    require(gate_changed.wait_for(lock, std::chrono::seconds(1), [] { return attempted; }),
      "updater did not attempt the clock mutex");
    released = true;
    gate_changed.notify_all();
  }
  updater.join();
  constructor.join();
  require(fixed, "legacy actual timer callback did not reproduce the null dereference");
  server.reset();
  node.reset();
  rclcpp::shutdown();
  event("passed");
  return 0;
}
