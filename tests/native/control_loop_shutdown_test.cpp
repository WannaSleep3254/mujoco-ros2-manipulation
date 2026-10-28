// Deterministic scheduling checks for the native shutdown ownership barrier.
#include "control_loop_shutdown.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
namespace gate = manipulation::control_loop_shutdown;
void require(bool value, const char* why) { if (!value) throw std::runtime_error(why); }
void held_update(bool protected_order) {
  gate::StopRequest stop;
  std::mutex mutex;
  std::condition_variable cv;
  bool entered=false, release=false;
  std::atomic<bool> updating{false}, cleanup{false}, overlap{false}, joined{false};
  std::atomic<int> writes{0};
  std::thread update([&] {
    gate::run_control_cycle([&]{return stop.requested();}, []{}, [&] {
      std::unique_lock<std::mutex> lock(mutex);
      updating.store(true); entered=true; cv.notify_all();
      cv.wait(lock,[&]{return release;});
      if (cleanup.load()) overlap.store(true);
      updating.store(false);
    }, [&]{++writes;});
  });
  { std::unique_lock<std::mutex> lock(mutex); cv.wait(lock,[&]{return entered;}); }
  stop.request();
  auto shutdown=[&] { if (updating.load()) overlap.store(true); cleanup.store(true); };
  if (protected_order) {
    std::thread owner([&] {gate::join_then_shutdown(update,[&]{joined.store(true);},shutdown);});
    std::this_thread::sleep_for(30ms);
    require(!cleanup.load() && !joined.load(), "teardown ran while an update was held");
    { std::lock_guard<std::mutex> lock(mutex); release=true; } cv.notify_all();
    owner.join();
    require(joined.load() && cleanup.load() && !overlap.load(), "join did not precede teardown");
    require(writes.load()==0,"write ran after the in-flight update observed shutdown");
  } else {
    // The old node initiates Context pre-shutdown before it joins its update thread.
    shutdown();
    { std::lock_guard<std::mutex> lock(mutex); release=true; } cv.notify_all();
    update.join();
    require(overlap.load(), "controlled old schedule did not expose teardown/update overlap");
  }
}
int main(int argc,char** argv) {
 try {
  require(argc==2,"one declared test case required"); const std::string mode(argv[1]);
  if (mode=="held_update") held_update(true);
  else if (mode=="unsafe_order_control") held_update(false);
  else if (mode=="cycle_boundaries") {
    for(int boundary=0;boundary<4;++boundary) {
      gate::StopRequest stop; std::vector<int> ops;
      if(boundary==0)stop.request();
      gate::run_control_cycle([&]{return stop.requested();},[&]{ops.push_back(1);if(boundary==1)stop.request();},[&]{ops.push_back(2);if(boundary==2)stop.request();},[&]{ops.push_back(3);});
      require(ops.size()==static_cast<size_t>(boundary),"operation crossed a requested stop boundary");
    }
  } else if (mode=="missing_clock" || mode=="paused_clock") {
    gate::StopRequest stop; std::atomic<int> pauses{0};
    const auto started=std::chrono::steady_clock::now();
    gate::ClockWaitResult result=gate::ClockWaitResult::ready;
    const int64_t frozen=mode=="missing_clock"?0:123;
    std::thread waiter([&]{result=gate::wait_for_ros_time([&]{return frozen;},frozen+1,[&]{return stop.requested();},[&](std::chrono::nanoseconds delay){require(delay>0ns && delay<=1ms,"clock wait was not bounded");++pauses;std::this_thread::sleep_for(delay);});});
    while(pauses.load()<3) {std::this_thread::yield();}
    stop.request(); waiter.join();
    require(result==gate::ClockWaitResult::stopped,"frozen ROS clock did not allow shutdown");
    require(std::chrono::steady_clock::now()-started<500ms,"frozen-clock stop was not bounded by wall time");
  } else if(mode=="backwards_clock") {
    std::vector<int64_t> times{10,11,9};size_t next=0;
    auto result=gate::wait_for_ros_time([&]{return times.at(next++);},12,[]{return false;},[](auto){});
    require(result==gate::ClockWaitResult::backwards,"backwards time was accepted");
  } else if(mode=="clock_error") {
    auto result=gate::wait_for_ros_time([]()->int64_t{throw std::runtime_error("clock failed");},1,[]{return false;},[](auto){});
    require(result==gate::ClockWaitResult::clock_error,"clock error escaped the typed boundary");
  } else if(mode=="stop_before_ready") {
    auto result=gate::wait_for_ros_time([]{return int64_t(100);},1,[]{return true;},[](auto){});
    require(result==gate::ClockWaitResult::stopped,"ready time won over an existing stop");
  } else if(mode=="ready_clock") {
    std::vector<int64_t> times{0,0,1,1,2};size_t next=0;
    auto result=gate::wait_for_ros_time([&]{return times.at(next++);},2,[]{return false;},[](auto){});
    require(result==gate::ClockWaitResult::ready && next==times.size(),"clock target was not honored");
  } else if(mode=="no_thread") {
    std::thread empty;bool joined=false,cleanup=false;
    gate::join_then_shutdown(empty,[&]{joined=true;},[&]{require(joined,"cleanup preceded join boundary");cleanup=true;});
    require(cleanup,"empty startup path did not finish cleanup");
  } else throw std::runtime_error("unknown declared test case");
  std::cout << "PASS " << mode << '\n';return 0;
 } catch(const std::exception& e) {std::cerr << e.what() << '\n';return 1;}
}
