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
  } else if(mode=="drain_pending_management") {
    std::atomic<bool> cancelled{true}, busy{true}, finished{false};
    std::atomic<int> stage{0}, reads{0}, updates{0};
    std::thread callback([&] {
      while(stage.load()<1) {std::this_thread::yield();}
      // A management callback publishes another controller list after its first ack.
      stage.store(2);
      while(stage.load()<3) {std::this_thread::yield();}
      busy.store(false); finished.store(true);
    });
    const auto cycles=gate::drain_until_executor_finished([&]{return finished.load();},
      [&]{return cancelled.load();},[&]{return busy.load();},[&]{++reads;},[&]{
        ++updates;int expected=0;
        if(!stage.compare_exchange_strong(expected,1) && stage.load()==2)stage.store(3);
      },[](auto delay){std::this_thread::sleep_for(delay);},2ms);
    callback.join();
    require(cycles>=2 && reads.load()==updates.load(),"pending management callback lost its RT acknowledgements");
    require(!busy.load() && finished.load(),"drain ended before the executor callback finished");
  } else if(mode=="drain_late_callback") {
    bool finished=false,busy=false;int pauses=0,reads=0,updates=0;
    const auto cycles=gate::drain_until_executor_finished([&]{return finished;},[]{return true;},
      [&]{return busy;},[&]{++reads;},[&]{++updates;busy=false;},[&](auto delay){
        require(delay>0ns && delay<=1ms,"drain wall slice was unbounded");
        ++pauses;if(pauses==3)busy=true;if(pauses==7)finished=true;
      },2ms);
    require(pauses==7 && cycles==1 && reads==1 && updates==1,
      "idle callback group ended the RT owner before late admitted work/spin completion");
  } else if(mode=="drain_cancel_ack") {
    bool finished=false,cancelled=false;int pauses=0,reads=0;
    const auto cycles=gate::drain_until_executor_finished([&]{return finished;},[&]{return cancelled;},
      []{return true;},[&]{require(cancelled,"read preceded executor cancellation");++reads;},
      [&]{finished=true;},[&](auto){if(++pauses==3)cancelled=true;},2ms);
    require(pauses==3 && cycles==1 && reads==1,"drain did not require cancellation acknowledgement");
  } else if(mode=="drain_missing_clock") {
    bool finished=false;int pauses=0,operations=0;const bool clock_started=false;
    const auto cycles=gate::drain_until_executor_finished([&]{return finished;},[]{return true;},
      [&]{return clock_started;},[&]{++operations;},[&]{++operations;},
      [&](auto){if(++pauses==4)finished=true;},2ms);
    require(cycles==0 && operations==0 && pauses==4,"never-started clock acquired a fabricated control cycle");
  } else if(mode=="drain_finished_during_read") {
    bool finished=false;int reads=0,updates=0;
    const auto cycles=gate::drain_until_executor_finished([&]{return finished;},[]{return true;},
      []{return true;},[&]{++reads;finished=true;},[&]{++updates;},[](auto){},2ms);
    require(reads==1 && updates==0 && cycles==0,"drain update crossed final executor completion");
  } else if(mode=="drain_before_cleanup") {
    std::atomic<bool> finished{false},busy{true},joined{false};
    std::atomic<int> updates{0};
    std::thread rt([&]{gate::drain_until_executor_finished([&]{return finished.load();},
      []{return true;},[&]{return busy.load();},[]{},[&]{++updates;busy.store(false);},
      [&](auto delay){if(!busy.load())finished.store(true);std::this_thread::sleep_for(delay);},2ms);});
    gate::join_then_shutdown(rt,[&]{joined.store(true);},[&]{
      require(joined.load() && finished.load() && !busy.load(),"Context cleanup raced an executor callback/drain");});
    require(updates.load()==1,"metadata drain failed before teardown");
  } else if(mode=="management_group_validation") {
    require(gate::management_group_state(0,false)==gate::ManagementGroupState::not_ready,
      "missing service group was confused with validated readiness");
    require(gate::management_group_state(1,true)==gate::ManagementGroupState::ready,
      "unique mutually-exclusive service group was rejected");
    for(int bad=0;bad<2;++bad) {
      bool threw=false;
      try {gate::management_group_state(bad==0?2:1,bad==0);}
      catch(const std::runtime_error&){threw=true;}
      require(threw,"duplicate or non-mutually-exclusive service group was silently treated as idle");
    }
  } else if(mode=="drain_invalid_period") {
    bool threw=false;
    try {gate::drain_until_executor_finished([]{return true;},[]{return true;},[]{return false;},
      []{},[]{},[](auto){},0ns);}catch(const std::invalid_argument&){threw=true;}
    require(threw,"invalid shutdown drain cadence was accepted");
  } else throw std::runtime_error("unknown declared test case");
  std::cout << "PASS " << mode << '\n';return 0;
 } catch(const std::exception& e) {std::cerr << e.what() << '\n';return 1;}
}
