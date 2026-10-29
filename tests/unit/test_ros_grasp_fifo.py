"""Compile and exercise the exact allocation-free native observation FIFO.

These tests do not load ROS or step physics. They test concurrent slot ownership,
ordering and capture-age admission, not middleware delivery or robot behavior.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


SOURCE = r'''
#include "ordered_fifo.hpp"
#include <array>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <thread>
using manipulation_grasp::OrderedFifo;
using manipulation_grasp::FifoWrite;
using manipulation_grasp::capture_age_valid;
struct Frame {std::uint64_t sequence = 0, captured = 0;std::array<std::uint64_t, 64> payload{};};

void ordering() {
  OrderedFifo<Frame,32> fifo;
  for (std::uint64_t i=0;i<32;++i)
    assert(fifo.try_write([&](Frame & f){f.sequence=i;f.captured=100+i;return true;})==FifoWrite::accepted);
  bool overwritten=false;
  assert(fifo.try_write([&](Frame &){overwritten=true;return true;})==FifoWrite::full);
  assert(!overwritten);
  std::uint64_t next=0;
  for (int i=0;i<8;++i) assert(fifo.try_read([&](const Frame & f){assert(f.sequence==next);assert(f.captured==100+next++);}));
  for (std::uint64_t i=32;i<40;++i)
    assert(fifo.try_write([&](Frame & f){f.sequence=i;f.captured=100+i;return true;})==FifoWrite::accepted);
  while(fifo.try_read([&](const Frame & f){assert(f.sequence==next);assert(f.captured==100+next++);})) {}
  assert(next==40);
}
void invalid_and_exception() {
  OrderedFifo<Frame,1> fifo;
  assert(fifo.try_write([](Frame & f){f.sequence=999;return false;})==FifoWrite::invalid);
  assert(!fifo.try_read([](const Frame &){assert(false);}));
  assert(fifo.try_write([](Frame & f){f.sequence=1;return true;})==FifoWrite::accepted);
  try {fifo.try_read([](const Frame & f){assert(f.sequence==1);throw std::runtime_error("consumer failed");});assert(false);}
  catch(const std::runtime_error &) {}
  assert(fifo.try_write([](Frame & f){f.sequence=2;return true;})==FifoWrite::accepted);
  assert(fifo.try_read([](const Frame & f){assert(f.sequence==2);}));
  assert(fifo.try_write([&](Frame &){fifo.close();return true;})==FifoWrite::closed);
  assert(!fifo.try_read([](const Frame &){assert(false);}));
}
void lifetime() {
  OrderedFifo<Frame,32> fifo;
  assert(fifo.try_write([](Frame & f){f.sequence=42;f.payload.fill(42);return true;})==FifoWrite::accepted);
  std::atomic<bool> entered{false},release{false};
  std::thread consumer([&]{assert(fifo.try_read([&](const Frame & f){
    entered.store(true,std::memory_order_release);
    while(!release.load(std::memory_order_acquire)) std::this_thread::yield();
    assert(f.sequence==42);for(auto value:f.payload) assert(value==42);
  }));});
  while(!entered.load(std::memory_order_acquire)) std::this_thread::yield();
  for(int i=0;i<31;++i) assert(fifo.try_write([&](Frame & f){f.sequence=i;return true;})==FifoWrite::accepted);
  assert(fifo.try_write([](Frame &){return true;})==FifoWrite::full);
  fifo.close();
  assert(fifo.try_write([](Frame &){assert(false);return true;})==FifoWrite::closed);
  release.store(true,std::memory_order_release);consumer.join();
  assert(!fifo.try_read([](const Frame &){assert(false);}));
}
void concurrent() {
  OrderedFifo<Frame,32> fifo;
  constexpr std::uint64_t count=30000;
  std::thread producer([&]{for(std::uint64_t i=0;i<count;++i){
    while(fifo.try_write([&](Frame & f){f.sequence=i;f.captured=1000+i;f.payload.fill(i^0xa5a5);return true;})==FifoWrite::full)
      std::this_thread::yield();
  }});
  std::uint64_t next=0;
  while(next<count) {
    if(!fifo.try_read([&](const Frame & f){assert(f.sequence==next);assert(f.captured==1000+next);
      for(auto value:f.payload) {assert(value==(next^0xa5a5));}
      ++next;})) std::this_thread::yield();
  }
  producer.join();assert(!fifo.try_read([](const Frame &){assert(false);}));
}
void source_age() {
  constexpr std::uint64_t maximum=500000000, capture=1000000000;
  assert(capture_age_valid(capture,capture,maximum));
  assert(capture_age_valid(capture,capture+maximum,maximum));
  assert(!capture_age_valid(capture,capture+maximum+1,maximum));
  assert(!capture_age_valid(capture,capture-1,maximum));
  assert(!capture_age_valid(0,capture,maximum));
  OrderedFifo<Frame,32> fifo;
  assert(fifo.try_write([](Frame & f){f.captured=capture;return true;})==FifoWrite::accepted);
  assert(fifo.try_read([](const Frame & f){assert(!capture_age_valid(f.captured,capture+maximum+1,maximum));}));
}
int main(int argc,char **argv) {
  assert(argc==2);std::string selected=argv[1];
  if(selected=="ordering") ordering();else if(selected=="invalid") invalid_and_exception();
  else if(selected=="lifetime") lifetime();else if(selected=="concurrent") concurrent();
  else if(selected=="age") source_age();else return 2;
}
'''


class NativeObservationFifoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("g++")
        if compiler is None:
            raise unittest.SkipTest("g++ is required for the native FIFO test")
        cls.temporary = tempfile.TemporaryDirectory(prefix="grasp-observation-fifo-")
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.executable = Path(cls.temporary.name) / "fifo-test"
        source = Path(cls.temporary.name) / "fifo-test.cpp"
        source.write_text(SOURCE)
        include = Path(__file__).resolve().parents[2] / "native/grasp_ros2_plugin/src"
        subprocess.run([compiler, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-pthread",
                        "-I", str(include), str(source), "-o", str(cls.executable)], check=True,
                       capture_output=True, text=True, timeout=30)

    def run_case(self, case):
        subprocess.run([str(self.executable), case], check=True, timeout=10)

    def test_order_and_overflow_never_replace_pending_frames(self):
        self.run_case("ordering")

    def test_invalid_capture_exception_and_closed_writer_keep_ownership(self):
        self.run_case("invalid")

    def test_close_during_consumer_does_not_reuse_its_frame(self):
        self.run_case("lifetime")

    def test_concurrent_producer_consumer_preserve_complete_frame_order(self):
        self.run_case("concurrent")

    def test_original_capture_age_expires_without_arrival_retimestamp(self):
        self.run_case("age")
