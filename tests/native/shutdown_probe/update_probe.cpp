// Test-only interposer for the installed Humble 2.54.1 JTC update entry.
// It holds a real virtual call after ControllerManager's active-state check.
// Never preload this library in an operator or physical robot process.
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <string>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#define JTC_UPDATE_SYMBOL "_ZN27joint_trajectory_controller25JointTrajectoryController6updateERKN6rclcpp4TimeERKNS1_8DurationE"
namespace {
using Update = std::uint8_t (*)(void *, const void *, const void *);
[[noreturn]] void fail(const char * reason) {
  ::dprintf(STDERR_FILENO, "SHUTDOWN_PROBE failure=%s pid=%ld\n", reason, static_cast<long>(::getpid()));
  ::_exit(125);
}
std::int64_t now_ns() {
  timespec value{};
  if (::clock_gettime(CLOCK_MONOTONIC, &value)) { fail("monotonic_clock"); }
  return static_cast<std::int64_t>(value.tv_sec) * 1000000000 + value.tv_nsec;
}
void write_all(int fd, const std::string & value) {
  std::size_t offset = 0;
  while (offset < value.size()) {
    const auto count = ::write(fd, value.data() + offset, value.size() - offset);
    if (count < 0 && errno == EINTR) { continue; }
    if (count <= 0) { fail("gate_write"); }
    offset += static_cast<std::size_t>(count);
  }
}
Update original() {
  static Update value = [] {
    ::dlerror();
    void * symbol = ::dlsym(RTLD_NEXT, JTC_UPDATE_SYMBOL);
    if (::dlerror() || !symbol) { fail("RTLD_NEXT_JTC_update_not_found"); }
    return reinterpret_cast<Update>(symbol);
  }();
  return value;
}
struct Config {
  int directory{-1};
  std::string run_id;
  std::int64_t hold_ns{0};
};
const Config & config() {
  static Config value = [] {
    Config result;
    const char * directory = ::getenv("MANIPULATION_SHUTDOWN_PROBE_DIR");
    if (!directory) { return result; }
    const char * run_id = ::getenv("MANIPULATION_SHUTDOWN_PROBE_RUN_ID");
    const char * hold_ms = ::getenv("MANIPULATION_SHUTDOWN_PROBE_MAX_HOLD_MS");
    if (!run_id || std::strlen(run_id) != 32 || std::strspn(run_id, "0123456789abcdef") != 32 || !hold_ms) {
      fail("invalid_probe_configuration");
    }
    char * end = nullptr;
    errno = 0;
    const long milliseconds = std::strtol(hold_ms, &end, 10);
    if (errno || !end || *end || milliseconds < 1000 || milliseconds > 15000) { fail("invalid_hold_timeout"); }
    result.directory = ::open(directory, O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    struct stat info{};
    if (result.directory < 0 || ::fstat(result.directory, &info) || info.st_uid != ::geteuid() ||
      !S_ISDIR(info.st_mode) || (info.st_mode & 0777) != 0700) { fail("directory_not_private_and_owned"); }
    result.run_id = run_id;
    result.hold_ns = static_cast<std::int64_t>(milliseconds) * 1000000;
    return result;
  }();
  return value;
}
bool gate(const Config & cfg, const char * name) {
  const int fd = ::openat(cfg.directory, name, O_RDONLY | O_NOFOLLOW | O_CLOEXEC);
  if (fd < 0 && errno == ENOENT) { return false; }
  struct stat info{};
  if (fd < 0 || ::fstat(fd, &info) || !S_ISREG(info.st_mode) || info.st_uid != ::geteuid() ||
    (info.st_mode & 0777) != 0600 || info.st_size != 33) { fail("invalid_gate_file"); }
  char contents[34]{};
  std::size_t total = 0;
  while (total < 33) {
    const auto count = ::read(fd, contents + total, 33 - total);
    if (count < 0 && errno == EINTR) { continue; }
    if (count <= 0) { fail("gate_read"); }
    total += static_cast<std::size_t>(count);
  }
  ::close(fd);
  if (std::string(contents, total) != cfg.run_id + "\n") { fail("gate_run_id_mismatch"); }
  return true;
}
void event(const Config & cfg, const char * name, bool entered_file = false) {
  char buffer[384];
  const int length = std::snprintf(buffer, sizeof(buffer),
    "{\"event\":\"%s\",\"run_id\":\"%s\",\"pid\":%ld,\"tid\":%ld,\"monotonic_ns\":%lld}\n",
    name, cfg.run_id.c_str(), static_cast<long>(::getpid()), static_cast<long>(::syscall(SYS_gettid)),
    static_cast<long long>(now_ns()));
  if (length <= 0 || length >= static_cast<int>(sizeof(buffer))) { fail("event_overflow"); }
  const std::string line(buffer, static_cast<std::size_t>(length));
  const auto save = [&](const char * filename, bool exclusive) {
    const int flags = O_WRONLY | O_CREAT | O_NOFOLLOW | O_CLOEXEC | (exclusive ? O_EXCL : O_APPEND);
    const int fd = ::openat(cfg.directory, filename, flags, 0600);
    struct stat info{};
    if (fd < 0 || ::fstat(fd, &info) || !S_ISREG(info.st_mode) || info.st_uid != ::geteuid() ||
      (info.st_mode & 0777) != 0600 || info.st_size > 8192) { fail("invalid_event_file"); }
    write_all(fd, line);
    if (::fsync(fd)) { fail("event_fsync"); }
    ::close(fd);
  };
  // Publish the durable stream entry before exposing entered.json to its owner.
  // Both files contain the same identity and timestamp; no second event can be
  // scheduled after the owner already sent SIGINT based on entered.json.
  save("events.jsonl", false);
  if (entered_file) { save("entered.json", true); }
  ::dprintf(STDERR_FILENO, "SHUTDOWN_PROBE event=%s run_id=%s pid=%ld\n",
    name, cfg.run_id.c_str(), static_cast<long>(::getpid()));
}
}  // namespace

extern "C" int manipulation_shutdown_probe_resolves() { return original() ? 1 : 0; }
extern "C" std::uint8_t manipulation_shutdown_probe_update(void *, const void *, const void *)
  __asm__(JTC_UPDATE_SYMBOL);
extern "C" std::uint8_t manipulation_shutdown_probe_update(void * self, const void * time, const void * period) {
  const auto actual = original();
  const auto & cfg = config();
  static std::atomic<bool> claimed{false};
  bool expected = false;
  const bool held = cfg.directory >= 0 && !claimed.load() && gate(cfg, "arm") &&
    claimed.compare_exchange_strong(expected, true);
  if (held) {
    event(cfg, "entered", true);
    const auto deadline = now_ns() + cfg.hold_ns;
    while (!gate(cfg, "release")) {
      if (now_ns() >= deadline) { event(cfg, "hold_timeout"); fail("hold_timeout"); }
      const timespec pause{0, 1000000};
      ::nanosleep(&pause, nullptr);
    }
    event(cfg, "released");
  }
  const auto result = actual(self, time, period);
  if (held) { event(cfg, "returned"); }
  return result;
}
