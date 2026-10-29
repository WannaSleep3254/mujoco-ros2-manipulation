// SPDX-License-Identifier: Apache-2.0
// Grasp observations never modify physics state. Only the declared userdata
// slots are written by pre_step; contact admission belongs to StepMonitor.
#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/string.hpp>
#include "ordered_fifo.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <time.h>

#ifndef MANIPULATION_GRASP_PLUGIN_SHA256
#error "The observer must carry its source SHA256"
#endif
#ifndef MANIPULATION_GRASP_FIFO_SHA256
#error "The observation FIFO must carry its source SHA256"
#endif

namespace manipulation_grasp
{
namespace
{
constexpr int kQpos = 21, kQvel = 20, kGeoms = 256, kContacts = 256;
constexpr std::size_t kJsonBytes = 256 * 1024;
constexpr std::uint64_t kHeartbeatMs = 1000;
constexpr std::size_t kQueueCapacity = 32, kMaximumDrain = 8;
constexpr std::uint64_t kMaximumSourceAgeNs = 500000000;
constexpr const char * kPolicy = "fr5_parallel_block_v1";
using Message = std_msgs::msg::String;

std::uint64_t steady_ns()
{
  struct timespec value{};
  if (::clock_gettime(CLOCK_MONOTONIC, &value) != 0 || value.tv_sec < 0 || value.tv_nsec < 0) return 0;
  return static_cast<std::uint64_t>(value.tv_sec) * 1000000000 + static_cast<std::uint64_t>(value.tv_nsec);
}
std::uint64_t steady_ms() {return steady_ns() / 1000000;}

bool hex(const char * value, std::size_t length)
{
  if (!value) return false;
  for (std::size_t i = 0; i < length; ++i) {
    if (!value[i] || !((value[i] >= '0' && value[i] <= '9') ||
      (value[i] >= 'a' && value[i] <= 'f'))) return false;
  }
  return value[length] == '\0';
}

std::string quoted(const std::string & value)
{
  std::ostringstream out;
  out << '"';
  constexpr char digits[] = "0123456789abcdef";
  for (unsigned char ch : value) {
    if (ch == '"' || ch == '\\') out << '\\' << ch;
    else if (ch < 32) out << "\\u00" << digits[ch >> 4] << digits[ch & 15];
    else out << ch;
  }
  out << '"';
  return out.str();
}

template<std::size_t N>
void vector(std::ostringstream & out, const std::array<double, N> & values)
{
  out << '[';
  for (std::size_t i = 0; i < N; ++i) {
    if (i) out << ',';
    out << values[i];
  }
  out << ']';
}

template<std::size_t N>
bool copy_finite(std::array<double, N> & output, const mjtNum * input)
{
  for (std::size_t i = 0; i < N; ++i) {
    if (!std::isfinite(input[i]) || std::abs(input[i]) > 1e6) return false;
    output[i] = input[i];
  }
  return true;
}

struct Pose
{
  std::array<double, 3> position{};
  std::array<double, 4> quaternion{};
};
struct Geom
{
  std::array<double, 3> position{};
  std::array<double, 9> rotation{};
};
struct Contact
{
  int geom1 = 0, geom2 = 0;
  double normal = 0;
};
struct Snapshot
{
  std::uint64_t sequence = 0, stamp_ns = 0, capture_monotonic_ns = 0;
  double time_sec = 0, force_evaluation_sec = 0;
  int phase = 0;
  bool armed = false, heartbeat_fresh = false;
  std::array<double, kQpos> qpos{};
  std::array<double, kQvel> qvel{};
  std::array<double, 12> arms{};
  std::array<double, 2> fingers{};
  std::array<Geom, kGeoms> geoms{};
  Pose tcp, block;
  std::array<double, 3> block_linear{}, block_angular{};
  std::array<Contact, kContacts> contacts{};
  int geom_count = 0, contact_count = 0;
};
struct DataDelete {void operator()(mjData * data) const {if (data) mj_deleteData(data);}};

struct Transport
{
  // Callback closures retain Transport. No timer/subscription captures the
  // plugin object; an already running callback can finish after cleanup.
  std::atomic<bool> closed{false}, fault{false};
  std::atomic_flag publishing = ATOMIC_FLAG_INIT;
  std::atomic_flag fault_reported = ATOMIC_FLAG_INIT;
  std::atomic<int> fault_code{0};
  std::atomic<std::uint64_t> overflow_count{0};
  std::atomic<std::uint64_t> command{0};  // one coherent receipt-ms/phase word
  OrderedFifo<Snapshot, kQueueCapacity> frames;
  std::mutex publisher_mutex;  // timer/cleanup only; never update/pre_step
  rclcpp::Publisher<Message>::SharedPtr publisher;
  std::string run_id, model_sha256;
  std::array<std::string, 12> arm_names;
  std::array<std::string, 2> finger_names{{"fr5_left_finger_slide", "fr5_right_finger_slide"}};
  std::array<std::string, kGeoms> geom_names;
  std::array<int, 12> arm_addresses{};
  std::array<int, 2> finger_addresses{};
  const mjModel * model = nullptr;
  std::unique_ptr<mjData, DataDelete> copy;
  int tcp_id = -1, block_id = -1;
  std::uint64_t sequence = 0, last_capture_ms = 0;
};

void transport_fault(Transport & state, int code)
{
  int expected = 0;
  state.fault_code.compare_exchange_strong(expected, code, std::memory_order_relaxed);
  state.fault.store(true, std::memory_order_release);
}

void report_transport_fault(Transport & state)
{
  if (state.fault.load(std::memory_order_acquire) &&
    !state.fault_reported.test_and_set(std::memory_order_acquire)) {
    // Non-real-time timer only. Preserve one bounded fault diagnostic even
    // when no old queued observation may be published as a current sample.
    RCLCPP_ERROR(rclcpp::get_logger("manipulation_grasp_observation"),
      "OBSERVATION_TRANSPORT_FAULT code=%d overflow_count=%llu fifo_capacity=%zu fifo_source_sha256=%s",
      state.fault_code.load(std::memory_order_relaxed),
      static_cast<unsigned long long>(state.overflow_count.load(std::memory_order_relaxed)),
      kQueueCapacity, MANIPULATION_GRASP_FIFO_SHA256);
  }
}

std::pair<int, bool> phase(const Transport & state, std::uint64_t now)
{
  const auto word = state.command.load(std::memory_order_acquire);
  if (!word) return {0, false};
  const auto received = word >> 4;
  if (now < received || now - received > kHeartbeatMs) return {-1, true};
  return {static_cast<int>(word & 15) - 1, true};
}

bool model_contract(const mjModel * model, Transport & state)
{
  if (!model || model->nq != kQpos || model->nv != kQvel || model->nu != 14 ||
    model->njnt != 15 || model->nuserdata != 2 || model->ngeom < 1 ||
    model->ngeom > kGeoms || !std::isfinite(model->opt.timestep) ||
    model->opt.timestep <= 0 || model->opt.timestep > .01) return false;
  const int marker = mj_name2id(model, mjOBJ_TEXT, "manipulation_grasp_policy");
  if (marker < 0 || model->text_size[marker] > 128 ||
    std::string(model->text_data + model->text_adr[marker]) != kPolicy) return false;
  const std::array<std::string, 6> right{{"right_shoulder_pan_joint", "right_shoulder_lift_joint",
    "right_elbow_joint", "right_wrist_1_joint", "right_wrist_2_joint", "right_wrist_3_joint"}};
  for (int i = 0; i < 12; ++i) {
    state.arm_names[i] = i < 6 ? "left_j" + std::to_string(i + 1) : right[i - 6];
    const int joint = mj_name2id(model, mjOBJ_JOINT, state.arm_names[i].c_str());
    if (joint < 0 || model->jnt_type[joint] != mjJNT_HINGE) return false;
    state.arm_addresses[i] = model->jnt_qposadr[joint];
  }
  for (int i = 0; i < 2; ++i) {
    const int joint = mj_name2id(model, mjOBJ_JOINT, state.finger_names[i].c_str());
    if (joint < 0 || model->jnt_type[joint] != mjJNT_SLIDE ||
      std::abs(model->jnt_range[2 * joint]) > 1e-12 ||
      std::abs(model->jnt_range[2 * joint + 1] - .034) > 1e-12) return false;
    state.finger_addresses[i] = model->jnt_qposadr[joint];
  }
  const int block_joint = mj_name2id(model, mjOBJ_JOINT, "fr5_block_free");
  state.block_id = mj_name2id(model, mjOBJ_BODY, "fr5_grasp_block");
  state.tcp_id = mj_name2id(model, mjOBJ_SITE, "fr5_grasp_tcp");
  if (block_joint < 0 || model->jnt_type[block_joint] != mjJNT_FREE ||
    state.block_id < 1 || model->jnt_bodyid[block_joint] != state.block_id || state.tcp_id < 0) return false;
  for (const auto * name : {"fr5_block_geom", "fr5_table_geom", "fr5_left_finger_geom", "fr5_right_finger_geom"}) {
    if (mj_name2id(model, mjOBJ_GEOM, name) < 0) return false;
  }
  for (int i = 0; i < model->ngeom; ++i) {
    const auto * name = mj_id2name(model, mjOBJ_GEOM, i);
    state.geom_names[i] = name ? name : "";
    if (state.geom_names[i].size() > 128) return false;
  }
  return true;
}

bool fill(Transport & state, const mjModel * model, const mjData * data, Snapshot & output)
{
  if (model != state.model || !data || !std::isfinite(data->time) ||
    data->time < 0 || data->time > 2147483647.0 || data->ncon < 0 || data->ncon > kContacts ||
    !copy_finite(output.qpos, data->qpos) || !copy_finite(output.qvel, data->qvel)) return false;
  const double observed_phase = data->userdata[0], observed_armed = data->userdata[1];
  if (!std::isfinite(observed_phase) || std::floor(observed_phase) != observed_phase ||
    observed_phase < -1 || observed_phase > 8 ||
    (observed_armed != 0 && observed_armed != 1)) return false;
  output.sequence = ++state.sequence;
  output.stamp_ns = static_cast<std::uint64_t>(std::llround(data->time * 1e9));
  output.time_sec = data->time;
  output.force_evaluation_sec = std::max(0., data->time - model->opt.timestep);
  output.phase = static_cast<int>(observed_phase);
  output.armed = observed_armed == 1;
  output.heartbeat_fresh = output.armed && output.phase >= 0;
  for (int i = 0; i < 12; ++i) output.arms[i] = output.qpos[state.arm_addresses[i]];
  for (int i = 0; i < 2; ++i) output.fingers[i] = output.qpos[state.finger_addresses[i]];
  // This is a private copy. Reconstruct post-integration kinematics without
  // rerunning a solver or changing live qpos, ctrl, contacts or warm starts.
  mj_copyData(state.copy.get(), model, data);
  mj_kinematics(model, state.copy.get());
  mj_comPos(model, state.copy.get());
  mj_comVel(model, state.copy.get());
  output.geom_count = model->ngeom;
  for (int i = 0; i < model->ngeom; ++i) {
    if (!copy_finite(output.geoms[i].position, state.copy->geom_xpos + 3 * i) ||
      !copy_finite(output.geoms[i].rotation, state.copy->geom_xmat + 9 * i)) return false;
  }
  mjtNum quaternion[4]{};
  mju_mat2Quat(quaternion, state.copy->site_xmat + 9 * state.tcp_id);
  if (!copy_finite(output.tcp.position, state.copy->site_xpos + 3 * state.tcp_id) ||
    !copy_finite(output.tcp.quaternion, quaternion) ||
    !copy_finite(output.block.position, state.copy->xpos + 3 * state.block_id) ||
    !copy_finite(output.block.quaternion, state.copy->xquat + 4 * state.block_id)) return false;
  mjtNum velocity[6]{};
  mj_objectVelocity(model, state.copy.get(), mjOBJ_BODY, state.block_id, velocity, 0);
  if (!copy_finite(output.block_angular, velocity) || !copy_finite(output.block_linear, velocity + 3)) return false;
  output.contact_count = data->ncon;
  for (int i = 0; i < data->ncon; ++i) {
    const auto & contact = data->contact[i];
    if (contact.geom[0] < 0 || contact.geom[0] >= model->ngeom ||
      contact.geom[1] < 0 || contact.geom[1] >= model->ngeom) return false;
    mjtNum force[6]{};
    if (contact.efc_address >= 0) mj_contactForce(model, data, i, force);
    if (!std::isfinite(force[0]) || std::abs(force[0]) > 1e6) return false;
    output.contacts[i] = {contact.geom[0], contact.geom[1], std::abs(double(force[0]))};
  }
  return true;
}

std::string serialize(const Transport & state, const Snapshot & frame)
{
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(17)
      << "{\"schema_version\":1,\"kind\":\"ros-grasp-observation-v1\",\"run_id\":" << quoted(state.run_id)
      << ",\"model_sha256\":" << quoted(state.model_sha256)
      << ",\"plugin_sha256\":\"" << MANIPULATION_GRASP_PLUGIN_SHA256 << "\",\"native_version\":" << quoted(mj_versionString())
      << ",\"sequence\":" << frame.sequence << ",\"time_sec\":" << frame.time_sec
      << ",\"stamp_ns\":\"" << frame.stamp_ns << "\",\"stamp_semantics\":\"rounded_native_mjData_time\""
      << ",\"capture_monotonic_ns\":\"" << frame.capture_monotonic_ns
      << "\",\"capture_clock_semantics\":\"linux_CLOCK_MONOTONIC\""
      << ",\"phase\":" << frame.phase << ",\"armed\":" << (frame.armed ? "true" : "false")
      << ",\"heartbeat_fresh\":" << (frame.heartbeat_fresh ? "true" : "false")
      << ",\"pose_sample_sec\":" << frame.time_sec << ",\"force_evaluation_sec\":" << frame.force_evaluation_sec
      << ",\"force_timing_verified\":false,\"force_time_semantics\":\"nominal last mj_step evaluation; invalid when native physics is held\""
      << ",\"local_overwrites_or_skips\":0,\"observation_delivery\":{\"schema_version\":1,"
      << "\"mode\":\"bounded_ordered_fifo\",\"capacity\":" << kQueueCapacity
      << ",\"max_drain_per_callback\":" << kMaximumDrain
      << ",\"max_source_age_ns\":\"" << kMaximumSourceAgeNs
      << "\",\"overflow_count\":" << state.overflow_count.load(std::memory_order_relaxed)
      << ",\"source_sha256\":\"" << MANIPULATION_GRASP_FIFO_SHA256 << "\"},\"qpos\":";
  vector(out, frame.qpos);out << ",\"qvel\":";vector(out, frame.qvel);
  out << ",\"arm_positions_rad\":{";
  for (int i = 0; i < 12; ++i) {if (i) out << ',';out << quoted(state.arm_names[i]) << ':' << frame.arms[i];}
  out << "},\"finger_positions_m\":{";
  for (int i = 0; i < 2; ++i) {if (i) out << ',';out << quoted(state.finger_names[i]) << ':' << frame.fingers[i];}
  out << "},\"geoms\":[";
  for (int i = 0; i < frame.geom_count; ++i) {
    if (i) out << ',';
    out << "{\"id\":" << i << ",\"position\":";vector(out, frame.geoms[i].position);
    out << ",\"rotation\":";vector(out, frame.geoms[i].rotation);out << '}';
  }
  out << "],\"tcp\":{\"position\":";vector(out, frame.tcp.position);
  out << ",\"quaternion_wxyz\":";vector(out, frame.tcp.quaternion);
  out << "},\"block\":{\"position\":";vector(out, frame.block.position);
  out << ",\"quaternion_wxyz\":";vector(out, frame.block.quaternion);
  out << ",\"linear_velocity_m_sec\":";vector(out, frame.block_linear);
  out << ",\"angular_velocity_rad_sec\":";vector(out, frame.block_angular);
  out << "},\"contacts\":[";
  for (int i = 0; i < frame.contact_count; ++i) {
    if (i) out << ',';
    const auto & contact = frame.contacts[i];
    out << "{\"geom1\":" << contact.geom1 << ",\"geom2\":" << contact.geom2
        << ",\"geom1_name\":" << quoted(state.geom_names[contact.geom1])
        << ",\"geom2_name\":" << quoted(state.geom_names[contact.geom2])
        << ",\"normal_force_n\":" << contact.normal << '}';
  }
  out << "]}";
  return out.str();
}

void publish_pending(const std::shared_ptr<Transport> & state)
{
  if (state->closed.load(std::memory_order_acquire) ||
    state->publishing.test_and_set(std::memory_order_acquire)) return;
  struct PublishingDone {Transport & state;~PublishingDone() {state.publishing.clear(std::memory_order_release);}} done{*state};
  report_transport_fault(*state);
  if (state->fault.load(std::memory_order_acquire)) return;
  for (std::size_t drained = 0; drained < kMaximumDrain; ++drained) {
    if (state->closed.load(std::memory_order_acquire) || state->fault.load(std::memory_order_acquire)) break;
    try {
      const bool consumed = state->frames.try_read([&](const Snapshot & frame) {
        // Original capture age survives queueing. No old frame is re-stamped
        // on publication; Python independently checks this same source clock.
        if (!capture_age_valid(frame.capture_monotonic_ns, steady_ns(), kMaximumSourceAgeNs)) {
          transport_fault(*state, 3);return;
        }
        Message message;
        message.data = serialize(*state, frame);
        if (message.data.size() > kJsonBytes) {transport_fault(*state, 4);return;}
        rclcpp::Publisher<Message>::SharedPtr publisher;
        {
          // This mutex is never shared with update or pre_step. A callback's
          // local publisher remains alive while cleanup closes admission.
          std::lock_guard<std::mutex> lock(state->publisher_mutex);
          if (!state->closed.load(std::memory_order_acquire)) publisher = state->publisher;
        }
        // Serialization and publisher acquisition must not extend the age.
        if (!capture_age_valid(frame.capture_monotonic_ns, steady_ns(), kMaximumSourceAgeNs)) {
          transport_fault(*state, 3);return;
        }
        if (publisher && !state->fault.load(std::memory_order_acquire)) publisher->publish(message);
      });
      if (!consumed) break;
    } catch (...) {
      // Do not throw through an executor callback during a transport failure
      // or concurrent ROS teardown. The next pre_step stops native admission.
      if (!state->closed.load(std::memory_order_acquire)) transport_fault(*state, 5);
      break;
    }
  }
  report_transport_fault(*state);
}

}  // namespace

class ObservationPlugin final : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel * model, mjData * /*data*/) override
  {
    cleanup();
    const auto * run = std::getenv("MANIPULATION_GRASP_RUN_ID");
    const auto * digest = std::getenv("MANIPULATION_GRASP_MODEL_SHA256");
    auto state = std::make_shared<Transport>();
    if (!node || !hex(run, 32) || !hex(digest, 64) || !model_contract(model, *state)) return false;
    state->run_id = run;state->model_sha256 = digest;state->model = model;
    state->copy.reset(mj_makeData(model));
    if (!state->copy) return false;
    state_ = state;
    node_ = std::move(node);
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    const auto observation_qos = rclcpp::QoS(rclcpp::KeepLast(kQueueCapacity)).reliable().durability_volatile();
    state->publisher = node_->create_publisher<Message>("/workcell/grasp_observation", observation_qos);
    phase_subscription_ = node_->create_subscription<Message>("/workcell/grasp_phase", qos,
      [state](Message::ConstSharedPtr message) {
        if (state->closed.load(std::memory_order_acquire) || message->data.size() != 34 ||
          message->data.compare(0, 32, state->run_id) != 0 || message->data[32] != ':' ||
          message->data[33] < '0' || message->data[33] > '8') return;
        const auto phase_value = static_cast<std::uint64_t>(message->data[33] - '0' + 1);
        state->command.store((steady_ms() << 4) | phase_value, std::memory_order_release);
      });
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), [state]() {publish_pending(state);});
    return true;
  }

  void update(const mjModel * model, mjData * data) override
  {
    const auto state = state_;
    if (!state || state->closed.load(std::memory_order_acquire)) return;
    if (state->fault.load(std::memory_order_acquire)) return;
    const auto capture_ns = steady_ns();
    if (!capture_ns) {transport_fault(*state, 6);return;}
    const auto now_ms = capture_ns / 1000000;
    if (now_ms >= state->last_capture_ms && now_ms - state->last_capture_ms < 50) return;
    state->last_capture_ms = now_ms;
    const auto result = state->frames.try_write([&](Snapshot & frame) {
      frame.capture_monotonic_ns = capture_ns;
      return state->sequence < (std::uint64_t{1} << 53) - 1 && fill(*state, model, data, frame);
    });
    if (result == FifoWrite::full) {
      state->overflow_count.fetch_add(1, std::memory_order_relaxed);
      transport_fault(*state, 1);
    } else if (result == FifoWrite::invalid) {
      transport_fault(*state, 2);
    }
  }

  void pre_step(mjData * data) override
  {
    const auto state = state_;
    if (!state || !data || state->closed.load(std::memory_order_acquire)) return;
    const auto current = phase(*state, steady_ms());
    data->userdata[0] = state->fault.load(std::memory_order_acquire) ? -1 : current.first;
    data->userdata[1] = state->fault.load(std::memory_order_relaxed) || current.second ? 1 : 0;
  }

  void on_reset(mjData * data) override
  {
    // A physical reset is not an authorized new grasp execution. Keep the
    // monitor fail-closed until a new process/run identity is created.
    const auto state = state_;
    if (!state) return;
    transport_fault(*state, 7);
    if (data) {data->userdata[0] = -1;data->userdata[1] = 1;}
  }

  void cleanup() override
  {
    const auto state = state_;
    if (state) {
      state->closed.store(true, std::memory_order_release);
      state->frames.close();
    }
    if (timer_) {timer_->cancel();timer_.reset();}
    phase_subscription_.reset();
    if (state) {
      std::lock_guard<std::mutex> lock(state->publisher_mutex);
      state->publisher.reset();
    }
    node_.reset();
    // Keep state_ stable for any last control hook. Timer/subscription
    // callbacks own only Transport, and release it when they finish.
  }

  ~ObservationPlugin() override {cleanup();}

private:
  std::shared_ptr<Transport> state_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<Message>::SharedPtr phase_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace manipulation_grasp

PLUGINLIB_EXPORT_CLASS(manipulation_grasp::ObservationPlugin,
  mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
