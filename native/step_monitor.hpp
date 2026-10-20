// Per-step diagnostics for the pinned native MuJoCo engine. No ROS sampling.
#pragma once
#include <mujoco/mujoco.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#ifndef MANIPULATION_MONITOR_SHA256
#error "Build the monitor with its source SHA-256"
#endif

namespace manipulation {
inline std::string json_string(const std::string &value) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\')
      out << '\\' << c;
    else if (c < 32)
      out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);
    else
      out << c;
  }
  out << '"';
  return out.str();
}

// Physics-thread policy. ROS callbacks only increment an atomic sequence on
// changed /clock stamps; repeated messages cannot refresh the steady deadline.
class ClockWatchdog {
  bool running_ = false, progressed_ = false;
  uint64_t sequence_ = 0;
  double last_progress_ = 0;

 public:
  static constexpr double timeout = 1.0, startup_timeout = 5.0;
  bool stalled(uint64_t sequence, bool running, double steady_now) {
    if (!running) {
      running_ = false;
      return false;
    }
    if (!running_) {
      running_ = true;
      sequence_ = sequence;
      progressed_ = sequence > 0;
      last_progress_ = steady_now;
    } else if (sequence != sequence_) {
      sequence_ = sequence;
      progressed_ = true;
      last_progress_ = steady_now;
    }
    return steady_now - last_progress_ >=
           (progressed_ ? timeout : startup_timeout);
  }
};

class StepMonitor {
  using Clock = std::chrono::steady_clock;
  struct Pair {
    uint64_t occurrences = 0, active = 0;
    double first = 0, last = 0, distance = 1e30, normal_force = 0;
  };
  struct Timing {
    // 10us buckets, bounded memory; overflow percentiles are null, never
    // understated.
    std::array<uint64_t, 4097> bins{};
    uint64_t count = 0;
    double total = 0, maximum = 0;
    void add(double value) {
      ++count;
      total += value;
      maximum = std::max(maximum, value);
      ++bins[std::min<size_t>(4096, static_cast<size_t>(value / 0.00001))];
    }
    std::string percentile(double p) const {
      if (!count) return "null";
      const auto rank =
          static_cast<uint64_t>(std::ceil(static_cast<double>(count) * p));
      uint64_t sum = 0;
      for (size_t i = 0; i < bins.size(); ++i) {
        sum += bins[i];
        if (sum >= rank) {
          if (i == 4096) return "null";
          std::ostringstream out;
          out << std::setprecision(12) << static_cast<double>(i + 1) * 0.00001;
          return out.str();
        }
      }
      return "null";
    }
    void write(std::ostream &out) const {
      out << "{\"count\":" << count << ",\"mean_sec\":"
          << (count ? total / static_cast<double>(count) : 0)
          << ",\"max_sec\":" << maximum
          << ",\"p50_upper_sec\":" << percentile(.50)
          << ",\"p95_upper_sec\":" << percentile(.95)
          << ",\"p99_upper_sec\":" << percentile(.99)
          << ",\"overflow_count\":" << bins.back() << '}';
    }
  };
  std::string path_, run_id_;
  const mjModel *model_ = nullptr;
  Clock::time_point first_wall_, last_wall_;
  uint64_t steps_ = 0, contact_steps_ = 0, contacts_ = 0, active_ = 0;
  uint64_t invalid_ = 0, discontinuities_ = 0, overflow_ = 0;
  uint64_t engine_steps_ = 0;
  double sim_seconds_ = 0, first_time_ = 0, last_time_ = 0;
  bool finished_ = false;
  bool stop_enabled_ = false, latched_ = false;
  std::string stop_reason_;
  uint64_t stop_step_ = 0, hold_ticks_ = 0;
  double stop_time_ = 0, hold_time_ = 0;
  std::vector<mjtNum> held_qpos_, held_mocap_pos_, held_mocap_quat_;
  void freeze(const mjModel *m, mjData *d) const {
    std::copy(held_qpos_.begin(), held_qpos_.end(), d->qpos);
    std::fill_n(d->qvel, m->nv, 0.0);
    std::fill_n(d->qacc, m->nv, 0.0);
    std::fill_n(d->ctrl, m->nu, 0.0);
    std::fill_n(d->act, m->na, 0.0);
    std::fill_n(d->qfrc_applied, m->nv, 0.0);
    std::fill_n(d->xfrc_applied, m->nbody * 6, 0.0);
    std::copy(held_mocap_pos_.begin(), held_mocap_pos_.end(), d->mocap_pos);
    std::copy(held_mocap_quat_.begin(), held_mocap_quat_.end(), d->mocap_quat);
    d->time = hold_time_;
    mj_kinematics(m, d);
    mj_comPos(m, d);
  }
  std::map<std::pair<int, int>, Pair> pairs_;
  Timing physics_, monitor_;
  static double seconds(Clock::duration value) {
    return std::chrono::duration<double>(value).count();
  }
  void inspect(const mjModel *m, const mjData *d, double contact_time,
               double post_time) {
    if (!steps_) {
      first_time_ = contact_time;
      first_wall_ = Clock::now();
    }
    if (!std::isfinite(post_time) || !std::isfinite(contact_time))
      ++invalid_;
    else {
      if (steps_ && std::abs(contact_time - last_time_) > 1e-8)
        ++discontinuities_;
      const double delta = post_time - contact_time;
      if (std::abs(delta - m->opt.timestep) > 1e-8) ++discontinuities_;
      sim_seconds_ += delta;
    }
    last_time_ = post_time;
    ++steps_;
    if (d->ncon) ++contact_steps_;
    for (int j = 0; j < m->nq; ++j)
      if (!std::isfinite(d->qpos[j])) ++invalid_;
    for (int j = 0; j < m->nv; ++j)
      if (!std::isfinite(d->qvel[j])) ++invalid_;
    for (int j = 0; j < m->nu; ++j)
      if (!std::isfinite(d->ctrl[j])) ++invalid_;
    for (int j = 0; j < d->ncon; ++j) {
      const auto &c = d->contact[j];
      ++contacts_;
      const bool active = c.efc_address >= 0;
      if (active) ++active_;
      if (c.geom[0] < 0 || c.geom[1] < 0) {
        ++invalid_;
        continue;
      }
      auto key = std::minmax(c.geom[0], c.geom[1]);
      if (!pairs_.count(key) && pairs_.size() >= 256) {
        ++overflow_;
        continue;
      }
      auto &pair = pairs_[key];
      if (!pair.occurrences) pair.first = contact_time;
      pair.last = contact_time;
      ++pair.occurrences;
      if (active) ++pair.active;
      if (std::isfinite(c.dist))
        pair.distance = std::min(pair.distance, double(c.dist));
      else
        ++invalid_;
      if (active) {
        mjtNum force[6]{};
        mj_contactForce(m, d, j, force);
        if (std::isfinite(force[0]))
          pair.normal_force =
              std::max(pair.normal_force, std::abs(double(force[0])));
        else
          ++invalid_;
      }
    }
    model_ = m;
  }
  void write() const {
    if (path_.empty()) return;
    const std::string temporary = path_ + ".tmp." + std::to_string(getpid());
    std::ofstream out(temporary);
    if (!out) {
      std::fprintf(stderr, "Cannot write step diagnostics: %s\n",
                   path_.c_str());
      return;
    }
    const double wall = steps_ ? seconds(last_wall_ - first_wall_) : 0;
    out << std::setprecision(17)
        << "{\"schema_version\":1,\"run_id\":" << json_string(run_id_)
        << ",\"monitor_source_sha256\":"
        << json_string(MANIPULATION_MONITOR_SHA256)
        << ",\"mujoco_version\":" << json_string(mj_versionString())
        << ",\"pid\":" << getpid()
        << ",\"complete\":" << (finished_ ? "true" : "false")
        << ",\"steps\":" << steps_ << ",\"engine_steps\":" << engine_steps_
        << ",\"timestep_sec\":" << (model_ ? model_->opt.timestep : 0)
        << ",\"integrator\":" << (model_ ? model_->opt.integrator : -1)
        << ",\"first_contact_evaluation_time_sec\":" << first_time_
        << ",\"last_state_time_sec\":" << last_time_
        << ",\"simulated_sec\":" << sim_seconds_
        << ",\"wall_span_sec\":" << wall << ",\"paced_real_time_factor\":"
        << (wall > 0 ? sim_seconds_ / wall : 0)
        << ",\"contact_steps\":" << contact_steps_
        << ",\"contacts\":" << contacts_ << ",\"active_contacts\":" << active_
        << ",\"invalid_values\":" << invalid_
        << ",\"time_discontinuities\":" << discontinuities_
        << ",\"pair_overflow\":" << overflow_ << ",\"physics_step\":";
    physics_.write(out);
    out << ",\"monitor_step\":";
    monitor_.write(out);
    out << ",\"pairs\":[";
    bool comma = false;
    for (const auto &item : pairs_) {
      if (comma) out << ',';
      comma = true;
      const auto &p = item.second;
      out << "{\"geom_ids\":[" << item.first.first << ',' << item.first.second
          << "],\"geoms\":[";
      for (int k = 0; k < 2; ++k) {
        const int id = k ? item.first.second : item.first.first;
        const char *name = mj_id2name(model_, mjOBJ_GEOM, id);
        if (k) out << ',';
        out << json_string(name ? name : "<unnamed>");
      }
      out << "],\"bodies\":[";
      for (int k = 0; k < 2; ++k) {
        const int id = k ? item.first.second : item.first.first;
        const char *name =
            mj_id2name(model_, mjOBJ_BODY, model_->geom_bodyid[id]);
        if (k) out << ',';
        out << json_string(name ? name : "world");
      }
      out << "],\"occurrences\":" << p.occurrences << ",\"active\":" << p.active
          << ",\"first_sec\":" << p.first << ",\"last_sec\":" << p.last
          << ",\"min_distance_m\":" << p.distance
          << ",\"max_normal_force_N\":" << p.normal_force << '}';
    }
    out << "],\"stop_enabled\":" << (stop_enabled_ ? "true" : "false")
        << ",\"stop_latched\":" << (latched_ ? "true" : "false")
        << ",\"stop_reason\":" << json_string(stop_reason_)
        << ",\"stop_step\":" << stop_step_
        << ",\"stop_time_sec\":" << stop_time_
        << ",\"hold_ticks\":" << hold_ticks_
        << ",\"hold_time_sec\":" << hold_time_ << ",\"stopped_qpos\":[";
    for (size_t j = 0; j < held_qpos_.size(); ++j) {
      if (j) out << ',';
      out << held_qpos_[j];
    }
    out << "]}\n";
    out.close();
    if (!out || std::rename(temporary.c_str(), path_.c_str()))
      std::fprintf(stderr, "Cannot publish step diagnostics: %s\n",
                   path_.c_str());
  }

 public:
  StepMonitor() {
    const char *stop = std::getenv("MANIPULATION_CONTACT_STOP");
    stop_enabled_ = stop && std::string(stop) == "1";
    const char *path = std::getenv("MANIPULATION_DIAGNOSTICS_PATH");
    const char *id = std::getenv("MANIPULATION_DIAGNOSTICS_RUN_ID");
    if (path && *path && id && *id) {
      path_ = path;
      run_id_ = id;
    }
  }
  ~StepMonitor() { write(); }
  bool latched() const { return latched_; }
  bool enabled() const { return stop_enabled_; }
  const std::string &stop_reason() const { return stop_reason_; }
  bool latch(const mjModel *m, mjData *d, const std::string &reason) {
    if (!stop_enabled_ || latched_) return false;
    latched_ = true;
    model_ = m;
    stop_reason_ = reason;
    stop_step_ = steps_;
    stop_time_ = hold_time_ = d->time;
    held_qpos_.assign(d->qpos, d->qpos + m->nq);
    held_mocap_pos_.assign(d->mocap_pos, d->mocap_pos + 3 * m->nmocap);
    held_mocap_quat_.assign(d->mocap_quat, d->mocap_quat + 4 * m->nmocap);
    freeze(m, d);
    mj_forward(m, d);
    freeze(m, d);
    return true;
  }
  // Also called before the viewer reset/paused paths: UI edits cannot release
  // the latch.
  void restore(const mjModel *m, mjData *d) const {
    if (latched_) freeze(m, d);
  }
  bool step(const mjModel *m, mjData *d) {
    if (latched_) {
      hold_time_ += m->opt.timestep;
      freeze(m, d);
      ++hold_ticks_;
      return false;  // Clock-only tick; no physics integration after contact.
    }
    if (path_.empty() && !stop_enabled_) {
      mj_step(m, d);
      return true;
    }
    const auto before = Clock::now();
    const double start = d->time;
    mj_step(m, d);
    const auto after = Clock::now();
    // Contacts describe the dynamics evaluation before integration, not
    // qpos(t+dt).
    inspect(m, d, start, d->time);
    const auto inspected = Clock::now();
    if (steps_ == 1) first_wall_ = before;
    last_wall_ = inspected;
    physics_.add(seconds(after - before));
    monitor_.add(seconds(inspected - after));
    if (stop_enabled_ && d->ncon > 0) latch(m, d, "contact");
    return true;
  }
  void finish(uint64_t engine_steps) {
    engine_steps_ = engine_steps;
    finished_ = true;
  }
};
}  // namespace manipulation
