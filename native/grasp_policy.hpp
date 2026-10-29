// Opt-in physics-thread contact policy for the fixed FR5 parallel-block fixture.
// No ROS, allocation, file I/O or physical-state modification after initialization.
#pragma once
#include <mujoco/mujoco.h>
#include <array>
#include <cmath>
#include <cstring>
#include <string>

#ifndef MANIPULATION_GRASP_POLICY_SHA256
#define MANIPULATION_GRASP_POLICY_SHA256 ""
#endif

namespace manipulation {
class GraspContactPolicy {
  const mjModel *model_ = nullptr;
  bool present_ = false, valid_ = false;
  int block_ = -1, table_ = -1;
  std::array<int, 2> fingers_{{-1, -1}};
 public:
  void prepare(const mjModel *m) {
    if (m == model_) return;
    model_ = m;
    present_ = valid_ = false;
    const int text = mj_name2id(m, mjOBJ_TEXT, "manipulation_grasp_policy");
    if (text < 0) return;
    present_ = true;
    if (std::strcmp(m->text_data+m->text_adr[text], "fr5_parallel_block_v1")
        || std::strlen(MANIPULATION_GRASP_POLICY_SHA256) != 64
        || m->nuserdata != 2 || m->nq != 21 || m->nv != 20 || m->nu != 14 || m->njnt != 15) return;
    block_ = mj_name2id(m, mjOBJ_GEOM, "fr5_block_geom");
    table_ = mj_name2id(m, mjOBJ_GEOM, "fr5_table_geom");
    fingers_[0] = mj_name2id(m, mjOBJ_GEOM, "fr5_left_finger_geom");
    fingers_[1] = mj_name2id(m, mjOBJ_GEOM, "fr5_right_finger_geom");
    if (block_ < 0 || table_ < 0 || fingers_[0] < 0 || fingers_[1] < 0) return;
    const int free = mj_name2id(m, mjOBJ_JOINT, "fr5_block_free");
    if (free < 0 || m->jnt_type[free] != mjJNT_FREE) return;
    for (const char *name : {"fr5_left_finger_slide", "fr5_right_finger_slide"}) {
      const int id = mj_name2id(m, mjOBJ_JOINT, name);
      if (id < 0 || m->jnt_type[id] != mjJNT_SLIDE || !m->jnt_limited[id]
          || std::abs(m->jnt_range[2*id]) > 1e-12
          || std::abs(m->jnt_range[2*id+1]-.034) > 1e-12) return;
    }
    valid_ = true;
  }
  bool present() const { return present_; }
  bool valid() const { return valid_; }
  const char *before_step(const mjModel *m, const mjData *d) {
    prepare(m);
    if (!present_) return nullptr;
    if (!valid_) return "grasp_policy_invalid";
    const double phase = d->userdata[0], armed = d->userdata[1];
    if (!std::isfinite(phase) || std::floor(phase) != phase || phase < 0 || phase > 8
        || (armed != 0 && armed != 1) || (!armed && phase != 0)) return "grasp_heartbeat";
    if (phase == 8) return "grasp_cancel";
    return nullptr;
  }
  bool allowed(int a, int b, int phase) const {
    if (!present_ || !valid_) return false;
    if (a != block_ && b != block_) return false;
    const int other = a == block_ ? b : a;
    if (other == table_)
      return phase == 0 || phase == 1 || phase == 2 || phase == 3 || phase == 5 || phase == 6 || phase == 7;
    if (other == fingers_[0] || other == fingers_[1])
      return phase >= 2 && phase <= 6;
    return false;
  }
  bool forbidden_contact(const mjModel *m, const mjData *d) {
    prepare(m);
    if (!present_) return d->ncon > 0;  // Original whole-world policy.
    if (!valid_) return true;
    const int phase = static_cast<int>(d->userdata[0]);
    for (int i = 0; i < d->ncon; ++i)
      if (!allowed(d->contact[i].geom[0], d->contact[i].geom[1], phase)) return true;
    return false;
  }
};
}  // namespace manipulation
