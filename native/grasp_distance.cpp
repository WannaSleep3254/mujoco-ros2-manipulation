// Offline candidate FK for the explicitly checked ROS grasp target. Never writes
// to the live model/data and never substitutes kinematic attachment for physics.
#include <mujoco/mujoco.h>
#include "grasp_policy.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#ifndef MANIPULATION_GRASP_DISTANCE_SHA256
#error MANIPULATION_GRASP_DISTANCE_SHA256 is required
#endif

namespace {
struct Context {
  mjModel* m = nullptr;
  mjData* d = nullptr;
  manipulation::GraspContactPolicy policy;
  std::vector<std::pair<int, int>> pairs;
  int tcp = -1, block_address = -1;
  ~Context() { if (d) mj_deleteData(d); if (m) mj_deleteModel(m); }
};
bool excluded(const mjModel* m, int a, int b) {
  if (a > b) std::swap(a, b);
  const unsigned signature = (static_cast<unsigned>(a) << 16) + static_cast<unsigned>(b);
  for (int i = 0; i < m->nexclude; ++i)
    if (static_cast<unsigned>(m->exclude_signature[i]) == signature) return true;
  return false;
}
bool pair_enabled(const mjModel* m, int a, int b) {
  if (!((m->geom_contype[a] & m->geom_conaffinity[b]) ||
        (m->geom_contype[b] & m->geom_conaffinity[a]))) return false;
  const int ba = m->geom_bodyid[a], bb = m->geom_bodyid[b];
  const int wa = m->body_weldid[ba], wb = m->body_weldid[bb];
  if (wa == wb || excluded(m, ba, bb)) return false;
  // MuJoCo's default adjacent-welded-body filtering, with world exception.
  if (!(m->opt.disableflags & mjDSBL_FILTERPARENT) && wa && wb &&
      (m->body_weldid[m->body_parentid[wa]] == wb ||
       m->body_weldid[m->body_parentid[wb]] == wa)) return false;
  return true;
}
bool unit_quaternion(const double* q) {
  double n = 0;
  for (int i = 0; i < 4; ++i) { if (!std::isfinite(q[i])) return false; n += q[i]*q[i]; }
  return std::abs(n-1.) < 1e-5;
}
void fk(Context* c) {
  mj_kinematics(c->m, c->d);
  mj_comPos(c->m, c->d);
}
}  // namespace

extern "C" {
const char* grasp_distance_source() { return MANIPULATION_GRASP_DISTANCE_SHA256; }
const char* grasp_distance_policy_source() { return MANIPULATION_GRASP_POLICY_SHA256; }
const char* grasp_distance_version() { return mj_versionString(); }
void* grasp_distance_open(const char* xml, char* error, int error_size) {
  if (!xml || !error || error_size <= 0) return nullptr;
  try {
    auto c = std::make_unique<Context>();
    c->m = mj_loadXML(xml, nullptr, error, error_size);
    if (!c->m) return nullptr;
    c->policy.prepare(c->m);
    if (!c->policy.present() || !c->policy.valid() || c->m->nflex || c->m->npair || c->m->nmocap)
      throw std::runtime_error("Expected checked rigid ROS grasp target without explicit pairs");
    c->d = mj_makeData(c->m);
    c->tcp = mj_name2id(c->m, mjOBJ_SITE, "fr5_grasp_tcp");
    const int block = mj_name2id(c->m, mjOBJ_JOINT, "fr5_block_free");
    if (!c->d || c->tcp < 0 || block < 0 || c->m->jnt_type[block] != mjJNT_FREE)
      throw std::runtime_error("Missing full-state grasp geometry");
    c->block_address = c->m->jnt_qposadr[block];
    for (int a = 0; a < c->m->ngeom; ++a) for (int b = a+1; b < c->m->ngeom; ++b) {
      if (!pair_enabled(c->m, a, b)) continue;
      for (int g : {a, b})
        if (!(c->m->geom_type[g] == mjGEOM_PLANE ||
              (c->m->geom_type[g] >= mjGEOM_SPHERE && c->m->geom_type[g] <= mjGEOM_MESH)))
          throw std::runtime_error("Unsupported grasp distance geometry");
      if (c->pairs.size() >= 4096) throw std::runtime_error("Grasp collision pair bound exceeded");
      c->pairs.emplace_back(a, b);
    }
    if (c->pairs.empty()) throw std::runtime_error("Empty grasp collision coverage");
    return c.release();
  } catch (const std::exception& e) {
    std::snprintf(error, static_cast<size_t>(error_size), "%s", e.what()); return nullptr;
  }
}
void grasp_distance_close(void* handle) { delete static_cast<Context*>(handle); }
int grasp_distance_pair_count(void* handle) {
  const auto* c = static_cast<Context*>(handle);
  return c ? static_cast<int>(c->pairs.size()) : 0;
}
int grasp_distance_geom_count(void* handle) {
  const auto* c = static_cast<Context*>(handle);
  return c ? c->m->ngeom : 0;
}
const char* grasp_distance_geom(void* handle, int id) {
  const auto* c = static_cast<Context*>(handle);
  return c && id >= 0 && id < c->m->ngeom ? mj_id2name(c->m, mjOBJ_GEOM, id) : nullptr;
}
// Optional relative pose is [TCP-local xyz, TCP-local wxyz]. It predicts only
// the held block's swept geometry for a candidate; it does not carry a live body.
int grasp_distance_eval(void* handle, const double* q, int count, int phase,
                        const double* relative_pose, double* output, int* pair,
                        double* tcp_pose) {
  auto* c = static_cast<Context*>(handle);
  if (!c || !q || count != c->m->nq || phase < 0 || phase > 7 || !output || !pair || !tcp_pose)
    return 0;
  for (int i = 0; i < count; ++i) if (!std::isfinite(q[i])) return 0;
  if (!unit_quaternion(q+c->block_address+3)) return 0;
  for (int j = 0; j < c->m->njnt; ++j) if (c->m->jnt_limited[j]) {
    const double value = q[c->m->jnt_qposadr[j]];
    if (value < c->m->jnt_range[2*j]-1e-6 || value > c->m->jnt_range[2*j+1]+1e-6) return 0;
  }
  std::copy(q, q+count, c->d->qpos);
  fk(c);
  std::copy(c->d->site_xpos+3*c->tcp, c->d->site_xpos+3*c->tcp+3, tcp_pose);
  mju_mat2Quat(tcp_pose+3, c->d->site_xmat+9*c->tcp);
  if (relative_pose) {
    if (phase < 3 || phase > 5 || !unit_quaternion(relative_pose+3)) return 0;
    for (int i = 0; i < 3; ++i)
      if (!std::isfinite(relative_pose[i]) || std::abs(relative_pose[i]) > .5) return 0;
    double rotated[3];
    mju_rotVecQuat(rotated, relative_pose, tcp_pose+3);
    for (int i = 0; i < 3; ++i) c->d->qpos[c->block_address+i] = tcp_pose[i]+rotated[i];
    mju_mulQuat(c->d->qpos+c->block_address+3, tcp_pose+3, relative_pose+3);
    fk(c);
  }
  output[0] = 100.; pair[0] = pair[1] = -1;
  for (const auto& candidate : c->pairs) {
    if (c->policy.allowed(candidate.first, candidate.second, phase)) continue;
    double nearest[6]{};
    const double d = mj_geomDistance(c->m, c->d, candidate.first, candidate.second, output[0], nearest);
    if (!std::isfinite(d)) return 0;
    if (d < output[0]) {
      output[0] = d; pair[0] = candidate.first; pair[1] = candidate.second;
      std::copy(nearest, nearest+6, output+1);
    }
  }
  return pair[0] >= 0 && std::all_of(output, output+7, [](double v) { return std::isfinite(v); });
}
}  // extern "C"
