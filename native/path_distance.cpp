// Offline kinematic distance/FK evaluator using the same pinned engine as ROS.
#include <mujoco/mujoco.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <vector>
#ifndef MANIPULATION_DISTANCE_SHA256
#error MANIPULATION_DISTANCE_SHA256 is required
#endif
namespace {
struct Context {
  mjModel* m = nullptr;
  mjData* d = nullptr;
  std::vector<int> addresses, geoms[2];
  int tips[2]{};
  ~Context() { if (d) mj_deleteData(d); if (m) mj_deleteModel(m); }
};
bool descendant(const mjModel* m, int body, int root) {
  while (body) { if (body == root) return true; body = m->body_parentid[body]; }
  return false;
}
}
extern "C" {
const char* path_distance_source() { return MANIPULATION_DISTANCE_SHA256; }
const char* path_distance_version() { return mj_versionString(); }
void* path_distance_open(const char* xml, const char* const* joints, int count,
                        const char* const* roots, const char* const* tips,
                        char* error, int error_size) {
  try {
    auto c = std::make_unique<Context>();
    c->m = mj_loadXML(xml, nullptr, error, error_size);
    if (!c->m) return nullptr;
    c->d = mj_makeData(c->m);
    if (!c->d || count != c->m->nq || count <= 0) throw std::runtime_error("Invalid model joint coverage");
    for (int n = 0; n < count; ++n) {
      const int j = mj_name2id(c->m, mjOBJ_JOINT, joints[n]);
      if (j < 0 || (c->m->jnt_type[j] != mjJNT_HINGE && c->m->jnt_type[j] != mjJNT_SLIDE))
        throw std::runtime_error("Unknown or non-scalar joint");
      c->addresses.push_back(c->m->jnt_qposadr[j]);
    }
    auto sorted = c->addresses; std::sort(sorted.begin(), sorted.end());
    if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end())
      throw std::runtime_error("Duplicate joint ownership");
    int root_ids[2];
    for (int arm = 0; arm < 2; ++arm) {
      root_ids[arm] = mj_name2id(c->m, mjOBJ_BODY, roots[arm]);
      c->tips[arm] = mj_name2id(c->m, mjOBJ_BODY, tips[arm]);
      if (root_ids[arm] <= 0 || c->tips[arm] <= 0 || !descendant(c->m, c->tips[arm], root_ids[arm]))
        throw std::runtime_error("Invalid root/tip frame");
      for (int g = 0; g < c->m->ngeom; ++g)
        if ((c->m->geom_contype[g] || c->m->geom_conaffinity[g]) && descendant(c->m, c->m->geom_bodyid[g], root_ids[arm]))
          c->geoms[arm].push_back(g);
      if (c->geoms[arm].empty()) throw std::runtime_error("No collision geometry for arm");
    }
    if (descendant(c->m, root_ids[0], root_ids[1]) || descendant(c->m, root_ids[1], root_ids[0]))
      throw std::runtime_error("Overlapping robot roots");
    return c.release();
  } catch (const std::exception& e) { std::snprintf(error, static_cast<size_t>(error_size), "%s", e.what()); return nullptr; }
}
void path_distance_close(void* handle) { delete static_cast<Context*>(handle); }
const char* path_distance_body(void* handle, int geom) {
  const auto* c = static_cast<Context*>(handle);
  if (!c || geom < 0 || geom >= c->m->ngeom) return nullptr;
  return mj_id2name(c->m, mjOBJ_BODY, c->m->geom_bodyid[geom]);
}
int path_distance_eval(void* handle, const double* q, int count, double* output, int* pair) {
  auto* c = static_cast<Context*>(handle);
  if (!c || count != static_cast<int>(c->addresses.size())) return 0;
  for (int n = 0; n < count; ++n) {
    if (!std::isfinite(q[n])) return 0;
    c->d->qpos[c->addresses[n]] = q[n];
  }
  mj_forward(c->m, c->d);
  output[0] = 100; pair[0] = pair[1] = -1;
  for (int a : c->geoms[0]) for (int b : c->geoms[1]) {
    if (!((c->m->geom_contype[a] & c->m->geom_conaffinity[b]) ||
          (c->m->geom_contype[b] & c->m->geom_conaffinity[a]))) continue;
    double nearest[6]{};
    const double distance = mj_geomDistance(c->m, c->d, a, b, output[0], nearest);
    if (!std::isfinite(distance)) return 0;
    if (distance < output[0]) {
      output[0] = distance; pair[0] = a; pair[1] = b;
      std::copy(nearest, nearest + 6, output + 1);
    }
  }
  for (int arm = 0; arm < 2; ++arm)
    std::copy(c->d->xpos + 3*c->tips[arm], c->d->xpos + 3*c->tips[arm] + 3, output + 7 + arm*3);
  return pair[0] >= 0 && std::all_of(output, output + 13, [](double v) { return std::isfinite(v); });
}
}
