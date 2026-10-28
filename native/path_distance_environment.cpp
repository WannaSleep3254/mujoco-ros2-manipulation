// Additive environment distances around the byte-pinned historical evaluator.
// Keep path_distance.cpp unchanged: public reproductions pin its exact source.
#define path_distance_eval path_distance_base_eval
#define path_distance_close path_distance_base_close
#include "path_distance.cpp"
#undef path_distance_eval
#undef path_distance_close

#include <limits>
#include <map>
#include <mutex>

#ifndef MANIPULATION_ENVIRONMENT_DISTANCE_SHA256
#error MANIPULATION_ENVIRONMENT_DISTANCE_SHA256 is required
#endif

namespace {
constexpr size_t max_environment_pairs = 4096;
struct EnvironmentContext {
  std::vector<std::pair<int, int>> pairs;
  int counts[4]{};
  bool state_ready = false;
};
std::map<void*, std::shared_ptr<EnvironmentContext>> environment_contexts;
std::mutex environment_contexts_mutex;

std::shared_ptr<EnvironmentContext> environment_context(void* handle) {
  const std::lock_guard<std::mutex> lock(environment_contexts_mutex);
  const auto found = environment_contexts.find(handle);
  return found == environment_contexts.end() ? nullptr : found->second;
}
bool body_excluded(const mjModel* m, int first, int second) {
  if (first > second) std::swap(first, second);
  const unsigned int signature = (static_cast<unsigned int>(first) << 16) +
                                static_cast<unsigned int>(second);
  for (int index = 0; index < m->nexclude; ++index)
    if (static_cast<unsigned int>(m->exclude_signature[index]) == signature) return true;
  return false;
}
bool distance_supported(int type) {
  return type == mjGEOM_PLANE || (type >= mjGEOM_SPHERE && type <= mjGEOM_MESH);
}
}  // namespace

extern "C" {
const char* path_distance_environment_source() {
  return MANIPULATION_ENVIRONMENT_DISTANCE_SHA256;
}

// Existing C ABI: same base implementation and results, with cache readiness
// stored only for handles whose caller explicitly enabled environment queries.
int path_distance_eval(void* handle, const double* q, int count, double* output, int* pair) {
  const auto environment = environment_context(handle);
  if (environment) environment->state_ready = false;
  const int result = path_distance_base_eval(handle, q, count, output, pair);
  if (environment) environment->state_ready = result != 0;
  return result;
}
void path_distance_close(void* handle) {
  {
    const std::lock_guard<std::mutex> lock(environment_contexts_mutex);
    environment_contexts.erase(handle);
  }
  path_distance_base_close(handle);
}

// Fixed-world rigid geometry only. Static geometry inside a robot subtree is
// its mounting structure. A mobile root's geometry remains a moving candidate.
int path_distance_environment_enable(void* handle, const char* const* roots,
                                     int* counts, char* error, int error_size) {
  auto* c = static_cast<Context*>(handle);
  if (!c || !roots || !counts || !error || error_size <= 0) return 0;
  {
    const std::lock_guard<std::mutex> lock(environment_contexts_mutex);
    environment_contexts.erase(handle);
  }
  try {
    if (c->m->nflex || c->m->npair)
      throw std::runtime_error("Environment distance does not support flex or explicit contact geom pairs");
    int root_ids[2]{};
    for (int arm = 0; arm < 2; ++arm) {
      if (!roots[arm]) throw std::runtime_error("Missing environment robot root");
      root_ids[arm] = mj_name2id(c->m, mjOBJ_BODY, roots[arm]);
      if (root_ids[arm] <= 0 || !descendant(c->m, c->tips[arm], root_ids[arm]))
        throw std::runtime_error("Environment robot root differs from the base evaluator");
      std::vector<int> base_geoms;
      for (int geom = 0; geom < c->m->ngeom; ++geom)
        if ((c->m->geom_contype[geom] || c->m->geom_conaffinity[geom]) &&
            descendant(c->m, c->m->geom_bodyid[geom], root_ids[arm])) base_geoms.push_back(geom);
      if (base_geoms != c->geoms[arm])
        throw std::runtime_error("Environment robot root differs from the base evaluator");
    }
    if (descendant(c->m, root_ids[0], root_ids[1]) || descendant(c->m, root_ids[1], root_ids[0]))
      throw std::runtime_error("Overlapping environment robot roots");
    std::vector<int> moving[2], fixed;
    for (int body = 1; body < c->m->nbody; ++body)
      if (!descendant(c->m, body, root_ids[0]) && !descendant(c->m, body, root_ids[1]) &&
          c->m->body_weldid[body] != 0)
        throw std::runtime_error("Movable geometry outside the two robot roots is unsupported");
    for (int geom = 0; geom < c->m->ngeom; ++geom) {
      if (!(c->m->geom_contype[geom] || c->m->geom_conaffinity[geom])) continue;
      const int body = c->m->geom_bodyid[geom];
      int arm = -1;
      for (int index = 0; index < 2; ++index)
        if (descendant(c->m, body, root_ids[index])) arm = index;
      if (arm >= 0 && c->m->body_weldid[body] == 0) continue;
      if (!distance_supported(c->m->geom_type[geom]))
        throw std::runtime_error("Unsupported collision geometry for environment distance");
      if (arm >= 0) moving[arm].push_back(geom);
      else fixed.push_back(geom);  // World body itself is a valid fixed owner.
    }
    if (fixed.empty()) throw std::runtime_error("No fixed environment collision geometry");
    auto environment = std::make_shared<EnvironmentContext>();
    for (int arm = 0; arm < 2; ++arm) {
      if (moving[arm].empty()) throw std::runtime_error("No moving collision geometry for a robot arm");
      size_t arm_pairs = 0;
      for (int robot : moving[arm]) for (int obstacle : fixed) {
        if (!((c->m->geom_contype[robot] & c->m->geom_conaffinity[obstacle]) ||
              (c->m->geom_contype[obstacle] & c->m->geom_conaffinity[robot]))) continue;
        if (body_excluded(c->m, c->m->geom_bodyid[robot], c->m->geom_bodyid[obstacle])) continue;
        if (environment->pairs.size() >= max_environment_pairs)
          throw std::runtime_error("Environment collision pair count exceeds 4096");
        environment->pairs.emplace_back(robot, obstacle);
        ++arm_pairs;
      }
      if (!arm_pairs) throw std::runtime_error("No eligible environment collision pairs for a robot arm");
    }
    environment->counts[0] = static_cast<int>(moving[0].size());
    environment->counts[1] = static_cast<int>(moving[1].size());
    environment->counts[2] = static_cast<int>(fixed.size());
    environment->counts[3] = static_cast<int>(environment->pairs.size());
    std::copy(environment->counts, environment->counts + 4, counts);
    const std::lock_guard<std::mutex> lock(environment_contexts_mutex);
    environment_contexts.emplace(handle, std::move(environment));
    return 1;
  } catch (const std::exception& failure) {
    std::snprintf(error, static_cast<size_t>(error_size), "%s", failure.what());
    return 0;
  }
}

// Reuse the immediately preceding successful base-evaluator FK snapshot. No
// second mj_forward, simulation step or controller operation is performed.
int path_distance_environment_eval(void* handle, double* output, int* pair) {
  const auto* c = static_cast<Context*>(handle);
  const auto environment = environment_context(handle);
  if (!c || !environment || !environment->state_ready || !output || !pair) return 0;
  output[0] = std::numeric_limits<double>::max();
  pair[0] = pair[1] = -1;
  for (const auto& candidate : environment->pairs) {
    double nearest[6]{};
    const double separation = mj_geomDistance(c->m, c->d, candidate.first, candidate.second,
                                             output[0], nearest);
    if (!std::isfinite(separation)) return 0;
    if (separation < output[0]) {
      output[0] = separation;
      pair[0] = candidate.first; pair[1] = candidate.second;
      std::copy(nearest, nearest + 6, output + 1);
    }
  }
  // No returned pair is an evaluation failure, never a large-distance success.
  return pair[0] >= 0 && std::all_of(output, output + 7, [](double v) { return std::isfinite(v); });
}
}  // extern "C"
