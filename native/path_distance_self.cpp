// Included by the additive native distance translation unit after Context.
// The historical cross-arm implementation and its source pin remain intact.
#ifndef MANIPULATION_SELF_DISTANCE_SHA256
#error MANIPULATION_SELF_DISTANCE_SHA256 is required
#endif

namespace {
constexpr size_t max_self_pairs = 4096;
struct SelfContext {
  std::vector<std::pair<int, int>> pairs;
  int counts[4]{};  // collision geoms left/right, eligible pairs left/right
  bool state_ready = false;
};
std::map<void*, std::shared_ptr<SelfContext>> self_contexts;
std::mutex self_contexts_mutex;

std::shared_ptr<SelfContext> self_context(void* handle) {
  const std::lock_guard<std::mutex> lock(self_contexts_mutex);
  const auto found = self_contexts.find(handle);
  return found == self_contexts.end() ? nullptr : found->second;
}
void self_forget(void* handle) {
  const std::lock_guard<std::mutex> lock(self_contexts_mutex);
  self_contexts.erase(handle);
}
void self_state_ready(void* handle, bool ready) {
  const auto context = self_context(handle);
  if (context) context->state_ready = ready;
}
bool self_body_excluded(const mjModel* m, int first, int second) {
  if (first > second) std::swap(first, second);
  const unsigned signature = (static_cast<unsigned>(first) << 16) + static_cast<unsigned>(second);
  for (int i = 0; i < m->nexclude; ++i)
    if (static_cast<unsigned>(m->exclude_signature[i]) == signature) return true;
  return false;
}
bool self_pair_enabled(const mjModel* m, int first, int second) {
  if (!((m->geom_contype[first] & m->geom_conaffinity[second]) ||
        (m->geom_contype[second] & m->geom_conaffinity[first]))) return false;
  const int a = m->geom_bodyid[first], b = m->geom_bodyid[second];
  const int wa = m->body_weldid[a], wb = m->body_weldid[b];
  if (wa == wb || self_body_excluded(m, a, b)) return false;
  // Native adjacent-weld filtering has a world-weld exception.
  if (!(m->opt.disableflags & mjDSBL_FILTERPARENT) && wa && wb &&
      (m->body_weldid[m->body_parentid[wa]] == wb ||
       m->body_weldid[m->body_parentid[wb]] == wa)) return false;
  return true;
}
bool self_distance_supported(int type) {
  return type == mjGEOM_PLANE || (type >= mjGEOM_SPHERE && type <= mjGEOM_MESH);
}
}  // namespace

extern "C" {
const char* path_distance_self_source() { return MANIPULATION_SELF_DISTANCE_SHA256; }

int path_distance_self_enable(void* handle, const char* const* roots,
                              int* counts, char* error, int error_size) {
  auto* c = static_cast<Context*>(handle);
  if (!c || !roots || !counts || !error || error_size <= 0) return 0;
  self_forget(handle);
  try {
    if (c->m->nflex || c->m->npair || c->m->nmocap || c->m->nplugin)
      throw std::runtime_error("Self distance does not support flex, mocap, plugins or explicit contact geom pairs");
    auto context = std::make_shared<SelfContext>();
    for (int arm = 0; arm < 2; ++arm) {
      if (!roots[arm]) throw std::runtime_error("Missing self-collision robot root");
      const int root = mj_name2id(c->m, mjOBJ_BODY, roots[arm]);
      if (root <= 0 || !descendant(c->m, c->tips[arm], root))
        throw std::runtime_error("Self-collision root differs from the base evaluator");
      std::vector<int> geoms;
      for (int geom = 0; geom < c->m->ngeom; ++geom)
        if ((c->m->geom_contype[geom] || c->m->geom_conaffinity[geom]) &&
            descendant(c->m, c->m->geom_bodyid[geom], root)) {
          if (!self_distance_supported(c->m->geom_type[geom]))
            throw std::runtime_error("Unsupported self-collision distance geometry");
          geoms.push_back(geom);
        }
      if (geoms != c->geoms[arm])
        throw std::runtime_error("Self-collision root differs from the base evaluator");
      context->counts[arm] = static_cast<int>(geoms.size());
      for (size_t a = 0; a < geoms.size(); ++a) for (size_t b = a+1; b < geoms.size(); ++b) {
        if (!self_pair_enabled(c->m, geoms[a], geoms[b])) continue;
        if (context->pairs.size() >= max_self_pairs)
          throw std::runtime_error("Self-collision pair count exceeds 4096");
        context->pairs.emplace_back(geoms[a], geoms[b]);
        ++context->counts[2+arm];
      }
      if (!context->counts[2+arm])
        throw std::runtime_error("No eligible self-collision pairs for a robot arm");
    }
    std::copy(context->counts, context->counts+4, counts);
    const std::lock_guard<std::mutex> lock(self_contexts_mutex);
    self_contexts.emplace(handle, std::move(context));
    return 1;
  } catch (const std::exception& failure) {
    std::snprintf(error, static_cast<size_t>(error_size), "%s", failure.what());
    return 0;
  }
}

// No mj_forward or mj_step: consume only the preceding successful base FK.
int path_distance_self_eval(void* handle, double* output, int* pair) {
  const auto context = self_context(handle);
  if (!context || !context->state_ready || !output || !pair) return 0;
  const auto* c = static_cast<Context*>(handle);
  output[0] = std::numeric_limits<double>::max();pair[0] = pair[1] = -1;
  for (const auto& candidate : context->pairs) {
    double nearest[6]{};
    const double distance = mj_geomDistance(c->m, c->d, candidate.first, candidate.second, output[0], nearest);
    if (!std::isfinite(distance)) return 0;
    if (distance < output[0]) {
      output[0] = distance;pair[0] = candidate.first;pair[1] = candidate.second;
      std::copy(nearest, nearest+6, output+1);
    }
  }
  return pair[0] >= 0 && std::all_of(output, output+7, [](double value) { return std::isfinite(value); });
}
}  // extern "C"
