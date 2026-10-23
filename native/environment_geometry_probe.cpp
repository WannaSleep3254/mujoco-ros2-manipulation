// Read-only installed-environment geometry probe. No ROS, renderer or mj_step.
#include <mujoco/mujoco.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>

namespace {
struct ModelDeleter { void operator()(mjModel* p) const { mj_deleteModel(p); } };
struct DataDeleter { void operator()(mjData* p) const { mj_deleteData(p); } };
template<class T> void array(const T* values, int count) {
  std::cout << '[';
  for (int i = 0; i < count; ++i) {
    if (!std::isfinite(static_cast<double>(values[i]))) throw std::runtime_error("Nonfinite geometry");
    if (i) std::cout << ',';
    std::cout << values[i];
  }
  std::cout << ']';
}
bool identifier(const std::string& value) {
  return !value.empty() && value.size() < 128 && value[0] >= 'a' && value[0] <= 'z'
    && std::all_of(value.begin(), value.end(), [](char c) {
      return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_';
    });
}
}

int main(int argc, char** argv) {
  if (argc != 3) { std::cerr << "Expected XML and complete named-state file\n"; return 2; }
  try {
    if (mj_version() != mjVERSION_HEADER || std::string(mj_versionString()) != "3.12.0")
      throw std::runtime_error("Expected pinned MuJoCo 3.12.0");
    char error[2048]{};
    std::unique_ptr<mjModel, ModelDeleter> model(mj_loadXML(argv[1], nullptr, error, sizeof(error)));
    if (!model) throw std::runtime_error(error);
    const auto* m = model.get();
    if (m->ngeom > 10000 || m->nq > 512 || m->nmeshvert > 2000000) throw std::runtime_error("Geometry bounds exceeded");
    std::unique_ptr<mjData, DataDeleter> data(mj_makeData(m));
    if (!data) throw std::runtime_error("Could not allocate data");
    std::ifstream input(argv[2]); int count = 0;
    if (!(input >> count) || count != m->nq || count != m->njnt) throw std::runtime_error("Incomplete scalar joint coverage");
    std::set<int> addresses;
    for (int i = 0; i < count; ++i) {
      std::string name; double value;
      if (!(input >> name >> value) || !std::isfinite(value)) throw std::runtime_error("Invalid named joint");
      const int j = mj_name2id(m, mjOBJ_JOINT, name.c_str());
      if (j < 0 || (m->jnt_type[j] != mjJNT_HINGE && m->jnt_type[j] != mjJNT_SLIDE)
          || !addresses.insert(m->jnt_qposadr[j]).second) throw std::runtime_error("Invalid or duplicate scalar joint");
      data->qpos[m->jnt_qposadr[j]] = value;
    }
    std::string extra;
    if (input >> extra) throw std::runtime_error("Unexpected state data");
    mj_forward(m, data.get());
    std::cout << std::setprecision(17) << "{\"engine\":\"MuJoCo " << mj_versionString()
      << "\",\"physics_steps\":0,\"home_contacts\":" << data->ncon << ",\"geoms\":[";
    bool first = true;
    for (int g = 0; g < m->ngeom; ++g) {
      const char* raw = mj_id2name(m, mjOBJ_GEOM, g);
      const std::string name(raw ? raw : "");
      if (name.rfind("environment_", 0) != 0) continue;
      if (!identifier(name)) throw std::runtime_error("Invalid environment identifier");
      if (!first) std::cout << ',';
      first = false;
      std::cout << "{\"name\":\"" << name << "\",\"type\":" << m->geom_type[g]
        << ",\"group\":" << m->geom_group[g] << ",\"contype\":" << m->geom_contype[g]
        << ",\"conaffinity\":" << m->geom_conaffinity[g] << ",\"position\":";
      array(data->geom_xpos + 3*g, 3);
      std::cout << ",\"rotation\":"; array(data->geom_xmat + 9*g, 9);
      std::cout << ",\"half_size\":"; array(m->geom_size + 3*g, 3);
      if (m->geom_type[g] == mjGEOM_MESH) {
        const int mesh = m->geom_dataid[g];
        if (mesh < 0 || m->mesh_vertnum[mesh] < 1) throw std::runtime_error("Missing environment mesh");
        double low[3], high[3];
        std::fill_n(low, 3, std::numeric_limits<double>::infinity());
        std::fill_n(high, 3, -std::numeric_limits<double>::infinity());
        for (int v = m->mesh_vertadr[mesh]; v < m->mesh_vertadr[mesh] + m->mesh_vertnum[mesh]; ++v) {
          for (int row = 0; row < 3; ++row) {
            double world = data->geom_xpos[3*g+row];
            for (int col = 0; col < 3; ++col) world += data->geom_xmat[9*g+3*row+col] * m->mesh_vert[3*v+col];
            low[row] = std::min(low[row], world); high[row] = std::max(high[row], world);
          }
        }
        std::cout << ",\"world_bbox_min_m\":"; array(low, 3);
        std::cout << ",\"world_bbox_max_m\":"; array(high, 3);
      }
      std::cout << '}';
    }
    std::cout << "]}\n";
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
