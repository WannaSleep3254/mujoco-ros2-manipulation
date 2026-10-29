// Static native MuJoCo geometry for the opt-in ROS grasp display. No mj_step,
// renderer, ROS connection, controller or live-state reconstruction exists here.
#include <mujoco/mujoco.h>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef MANIPULATION_GRASP_SCENE_EXPORT_SHA256
#error MANIPULATION_GRASP_SCENE_EXPORT_SHA256 is required
#endif

namespace {
constexpr int kGeoms = 256, kMeshes = 256, kVertices = 2000000, kIndices = 6000000;
constexpr mjtSize kModelBytes = 256 * 1024 * 1024;
constexpr std::size_t kJsonBytes = 128 * 1024 * 1024;
struct ModelDelete {void operator()(mjModel* value) const {if(value) mj_deleteModel(value);}};
struct DataDelete {void operator()(mjData* value) const {if(value) mj_deleteData(value);}};
struct Scene {
  mjvScene value;
  Scene() {mjv_defaultScene(&value);}
  ~Scene() {mjv_freeScene(&value);}
};
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
std::string quoted(const std::string& value) {
  require(value.size() <= 256, "Unbounded compiled element name");
  std::ostringstream out;
  constexpr char digits[] = "0123456789abcdef";
  out << '"';
  for (unsigned char ch : value) {
    if (ch == '"' || ch == '\\') out << '\\' << ch;
    else if (ch < 32) out << "\\u00" << digits[ch >> 4] << digits[ch & 15];
    else out << ch;
  }
  out << '"';
  return out.str();
}
std::string name(const mjModel* model, mjtObj kind, int id) {
  const char* value = mj_id2name(model, kind, id);
  return quoted(value ? value : "");
}
template<class T>
void vector(std::ostringstream& out, const T* values, int count, double maximum = 1e9) {
  out << '[';
  for (int i = 0; i < count; ++i) {
    const double value = static_cast<double>(values[i]);
    require(std::isfinite(value) && std::abs(value) <= maximum, "Nonfinite or unbounded native geometry");
    if (i) out << ',';
    out << value;
  }
  out << ']';
}
const char* geom_type(int value) {
  switch (value) {
    case mjGEOM_PLANE: return "plane";
    case mjGEOM_SPHERE: return "sphere";
    case mjGEOM_CAPSULE: return "capsule";
    case mjGEOM_CYLINDER: return "cylinder";
    case mjGEOM_BOX: return "box";
    case mjGEOM_MESH: return "mesh";
    default: throw std::runtime_error("Unsupported native display geometry");
  }
}
bool hexadecimal(const std::string& value) {
  if(value.size() != 64) return false;
  for (char ch : value) if (!((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f'))) return false;
  return true;
}
void contract(const mjModel* model) {
  require(model->nq == 21 && model->nv == 20 && model->nu == 14 && model->njnt == 15 && model->nuserdata == 2,
          "Expected the full fixed ROS grasp state contract");
  require(!model->nflex && !model->nskin && !model->nhfield && !model->nmocap && !model->nplugin,
          "Deforming, heightfield, mocap or executable geometry is unsupported");
  require(model->ngeom > 0 && model->ngeom <= kGeoms && model->nmesh <= kMeshes &&
          model->nmeshvert <= kVertices && model->nmeshface <= kIndices / 3,
          "Native display exceeds geometry resource limits");
  const int policy = mj_name2id(model, mjOBJ_TEXT, "manipulation_grasp_policy");
  require(policy >= 0 && model->text_size[policy] <= 128 &&
          std::string(model->text_data + model->text_adr[policy]) == "fr5_parallel_block_v1",
          "Missing fixed grasp policy marker");
  for (const char* item : {"fr5_block_geom", "fr5_table_geom", "fr5_left_finger_geom", "fr5_right_finger_geom"})
    require(mj_name2id(model, mjOBJ_GEOM, item) >= 0, "Missing fixed grasp geometry");
  require(mj_name2id(model, mjOBJ_SITE, "fr5_grasp_tcp") >= 0, "Missing fixed grasp TCP");
  const mjtSize bytes = mj_sizeModel(model);
  require(bytes > 0 && bytes <= kModelBytes, "Compiled native model exceeds byte limit");
}
void identity(std::ostream& out) {
  out << "{\"schema_version\":1,\"native_version\":" << quoted(mj_versionString())
      << ",\"source_sha256\":\"" << MANIPULATION_GRASP_SCENE_EXPORT_SHA256 << "\"}";
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--identity") {
      identity(std::cout);std::cout << '\n';return 0;
    }
    require(argc == 4, "Usage: grasp_scene_export model.xml model_sha256 fresh-output-directory");
    const std::filesystem::path input(argv[1]), output(argv[3]);
    require(hexadecimal(argv[2]), "Expected lowercase model SHA256 supplied by the checked owner");
    require(std::filesystem::is_regular_file(input) && std::filesystem::file_size(input) <= 16 * 1024 * 1024,
            "Expected bounded native XML input");
    require(std::filesystem::is_directory(output) && std::filesystem::is_empty(output),
            "Select an empty private output directory");
    char error[2048]{};
    std::unique_ptr<mjModel, ModelDelete> model(mj_loadXML(argv[1], nullptr, error, sizeof(error)));
    if (!model) throw std::runtime_error(error);
    contract(model.get());
    std::unique_ptr<mjData, DataDelete> data(mj_makeData(model.get()));
    require(data != nullptr, "Could not allocate native static geometry data");
    if (model->nkey) mj_resetDataKeyframe(model.get(), data.get(), 0);
    // Static FK only. In particular there is no dynamics solve and no mj_step.
    mj_kinematics(model.get(), data.get());
    mj_comPos(model.get(), data.get());
    std::vector<std::array<float, 4>> colors(static_cast<std::size_t>(model->ngeom));
    for (int g = 0; g < model->ngeom; ++g)
      for (int i = 0; i < 4; ++i) colors[static_cast<std::size_t>(g)][static_cast<std::size_t>(i)] = model->geom_rgba[4*g+i];
    // Resolve the same CPU material precedence as the existing CompiledScene.
    mjvOption options;mjv_defaultOption(&options);
    for (auto& group : options.geomgroup) group = 1;
    mjvCamera camera;mjv_defaultCamera(&camera);
    Scene scene;
    mjv_makeScene(model.get(), &scene.value, model->ngeom + model->nsite + 16);
    mjv_updateScene(model.get(), data.get(), &options, nullptr, &camera, mjCAT_ALL, &scene.value);
    for (int i = 0; i < scene.value.ngeom; ++i) {
      const auto& visual = scene.value.geoms[i];
      if (visual.objtype == mjOBJ_GEOM && visual.objid >= 0 && visual.objid < model->ngeom)
        for (int j = 0; j < 4; ++j)
          colors[static_cast<std::size_t>(visual.objid)][static_cast<std::size_t>(j)] = visual.rgba[j];
    }
    const auto compiled = output / "compiled.mjb";
    mj_saveModel(model.get(), compiled.string().c_str(), nullptr, 0);
    require(std::filesystem::is_regular_file(compiled) &&
            std::filesystem::file_size(compiled) == static_cast<std::uintmax_t>(mj_sizeModel(model.get())),
            "Native compiled-model snapshot was not completely written");
    std::ostringstream out;
    out.imbue(std::locale::classic());out << std::setprecision(17);
    out << "{\"schema_version\":1,\"kind\":\"native-ros-grasp-static-scene-v1\",\"native_version\":"
        << quoted(mj_versionString()) << ",\"source_sha256\":\"" << MANIPULATION_GRASP_SCENE_EXPORT_SHA256
        << "\",\"model_sha256\":" << quoted(argv[2]) << ",\"physics_steps\":0,\"compiled_bytes\":"
        << mj_sizeModel(model.get()) << ",\"dimensions\":{\"nq\":" << model->nq << ",\"nv\":" << model->nv
        << ",\"nu\":" << model->nu << ",\"njnt\":" << model->njnt << "},\"qpos_layout\":[";
    for (int j = 0; j < model->njnt; ++j) {
      if (j) out << ',';
      const int kind = model->jnt_type[j];
      require(kind == mjJNT_HINGE || kind == mjJNT_SLIDE || kind == mjJNT_FREE, "Unsupported native state coordinate");
      out << "{\"name\":" << name(model.get(), mjOBJ_JOINT, j) << ",\"type\":\""
          << (kind == mjJNT_HINGE ? "hinge" : kind == mjJNT_SLIDE ? "slide" : "free")
          << "\",\"qpos_address\":" << model->jnt_qposadr[j] << ",\"qpos_width\":"
          << (kind == mjJNT_FREE ? 7 : 1) << ",\"velocity_address\":" << model->jnt_dofadr[j] << '}';
    }
    out << "],\"initial_qpos\":";vector(out, data->qpos, model->nq);
    out << ",\"bodies\":[";
    for (int b = 0; b < model->nbody; ++b) {
      if (b) out << ',';
      out << "{\"id\":" << b << ",\"parent_id\":" << model->body_parentid[b]
          << ",\"name\":" << name(model.get(), mjOBJ_BODY, b) << '}';
    }
    out << "],\"meshes\":[";
    for (int mesh = 0; mesh < model->nmesh; ++mesh) {
      if (mesh) out << ',';
      const int vertices = model->mesh_vertnum[mesh], faces = model->mesh_facenum[mesh];
      require(vertices >= 3 && faces >= 1, "Mesh has no compiled triangle geometry");
      out << "{\"id\":" << mesh << ",\"vertices\":[";
      for (int v = 0; v < vertices; ++v) {
        if (v) out << ',';
        vector(out, model->mesh_vert + 3*(model->mesh_vertadr[mesh]+v), 3);
      }
      out << "],\"faces\":[";
      for (int f = 0; f < faces; ++f) {
        if (f) out << ',';
        out << '[';
        for (int i = 0; i < 3; ++i) {
          const int index = model->mesh_face[3*(model->mesh_faceadr[mesh]+f)+i];
          require(index >= 0 && index < vertices, "Native mesh contains an invalid triangle index");
          if (i) out << ',';
          out << index;
        }
        out << ']';
      }
      out << "]}";
      require(static_cast<std::size_t>(out.tellp()) <= kJsonBytes, "Native geometry JSON exceeds byte limit");
    }
    out << "],\"geoms\":[";
    for (int g = 0; g < model->ngeom; ++g) {
      if (g) out << ',';
      const bool mesh = model->geom_type[g] == mjGEOM_MESH;
      require(model->geom_group[g] >= 0 && model->geom_group[g] <= 5, "Unsupported native display group");
      if (mesh) require(model->geom_dataid[g] >= 0 && model->geom_dataid[g] < model->nmesh, "Missing native display mesh");
      out << "{\"id\":" << g << ",\"name\":" << name(model.get(), mjOBJ_GEOM, g)
          << ",\"body_id\":" << model->geom_bodyid[g] << ",\"type\":" << quoted(geom_type(model->geom_type[g]))
          << ",\"group\":" << model->geom_group[g] << ",\"mesh_id\":" << (mesh ? model->geom_dataid[g] : -1)
          << ",\"size\":";vector(out, model->geom_size+3*g, 3);
      out << ",\"rgba\":";vector(out, colors[static_cast<std::size_t>(g)].data(), 4, 1.);
      out << ",\"initial_position\":";vector(out, data->geom_xpos+3*g, 3);
      out << ",\"initial_rotation\":";vector(out, data->geom_xmat+9*g, 9, 1.000001);
      out << '}';
    }
    out << "]}";
    const std::string json = out.str();
    require(json.size() <= kJsonBytes, "Native geometry JSON exceeds byte limit");
    std::cout << json << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "grasp_scene_export: " << error.what() << '\n';return 1;
  }
}
