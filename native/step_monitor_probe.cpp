// Regression probe: real engine, one deliberately transient contact, no ROS
// commands.
#include <memory>
#include <stdexcept>

#include "step_monitor.hpp"

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  char error[1024]{};
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
      mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
  if (!model) {
    std::fprintf(stderr, "%s\n", error);
    return 3;
  }
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
      mj_makeData(model.get()), mj_deleteData);
  const std::string mode = argv[2];
  if (model->nkey) mj_resetDataKeyframe(model.get(), data.get(), 0);
  uint64_t count = 24, integrated = 0;
  if (mode == "workcell_contact" || mode == "stop_workcell_contact") {
    count = mode == "workcell_contact" ? 1 : 24;
    // Pinned single-model targets collide when combined in this workcell.
    // Workcell coordinates are supplied by a simple joint/value file.
    std::ifstream pose(std::string(argv[1]) + ".pose");
    std::string name;
    double value;
    while (pose >> name >> value) {
      const int id = mj_name2id(model.get(), mjOBJ_JOINT, name.c_str());
      if (id < 0) return 4;
      data->qpos[model->jnt_qposadr[id]] = value;
    }
    if (!pose.eof()) return 5;
  }
  manipulation::StepMonitor monitor;
  for (uint64_t j = 0; j < count; ++j) {
    if (mode == "transient" || mode == "stop_transient") {
      // No mj_forward between the edits and step: detection must occur inside
      // mj_step.
      data->qpos[2] = j == 11 ? 0.05 : 1.0;
      std::fill_n(data->qvel, model->nv, 0.0);
    }
    const bool was_latched = monitor.latched();
    std::vector<mjtNum> before(data->qpos, data->qpos + model->nq);
    if (was_latched) {
      data->qpos[0] += 10;
      std::fill_n(data->qvel, model->nv, 100.0);
      std::fill_n(data->ctrl, model->nu, 999.0);
      data->time =
          0;  // A reset/control edit must not release or move a latched world.
    }
    if (monitor.step(model.get(), data.get())) ++integrated;
    if (was_latched && mode == "stop_workcell_contact" &&
        !std::equal(before.begin(), before.end(), data->qpos))
      return 6;
    if (monitor.latched() && std::any_of(data->qvel, data->qvel + model->nv,
                                         [](double v) { return v != 0; }))
      return 7;
  }
  if (monitor.latched() && std::any_of(data->ctrl, data->ctrl + model->nu,
                                       [](double v) { return v != 0; }))
    return 8;
  monitor.finish(integrated);
}
