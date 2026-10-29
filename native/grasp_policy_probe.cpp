// Real-engine policy regression fixture. Pose edits below are fault injection
// in this test executable only; the production observer never writes qpos.
#include <memory>
#include <string>
#include "step_monitor.hpp"

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  char error[1024]{};
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
    mj_loadXML(argv[1],nullptr,error,sizeof(error)),mj_deleteModel);
  if (!model) {std::fprintf(stderr,"%s\n",error);return 3;}
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()),mj_deleteData);
  mj_resetDataKeyframe(model.get(),data.get(),0);
  const std::string mode=argv[2];
  const int block=mj_name2id(model.get(),mjOBJ_GEOM,"fr5_block_geom");
  const int table=mj_name2id(model.get(),mjOBJ_GEOM,"fr5_table_geom");
  const int finger=mj_name2id(model.get(),mjOBJ_GEOM,"fr5_left_finger_geom");
  manipulation::GraspContactPolicy policy;policy.prepare(model.get());
  if (mode=="matrix") {
    if (!policy.valid()) return 4;
    for (int phase=0;phase<=8;++phase) {
      const bool finger_allowed=phase>=2&&phase<=6;
      const bool table_allowed=phase==0||phase==1||phase==2||phase==3||phase==5||phase==6||phase==7;
      if (policy.allowed(block,finger,phase)!=finger_allowed ||
          policy.allowed(table,block,phase)!=table_allowed || policy.allowed(table,finger,phase)) return 5;
    }
    return 0;
  }
  data->userdata[0]=mode=="transport_table"?4:mode=="cancel"?8:mode=="expired"?-1:0;
  data->userdata[1]=mode=="idle"?0:1;
  if (mode=="unexpected") {
    mj_forward(model.get(),data.get());
    const int body=mj_name2id(model.get(),mjOBJ_BODY,"left_forearm_link");
    const int joint=mj_name2id(model.get(),mjOBJ_JOINT,"fr5_block_free");
    if (body<0||joint<0) return 6;
    std::copy_n(data->xpos+3*body,3,data->qpos+model->jnt_qposadr[joint]);
  }
  manipulation::StepMonitor monitor;
  std::vector<mjtNum> held;
  uint64_t integrated=0;
  for (int i=0;i<150;++i) {
    const bool before=monitor.latched();
    if (before&&held.empty()) held.assign(data->qpos,data->qpos+model->nq);
    if (monitor.step(model.get(),data.get())) ++integrated;
    if (before&&!std::equal(held.begin(),held.end(),data->qpos)) return 7;
  }
  monitor.finish(integrated);
  if (mode=="idle") return monitor.latched()?8:0;
  if (!monitor.latched()) return 9;
  if ((mode=="cancel"||mode=="expired"||mode=="unverified")&&integrated!=0) return 10;
  return 0;
}
