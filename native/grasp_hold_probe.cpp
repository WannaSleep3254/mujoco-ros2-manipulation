// Offline diagnostic only: initialize an owned mjData from a recorded state,
// then hold fixed position-servo commands. No ROS, controller plugin, live state
// writes, welds or carried-pose reconstruction are used.
// State input (whitespace separated): nq nv nu native_time; nq qpos; nv qvel;
// nu position-servo controls; 3 original closed-reference position coordinates;
// 4 original closed-reference quaternion coordinates (wxyz). Caller preserves
// the CI input stream/hash, selected stable-lift and close-anchor sequences.
#include <mujoco/mujoco.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef MANIPULATION_GRASP_HOLD_PROBE_SHA256
#error MANIPULATION_GRASP_HOLD_PROBE_SHA256 is required
#endif

namespace {
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
struct ModelDelete { void operator()(mjModel* m) const { if (m) mj_deleteModel(m); } };
struct DataDelete { void operator()(mjData* d) const { if (d) mj_deleteData(d); } };
struct Evidence {
  double elapsed, position_error, angle_error, block_z, speed, left_force, right_force, table_force;
  double relative_position[3];
};
void relative_pose(const mjModel* m, const mjData* d, int tcp, int block,
                   double* position, double* quaternion) {
  double delta[3], tcp_quat[4], inverse[4];
  mju_sub3(delta, d->xpos + 3*block, d->site_xpos + 3*tcp);
  mju_mulMatTVec(position, d->site_xmat + 9*tcp, delta, 3, 3);
  mju_mat2Quat(tcp_quat, d->site_xmat + 9*tcp);
  mju_negQuat(inverse, tcp_quat);
  mju_mulQuat(quaternion, inverse, d->xquat + 4*block);
  (void)m;
}
Evidence sample(const mjModel* m, mjData* d, int tcp, int block, int block_geom,
                int left, int right, int table, const double* reference_position,
                const double* reference_quaternion, double start) {
  // Recompute geometry and force for this offline sample's current state. This
  // does not advance time and is separate from native callback timing claims.
  mj_forward(m, d);
  double relative[3], quaternion[4], delta[3], velocity[6];
  relative_pose(m, d, tcp, block, relative, quaternion);
  mju_sub3(delta, relative, reference_position);
  const double dot = std::abs(mju_dot(quaternion, reference_quaternion, 4));
  mj_objectVelocity(m, d, mjOBJ_BODY, block, velocity, 0);
  Evidence out{d->time-start, mju_norm3(delta), 2*std::acos(std::min(1., dot)),
               d->xpos[3*block+2], mju_norm3(velocity+3), 0., 0., 0., {0., 0., 0.}};
  std::copy(relative, relative+3, out.relative_position);
  for (int i=0; i<d->ncon; ++i) {
    const mjContact& c = d->contact[i];
    if (c.geom[0] != block_geom && c.geom[1] != block_geom) continue;
    const int other = c.geom[0] == block_geom ? c.geom[1] : c.geom[0];
    double force[6]{};
    mj_contactForce(m, d, i, force);
    if (other == left) out.left_force += force[0];
    if (other == right) out.right_force += force[0];
    if (other == table) out.table_force += force[0];
  }
  require(std::isfinite(out.position_error) && std::isfinite(out.angle_error)
          && std::isfinite(out.left_force) && std::isfinite(out.right_force),
          "Nonfinite offline observation");
  return out;
}
void emit(const Evidence& e) {
  std::cout << "{\"elapsed_sec\":" << e.elapsed
            << ",\"relative_position_error_m\":" << e.position_error
            << ",\"relative_angle_error_rad\":" << e.angle_error
            << ",\"relative_position_m\":[" << e.relative_position[0] << ','
            << e.relative_position[1] << ',' << e.relative_position[2] << ']'
            << ",\"block_z_m\":" << e.block_z << ",\"block_speed_m_s\":" << e.speed
            << ",\"finger_force_n\":[" << e.left_force << ',' << e.right_force << ']'
            << ",\"table_force_n\":" << e.table_force << '}';
}
}  // namespace

int main(int argc, char** argv) {
  try {
    require(argc == 7, "Usage: grasp_hold_probe model.xml state.txt seconds original|elliptic|elliptic10 low-friction|normal open|closed");
    require(std::string(mj_versionString()) == "3.12.0", "Expected native MuJoCo 3.12.0");
    const double duration = std::stod(argv[3]);
    require(std::isfinite(duration) && duration >= .1 && duration <= 60., "Duration must be .1..60 simulation seconds");
    const std::string cone = argv[4], friction = argv[5], fingers = argv[6];
    require(cone == "original" || cone == "elliptic" || cone == "elliptic10", "Unknown solver candidate");
    require(friction == "normal" || friction == "low-friction", "Unknown friction control");
    require(fingers == "open" || fingers == "closed", "Unknown finger control");
    char error[2048]{};
    std::unique_ptr<mjModel, ModelDelete> m(mj_loadXML(argv[1], nullptr, error, sizeof(error)));
    if (!m) throw std::runtime_error(error);
    require(m->nq <= 100 && m->nv <= 100 && m->nu <= 100 && m->nplugin == 0 && m->nmocap == 0,
            "Unexpected or executable diagnostic model");
    require(m->opt.timestep > 0 && m->opt.timestep <= .01, "Invalid diagnostic timestep");
    const int tcp=mj_name2id(m.get(), mjOBJ_SITE, "fr5_grasp_tcp");
    const int block=mj_name2id(m.get(), mjOBJ_BODY, "fr5_grasp_block");
    const int block_geom=mj_name2id(m.get(), mjOBJ_GEOM, "fr5_block_geom");
    const int left=mj_name2id(m.get(), mjOBJ_GEOM, "fr5_left_finger_geom");
    const int right=mj_name2id(m.get(), mjOBJ_GEOM, "fr5_right_finger_geom");
    const int table=mj_name2id(m.get(), mjOBJ_GEOM, "fr5_table_geom");
    require(std::min({tcp,block,block_geom,left,right,table}) >= 0, "Missing diagnostic grasp geometry");
    const int original_cone=m->opt.cone;
    const double original_impratio=m->opt.impratio;
    if (cone != "original") { m->opt.cone=mjCONE_ELLIPTIC; m->opt.impratio=cone=="elliptic" ? 5. : 10.; }
    if (friction == "low-friction") {
      for (int geom : {block_geom,left,right,table}) m->geom_friction[3*geom] *= .001;
    }
    std::unique_ptr<mjData, DataDelete> d(mj_makeData(m.get()));
    require(d != nullptr, "Could not allocate offline state");
    std::ifstream input(argv[2]);
    int nq=0,nv=0,nu=0;
    double start=0;
    require(bool(input>>nq>>nv>>nu>>start) && nq==m->nq && nv==m->nv && nu==m->nu,
            "Diagnostic input dimensions differ");
    require(std::isfinite(start) && start >= 0, "Invalid input time");
    auto read = [&](double* values, int count) {
      for (int i=0;i<count;++i) require(bool(input>>values[i]) && std::isfinite(values[i]), "Invalid diagnostic input scalar");
    };
    read(d->qpos,m->nq);read(d->qvel,m->nv);read(d->ctrl,m->nu);d->time=start;
    // Preserve the closed-grasp lifetime reference. Initializing at a later
    // stable-lift state must not silently reset the slip acceptance gate.
    double reference_position[3],reference_quaternion[4];
    read(reference_position,3);read(reference_quaternion,4);
    require(std::abs(mju_norm(reference_quaternion,4)-1.) < 1e-6, "Reference quaternion must be unit length");
    std::string trailing;require(!(input>>trailing), "Unexpected diagnostic input tail");
    if (fingers == "open") {
      for (const char* name : {"fr5_left_finger_servo","fr5_right_finger_servo"}) {
        const int id=mj_name2id(m.get(),mjOBJ_ACTUATOR,name);
        require(id >= 0,"Missing diagnostic finger actuator");d->ctrl[id]=0.;
      }
    }
    mj_forward(m.get(),d.get());
    std::vector<Evidence> samples;
    samples.push_back(sample(m.get(),d.get(),tcp,block,block_geom,left,right,table,
                             reference_position,reference_quaternion,start));
    double max_position=0.,max_angle=0.,first_position_failure=-1.,first_contact_failure=-1.;
    const auto wall_start=std::chrono::steady_clock::now();
    int steps=0;double next_sample=.25;
    while (d->time-start < duration-m->opt.timestep/2) {
      mj_step(m.get(),d.get());++steps;
      require(std::all_of(d->qpos,d->qpos+m->nq,[](double x){return std::isfinite(x);}), "Nonfinite integrated state");
      if (d->time-start+1e-9 >= next_sample || d->time-start >= duration-m->opt.timestep/2) {
        const auto e=sample(m.get(),d.get(),tcp,block,block_geom,left,right,table,
                             reference_position,reference_quaternion,start);
        samples.push_back(e);next_sample+=.25;
        max_position=std::max(max_position,e.position_error);max_angle=std::max(max_angle,e.angle_error);
        if (first_position_failure < 0 && e.position_error > .005) first_position_failure=e.elapsed;
        if (first_contact_failure < 0 && std::min(e.left_force,e.right_force) < .2) first_contact_failure=e.elapsed;
      }
    }
    const double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-wall_start).count();
    std::cout << std::setprecision(17) << "{\"schema_version\":1,\"kind\":\"offline-held-grasp-diagnostic\","
              << "\"native_version\":\"" << mj_versionString() << "\",\"source_sha256\":\""
              << MANIPULATION_GRASP_HOLD_PROBE_SHA256 << "\",\"live_state_writes\":false,"
              << "\"initialization\":\"recorded qpos/qvel and reconstructed fixed servo commands in owned mjData only\","
              << "\"reference\":\"provided original closed-grasp lifetime relative pose; not re-anchored at initialization\","
              << "\"original_cone\":" << original_cone << ",\"original_impratio\":" << original_impratio
              << ",\"cone\":" << m->opt.cone << ",\"impratio\":" << m->opt.impratio
              << ",\"friction_scale\":" << (friction=="normal"?1.:.001)
              << ",\"finger_open_control\":" << (fingers=="open"?"true":"false")
              << ",\"sample_period_sec\":0.25"
              << ",\"steps\":" << steps << ",\"timestep_sec\":" << m->opt.timestep
              << ",\"elapsed_sec\":" << d->time-start << ",\"wall_sec\":" << wall
              << ",\"max_relative_position_error_m\":" << max_position
              << ",\"max_relative_angle_error_rad\":" << max_angle
              << ",\"first_position_gate_failure_sec\":" << first_position_failure
              << ",\"first_contact_gate_failure_sec\":" << first_contact_failure << ",\"samples\":[";
    for (std::size_t i=0;i<samples.size();++i) {if(i)std::cout<<',';emit(samples[i]);}
    std::cout << "]}\n";
    return 0;
  } catch(const std::exception& error) {std::cerr << error.what() << '\n';return 2;}
}
