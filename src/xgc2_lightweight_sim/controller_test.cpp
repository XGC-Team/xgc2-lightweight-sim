// Numerical ABI coupling with the actual, separately built ctl-px4 ELF.
// No real FCU/ROS/network; this verifies the unchanged SMC lifecycle on the
// plant.
#include <xgc_rt.h>
#include <xgc-robotics-interfaces/robotics_interfaces_v1.h>
#include <xgc-lightweight-sim/simulation_records_v1.h>
#include <hover_thrust_estimator/native/hover_thrust_wire.h>
#include <multirotor_reference_trajectory/reference_wire_v1.h>

using hover_thrust_native::xgc_hover_thrust_v1;
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <dlfcn.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <stdexcept>
#include <sstream>
#include <vector>

struct Module {
  struct Sample {
    int64_t at;
    std::vector<uint8_t> data;
  };
  struct Link {
    uint32_t out;
    Module *target;
    uint32_t in;
  };
  void *library;
  void *instance;
  const xgc_plugin_descriptor *descriptor;
  xgc_host_api host{};
  std::map<std::string, uint32_t> ports;
  std::string source_path;
  std::map<uint32_t, std::deque<Sample>> inbox;
  std::map<uint32_t, std::vector<uint8_t>> last;
  std::map<uint32_t, uint64_t> publications;
  std::map<uint32_t, std::vector<Sample>> events;
  std::vector<uint8_t> popped;
  std::vector<Link> links;
  int64_t now{1000000000};
  int smc_outputs{0}, attitude_outputs{0};
  int attitude_command_outputs{0};
  Module(const char *path, const char *config) {
    source_path=path;
    library = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    if (!library) {
      std::cerr << dlerror() << "\n";
      std::abort();
    }
    auto get = reinterpret_cast<const xgc_plugin_descriptor *(*)()>(
        dlsym(library, "xgc_rt_plugin_v1"));
    descriptor = get();
    for (uint32_t i = 0; i < descriptor->port_count; ++i)
      ports[descriptor->ports[i].name] = i;
    host.abi_version = 1;
    host.abi_minor = 2;
    host.host = this;
    host.now = [](void *p) { return static_cast<Module *>(p)->now; };
    host.next = [](void *p, uint32_t port, xgc_sample_view *v) {
      auto &s = *static_cast<Module *>(p);
      auto &q = s.inbox[port];
      if (q.empty())
        return XGC_ERR_AGAIN;
      auto t = q.front().at;
      s.popped = std::move(q.front().data);
      q.pop_front();
      *v = {};
      v->data = s.popped.data();
      v->len = s.popped.size();
      v->t_rx = v->t_produce = v->t_tx = t;
      return XGC_OK;
    };
    host.publish = [](void *p, uint32_t port, uint64_t, const uint8_t *data,
                      uint32_t len) {
      auto &s = *static_cast<Module *>(p);
      s.last[port] = {data, data + len};
      ++s.publications[port];
      if (s.descriptor->ports[port].qos==XGC_QOS_EVENT)
        s.events[port].push_back({s.now,{data,data+len}});
      if (std::strcmp(s.descriptor->ports[port].name,"fcu_result")==0 && len==sizeof(xgc_fcu_result_v1)) {
        xgc_fcu_result_v1 r{}; std::memcpy(&r,data,sizeof r);
        std::cout << std::setprecision(17) << "fcu_result stamp=" << r.stamp
                  << " request_stamp=" << r.request_stamp << " id=" << r.request_id
                  << " robot=" << r.robot_index << " kind=" << r.kind
                  << " MAV_RESULT=" << r.result << std::endl;
      }
      if (std::strcmp(s.descriptor->ports[port].name, "setpoint") == 0 &&
          len == sizeof(xgc_position_target_v1)) {
        xgc_position_target_v1 v;
        std::memcpy(&v, data, len);
        if (v.type_mask == 3135)
          ++s.smc_outputs;
      }
      if (std::strcmp(s.descriptor->ports[port].name, "attitude_rate") == 0)
        ++s.attitude_outputs;
      if (std::strcmp(s.descriptor->ports[port].name, "attitude_command") == 0)
        ++s.attitude_command_outputs;
      for (auto &l : s.links)
        if (l.out == port)
          l.target->inbox[l.in].push_back({s.now, {data, data + len}});
      return XGC_OK;
    };
    host.log = [](void *, xgc_log_level level, const char *msg) {
      if (std::strstr(msg,"fs150_state_audit/1")) std::cout << msg << '\n';
      if (level >= XGC_LOG_WARN)
        std::cerr << msg << "\n";
    };
    host.request_degrade = [](void *, const char *reason) {
      std::cerr << "degrade " << reason << "\n";
    };
    host.request_recover = [](void *) {};
    host.node_id = [](void *) -> uint16_t { return 0; };
    host.port_origins = [](void *, uint32_t, uint16_t *out,
                           uint32_t cap) -> uint32_t {
      if (cap)
        out[0] = 0;
      return 1;
    };
    instance = descriptor->vtbl->create(&host);
    assert(instance);
    assert(descriptor->vtbl->configure(instance, config) == XGC_OK);
    assert(descriptor->vtbl->activate(instance) == XGC_OK);
  }
  ~Module() {
    descriptor->vtbl->deactivate(instance);
    descriptor->vtbl->destroy(instance);
    dlclose(library);
  }
  void link(const char *out, Module &target, const char *in) {
    if (std::strcmp(descriptor->ports[port(out)].schema_id,
                    target.descriptor->ports[target.port(in)].schema_id)!=0)
      throw std::runtime_error("schema mismatch: "+source_path+"/"+out+" -> "+target.source_path+"/"+in);
    links.push_back({port(out), &target, target.port(in)});
  }
  uint32_t port(const char *name) const {
    const auto at=ports.find(name);
    if (at==ports.end()) throw std::runtime_error("missing port "+std::string(name)+" in "+source_path);
    return at->second;
  }
  template <class T> void input(const char *name, const T &v) {
    auto *b = reinterpret_cast<const uint8_t *>(&v);
    inbox[ports.at(name)].push_back({now, {b, b + sizeof v}});
  }
  void input_bytes(const char *name, std::vector<uint8_t> bytes) {
    inbox[ports.at(name)].push_back({now,std::move(bytes)});
  }
  template <class T> T output(const char *name) {
    T v{};
    auto &b = last[ports.at(name)];
    if (!b.empty()) {
      assert(b.size() == sizeof v);
      std::memcpy(&v, b.data(), sizeof v);
    }
    return v;
  }
  void tick(int64_t t) {
    now = t;
    xgc_step_ctx c{};
    c.now = c.round_start = t;
    c.round = (t - 1000000000) / 1000000;
    c.round_advanced = 1;
    assert(descriptor->vtbl->step(instance, &c) == XGC_OK);
  }
  void command(const char *text) {
    char b[64]{};
    std::strcpy(b, text);
    input("command", b);
  }
};

using Vec = std::array<double, 3>;
using Quat = std::array<double, 4>; // wxyz, body FLU -> world ENU

void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

double norm(const Vec &v) { return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); }
Quat product(const Quat &a, const Quat &b) {
  return {a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
          a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
          a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
          a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
}
Quat conjugate(const Quat &q) { return {q[0], -q[1], -q[2], -q[3]}; }
Vec rotate(const Quat &q, const Vec &v) {
  const auto r = product(product(q, {0, v[0], v[1], v[2]}), conjugate(q));
  return {r[1], r[2], r[3]};
}
Quat aligned(Quat q, const Quat &reference) {
  double dot = 0;
  for (int j=0; j!=4; ++j) dot += q[j]*reference[j];
  if (dot < 0) for (auto &value : q) value = -value;
  return q;
}

struct FlightSample {
  xgc_pose_v1 pose;
  xgc_twist_v1 velocity;
  xgc_imu_v1 imu;
  xgc_fcu_extended_state_v1 extended;
  double custom_elapsed;
  std::string state;
  Quat q() const { return {pose.q_wxyz[0], pose.q_wxyz[1], pose.q_wxyz[2], pose.q_wxyz[3]}; }
};

struct Errors {
  std::vector<double> values;
  void add(double value) { require(std::isfinite(value), "nonfinite residual"); values.push_back(value); }
  double rms() const {
    double sum=0;
    for (auto value : values) sum += value*value;
    return std::sqrt(sum / values.size());
  }
  double p95() const {
    auto sorted = values;
    std::sort(sorted.begin(), sorted.end());
    return sorted.at(static_cast<size_t>(std::ceil(.95*sorted.size())) - 1);
  }
  void report(const char *name) const {
    require(!values.empty(), "no residual samples");
    std::cout << name << " samples=" << values.size() << " rms=" << rms()
              << " p95=" << p95() << " max=" << *std::max_element(values.begin(), values.end()) << "\n";
  }
};

// Original 1 m / 6 s quintic, with a smooth 0.25 m lateral excursion to
// exercise roll as well as pitch. The original (1,0,1) endpoint is unchanged.
xgc_position_target_v1 reference(double elapsed, double stamp) {
  const double q = std::clamp(elapsed/6.0, 0.0, 1.0), u = q*(1-q);
  xgc_position_target_v1 p{};
  p.stamp=stamp; p.coordinate_frame=1; p.type_mask=3072;
  p.position[0]=10*q*q*q-15*q*q*q*q+6*q*q*q*q*q;
  p.position[1]=16*u*u*u;
  p.position[2]=1;
  if (q<1) {
    p.velocity[0]=(30*q*q-60*q*q*q+30*q*q*q*q)/6;
    p.acceleration[0]=(60*q-180*q*q+120*q*q*q)/36;
    p.velocity[1]=48*u*u*(1-2*q)/6;
    p.acceleration[1]=96*(u*(1-2*q)*(1-2*q)-u*u)/36;
  }
  return p;
}

std::vector<uint8_t> sampled_reference(int64_t start) {
  xgc_ref_sampled_v1 head{};
  head.header.stamp_sec=head.start_sec=static_cast<uint32_t>(start/1000000000);
  head.header.stamp_nsec=head.start_nsec=static_cast<uint32_t>(start%1000000000);
  std::strcpy(head.header.frame_id,"map");
  head.trajectory_id=head.revision=1;
  head.sample_dt=.01; head.points_len=1201;
  std::vector<uint8_t> bytes(sizeof head+head.points_len*sizeof(xgc_ref_flat_point_v1));
  std::memcpy(bytes.data(),&head,sizeof head);
  const std::array<std::vector<double>,3> coefficients{{{0,0,0,10,-15,6},{0,0,0,16,-48,48,-16},{1}}};
  for (uint32_t i=0; i<head.points_len; ++i) {
    xgc_ref_flat_point_v1 point{};
    point.t_from_start=i*head.sample_dt;
    const double q=std::min(1.0,point.t_from_start/6.0);
    double *fields[]{point.position,point.velocity,point.acceleration,point.jerk,point.snap};
    for (int order=0; order<5; ++order) for (int axis=0; axis<3; ++axis) {
      if (order && point.t_from_start>=6) continue;
      const auto &coeff=coefficients[axis];
      for (size_t k=order; k<coeff.size(); ++k) {
        double factor=coeff[k]/std::pow(6.0,order);
        for (int d=0; d<order; ++d) factor*=k-d;
        fields[order][axis]+=factor*std::pow(q,int(k)-order);
      }
    }
    std::memcpy(bytes.data()+sizeof head+i*sizeof point,&point,sizeof point);
  }
  return bytes;
}

void verify_hte(const std::vector<xgc_hover_thrust_v1> &samples, double ratio,
                double initial) {
  require(!samples.empty(), "real HTE published no estimate");
  Errors tail;
  size_t used=0;
  double first= samples.front().hover_thrust, last=first;
  double latest_good_stamp=0;
  for (const auto &s : samples)
    if (s.state==12 && s.sample_used && s.flags==0) latest_good_stamp=s.stamp;
  for (const auto &s : samples) {
    require(std::isfinite(s.hover_thrust) && std::abs(s.initial_hover_thrust-initial)<1e-9,
            "HTE initial/value does not come from configured real estimator");
    if (s.state==12 && s.sample_used && s.flags==0) {
      ++used; last=s.hover_thrust;
      if (s.stamp>=latest_good_stamp-2) tail.add(std::abs(s.hover_thrust-ratio));
    }
  }
  std::cout << "hte initial=" << initial << " first=" << first << " last=" << last
            << " fixed_px4_hover_command=" << ratio << " healthy_used=" << used << "\n";
  tail.report("hte_convergence_tail_error");
  require(tail.values.size()>=10 && *std::max_element(tail.values.begin(),tail.values.end())<.02,
          "real HTE did not converge to physical hover ratio within .02");
}

void verify_fcu_receipts(Module &ctl, Module &plant, double landing_stamp) {
  const auto &requests=ctl.events[ctl.port("fcu_request_full")];
  const auto &receipts=plant.events[plant.port("fcu_result")];
  require(!requests.empty() && !receipts.empty(),"no real FCU request/result events");
  std::map<uint64_t,xgc_fcu_result_v1> executed;
  for (const auto &sample : receipts) {
    require(sample.data.size()==sizeof(xgc_fcu_result_v1),"wrong shared FCU result payload size");
    xgc_fcu_result_v1 r{}; std::memcpy(&r,sample.data.data(),sizeof r);
    require(r.request_id && r.robot_index==0,"FCU result ID/robot correlation invalid");
    executed[r.request_id]=r;
  }
  bool arm=false, offboard=false, landed_disarm=false;
  for (const auto &sample : requests) {
    require(sample.data.size()==sizeof(xgc_fcu_request_v2),"wrong FCU request /2 payload size");
    xgc_fcu_request_v2 r{}; std::memcpy(&r,sample.data.data(),sizeof r);
    const auto at=executed.find(r.request_id);
    require(r.request_id && at!=executed.end(),"FCU request has no correlated execution result");
    const auto &ack=at->second;
    require(ack.request_stamp==r.stamp && ack.kind==r.kind && ack.stamp>=r.stamp,
            "FCU execution result does not echo request stamp/kind");
    if (ack.result==0) {
      arm=arm || (r.kind==1 && r.arm);
      offboard=offboard || (r.kind==2 && std::string(r.mode)=="OFFBOARD");
      landed_disarm=landed_disarm || (r.kind==1 && !r.arm && r.stamp>=landing_stamp);
    }
  }
  require(arm && offboard && landed_disarm,"missing authoritative accepted ARM/OFFBOARD/landing-disarm receipts");
}

void verify_flight(const std::vector<FlightSample> &samples) {
  require(samples.size()>300, "insufficient complete state/IMU samples");
  Errors position, velocity, gyro_fd, force_fd, world_rate;
  double max_tilt=0, max_roll=0, max_pitch=0, max_gyro=0;
  int tilted_acceleration=0, aligned_acceleration=0;
  for (size_t i=0; i<samples.size(); ++i) {
    const auto &s=samples[i];
    require(s.pose.stamp==s.velocity.stamp && s.pose.stamp==s.imu.stamp,
            "pose/velocity/IMU must have identical source stamps");
    require(i==0 || s.pose.stamp>samples[i-1].pose.stamp, "state stamps must strictly increase");
    double qnorm=0;
    for (auto value : s.q()) { require(std::isfinite(value), "nonfinite quaternion"); qnorm+=value*value; }
    require(std::abs(qnorm-1)<1e-9, "measured quaternion is not unit length");
    const Vec g=rotate(s.q(), {s.imu.gyro[0],s.imu.gyro[1],s.imu.gyro[2]});
    world_rate.add(norm({g[0]-s.velocity.angular[0],g[1]-s.velocity.angular[1],g[2]-s.velocity.angular[2]}));
    if (s.custom_elapsed<0 || s.custom_elapsed>10 || s.state!="Custom1") continue;
    const auto desired=reference(s.custom_elapsed,s.pose.stamp);
    Vec pe{}, ve{};
    for (int j=0; j<3; ++j) { pe[j]=s.pose.position[j]-desired.position[j]; ve[j]=s.velocity.linear[j]-desired.velocity[j]; }
    position.add(norm(pe)); velocity.add(norm(ve));
    const Vec z=rotate(s.q(), {0,0,1});
    max_tilt=std::max(max_tilt,std::acos(std::clamp(z[2],-1.0,1.0)));
    max_roll=std::max(max_roll,std::abs(std::atan2(z[1],z[2])));
    max_pitch=std::max(max_pitch,std::abs(std::atan2(z[0],z[2])));
    max_gyro=std::max(max_gyro,norm({s.imu.gyro[0],s.imu.gyro[1],s.imu.gyro[2]}));
    if (i==0 || i+1==samples.size()) continue;
    const auto &before=samples[i-1], &after=samples[i+1];
    // Only airborne Custom1 triples, away from takeoff/ground/landing contact
    // impulses. Keep every such triple; do not discard large residuals.
    if (before.state!="Custom1" || after.state!="Custom1" ||
        std::min({before.pose.position[2],s.pose.position[2],after.pose.position[2]})<.3) continue;
    const double hm=s.pose.stamp-before.pose.stamp, hp=after.pose.stamp-s.pose.stamp;
    require(hm>0 && hp>0 && hm<=.030001 && hp<=.030001, "state gap too large for finite differences");
    const double a=-hp/(hm*(hm+hp)), b=(hp-hm)/(hm*hp), c=hm/(hp*(hm+hp));
    const auto qm=aligned(before.q(),s.q()), qp=aligned(after.q(),s.q());
    Quat qdot{};
    for (int j=0; j<4; ++j) qdot[j]=a*qm[j]+b*s.q()[j]+c*qp[j];
    const auto omega=product(conjugate(s.q()),qdot);
    gyro_fd.add(norm({2*omega[1]-s.imu.gyro[0],2*omega[2]-s.imu.gyro[1],2*omega[3]-s.imu.gyro[2]}));
    Vec acceleration{};
    for (int j=0; j<3; ++j) acceleration[j]=a*before.velocity.linear[j]+b*s.velocity.linear[j]+c*after.velocity.linear[j];
    const auto measured=rotate(s.q(),{s.imu.accel[0],s.imu.accel[1],s.imu.accel[2]});
    force_fd.add(norm({measured[0]-acceleration[0],measured[1]-acceleration[1],measured[2]-9.8066-acceleration[2]}));
    if (std::hypot(acceleration[0],acceleration[1])>.08) {
      ++tilted_acceleration;
      if (z[0]*acceleration[0]+z[1]*acceleration[1]>0) ++aligned_acceleration;
    }
  }
  position.report("trajectory_position_error_m"); velocity.report("trajectory_velocity_error_m_s");
  gyro_fd.report("q_derivative_gyro_residual_rad_s"); force_fd.report("imu_velocity_derivative_residual_m_s2");
  world_rate.report("world_twist_body_gyro_residual_rad_s");
  std::cout << "flight_excitation tilt_rad=" << max_tilt << " roll_rad=" << max_roll
            << " pitch_rad=" << max_pitch << " gyro_rad_s=" << max_gyro
            << " acceleration_aligned=" << aligned_acceleration << "/" << tilted_acceleration << "\n";
  std::cout.flush();
  require(position.values.size()>=500 && gyro_fd.values.size()>=300, "insufficient airborne trajectory coverage");
  require(max_roll>.005 && max_pitch>.005 && max_gyro>.005, "trajectory must excite roll, pitch and measured gyro");
  require(tilted_acceleration>100 && aligned_acceleration>.9*tilted_acceleration, "attitude must tilt with realized horizontal acceleration");
  require(world_rate.p95()<1e-8, "world twist angular rate does not equal rotated body gyro");
  // 100 Hz central differences of a 1 ms-integrated state: declared before
  // running the candidate; sensor/frame errors cannot be excused by endpoint arrival.
  require(gyro_fd.rms()<.02 && gyro_fd.p95()<.04, "gyro inconsistent with quaternion derivative");
  require(force_fd.rms()<.10 && force_fd.p95()<.20, "body specific force inconsistent with velocity derivative and gravity");
  require(position.rms()<.10 && position.p95()<.20, "whole-trajectory error exceeds flight fixture gate");
}

int run(int argc, char **argv) {
  require(argc>=3, "usage: controller-test PLANT.so CTL.so --hte HTE.so [--backend smc|dfbc] [--trace trace.csv] [--thr-mdl-fac VALUE --hte-initial .5]");
  const char *hte_path=std::getenv("XGC_HTE_PLUGIN"), *trace_path=nullptr;
  std::string backend="smc";
  const char *rigid_path=nullptr;
  double thr_mdl_fac=0, hte_initial=.3;
  bool override_initial=false;
  std::string startup_profile="actual";
  for (int i=3; i<argc; i+=2) {
    require(i+1<argc,"option requires a value");
    const std::string option=argv[i];
    if (option=="--hte") hte_path=argv[i+1];
    else if (option=="--backend") backend=argv[i+1];
    else if (option=="--trace") trace_path=argv[i+1];
    else if (option=="--thr-mdl-fac") thr_mdl_fac=std::stod(argv[i+1]);
    else if (option=="--hte-initial") { hte_initial=std::stod(argv[i+1]); override_initial=true; }
    else if (option=="--rigid-estimator") rigid_path=argv[i+1];
    else if (option=="--startup-profile") startup_profile=argv[i+1];
    else require(false,"unknown fixture option");
  }
  require(hte_path && (backend=="smc" || backend=="dfbc" || backend=="px4_local"),"real HTE path and existing controller backend required");
  const bool exercise_loss=backend!="px4_local";
  require(startup_profile=="actual" || startup_profile=="calibrated","startup profile must be actual or calibrated");
  const bool calibrate=backend!="px4_local" || startup_profile=="calibrated";
  require(thr_mdl_fac>=0 && thr_mdl_fac<=1 && hte_initial>.05 && hte_initial<.95,"invalid fixture THR_MDL_FAC/initial");
  const double hover_omega=std::sqrt(.310*9.8066/(4*5.33969944334e-6));
  const double hover_motor=(hover_omega-100)/1000;
  const double expected_hover=(1-thr_mdl_fac)*hover_motor+thr_mdl_fac*hover_motor*hover_motor;
  std::cout << "fixed_px4_physics hover_omega=" << hover_omega << " motor=" << hover_motor << " command=" << expected_hover << "\n";
  std::ofstream trace;
  if (trace_path) {
    trace.open(trace_path);
    require(bool(trace), "cannot open flight trace");
    trace << "stamp,custom_elapsed,state,px,py,pz,qw,qx,qy,qz,vx,vy,vz,wx_world,wy_world,wz_world,gx_body,gy_body,gz_body,fx_body,fy_body,fz_body,extended_stamp,extended_count,landed,vtol\n" << std::setprecision(17);
  }
  std::vector<FlightSample> samples;
  std::vector<std::string> states;
  uint64_t pose_publications=0;
  uint64_t hte_publications=0;
  std::vector<xgc_hover_thrust_v1> estimates;
  std::ostringstream startup;
  startup << std::setprecision(17) << "model=\"fs150\"\nepoch_ns=1000000000\nstep_ms=1\noutput_ms=10\ntrace_state=true\n"
          ;
  if (calibrate)
    startup << "[fcu_parameters]\nMPC_THR_HOVER=" << expected_hover
            << "\nTHR_MDL_FAC=" << thr_mdl_fac << "\n";
  else require(thr_mdl_fac==0,"ordinary PVA uses actual owning FS startup profile without override");
  const std::string plant_config=startup.str();
  std::cout << (!calibrate ? "startupProfile=actual_fs_overlay_no_override\n" :
              "startupoverride provenance=explicit_private_ACC_physical_equilibrium_not_asset_default\n") << plant_config;
  Module plant(
      argv[1],
      plant_config.c_str());
  xgc_sim_provider_request_v1 start_provider{};
  start_provider.stamp=1.0; start_provider.request_id=1;
  start_provider.robot_index=0; start_provider.generation=0; start_provider.action=1;
  plant.input("provider_request",start_provider);
  plant.tick(1000000000);
  const auto started=plant.output<xgc_sim_provider_result_v1>("provider_result");
  require(started.request_id==1 && started.robot_index==0 && started.accepted &&
          started.enabled && started.generation==1,"provider start must execute and ACK before controller ingress");
  std::cout << "provider_started generation=" << started.generation << " accepted=" << started.accepted << "\n";
  const std::string ctl_config="time_source=\"session\"\ntracking_backend=\""+backend+"\"\ntakeoff_altitude=1\nplanning_period=0.1\n";
  Module ctl(argv[2],ctl_config.c_str());
  const std::string hte_config="time_source=\"session\"\n"+
      (override_initial ? "initial_hover_thrust="+std::to_string(hte_initial)+"\n" : "");
  Module hte(hte_path,hte_config.c_str());
  std::unique_ptr<Module> rigid;
  if (backend=="dfbc") {
    require(rigid_path,"DFBC requires the real est-rigid-state ELF, not raw-truth substitution");
    // Native pose and IMU are measured at the same model body origin in
    // ENU/FLU. This is the fixture's known identity extrinsic, not a physical
    // mocap calibration or a bypass of the real estimator's health checks.
    rigid=std::make_unique<Module>(rigid_path,"time_source=\"session\"\nextrinsic_verified=true\nimu_to_vrpn_marker_xyz=[0,0,0]\nimu_to_vrpn_marker_rpy=[0,0,0]\n");
    plant.link("pose",*rigid,"pose"); plant.link("imu",*rigid,"imu");
    rigid->link("estimate",ctl,"estimate");
  }
  plant.link("pose", ctl, "local_pose");
  plant.link("pose", ctl, "vrpn_pose");
  plant.link("velocity", ctl, "local_velocity");
  plant.link("imu", ctl, "imu");
  plant.link("fcu_state", ctl, "fcu_state");
  ctl.link("setpoint", plant, "setpoint");
  ctl.link("attitude_command",plant,"attitude_command");
  ctl.link("fcu_request_full", plant, "fcu_request");
  require(std::string(plant.descriptor->ports[plant.port("fcu_result")].schema_id)=="xgc.fcu_result/1" &&
          std::string(plant.descriptor->ports[plant.port("fcu_extended_state")].schema_id)=="xgc.fcu_extended_state/1",
          "matched FCU result/extended-state candidate required");
  plant.link("pose",hte,"pose"); plant.link("imu",hte,"imu");
  plant.link("attitude_target",hte,"attitude_target_full");
  hte.link("hover_thrust",ctl,"hover_thrust");
  bool takeoff = false, custom = false, land = false, landed = false;
  bool stream_lost=false, stream_recovered=false;
  int loss_ms=-1;
  Vec loss_position{};
  bool saw_airborne_extended=false;
  double landing_stamp=0;
  int custom_ms = -1;
  std::string last_state;
  double error = -1;
  for (int ms = 0; ms != 60000; ++ms) {
    const int64_t now = 1000000000 + int64_t(ms) * 1000000;
    ctl.now = plant.now = hte.now = now;
    const auto status = ctl.output<xgc_controller_status_v1>("status");
    const std::string state(status.state);
    if (state != last_state) {
      std::cout << "state " << ms << " " << state << "\n";
      last_state = state;
      states.push_back(state);
    }
    if (!takeoff && state == "Ready") {
      ctl.command("takeoff");
      takeoff = true;
    }
    const auto hover=hte.output<xgc_hover_thrust_v1>("hover_thrust");
    if (takeoff && !custom && state == "Hover" && (backend=="px4_local" ||
        (hover.state==12 && std::abs(hover.hover_thrust-expected_hover)<.02))) {
      custom = true;
      custom_ms = ms;
      if (backend=="dfbc") ctl.input_bytes("ref_active_sampled",sampled_reference(now));
      ctl.command("custom1");
    }
    if (backend!="dfbc" && custom && loss_ms<0 && ((ms - custom_ms) % 100) == 0) {
      ctl.input("alg_setpoint", reference(double(ms-custom_ms)*.001,double(now)*1e-9));
    }
    if (custom && loss_ms<0 && ms - custom_ms >= 10000) {
      auto p = plant.output<xgc_pose_v1>("pose");
      error = norm({p.position[0]-1,p.position[1],p.position[2]-1});
      std::cout << "tracking_error " << error << "\n";
      assert(state == "Custom1");
      std::cout.flush();
      require(error < .03,"original 3cm tracking endpoint gate failed; complete state trace retained");
      loss_ms=ms;
      std::copy(p.position,p.position+3,loss_position.begin());
      if (exercise_loss) std::cout << "control_stream_loss_begin " << ms << " " << backend << "\n";
      else { ctl.command("land"); land=true; stream_recovered=true; landing_stamp=double(now)*1e-9; }
    }
    plant.tick(now);
    hte.tick(now);
    if (rigid) rigid->tick(now);
    if (loss_ms<0 || stream_recovered) ctl.tick(now);
    if (loss_ms>=0 && !stream_recovered &&
        std::string(plant.output<xgc_fcu_state_v1>("fcu_state").mode)=="AUTO.LOITER") {
      const auto fcu=plant.output<xgc_fcu_state_v1>("fcu_state");
      const auto p=plant.output<xgc_pose_v1>("pose");
      const double drift=norm({p.position[0]-loss_position[0],p.position[1]-loss_position[1],p.position[2]-loss_position[2]});
      require(fcu.armed && p.position[2]>.3 && ms-loss_ms<=3000,
              "control stream loss did not yield timely airborne AUTO.LOITER fallback");
      stream_lost=true; stream_recovered=true;
      std::cout << "control_stream_loss_pass mode=AUTO.LOITER elapsed_s=" << double(ms-loss_ms)*.001
                << " drift_m=" << drift << " source_stamp=" << p.stamp << "\n";
      ctl.command("land"); land=true; landing_stamp=double(now)*1e-9;
    }
    if (loss_ms>=0 && !stream_recovered && ms-loss_ms>3000)
      require(false,"no original FCU loss response in bounded three-second observation");
    const auto hc=hte.publications[hte.ports.at("hover_thrust")];
    if (hc!=hte_publications) {
      hte_publications=hc;
      estimates.push_back(hte.output<xgc_hover_thrust_v1>("hover_thrust"));
    }
    const auto pose = plant.output<xgc_pose_v1>("pose");
    for (double v : pose.position)
      assert(std::isfinite(v));
    const auto count=plant.publications[plant.ports.at("pose")];
    if (count!=pose_publications) {
      pose_publications=count;
      FlightSample s{pose,plant.output<xgc_twist_v1>("velocity"),plant.output<xgc_imu_v1>("imu"),
                     plant.output<xgc_fcu_extended_state_v1>("fcu_extended_state"),
                     custom ? double(ms-custom_ms)*.001 : -1,
                     ctl.output<xgc_controller_status_v1>("status").state};
      require(s.pose.stamp==plant.output<xgc_fcu_state_v1>("fcu_state").stamp, "FCU and pose source stamps differ");
      const auto extended=plant.output<xgc_fcu_extended_state_v1>("fcu_extended_state");
      require(extended.count==1 && extended.stamp==s.pose.stamp && extended.vtol_state[0]==0,
              "extended state must describe this same-stamp single quadrotor");
      saw_airborne_extended=saw_airborne_extended || extended.landed_state[0]==2;
      samples.push_back(s);
      if (trace) {
        trace << s.pose.stamp << ',' << s.custom_elapsed << ',' << s.state;
        for (auto value : s.pose.position) trace << ',' << value;
        for (auto value : s.q()) trace << ',' << value;
        for (auto value : s.velocity.linear) trace << ',' << value;
        for (auto value : s.velocity.angular) trace << ',' << value;
        for (auto value : s.imu.gyro) trace << ',' << value;
        for (auto value : s.imu.accel) trace << ',' << value;
        trace << ',' << s.extended.stamp << ',' << s.extended.count << ','
              << unsigned(s.extended.landed_state[0]) << ',' << unsigned(s.extended.vtol_state[0]);
        trace << '\n';
        trace.flush();
      }
    }
    if (land && !plant.output<xgc_fcu_state_v1>("fcu_state").armed &&
        pose.position[2] < .03) {
      landed = true;
      break;
    }
  }
  // Execute any FCU request issued on the last controller step before stopping
  // the fixture. Do not fabricate an ACK for a queued, unexecuted request.
  for (int k=1; k<=20; ++k) {
    const auto now=plant.now+1000000;
    plant.tick(now); hte.tick(now);
  }
  require(takeoff && custom && land && landed && (!exercise_loss || (stream_lost && stream_recovered)),"flight lifecycle or stream-loss recovery missing");
  if (backend=="smc") require(ctl.smc_outputs>100 && ctl.attitude_outputs==0,"SMC must preserve acceleration-only controller output");
  else if (backend=="dfbc") require(ctl.attitude_outputs>100 && ctl.attitude_command_outputs>100,"DFBC must produce real controller body-rate /1 and full /2 output");
  else require(ctl.publications[ctl.port("setpoint")]>100 && ctl.attitude_outputs==0,"px4_local must retain original PVA controller output");
  size_t next=0;
  const std::vector<std::string> required{"Ready","Takeoff","Hover","Custom1","Landing"};
  for (const auto &state : states) {
    const std::string family=state.find("Takeoff")==0 ? "Takeoff" : state;
    if (next<required.size() && family==required[next]) ++next;
  }
  require(next==required.size(), "missing ordered Ready/Takeoff/Hover/Custom1/Landing sequence");
  verify_flight(samples);
  if (backend!="px4_local") verify_hte(estimates,expected_hover,hte_initial);
  verify_fcu_receipts(ctl,plant,landing_stamp);
  require(saw_airborne_extended && plant.output<xgc_fcu_extended_state_v1>("fcu_extended_state").landed_state[0]==1,
          "extended state must show airborne then on-ground landing");
  std::cout << "PASS " << backend << " unchanged controller + real HTE + lightweight plugin "
               "takeoff/tracking/landing; error="
            << error << " acceleration_outputs=" << ctl.smc_outputs << "\n";
  return 0;
}

int main(int argc, char **argv) {
  try {
    return run(argc,argv);
  } catch (const std::exception &error) {
    std::cout.flush();
    std::cerr << "FAIL controller flight fixture: " << error.what() << "\n";
    return 1;
  }
}
