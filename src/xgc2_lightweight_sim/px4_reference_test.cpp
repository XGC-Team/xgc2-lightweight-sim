// Offline oracle: call the pinned PX4 classes directly, preserving every
// integrator and mixer history. Fixture tuning stresses branches; it is not
// a runtime/default parameter table. A wrapper can replay the emitted JSONL.
#include "px4_v1_12_3/include/px4_reference.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using matrix::Vector3f;
using matrix::Quatf;
namespace {
constexpr float dt = 0.001f;
int checks = 0;
std::ofstream trace_file;
void check(bool condition, const char *detail) {
  ++checks;
  if (!condition) throw std::runtime_error(detail);
}
void values(const std::vector<float> &v) {
  trace_file << '[';
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i) trace_file << ',';
    if (std::isfinite(v[i])) trace_file << v[i];
    else trace_file << (std::isnan(v[i]) ? "\"nan\"" : (v[i] > 0 ? "\"inf\"" : "\"-inf\""));
  }
  trace_file << ']';
}
void row(const std::string &kind, int step, const std::vector<float> &input,
         const std::vector<float> &output) {
  if (!trace_file.is_open()) return;
  trace_file << "{\"case\":\"" << kind << "\",\"step\":" << step << ",\"input\":";
  values(input); trace_file << ",\"output\":"; values(output); trace_file << "}\n";
}
void append(std::vector<float> &v, const Vector3f &p) {
  for (int i = 0; i < 3; ++i) v.push_back(p(i));
}
Vector3f integral(RateControl &control) {
  rate_ctrl_status_s status{};
  control.getRateControlStatus(status);
  return Vector3f(status.rollspeed_integ, status.pitchspeed_integ, status.yawspeed_integ);
}
void rate_tuning(RateControl &control) {
  control.setGains(Vector3f(.4f,.5f,.6f), Vector3f(2.f,2.f,2.f), Vector3f(.01f,.02f,.03f));
  control.setIntegratorLimit(Vector3f(.03f,.03f,.03f));
  control.setFeedForwardGain(Vector3f(.02f,.03f,.04f));
}
void attitudes() {
  for (float weight : {0.f, .4f, 1.f}) {
    AttitudeControl control;
    control.setProportionalGain(Vector3f(6.5f,6.5f,2.8f), weight);
    control.setRateLimit(Vector3f(2.f,3.f,1.f));
    for (int step = 0; step < 48; ++step) {
      const Quatf current(matrix::Eulerf(.2f,-.3f,.4f));
      Quatf desired(matrix::Eulerf(.08f*step, .04f*step, .13f*step));
      if (step == 47) desired = Quatf(matrix::Eulerf(M_PI_F,0.f,0.f));
      control.setAttitudeSetpoint(desired, .2f);
      const Vector3f output = control.update(current);
      control.setAttitudeSetpoint(Quatf(-desired(0),-desired(1),-desired(2),-desired(3)), .2f);
      const Vector3f antipodal = control.update(current);
      for (int i = 0; i < 3; ++i) {
        check(std::isfinite(output(i)), "attitude output finite, including inverted thrust direction");
        check(std::fabs(output(i)-antipodal(i)) < 3e-6f, "q and -q have identical rate output");
        check(std::fabs(output(i)) <= Vector3f(2.f,3.f,1.f)(i)+1e-6f, "attitude rate limits");
      }
      std::vector<float> input{weight,.2f,6.5f,6.5f,2.8f,2.f,3.f,1.f};
      for (int i=0;i<4;++i) input.push_back(current(i));
      for (int i=0;i<4;++i) input.push_back(desired(i));
      std::vector<float> out; append(out,output); append(out,antipodal);
      row("attitude_weight_"+std::to_string(weight),step,input,out);
    }
  }
}
void rate_history() {
  RateControl control;
  rate_tuning(control);
  row("rate_history_config",-1,{.4f,.5f,.6f,2.f,2.f,2.f,.01f,.02f,.03f,.03f,.03f,.03f,.02f,.03f,.04f},{});
  Vector3f previous;
  for (int step = 0; step < 160; ++step) {
    MultirotorMixer::saturation_status saturation{};
    saturation.flags.valid = true;
    if (step >= 40 && step < 80) {
      saturation.flags.roll_pos=true; saturation.flags.pitch_neg=true; saturation.flags.yaw_pos=true;
    }
    control.setSaturationStatus(saturation);
    const bool reset = step == 100;
    if (reset) control.resetIntegral();
    const bool landed = step >= 120 && step < 140;
    const Vector3f setpoint = step < 80 ? Vector3f(1.f,-1.f,.5f) : Vector3f(-1.f,1.f,-.5f);
    const Vector3f accel(.1f,-.2f,.3f);
    const Vector3f output = control.update(Vector3f(),setpoint,accel,dt,landed);
    const Vector3f state = integral(control);
    for (int i=0;i<3;++i) {
      check(std::fabs(state(i)) <= .030001f, "rate integrator limit");
      if ((step>=40 && step<80) || landed)
        check(state(i)==previous(i), "saturation/landed preserves integral history");
    }
    if (reset) {
      RateControl fresh; rate_tuning(fresh);
      check((output-fresh.update(Vector3f(),setpoint,accel,dt,false)).norm()<1e-7f,
            "rate reset output equals fresh controller");
    }
    std::vector<float> input; append(input,Vector3f()); append(input,setpoint); append(input,accel);
    input.insert(input.end(),{dt,float(landed),float(reset),float(saturation.value)});
    std::vector<float> out; append(out,output); append(out,state);
    row("rate_history",step,input,out);
    previous=state;
  }
}
struct Controls { float value[4]{}; };
int read_control(uintptr_t handle, uint8_t group, uint8_t index, float &value) {
  if (group!=0 || index>=4) return -1;
  value=reinterpret_cast<Controls*>(handle)->value[index]; return 0;
}
void mixer_history() {
  for (int mode=0;mode<3;++mode) for (float factor : {0.f,.35f,.8f}) {
    Controls controls;
    MultirotorMixer mixer(read_control,reinterpret_cast<uintptr_t>(&controls),MultirotorGeometry::QUAD_X);
    mixer.set_airmode(static_cast<Mixer::Airmode>(mode)); mixer.set_thrust_factor(factor);
    RateControl rates; rate_tuning(rates);
    float previous[4]{-1.f,-1.f,-1.f,-1.f};
    uint16_t previous_status=0;
    const std::string name="mixer_airmode_"+std::to_string(mode)+"_factor_"+std::to_string(factor);
    for (int step=0;step<200;++step) {
      MultirotorMixer::saturation_status feedback{};
      feedback.value=previous_status;
      // Feed the previous actual mixer's per-axis flags into this rate tick.
      // Keep this call before update: recording a status word is insufficient.
      rates.setSaturationStatus(feedback);
      const Vector3f rate_sp(step<100?3.f:-3.f,step<120?-2.f:2.f,step<140?3.f:-3.f);
      const bool reset=step==150;
      if (reset) rates.resetIntegral();
      const Vector3f torque=rates.update(Vector3f(),rate_sp,Vector3f(),dt,false);
      for (int i=0;i<3;++i) controls.value[i]=torque(i);
      controls.value[3]=step<80?.08f:step<160?.95f:.15f;
      const bool slew=step!=80; // Verify that the once-only limit does not persist.
      if (slew) mixer.set_max_delta_out_once(.03f);
      float outputs[4];
      check(mixer.mix(outputs,4)==4,"mixer writes four motor outputs");
      std::vector<float> out;
      for (int i=0;i<4;++i) {
        check(std::isfinite(outputs[i]) && outputs[i]>=-1.f && outputs[i]<=1.f,"original mixer output range");
        if (slew) check(std::fabs(outputs[i]-previous[i])<.030001f,"mixer slew preserves motor history");
        out.push_back(outputs[i]); previous[i]=outputs[i];
      }
      if (step==80) {
        MultirotorMixer fresh(read_control,reinterpret_cast<uintptr_t>(&controls),MultirotorGeometry::QUAD_X);
        fresh.set_airmode(static_cast<Mixer::Airmode>(mode));fresh.set_thrust_factor(factor);
        float unrestricted[4];fresh.mix(unrestricted,4);
        for (int i=0;i<4;++i) check(outputs[i]==unrestricted[i],"once-only slew resets to unrestricted mix");
      }
      // This explicit execution boundary is the sole [-1,1] -> [0,1] conversion.
      for (float output:outputs) out.push_back(.5f*(output+1.f));
      append(out,torque); append(out,integral(rates));
      out.push_back(float(mixer.get_saturation_status()));
      std::vector<float> input{float(mode),factor,dt,float(slew),.03f,float(reset),float(previous_status)};
      append(input,rate_sp); append(input,Vector3f()); append(input,Vector3f());
      input.push_back(controls.value[3]);
      row(name,step,input,out);
      previous_status=mixer.get_saturation_status();
    }
    // A pure collective step is guaranteed to demand a jump, unlike a
    // torque-saturated airmode case whose ideal outputs may already be fixed.
    Controls collective;collective.value[3]=.9f;
    MultirotorMixer once(read_control,reinterpret_cast<uintptr_t>(&collective),MultirotorGeometry::QUAD_X);
    once.set_airmode(static_cast<Mixer::Airmode>(mode));once.set_thrust_factor(factor);
    float limited[4],unlimited[4];once.set_max_delta_out_once(.02f);once.mix(limited,4);once.mix(unlimited,4);
    for (int i=0;i<4;++i) check(unlimited[i]-limited[i]>.02f,"pure collective proves once-only slew does not persist");
  }
}
void position_tuning(PositionControl &control) {
  control.setPositionGains(Vector3f(1.f,1.f,1.f));
  control.setVelocityGains(Vector3f(2.f,2.f,3.f),Vector3f(.5f,.5f,.6f),Vector3f(.2f,.2f,.1f));
  control.setVelocityLimits(3.f,2.f,1.f); control.setThrustLimits(.1f,.8f);
  control.setTiltLimit(.7f); control.setHoverThrust(.5f);
}
vehicle_local_position_setpoint_s empty_setpoint() {
  vehicle_local_position_setpoint_s sp{};
  const float nan=std::numeric_limits<float>::quiet_NaN();
  sp.x=sp.y=sp.z=sp.vx=sp.vy=sp.vz=sp.yaw=sp.yawspeed=nan;
  for (int i=0;i<3;++i) sp.acceleration[i]=sp.thrust[i]=sp.jerk[i]=nan;
  return sp;
}
void positions() {
  PositionControl control; position_tuning(control);
  row("position_history_config",-1,{1.f,1.f,1.f,2.f,2.f,3.f,.5f,.5f,.6f,.2f,.2f,.1f,3.f,2.f,1.f,.1f,.8f,.7f,.5f},{});
  PositionControlStates state{Vector3f(),Vector3f(),Vector3f(),.4f};
  for (int step=0;step<200;++step) {
    auto sp=empty_setpoint();
    if (step<80) {
      sp.x=10.f;sp.y=-8.f;sp.z=-4.f;
      if (step>=40) {sp.vx=8.f;sp.vy=-4.f;sp.vz=-5.f;}
    } else if (step<120 || step>=160) {sp.vx=3.f;sp.vy=-2.f;sp.vz=-2.f;}
    if (step>=80) {sp.acceleration[0]=10.f;sp.acceleration[1]=-10.f;sp.acceleration[2]=-12.f;}
    const bool reset=step==100;
    if (reset) control.resetIntegral();
    const float hover_update=step==180?.6f:0.f;
    if (hover_update) control.updateHoverThrust(hover_update);
    control.setState(state);control.setInputSetpoint(sp);
    check(control.update(dt),"position/velocity/acceleration feedforward update valid");
    vehicle_local_position_setpoint_s local{}; vehicle_attitude_setpoint_s attitude{};
    control.getLocalPositionSetpoint(local);control.getAttitudeSetpoint(attitude);
    const Vector3f thrust(local.thrust);
    check(thrust.norm()<=.80001f,"position thrust magnitude limit");
    if (std::isfinite(local.vx) && std::isfinite(local.vy))
      check(std::hypot(local.vx,local.vy)<=3.00001f,"position horizontal velocity limit");
    if (std::isfinite(local.vz))
      check(local.vz>=-2.00001f && local.vz<=1.00001f,"position vertical velocity limits");
    check(local.yaw==state.yaw && local.yawspeed==0.f,"NaN yaw defaults are preserved");
    const matrix::Dcmf rotation{Quatf(attitude.q_d)};
    check(std::acos(std::clamp(rotation(2,2),-1.f,1.f))<=.70001f,"position tilt limit");
    if (reset) {
      PositionControl fresh;position_tuning(fresh);fresh.setState(state);fresh.setInputSetpoint(sp);fresh.update(dt);
      vehicle_local_position_setpoint_s output{};fresh.getLocalPositionSetpoint(output);
      check((thrust-Vector3f(output.thrust)).norm()<1e-7f,"position reset equals fresh controller");
    }
    std::vector<float> input{dt,float(reset),hover_update};
    append(input,state.position);append(input,state.velocity);append(input,state.acceleration);input.push_back(state.yaw);
    input.insert(input.end(),{sp.x,sp.y,sp.z,sp.vx,sp.vy,sp.vz});
    for (float value:sp.acceleration) input.push_back(value);
    input.insert(input.end(),{sp.yaw,sp.yawspeed});
    std::vector<float> out{local.vx,local.vy,local.vz};
    for (float value:local.acceleration) out.push_back(value);
    append(out,thrust);out.insert(out.end(),{local.yaw,local.yawspeed});
    for (float value:attitude.q_d) out.push_back(value);
    row("position_history",step,input,out);
  }
}
} // namespace
int main(int argc,char **argv) {
  try {
    if (argc==3 && std::string(argv[1])=="--trace") {
      trace_file.open(argv[2]);check(bool(trace_file),"trace file writable");
      trace_file<<std::setprecision(std::numeric_limits<float>::max_digits10);
    } else if (argc!=1) throw std::runtime_error("usage: px4_reference_test [--trace output.jsonl]");
    attitudes();rate_history();positions();mixer_history();
    std::cout<<"PX4 original history oracle: "<<checks<<" checks passed\n";
  } catch (const std::exception &error) {std::cerr<<error.what()<<'\n';return 1;}
}
