#include "xgc_rt.h"
#include "xgc_schemas_v1.h"
#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <deque>
#include <dlfcn.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>

struct Sample { int64_t time; std::vector<uint8_t> bytes; };
struct World {
  std::array<std::deque<Sample>,64> queue;
  std::map<uint32_t,std::vector<uint8_t>> output;
  std::vector<uint8_t> popped;
  const xgc_plugin_descriptor *plugin;
  void *instance{};
  xgc_host_api host{};
  int64_t now{1000000000};
  explicit World(const xgc_plugin_descriptor *p):plugin(p) {
    assert(p->port_count==64);
    host.abi_version=XGC_RT_ABI_VERSION; host.abi_minor=XGC_RT_ABI_MINOR; host.host=this;
    host.next=[](void *ptr,uint32_t port,xgc_sample_view *s){
      auto &w=*static_cast<World*>(ptr);auto &q=w.queue.at(port);
      if(q.empty())return XGC_ERR_AGAIN;
      auto t=q.front().time;w.popped=std::move(q.front().bytes);q.pop_front();
      *s={};s->t_rx=t;s->data=w.popped.data();s->len=w.popped.size();return XGC_OK;
    };
    host.publish=[](void *ptr,uint32_t port,uint64_t,const uint8_t *data,uint32_t size){
      static_cast<World*>(ptr)->output[port]={data,data+size};return XGC_OK;
    };
    host.log=[](void*,xgc_log_level level,const char *message){
      if(level>=XGC_LOG_ERROR){std::cerr<<message<<'\n';assert(false);}
    };
    instance=p->vtbl->create(&host);assert(instance);
    assert(p->vtbl->configure(instance,"model=\"fs150\"\nepoch_ns=1000000000\nrobots=2\ninitial_poses=[0,0,1,0,4,0,1,0]\noutput_ms=1\n[fcu_parameters]\nMPC_THR_HOVER=.27726948\n")==XGC_OK);
    assert(p->vtbl->activate(instance)==XGC_OK);
  }
  ~World(){plugin->vtbl->deactivate(instance);plugin->vtbl->destroy(instance);}
  template<class T>void send(uint32_t port,const T &x){const auto *b=reinterpret_cast<const uint8_t*>(&x);queue[port].push_back({now,{b,b+sizeof x}});}
  template<class T>T read(uint32_t port)const{T x{};assert(output.at(port).size()==sizeof x);std::memcpy(&x,output.at(port).data(),sizeof x);return x;}
  void tick(int ms=1){now+=ms*1000000LL;xgc_step_ctx ctx{};ctx.now=ctx.round_start=now;ctx.round=(now-1000000000)/1000000;ctx.round_advanced=1;assert(plugin->vtbl->step(instance,&ctx)==XGC_OK);}
  xgc_sim_provider_result_v1 provider(uint32_t slot,uint32_t action,uint64_t generation){
    xgc_sim_provider_request_v1 r{};r.stamp=now*1e-9;r.request_id=now;r.robot_index=slot;r.action=action;r.generation=generation;send(62,r);tick();auto v=read<xgc_sim_provider_result_v1>(63);assert(v.request_id==r.request_id);return v;
  }
  void target(uint32_t slot,double x){xgc_position_target_v1 p{};p.stamp=now*1e-9;p.coordinate_frame=1;p.type_mask=2552;p.position[0]=x;p.position[2]=1;send(slot*10,p);}
  void armOffboard(uint32_t slot){xgc_fcu_request_v2 r{};r.stamp=now*1e-9;r.kind=1;r.arm=1;r.request_id=now;send(slot*10+2,r);r.kind=2;std::strcpy(r.mode,"OFFBOARD");send(slot*10+2,r);}
};
int main(int argc,char **argv){
  assert(argc==2);void *lib=dlopen(argv[1],RTLD_NOW|RTLD_LOCAL);if(!lib){std::cerr<<dlerror()<<'\n';return 1;}
  auto entry=reinterpret_cast<const xgc_plugin_descriptor*(*)()>(dlsym(lib,"xgc_rt_plugin_v1"));assert(entry);
  {
    World w(entry());w.tick();assert(!w.read<xgc_fcu_state_v1>(6).connected);
    auto v=w.provider(0,0,0);assert(v.accepted&&!v.enabled&&v.generation==0);
    v=w.provider(0,1,0);assert(v.accepted&&v.enabled&&v.generation==1);
    assert(!w.read<xgc_fcu_state_v1>(16).connected); // only A is online
    assert(w.provider(1,1,0).accepted);
    w.target(0,1);w.target(1,5);w.armOffboard(0);w.armOffboard(1);w.tick();
    for(int i=0;i<600;++i){w.target(0,1);w.target(1,5);w.tick();}
    auto moved=w.read<xgc_pose_v1>(3);auto beforeB=w.read<xgc_pose_v1>(13);assert(std::abs(moved.position[0])>.01);
    // Queue a command for t+0.8 before reset. It cannot cross the generation.
    xgc_position_target_v1 old{};old.stamp=w.now*1e-9+.8;old.coordinate_frame=1;old.type_mask=2552;old.position[0]=100;old.position[2]=10;w.send(0,old);w.tick();
    assert(w.provider(0,2,1).accepted);v=w.provider(0,1,1);assert(v.accepted&&v.generation==2);
    auto reset=w.read<xgc_pose_v1>(3);assert(std::abs(reset.position[0])<1e-12&&std::abs(reset.position[2]-(1-.5*9.8066e-6))<1e-8);
    auto stale=w.provider(0,2,1);assert(!stale.accepted&&stale.enabled&&stale.generation==2);
    stale=w.provider(0,1,0);assert(!stale.accepted&&stale.generation==2);
    auto repeated=w.provider(0,1,1);assert(repeated.accepted&&repeated.generation==2);
    w.target(0,0);w.armOffboard(0);w.tick();
    for(int i=0;i<1000;++i){w.target(1,5);w.tick();} // source-loss A never resets
    auto afterA=w.read<xgc_pose_v1>(3);auto afterB=w.read<xgc_pose_v1>(13);
    assert(std::abs(afterA.position[0])<.02);assert(afterA.position[2]<2);
    assert(afterB.stamp>beforeB.stamp&&w.read<xgc_fcu_state_v1>(16).connected);
    assert(w.provider(0,0,0).generation==2);
    std::cout<<"PASS actual installed provider: observe/offline/start/reset/stale/future/source-loss/sibling clock\n";
    std::cout<<"A displaced_x="<<moved.position[0]<<" reset_z="<<reset.position[2]<<" B_stamp="<<afterB.stamp<<" A_after="<<afterA.position[0]<<','<<afterA.position[2]<<'\n';
  }
  dlclose(lib);
}
