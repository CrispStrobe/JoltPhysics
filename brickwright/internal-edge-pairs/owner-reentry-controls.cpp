#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <iostream>
#include <memory>
#include <cstdlib>
using namespace JPH;
static unsigned checks=0;
static void need(bool value,const char*message){++checks;if(!value){std::cerr<<"FAIL "<<message<<'\n';std::exit(1);}}
struct BP final:BroadPhaseLayerInterface{uint GetNumBroadPhaseLayers()const override{return 1;}BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer)const override{return BroadPhaseLayer(0);}
#if defined(JPH_EXTERNAL_PROFILE)||defined(JPH_PROFILE_ENABLED)
const char*GetBroadPhaseLayerName(BroadPhaseLayer)const override{return "all";}
#endif
};
struct OP final:ObjectLayerPairFilter{bool ShouldCollide(ObjectLayer,ObjectLayer)const override{return true;}};
struct OV final:ObjectVsBroadPhaseLayerFilter{bool ShouldCollide(ObjectLayer,BroadPhaseLayer)const override{return true;}};
static void ownerTeardownControls(){
 struct State{PhysicsSystem *system=nullptr;BodyID source,surface;bool armed=false,entered=false,set=true,clear=true,reset=true;uint bodies=0;};
 struct Functor{std::shared_ptr<State> state;
  void operator()(const Body&a,const Body&b,Mat44Arg t1,Mat44Arg t2,CollideShapeSettings&s,CollideShapeCollector&c,const ShapeFilter&f)const{PhysicsSystem::sDefaultSimCollideBodyVsBody(a,b,t1,t2,s,c,f);}
  ~Functor(){if(state&&state->armed){auto held=state;held->armed=false;held->entered=true;held->bodies=held->system->GetNumBodies();held->set=held->system->SetSimInternalEdgeRemovalPair(held->source,held->surface,false);held->clear=held->system->ClearSimInternalEdgeRemovalPairs();held->reset=held->system->ResetSimCollideBodyVsBody();}}
 };
 for(bool teardown:{false,true}){
  std::cerr<<"CASE owner-functor "<<teardown<<'\n';auto state=std::make_shared<State>();
  {BP bp;OP op;OV ov;PhysicsSystem system;system.Init(16,0,16,16,bp,ov,op);state->system=&system;
   auto &bi=system.GetBodyInterface();BodyCreationSettings settings(new SphereShape(.01f),RVec3::sZero(),Quat::sIdentity(),EMotionType::Dynamic,0);
   Body *a=bi.CreateBody(settings),*b=bi.CreateBody(settings);need(a&&b,"teardown live body allocation");state->source=a->GetID();state->surface=b->GetID();bi.AddBody(state->source,EActivation::Activate);bi.AddBody(state->surface,EActivation::Activate);
   system.SetSimCollideBodyVsBody(Functor{state});state->armed=true;
   if(!teardown)need(system.ResetSimCollideBodyVsBody(),"idle explicit callback reset");
   // The owner retains both bodies. Its BodyManager destroys them after the
   // collide functor, making Set(false) a nonvacuous live-ID reentry control.
  }
  need(state->entered&&state->bodies==2,"functor invoked while both owned bodies remain live");
  need(state->set==!teardown&&state->clear==!teardown&&state->reset==!teardown,"only owner teardown refuses configuration; idle functor cleanup remains allowed");
 }
}
int main(){RegisterDefaultAllocator();Factory::sInstance=new Factory;RegisterTypes();ownerTeardownControls();UnregisterTypes();delete Factory::sInstance;Factory::sInstance=nullptr;std::cout<<"PASS "<<checks<<" owner checks\n";return checks==7?0:1;}
