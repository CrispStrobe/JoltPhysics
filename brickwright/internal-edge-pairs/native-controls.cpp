// SPDX-License-Identifier: MIT
// Real native selected/unselected, cache, lifetime, callback and replay controls.
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/PhysicsStepListener.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include "part-pair-filter.h"
#include <atomic>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>
using namespace JPH;
static unsigned checks=0;
static void need(bool value,const char *message){++checks;if(!value)throw std::runtime_error(message);}
#ifdef JPH_ENABLE_ASSERTS
static bool assertion(const char *expression,const char *message,const char *file,uint line){std::cerr<<"JOLT_ASSERT "<<expression<<' '<<(message?message:"")<<' '<<file<<':'<<line<<'\n';std::abort();}
#endif
struct BP final:BroadPhaseLayerInterface{
 uint GetNumBroadPhaseLayers() const override{return 1;}
 BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer) const override{return BroadPhaseLayer(0);}
#if defined(JPH_EXTERNAL_PROFILE)||defined(JPH_PROFILE_ENABLED)
 const char *GetBroadPhaseLayerName(BroadPhaseLayer) const override{return "all";}
#endif
};
struct OP final:ObjectLayerPairFilter{bool ShouldCollide(ObjectLayer,ObjectLayer)const override{return true;}};
struct OV final:ObjectVsBroadPhaseLayerFilter{bool ShouldCollide(ObjectLayer,BroadPhaseLayer)const override{return true;}};
struct Filter final:PartPairSimShapeFilter{
 mutable unsigned calls=0,associationErrors=0;bool reenter=false;mutable bool attempted=false;mutable bool refused=false;
 PhysicsSystem *system=nullptr;BodyID frame,roller;
 bool ShouldCollide(const Body&a,const Shape*sa,const SubShapeID&ia,const Body&b,const Shape*sb,const SubShapeID&ib)const override{
  ++calls;
  auto valid=[this](const Body&body,const Shape*shape){if(shape->GetType()==EShapeType::Compound)return true;auto u=shape->GetUserData();return body.GetID()==frame?u==100||u==101:u==200||u==201;};
  if(!valid(a,sa)||!valid(b,sb))++associationErrors;
  if(reenter&&!attempted){attempted=true;bool x=system->SetSimInternalEdgeRemovalPair(roller,frame,true),y=system->ClearSimInternalEdgeRemovalPairs(),z=system->ResetSimCollideBodyVsBody();refused=!x&&!y&&!z;}
  return PartPairSimShapeFilter::ShouldCollide(a,sa,ia,b,sb,ib);
 }
};
static RefConst<Shape> compound(bool roller){
 StaticCompoundShapeSettings s;
 for(int i=0;i<2;i++){Ref<BoxShape> leaf=new BoxShape(Vec3(.05f,.05f,.05f),.001f);leaf->SetUserData((roller?200:100)+i);s.AddShape(Vec3(i?(roller?.8f:.4f):0,0,0),Quat::sIdentity(),leaf);}
 auto result=s.Create();need(!result.HasError(),"compound construction");return result.Get();
}
struct World{
 BP bp;OP op;OV ov;Filter filter;PhysicsSystem system;TempAllocatorImpl allocator{8*1024*1024};JobSystemSingleThreaded jobs{cMaxPhysicsJobs};
 Body *frame=nullptr,*roller=nullptr,*other=nullptr;std::vector<BodyID> ids;
 BodyCreationSettings settings(bool isRoller){BodyCreationSettings s(compound(isRoller),RVec3(0,isRoller?.06f:0,0),Quat::sIdentity(),EMotionType::Dynamic,0);s.mGravityFactor=0;s.mAllowSleeping=false;s.mLinearDamping=0;s.mAngularDamping=0;s.mAllowedDOFs=EAllowedDOFs::TranslationY;return s;}
 World(bool frameFirst=true,int filterMode=0){
  system.Init(16,0,64,64,bp,ov,op);system.SetGravity(Vec3::sZero());auto &bi=system.GetBodyInterface();
  if(frameFirst){frame=bi.CreateBody(settings(false));roller=bi.CreateBody(settings(true));}else{roller=bi.CreateBody(settings(true));frame=bi.CreateBody(settings(false));}
  need(frame&&roller,"body allocation");for(Body*b:{frame,roller}){ids.push_back(b->GetID());bi.AddBody(b->GetID(),EActivation::Activate);need(!b->GetEnhancedInternalEdgeRemoval(),"legacy flags remain off");}
  BodyCreationSettings os(new SphereShape(.001f),RVec3(10,10,10),Quat::sIdentity(),EMotionType::Dynamic,0);os.mGravityFactor=0;os.mAllowSleeping=false;other=bi.CreateBody(os);need(other,"unselected body allocation");ids.push_back(other->GetID());bi.AddBody(other->GetID(),EActivation::Activate);
  filter.system=&system;filter.frame=frame->GetID();filter.roller=roller->GetID();
  for(uint i=0;i<2;i++){filter.SetColliderPart(frame->GetID(),100+i,10+i);filter.SetColliderPart(roller->GetID(),200+i,20+i);}
  filter.AddBodyPair(frame->GetID(),roller->GetID(),filterMode!=1);
  if(filterMode!=0){uint f=filterMode==2?11:10,r=filterMode==2?21:20;filter.AddPartPair(roller->GetID(),frame->GetID(),r,f);system.SetSimShapeFilter(&filter);}
 }
 ~World(){auto &bi=system.GetBodyInterface();for(auto id:ids){bi.RemoveBody(id);bi.DestroyBody(id);}}
 void step(){need(system.Update(1.f/60,1,&allocator,&jobs)==EPhysicsUpdateError::None,"native update");}
 void steps(int n){for(int i=0;i<n;i++)step();}
 bool select(bool on){return system.SetSimInternalEdgeRemovalPair(roller->GetID(),frame->GetID(),on);}
 bool invalid()const{return frame->IsCollisionCacheInvalid()||roller->IsCollisionCacheInvalid();}
 void eraseRoller(){auto id=roller->GetID();system.GetBodyInterface().RemoveBody(id);system.GetBodyInterface().DestroyBody(id);ids.erase(std::find(ids.begin(),ids.end(),id));roller=nullptr;}
 void sameIdRoller(BodyID id){auto &bi=system.GetBodyInterface();roller=bi.CreateBodyWithID(id,settings(true));need(roller,"custom complete-ID recreation");ids.push_back(id);bi.AddBody(id,EActivation::Activate);}
};
static void filterControls(){
 for(bool first:{true,false})for(int mode:{0,1,2,3})for(int policy:{0,1,2}){
  std::cerr<<"CASE filter "<<first<<' '<<mode<<' '<<policy<<'\n';World w(first,mode);
  if(policy==1)need(w.select(true),"explicit ordered pair accepted");
  if(policy==2)need(w.system.SetSimInternalEdgeRemovalPair(w.other->GetID(),w.frame->GetID(),true),"unselected override accepted");
  w.steps(30);float gap=float(w.roller->GetPosition().GetY()-w.frame->GetPosition().GetY());bool push=mode==0||mode==3;
  need(push?gap>.075f:std::abs(gap-.06f)<1e-7f,"real asymmetric filter outcome");need(w.filter.associationErrors==0,"body/leaf association");
  if(mode)need(w.filter.calls>0,"native filter consulted");std::cout<<"FILTER "<<first<<' '<<mode<<' '<<policy<<' '<<gap<<'\n';
 }
}
static void cacheControls(){
 std::cerr<<"CASE cache\n";World w(true,1);w.step();auto count=w.filter.calls;w.step();need(w.filter.calls==count,"initial no-hit cache reused");need(!w.invalid(),"cache flags cleared");
 need(w.select(true),"enable");need(w.frame->IsCollisionCacheInvalid()&&w.roller->IsCollisionCacheInvalid(),"enable invalidates both");w.step();need(w.filter.calls>count,"enable refreshes narrow phase");count=w.filter.calls;
 need(w.select(true),"duplicate idempotent");need(!w.invalid(),"duplicate does not invalidate");w.step();need(w.filter.calls==count,"duplicate preserves cache");
 need(w.system.SetSimInternalEdgeRemovalPair(w.frame->GetID(),w.roller->GetID(),true),"orientation replacement");need(w.invalid(),"orientation invalidates");w.step();need(w.filter.calls>count,"orientation refreshes");count=w.filter.calls;
 need(w.select(false),"disable in opposite argument order");need(w.invalid(),"disable invalidates");w.step();need(w.filter.calls>count,"disable refreshes");
 need(w.select(true),"enable again");w.step();need(w.system.ClearSimInternalEdgeRemovalPairs(),"clear");need(w.invalid(),"clear invalidates");w.step();
 need(!w.system.SetSimInternalEdgeRemovalPair(BodyID(),w.frame->GetID(),true),"invalid ID refused");need(!w.system.SetSimInternalEdgeRemovalPair(w.frame->GetID(),w.frame->GetID(),true),"same ID refused");
}
static void lifetimeControls(){
 for(int mode:{0,1,2}){
  std::cerr<<"CASE lifetime "<<mode<<'\n';World w(true,1);need(w.select(true),"lifetime enable");w.step();BodyID old=w.roller->GetID();auto &bi=w.system.GetBodyInterface();
  if(mode==1){bi.RemoveBody(old);Body *body=bi.UnassignBodyID(old);need(body,"unassign");need(bi.AssignBodyID(body,old),"reassign same complete ID");bi.AddBody(old,EActivation::Activate);}
  else{w.eraseRoller();if(mode==2){for(int i=0;i<255;i++){Body *body=bi.CreateBody(w.settings(true));need(body,"sequence recycle allocation");bi.DestroyBody(body->GetID());}w.roller=bi.CreateBody(w.settings(true));need(w.roller&&w.roller->GetID()==old,"finite sequence wrapped to old complete ID");w.ids.push_back(old);bi.AddBody(old,EActivation::Activate);}else w.sameIdRoller(old);}
  need(w.frame->IsCollisionCacheInvalid(),"survivor invalidated on ID release");w.step();need(!w.invalid(),"warm recreated world");need(w.select(false),"remove absent preference");need(!w.invalid(),"no policy inherited across ID reuse");
 }
}
struct PausedStep final:PhysicsStepListener{std::atomic<bool> entered{false},release{false};void OnStep(const PhysicsStepListenerContext&)override{entered.store(true);while(!release.load())std::this_thread::yield();}};
static void callbackControls(){
 std::cerr<<"CASE callback\n";World w(true,1);need(w.select(true),"callback enable");w.filter.reenter=true;w.step();need(w.filter.attempted&&w.filter.refused,"shape callback reentry refused without deadlock");
 unsigned custom=0;w.system.SetSimCollideBodyVsBody([&](const Body&a,const Body&b,Mat44Arg t1,Mat44Arg t2,CollideShapeSettings&s,CollideShapeCollector&c,const ShapeFilter&f){++custom;PhysicsSystem::sDefaultSimCollideBodyVsBody(a,b,t1,t2,s,c,f);});
 need(w.invalid(),"custom replacement clears policy/cache");need(!w.select(true),"custom callback not silently replaced");w.step();need(custom>0,"custom callback actually runs");
 need(w.system.ResetSimCollideBodyVsBody(),"explicit default reset");need(w.select(true),"pair accepted after explicit reset");
 PausedStep pause;w.system.AddStepListener(&pause);std::thread update([&]{w.step();});while(!pause.entered.load())std::this_thread::yield();
 bool a=w.select(false),b=w.system.ClearSimInternalEdgeRemovalPairs(),c=w.system.ResetSimCollideBodyVsBody();pause.release.store(true);update.join();w.system.RemoveStepListener(&pause);need(!a&&!b&&!c,"cross-thread update mutations refused without waiting");
}
static void replayControls(){
 std::cerr<<"CASE replay\n";World w(true,3);need(w.select(true),"replay enable");w.steps(3);StateRecorderImpl recorder;w.system.SaveState(recorder);std::vector<float> first,second;
 for(int i=0;i<12;i++){w.step();first.push_back(float(w.roller->GetPosition().GetY()));}
 recorder.Rewind();need(w.system.RestoreState(recorder),"same-config restore");for(int i=0;i<12;i++){w.step();second.push_back(float(w.roller->GetPosition().GetY()));}
 need(first.size()==second.size()&&std::memcmp(first.data(),second.data(),first.size()*sizeof(float))==0,"unchanged policy replay trajectory bytes");need(w.select(false),"replay policy remains present");need(w.invalid(),"restored session retained policy configuration");
}
int main(){try{RegisterDefaultAllocator();Factory::sInstance=new Factory;RegisterTypes();
#ifdef JPH_ENABLE_ASSERTS
 AssertFailed=assertion;
#endif
 filterControls();cacheControls();lifetimeControls();callbackControls();replayControls();{World fresh(true,1);fresh.steps(2);need(fresh.select(false)&&!fresh.invalid(),"new world default off");}
 UnregisterTypes();delete Factory::sInstance;Factory::sInstance=nullptr;std::cout<<"PASS "<<checks<<" real native checks\n";return 0;}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
