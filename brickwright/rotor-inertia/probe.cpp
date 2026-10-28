// SPDX-FileCopyrightText: 2026 Brickwright contributors
// SPDX-License-Identifier: MIT
// Bounded no-contact native constraint/analytic controls. Existing full suites
// and later browser/model qualification remain necessary for contact acceptance.
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>
#include <sstream>
#include <algorithm>
using namespace JPH;
#ifndef BW_NATIVE_SOURCE_SHA
#define BW_NATIVE_SOURCE_SHA "UNBOUND"
#endif
#ifndef BW_PROBE_SOURCE_SHA
#define BW_PROBE_SOURCE_SHA "UNBOUND"
#endif
static int checks=0,failures=0;
static bool mutation=false;
static void Check(bool ok,const char *name){++checks;if(!ok){++failures;std::cerr<<"FAIL "<<name<<'\n';}}
static bool Near(float a,float b,float eps=1e-4f){return std::abs(a-b)<=eps;}
class Broad final:public BroadPhaseLayerInterface{
public:uint GetNumBroadPhaseLayers()const override{return 1;}BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer)const override{return BroadPhaseLayer(0);}
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
 const char *GetBroadPhaseLayerName(BroadPhaseLayer)const override{return "bench";}
#endif
};
class Pair final:public ObjectLayerPairFilter{public:bool ShouldCollide(ObjectLayer,ObjectLayer)const override{return false;}};
class Versus final:public ObjectVsBroadPhaseLayerFilter{public:bool ShouldCollide(ObjectLayer,BroadPhaseLayer)const override{return true;}};
static bool SetRotor(HingeConstraint &h,float jr){
#ifdef BW_ROTOR_BASELINE
 (void)h;(void)jr;return true;
#else
 return h.SetReflectedRotorInertia(mutation?0:jr);
#endif
}
static float Speed(HingeConstraint &h){
#ifdef BW_ROTOR_BASELINE
 (void)h;return 0;
#else
 return h.GetRotorAngularVelocity();
#endif
}
static float Inertia(HingeConstraint &h){
#ifdef BW_ROTOR_BASELINE
 (void)h;return 0;
#else
 return h.GetReflectedRotorInertia();
#endif
}
static float RotorLambda(HingeConstraint &h){
#ifdef BW_ROTOR_BASELINE
 (void)h;return 0;
#else
 return h.GetTotalLambdaRotor();
#endif
}
static void VectorJSON(std::ostream &s,Vec3Arg v){s<<'['<<v.GetX()<<','<<v.GetY()<<','<<v.GetZ()<<']';}
struct World{
 Broad broad;Pair pair;Versus versus;PhysicsSystem system;TempAllocatorImpl temp{10*1024*1024};JobSystemSingleThreaded jobs{cMaxPhysicsJobs};
 std::vector<Body *> bodies;std::vector<Ref<HingeConstraint>> hinges;
 std::string label;int stepIndex=0;std::vector<std::pair<size_t,Vec3>> externalTorques;
 explicit World(const char *name="bench"):label(name){system.Init(32,0,64,64,broad,versus,pair);system.SetGravity(Vec3::sZero());const auto &settings=system.GetPhysicsSettings();std::cout<<"ROTOR_CASE {\"case\":\""<<label<<"\",\"gravity\":[0,0,0],\"contactsDisabled\":true,\"velocityIterations\":"<<settings.mNumVelocitySteps<<",\"positionIterations\":"<<settings.mNumPositionSteps<<"}\n";}
 ~World(){for(auto &h:hinges)system.RemoveConstraint(h);hinges.clear();for(auto b:bodies){system.GetBodyInterface().RemoveBody(b->GetID());system.GetBodyInterface().DestroyBody(b->GetID());}}
 Body &BodyAt(RVec3Arg p,Vec3Arg moments,EMotionType motion=EMotionType::Dynamic,QuatArg rotation=Quat::sIdentity()){
  BodyCreationSettings s(new SphereShape(.005f),p,rotation,motion,0);s.mAngularDamping=0;s.mLinearDamping=0;s.mAllowSleeping=false;
  s.mOverrideMassProperties=EOverrideMassProperties::MassAndInertiaProvided;s.mMassPropertiesOverride.mMass=.1f;s.mMassPropertiesOverride.mInertia=Mat44::sScale(moments);
  Body *b=system.GetBodyInterface().CreateBody(s);b->SetApplyGyroscopicForce(true);system.GetBodyInterface().AddBody(b->GetID(),motion==EMotionType::Static?EActivation::DontActivate:EActivation::Activate);bodies.push_back(b);
  std::cout<<"ROTOR_CONFIG {\"case\":\""<<label<<"\",\"body\":"<<bodies.size()-1<<",\"massKg\":0.1,\"sphereRadiusM\":0.005,\"linearDamping\":0,\"angularDamping\":0,\"gyro\":true,\"momentsKgM2\":";VectorJSON(std::cout,moments);std::cout<<",\"motion\":"<<int(motion)<<"}\n";
  return *b;
 }
 HingeConstraint &Hinge(Body &a,Body &b,RVec3Arg p,Vec3Arg axis,float jr=0){
  HingeConstraintSettings s;s.mPoint1=s.mPoint2=p;s.mHingeAxis1=s.mHingeAxis2=axis;
  s.mNormalAxis1=s.mNormalAxis2=axis.Cross(std::abs(axis.GetX())<.8f?Vec3::sAxisX():Vec3::sAxisY()).Normalized();
  Ref<HingeConstraint> h=static_cast<HingeConstraint *>(s.Create(a,b));system.AddConstraint(h);hinges.push_back(h);Check(SetRotor(*h,jr),"valid native rotor setter");
  std::cout<<"ROTOR_HINGE {\"case\":\""<<label<<"\",\"hinge\":"<<hinges.size()-1<<",\"body1\":"<<Index(&a)<<",\"body2\":"<<Index(&b)<<",\"declaredJr\":"<<jr<<",\"point\":["<<p.GetX()<<','<<p.GetY()<<','<<p.GetZ()<<"],\"axis\":";VectorJSON(std::cout,axis);std::cout<<"}\n";Event("hinge-created");return *h;
 }
 size_t Index(const Body *body)const{for(size_t i=0;i<bodies.size();i++)if(bodies[i]==body)return i;return bodies.size();}
 void State(std::ostream &s){
  s<<"{\"bodies\":[";
  for(size_t i=0;i<bodies.size();i++){
   if(i)s<<',';auto &b=*bodies[i];auto q=b.GetRotation();auto x=b.GetCenterOfMassPosition();
   s<<"{\"id\":"<<i<<",\"dynamic\":"<<(b.IsDynamic()?"true":"false")<<",\"active\":"<<(b.IsActive()?"true":"false")<<",\"allowSleeping\":"<<(!b.IsStatic()&&b.GetAllowSleeping()?"true":"false")<<",\"gyro\":"<<(b.GetApplyGyroscopicForce()?"true":"false")<<",\"position\":["<<x.GetX()<<','<<x.GetY()<<','<<x.GetZ()<<"],\"rotation\":["<<q.GetX()<<','<<q.GetY()<<','<<q.GetZ()<<','<<q.GetW()<<"],\"linear\":";
   VectorJSON(s,b.GetLinearVelocity());s<<",\"angular\":";VectorJSON(s,b.GetAngularVelocity());s<<",\"inverseMass\":"<<(b.IsDynamic()?b.GetMotionProperties()->GetInverseMass():0)<<",\"inverseInertiaWorld\":[";
   auto inv=b.IsDynamic()?b.GetInverseInertia():Mat44::sZero();VectorJSON(s,inv.GetAxisX());s<<',';VectorJSON(s,inv.GetAxisY());s<<',';VectorJSON(s,inv.GetAxisZ());s<<"]}";
  }
  s<<"],\"hinges\":[";
  for(size_t i=0;i<hinges.size();i++){
   if(i)s<<',';auto &h=*hinges[i];s<<"{\"id\":"<<i<<",\"active\":"<<(h.IsActive()?"true":"false")<<",\"body1\":"<<Index(h.GetBody1())<<",\"body2\":"<<Index(h.GetBody2())<<",\"jr\":"<<Inertia(h)<<",\"wr\":"<<Speed(h)<<",\"rotorLambda\":"<<RotorLambda(h)<<",\"motorLambda\":"<<h.GetTotalLambdaMotor()<<",\"mode\":"<<int(h.GetMotorState())<<",\"targetVelocity\":"<<h.GetTargetAngularVelocity()<<",\"minTorque\":"<<h.GetMotorSettings().mMinTorqueLimit<<",\"maxTorque\":"<<h.GetMotorSettings().mMaxTorqueLimit<<",\"axis\":";
   VectorJSON(s,h.GetBody1()->GetRotation()*h.GetLocalSpaceHingeAxis1());s<<",\"localPoint1\":";VectorJSON(s,h.GetLocalSpacePoint1());s<<",\"localPoint2\":";VectorJSON(s,h.GetLocalSpacePoint2());s<<'}';
  }
  s<<"]}";
 }
 void Event(const char *action){std::cout<<"ROTOR_EVENT {\"case\":\""<<label<<"\",\"nextStep\":"<<stepIndex<<",\"action\":\""<<action<<"\",\"state\":";State(std::cout);std::cout<<"}\n";}
 void Torque(Body &b,Vec3Arg value){system.GetBodyInterface().AddTorque(b.GetID(),value);externalTorques.emplace_back(Index(&b),value);}
 void Step(float dt=.005f){
  std::ostringstream before;before<<std::setprecision(9);State(before);
  Check(uint(system.Update(dt,1,&temp,&jobs))==0,"native update successful");
  std::cout<<"ROTOR_STEP {\"schema\":\"bw.native-rotor.step.v1\",\"case\":\""<<label<<"\",\"index\":"<<stepIndex++<<",\"dt\":"<<dt<<",\"collisionSteps\":1,\"externalTorques\":[";
  for(size_t i=0;i<externalTorques.size();i++){if(i)std::cout<<',';std::cout<<"{\"body\":"<<externalTorques[i].first<<",\"torque\":";VectorJSON(std::cout,externalTorques[i].second);std::cout<<'}';}
  std::cout<<"],\"before\":"<<before.str()<<",\"after\":";State(std::cout);std::cout<<"}\n";externalTorques.clear();
 }
};
static void Analytic(float torque){
 constexpr float i1=.00224f,i2=.000058f,jr=.0003f;
 World w(torque>0?"analytic-positive-variable-dt":"analytic-negative-variable-dt");Body &a=w.BodyAt(RVec3(0,0,0),Vec3::sReplicate(i1)),&b=w.BodyAt(RVec3(0,.05f,0),Vec3::sReplicate(i2));auto &h=w.Hinge(a,b,RVec3(0,.025f,0),Vec3::sAxisY(),jr);
 float time=0;for(int i=0;i<60;i++){float dt=i%2?.005f:.01f;w.Torque(a,Vec3(0,-torque,0));w.Torque(b,Vec3(0,torque,0));w.Step(dt);time+=dt;}
 float relative=b.GetAngularVelocity().GetY()-a.GetAngularVelocity().GetY(),mu=i1*i2/(i1+i2),expected=torque*time/(mu+jr);
 Check(Near(relative,expected,.002f),"analytic relative inertia with variable dt");Check(std::abs(i1*a.GetAngularVelocity().GetY()+i2*b.GetAngularVelocity().GetY())<1e-7f,"bare angular momentum");Check(Near(Speed(h),relative,.002f),"physical scalar relative speed");
 float energy=.5f*(i1*Square(a.GetAngularVelocity().GetY())+i2*Square(b.GetAngularVelocity().GetY())+jr*Square(Speed(h)));
 Check(Near(energy,.5f*Square(torque*time)/(mu+jr),1e-5f),"body plus scalar energy matches torque impulse work");
 std::cout<<"analytic relative="<<relative<<" expected="<<expected<<" scalar="<<Speed(h)<<'\n';
 // Native state restores Jr as well as scalar/lambda after reconfiguration.
#ifndef BW_ROTOR_BASELINE
 StateRecorderImpl save;w.system.SaveState(save);w.Event("checkpoint-before-disable");float savedSpeed=Speed(h);SetRotor(h,0);w.Event("disable");w.Step();save.Rewind();Check(w.system.RestoreState(save),"native state restore");w.Event("restore-before-disable");Check(h.GetReflectedRotorInertia()==jr&&Speed(h)==savedSpeed,"rotor configuration and speed restored");
 StateRecorderImpl checkpoint;w.system.SaveState(checkpoint);w.Event("trajectory-checkpoint");for(int i=0;i<10;i++)w.Step();StateRecorderImpl first;w.system.SaveState(first);checkpoint.Rewind();Check(w.system.RestoreState(checkpoint),"second restore");w.Event("trajectory-restore");for(int i=0;i<10;i++)w.Step();StateRecorderImpl second;w.system.SaveState(second);Check(first.GetData()==second.GetData(),"exact next trajectory rollback");
#endif
}
static void Clamped(){
 for(auto fixed:{EMotionType::Static,EMotionType::Kinematic})for(bool reverse:{false,true}){
  World w(fixed==EMotionType::Static?(reverse?"static-body2":"static-body1"):(reverse?"kinematic-body2":"kinematic-body1"));Body &anchor=w.BodyAt(RVec3(0,0,0),Vec3::sReplicate(.001f),fixed),&moving=w.BodyAt(RVec3(0,0,0),Vec3::sReplicate(.002f));
  if(fixed==EMotionType::Kinematic)w.system.GetBodyInterface().SetAngularVelocity(anchor.GetID(),Vec3(0,1,0));
  auto &h=w.Hinge(reverse?moving:anchor,reverse?anchor:moving,RVec3::sZero(),Vec3::sAxisY(),.001f);
  float initial=moving.GetAngularVelocity().GetY();for(int i=0;i<20;i++){w.Torque(moving,Vec3(0,.003f,0));w.Step();}
  Check(Near(moving.GetAngularVelocity().GetY()-initial,.003f*.1f/.003f,.001f),"clamped dynamic partner sees I+Jr in both orientations");
  float relative=(reverse?-1:1)*(moving.GetAngularVelocity().GetY()-anchor.GetAngularVelocity().GetY());Check(Near(Speed(h),relative,.001f),"moving kinematic axis enters relative error");
 }
}
static float PointError(Body &a,Body &b,Vec3Arg la,Vec3Arg lb){
 Vec3 ra=a.GetRotation()*la,rb=b.GetRotation()*lb;
 return (a.GetLinearVelocity()+a.GetAngularVelocity().Cross(ra)-b.GetLinearVelocity()-b.GetAngularVelocity().Cross(rb)).Length();
}
static void ConstraintControls(float jr){
 World w(jr>0?"anisotropic-neighbor-rotor":"anisotropic-neighbor-zero");Vec3 ia(.004f,.002f,.003f),ib(.003f,.0015f,.002f),ic(.002f,.002f,.002f);
 Body &a=w.BodyAt(RVec3(0,0,0),ia,EMotionType::Dynamic,Quat::sRotation(Vec3::sAxisZ(),.4f));
 Body &b=w.BodyAt(RVec3(.02f,.05f,0),ib,EMotionType::Dynamic,Quat::sRotation(Vec3::sAxisX(),.3f));Body &c=w.BodyAt(RVec3(.02f,.1f,0),ic);
 RVec3 point(.01f,.025f,0);auto &h=w.Hinge(a,b,point,Vec3::sAxisY(),jr);auto &neighbor=w.Hinge(b,c,RVec3(.02f,.075f,0),Vec3::sAxisX());
 Vec3 la=a.GetRotation().Conjugated()*Vec3(point-a.GetCenterOfMassPosition()),lb=b.GetRotation().Conjugated()*Vec3(point-b.GetCenterOfMassPosition());
 h.GetMotorSettings().mMinTorqueLimit=-.2f;h.GetMotorSettings().mMaxTorqueLimit=.2f;h.SetMotorState(EMotorState::Velocity);h.SetTargetAngularVelocity(3);
 float angular=0,pointError=0,postAngular=0,postPoint=0;
 for(int i=0;i<200;i++){
  // SetupVelocityConstraint caches these Jacobians before integration. Compare
  // solved velocities in that same frame, not the subsequently moved poses.
  Vec3 axis=b.GetRotation()*neighbor.GetLocalSpaceHingeAxis1(),ra=a.GetRotation()*la,rb=b.GetRotation()*lb;
  w.Step();Vec3 relative=b.GetAngularVelocity()-c.GetAngularVelocity();
  angular=std::max(angular,(relative-axis*relative.Dot(axis)).Length());
  pointError=std::max(pointError,(a.GetLinearVelocity()+a.GetAngularVelocity().Cross(ra)-b.GetLinearVelocity()-b.GetAngularVelocity().Cross(rb)).Length());
  Vec3 postAxis=b.GetRotation()*neighbor.GetLocalSpaceHingeAxis1();
  postAngular=std::max(postAngular,(relative-postAxis*relative.Dot(postAxis)).Length());postPoint=std::max(postPoint,PointError(a,b,la,lb));
 }
 std::cout<<"ordinaryNeighborTransverse="<<angular<<" anchorPointVelocity="<<pointError<<" postPoseGeometricTransverse="<<postAngular<<" postPoseGeometricPointVelocity="<<postPoint<<'\n';
 Check(std::abs(h.GetCurrentAngle())>.1f,"constraint controls actually driven");Check(angular<1e-4f,"ordinary neighboring transverse hinge <=1e-4 rad/s");Check(pointError<1e-4f,"anisotropic offset hinge anchor-point <=1e-4 m/s");
}
static void SharedAndConfiguration(){
#ifndef BW_ROTOR_BASELINE
 World w("shared-configuration-activation");Body &a=w.BodyAt(RVec3(0,0,0),Vec3::sReplicate(.003f)),&b=w.BodyAt(RVec3(0,.05f,0),Vec3::sReplicate(.002f)),&c=w.BodyAt(RVec3(0,.1f,0),Vec3::sReplicate(.001f));
 auto &h=w.Hinge(a,b,RVec3(0,.025f,0),Vec3::sAxisY(),.0003f),&g=w.Hinge(b,c,RVec3(0,.075f,0),Vec3::sAxisY(),.0002f);
 for(auto p:{&h,&g}){p->GetMotorSettings().mMinTorqueLimit=-.2f;p->GetMotorSettings().mMaxTorqueLimit=.2f;p->SetMotorState(EMotorState::Velocity);p->SetTargetAngularVelocity(p==&h?2:-1);}
 for(int i=0;i<100;i++)w.Step();Check(Near(b.GetAngularVelocity().GetY()-a.GetAngularVelocity().GetY(),2,.001f),"shared first servo");Check(Near(c.GetAngularVelocity().GetY()-b.GetAngularVelocity().GetY(),-1,.001f),"shared second servo");
 StateRecorderImpl shared;w.system.SaveState(shared);w.Event("shared-checkpoint");for(int i=0;i<5;i++)w.Step();StateRecorderImpl sharedExpected;w.system.SaveState(sharedExpected);
 SetRotor(h,0);SetRotor(g,.0006f);w.Event("shared-configuration-changed");w.Step();shared.Rewind();Check(w.system.RestoreState(shared),"shared rotor state restores");w.Event("shared-restore");for(int i=0;i<5;i++)w.Step();StateRecorderImpl sharedActual;w.system.SaveState(sharedActual);Check(sharedExpected.GetData()==sharedActual.GetData(),"shared configuration and next trajectory restore exactly");
 float speed=h.GetRotorAngularVelocity(),lambda=h.GetTotalLambdaRotor();w.Event("before-same-value-configuration");Check(h.SetReflectedRotorInertia(.0003f),"same configuration");Check(speed==h.GetRotorAngularVelocity()&&lambda==h.GetTotalLambdaRotor(),"same configuration exact no-op");w.Event("after-same-value-configuration");
 for(float bad:{-1.0f,std::numeric_limits<float>::infinity(),std::numeric_limits<float>::quiet_NaN()}){Check(!h.SetReflectedRotorInertia(bad),"invalid setter refused");Check(speed==h.GetRotorAngularVelocity()&&lambda==h.GetTotalLambdaRotor()&&h.GetReflectedRotorInertia()==.0003f,"invalid setter atomic");}
 w.Event("invalid-setters-refused");SetRotor(h,0);w.Event("disable");Check(Speed(h)==0&&h.GetTotalLambdaRotor()==0,"disable clears scalar and impulse");SetRotor(h,.0003f);w.Event("enable");Vec3 axis=a.GetRotation()*h.GetLocalSpaceHingeAxis1();Check(Near(Speed(h),axis.Dot(b.GetAngularVelocity()-a.GetAngularVelocity()),1e-6f),"enable uses current relative speed");
 auto mass=a.GetMotionProperties()->GetInverseMass();auto inertia=a.GetMotionProperties()->GetInverseInertiaDiagonal();auto com=a.GetCenterOfMassPosition();for(int i=0;i<10;i++){SetRotor(h,.0004f);SetRotor(h,0);w.Event("configuration-roundtrip");}Check(mass==a.GetMotionProperties()->GetInverseMass()&&inertia==a.GetMotionProperties()->GetInverseInertiaDiagonal()&&com==a.GetCenterOfMassPosition(),"configuration preserves bare mass inertia COM");
 // Settings reject invalid input without constructing another constraint.
 HingeConstraintSettings invalid;invalid.mReflectedRotorInertia=-1;Check(invalid.Create(a,b)==nullptr,"invalid settings rejected");
 // Native deactivation zeros BODY velocities without calling constraints. Scalar
 // energy is retained unless the caller explicitly drops/resynchronizes it.
 SetRotor(h,.0003f);float sleepingSpeed=Speed(h);Check(std::abs(sleepingSpeed)>.1f,"sleep control actually retains nonzero scalar speed");h.SetMotorState(EMotorState::Off);g.SetMotorState(EMotorState::Off);
 for(auto p:{&a,&b,&c}){p->SetAllowSleeping(true);w.system.GetBodyInterface().DeactivateBody(p->GetID());}
 w.Event("participants-inactive-with-configured-rotors");Check(Speed(h)==sleepingSpeed,"deactivation retains scalar without inventing a callback");
 h.SetReflectedRotorInertia(.0005f);w.Event("positive-change-while-inactive");Check(Speed(h)==sleepingSpeed&&h.GetTotalLambdaRotor()==0,"positive change preserves scalar speed and clears old impulse");Check(!a.IsActive()&&!b.IsActive()&&!c.IsActive(),"setter does not implicitly wake");
 h.SetReflectedRotorInertia(0);h.SetReflectedRotorInertia(.0005f);g.SetReflectedRotorInertia(0);g.SetReflectedRotorInertia(.0002f);w.Event("explicit-caller-drop-and-resynchronize-scalar-energy");
 Check(Speed(h)==0&&Speed(g)==0&&h.GetTotalLambdaRotor()==0&&g.GetTotalLambdaRotor()==0,"explicit resynchronization clears scalar and warm start");
 for(auto p:{&a,&b,&c})w.system.GetBodyInterface().ActivateBody(p->GetID());w.Event("caller-activation");Check(a.IsActive()&&b.IsActive()&&c.IsActive(),"caller explicitly activates participants");w.Step();
 Check(a.GetAngularVelocity().Length()<1e-8f&&b.GetAngularVelocity().Length()<1e-8f&&c.GetAngularVelocity().Length()<1e-8f&&Speed(h)==0&&Speed(g)==0,"idle wake after explicit resync has no stale impulse kick");
#endif
}
static void ZeroTrace(std::ostream &trace){
 World w("zero-compatibility");Body &a=w.BodyAt(RVec3::sZero(),Vec3::sReplicate(.003f)),&b=w.BodyAt(RVec3(0,.05f,0),Vec3::sReplicate(.002f));auto &h=w.Hinge(a,b,RVec3(0,.025f,0),Vec3::sAxisY());
 h.GetMotorSettings().mMinTorqueLimit=-.2f;h.GetMotorSettings().mMaxTorqueLimit=.2f;h.SetMotorState(EMotorState::Velocity);h.SetTargetAngularVelocity(2);
 trace<<std::hexfloat;for(int i=0;i<100;i++){w.Step();for(auto p:{&a,&b}){auto v=p->GetLinearVelocity(),av=p->GetAngularVelocity();auto q=p->GetRotation();auto x=p->GetPosition();trace<<i<<','<<x.GetX()<<','<<x.GetY()<<','<<x.GetZ()<<','<<q.GetX()<<','<<q.GetY()<<','<<q.GetZ()<<','<<q.GetW()<<','<<v.GetX()<<','<<v.GetY()<<','<<v.GetZ()<<','<<av.GetX()<<','<<av.GetY()<<','<<av.GetZ()<<'\n';}}
}
int main(int argc,char **argv){
 bool zeroOnly=false;std::string tracePath;for(int i=1;i<argc;i++){std::string a=argv[i];if(a=="--zero-only")zeroOnly=true;else if(a=="--disable-rotor-control")mutation=true;else if(a=="--trace"&&i+1<argc)tracePath=argv[++i];else return 2;}
 std::cout<<std::setprecision(9);std::cout<<"ROTOR_META {\"schema\":\"bw.native-rotor.trace.v1\",\"nativeSource\":\""<<BW_NATIVE_SOURCE_SHA<<"\",\"probeSource\":\""<<BW_PROBE_SOURCE_SHA<<"\",\"disableRotorControl\":"<<(mutation?"true":"false")<<"}\n";RegisterDefaultAllocator();Factory::sInstance=new Factory();RegisterTypes();
 if(zeroOnly){std::ofstream trace(tracePath);Check(bool(trace),"zero control trace opened");ZeroTrace(trace);}else{Analytic(.003f);Analytic(-.003f);Clamped();ConstraintControls(0);ConstraintControls(.0003f);SharedAndConfiguration();}
 UnregisterTypes();delete Factory::sInstance;Factory::sInstance=nullptr;std::cout<<"{\"checks\":"<<checks<<",\"failures\":"<<failures<<"}\n";return failures?1:0;
}
