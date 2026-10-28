// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2025 Jorrit Rouwe
// SPDX-License-Identifier: CC0-1.0
// This file is in the public domain. It serves as an example to start building your own application using Jolt Physics. Feel free to copy paste without attribution!

// The Jolt headers don't include Jolt.h. Always include Jolt.h before including any other Jolt header.
// You can use Jolt.h in your precompiled header to speed up compilation.
#include <Jolt/Jolt.h>

// Jolt includes
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <cassert>
#include <cmath>
#include <vector>
#include <fstream>
#include <iomanip>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Physics/StateRecorderImpl.h>

// STL includes
#include <iostream>
#include <cstdarg>
#include <thread>

// Disable common warnings triggered by Jolt, you can use JPH_SUPPRESS_WARNING_PUSH / JPH_SUPPRESS_WARNING_POP to store and restore the warning state
JPH_SUPPRESS_WARNINGS

// All Jolt symbols are in the JPH namespace
using namespace JPH;

// If you want your code to compile using single or double precision write 0.0_r to get a Real value that compiles to double or float depending if JPH_DOUBLE_PRECISION is set or not.
using namespace JPH::literals;

// We're also using STL classes in this example
using namespace std;

// Callback for traces, connect this to your own trace function if you have one
static void TraceImpl(const char *inFMT, ...)
{
	// Format the message
	va_list list;
	va_start(list, inFMT);
	char buffer[1024];
	vsnprintf(buffer, sizeof(buffer), inFMT, list);
	va_end(list);

	// Print to the TTY
	cout << buffer << endl;
}

#ifdef JPH_ENABLE_ASSERTS

// Callback for asserts, connect this to your own assert handler if you have one
static bool AssertFailedImpl(const char *inExpression, const char *inMessage, const char *inFile, uint inLine)
{
	// Print to the TTY
	cout << inFile << ":" << inLine << ": (" << inExpression << ") " << (inMessage != nullptr? inMessage : "") << endl;

	// Breakpoint
	return true;
};

#endif // JPH_ENABLE_ASSERTS

// Layer that objects can be in, determines which other objects it can collide with
// Typically you at least want to have 1 layer for moving bodies and 1 layer for static bodies, but you can have more
// layers if you want. E.g. you could have a layer for high detail collision (which is not used by the physics simulation
// but only if you do collision testing).
namespace Layers
{
	static constexpr ObjectLayer NON_MOVING = 0;
	static constexpr ObjectLayer MOVING = 1;
	static constexpr ObjectLayer NUM_LAYERS = 2;
};

/// Class that determines if two object layers can collide
class ObjectLayerPairFilterImpl : public ObjectLayerPairFilter
{
public:
	virtual bool					ShouldCollide(ObjectLayer inObject1, ObjectLayer inObject2) const override
	{
		switch (inObject1)
		{
		case Layers::NON_MOVING:
			return inObject2 == Layers::MOVING; // Non moving only collides with moving
		case Layers::MOVING:
			return true; // Moving collides with everything
		default:
			JPH_ASSERT(false);
			return false;
		}
	}
};

// Each broadphase layer results in a separate bounding volume tree in the broad phase. You at least want to have
// a layer for non-moving and moving objects to avoid having to update a tree full of static objects every frame.
// You can have a 1-on-1 mapping between object layers and broadphase layers (like in this case) but if you have
// many object layers you'll be creating many broad phase trees, which is not efficient. If you want to fine tune
// your broadphase layers define JPH_TRACK_BROADPHASE_STATS and look at the stats reported on the TTY.
namespace BroadPhaseLayers
{
	static constexpr BroadPhaseLayer NON_MOVING(0);
	static constexpr BroadPhaseLayer MOVING(1);
	static constexpr uint NUM_LAYERS(2);
};

// BroadPhaseLayerInterface implementation
// This defines a mapping between object and broadphase layers.
class BPLayerInterfaceImpl final : public BroadPhaseLayerInterface
{
public:
									BPLayerInterfaceImpl()
	{
		// Create a mapping table from object to broad phase layer
		mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
		mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
	}

	virtual uint					GetNumBroadPhaseLayers() const override
	{
		return BroadPhaseLayers::NUM_LAYERS;
	}

	virtual BroadPhaseLayer			GetBroadPhaseLayer(ObjectLayer inLayer) const override
	{
		JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
		return mObjectToBroadPhase[inLayer];
	}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char *			GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override
	{
		switch ((BroadPhaseLayer::Type)inLayer)
		{
		case (BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING:	return "NON_MOVING";
		case (BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:		return "MOVING";
		default:													JPH_ASSERT(false); return "INVALID";
		}
	}
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

private:
	BroadPhaseLayer					mObjectToBroadPhase[Layers::NUM_LAYERS];
};

/// Class that determines if an object layer can collide with a broadphase layer
class ObjectVsBroadPhaseLayerFilterImpl : public ObjectVsBroadPhaseLayerFilter
{
public:
	virtual bool				ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override
	{
		switch (inLayer1)
		{
		case Layers::NON_MOVING:
			return inLayer2 == BroadPhaseLayers::MOVING;
		case Layers::MOVING:
			return true;
		default:
			JPH_ASSERT(false);
			return false;
		}
	}
};



static int checks=0,failures=0;
static float LastImpulse(const DistanceConstraint &c){
#ifdef BW_DISTANCE_IMPULSE_BASELINE
 return c.GetTotalLambdaPosition();
#else
 return c.GetLastVelocityImpulse();
#endif
}
static void Check(bool ok,const char *label){if(!ok){std::cerr<<"FAIL "<<label<<"\n";++failures;}++checks;}
int main(int argc,char **argv){
 const bool legacy=argc>1&&std::string(argv[1])=="--legacy";
 std::ofstream trace;if(argc>2){trace.open(argv[2]);trace<<std::hexfloat;}
 RegisterDefaultAllocator();Factory::sInstance=new Factory();RegisterTypes();
 {
 BPLayerInterfaceImpl bp;ObjectVsBroadPhaseLayerFilterImpl vs;ObjectLayerPairFilterImpl pairs;
 PhysicsSystem system;system.Init(16,0,32,32,bp,vs,pairs);auto &bi=system.GetBodyInterface();
 RefConst<Shape> shape=new SphereShape(.001f);
 Body *anchor=bi.CreateBody(BodyCreationSettings(shape,RVec3::sZero(),Quat::sIdentity(),EMotionType::Static,Layers::NON_MOVING));
 Body *payload=bi.CreateBody(BodyCreationSettings(shape,RVec3(1.001f,0,0),Quat::sIdentity(),EMotionType::Dynamic,Layers::MOVING));
 bi.AddBody(anchor->GetID(),EActivation::DontActivate);bi.AddBody(payload->GetID(),EActivation::Activate);
 for(bool lower:{false,true}){
  float start=lower?.999f:1.001f,end=lower?1.001f:.999f;
  bi.SetPosition(payload->GetID(),RVec3(start,0,0),EActivation::DontActivate);
  DistanceConstraintSettings settings;settings.mPoint1=RVec3::sZero();settings.mPoint2=RVec3(start,0,0);settings.mMinDistance=lower?1:0;settings.mMaxDistance=lower?2:1;
  Ref<DistanceConstraint> rope=static_cast<DistanceConstraint *>(settings.Create(*anchor,*payload));
  bi.SetLinearVelocity(payload->GetID(),Vec3(lower?-1:1,0,0));rope->SetupVelocityConstraint(.005f);rope->WarmStartVelocityConstraint(0);
  rope->SolveVelocityConstraint(.005f);float applied=rope->GetTotalLambdaPosition();Vec3 velocity=payload->GetLinearVelocity();
  Check(lower?applied>0:applied<0,"signed applied impulse");
  Check(LastImpulse(*rope)==applied,"observe final velocity impulse");
  StateRecorderImpl recorder;rope->SaveState(recorder);
  bi.SetPosition(payload->GetID(),RVec3(end,0,0),EActivation::DontActivate);
  rope->SolvePositionConstraint(.005f,.2f);
  Check(rope->GetTotalLambdaPosition()==0,"legacy getter still clears");
  Check((payload->GetLinearVelocity()-velocity).Length()==0,"position solve does not undo momentum");
  Check((legacy?rope->GetTotalLambdaPosition():LastImpulse(*rope))==applied,"retained actual impulse after correction");
  rope->SetupVelocityConstraint(.005f);Check(LastImpulse(*rope)==0,"fresh velocity setup clears observation");
  rope->WarmStartVelocityConstraint(.005f);Check(LastImpulse(*rope)==0,"inactive warm start has zero observation");
  rope->ResetWarmStart();Check(LastImpulse(*rope)==0,"explicit reset clears observation");
  recorder.Rewind();rope->RestoreState(recorder);Check(LastImpulse(*rope)==0,"restore clears observation without extending solver state");
  Check(rope->GetTotalLambdaPosition()==applied,"restore preserves original serialized solver impulse");
 }
 bi.RemoveBody(payload->GetID());bi.DestroyBody(payload->GetID());
 // A four-rope suspension with the Cable Compass anchor dimensions. No ground
 // contact or driven output; collect every native 5ms step after settling.
 BodyCreationSettings bodySettings(shape,RVec3(0,-.1f,0),Quat::sIdentity(),EMotionType::Dynamic,Layers::MOVING);
 bodySettings.mOverrideMassProperties=EOverrideMassProperties::CalculateInertia;bodySettings.mMassPropertiesOverride.mMass=.0109f;bodySettings.mAllowSleeping=false;
 payload=bi.CreateBody(bodySettings);bi.AddBody(payload->GetID(),EActivation::Activate);
 std::vector<Ref<DistanceConstraint>> ropes;std::vector<Vec3> anchors;
 for(float x:{-.064f,.064f})for(float z:{-.024f,.024f}){
  anchors.emplace_back(x,0,z);DistanceConstraintSettings ds;ds.mPoint1=RVec3(x,0,z);ds.mPoint2=payload->GetPosition();ds.mMinDistance=0;ds.mMaxDistance=std::sqrt(x*x+.1f*.1f+z*z);
  Ref<DistanceConstraint> c=static_cast<DistanceConstraint *>(ds.Create(*anchor,*payload));system.AddConstraint(c);ropes.push_back(c);
 }
 TempAllocatorImpl allocator(10*1024*1024);JobSystemSingleThreaded jobs(cMaxPhysicsJobs);
 Vec3 impulse=Vec3::sZero(),oldImpulse=Vec3::sZero(),drag=Vec3::sZero();const float dt=.005f,mass=.0109f;Vec3 initialVelocity=Vec3::sZero();
 for(int i=0;i<400;i++){
  Vec3 before=payload->GetLinearVelocity(),pos=Vec3(payload->GetPosition());if(i==300)initialVelocity=before;
  system.Update(dt,1,&allocator,&jobs);
  if(trace){auto pos=payload->GetPosition();auto rot=payload->GetRotation();auto v=payload->GetLinearVelocity();
   trace<<i<<","<<pos.GetX()<<","<<pos.GetY()<<","<<pos.GetZ()<<","<<rot.GetX()<<","<<rot.GetY()<<","<<rot.GetZ()<<","<<rot.GetW()<<","<<v.GetX()<<","<<v.GetY()<<","<<v.GetZ()<<"\n";}

  if(i>=300){
   for(size_t j=0;j<ropes.size();j++){
    Vec3 normal=(pos-anchors[j]).Normalized();impulse+=normal*LastImpulse(*ropes[j]);oldImpulse+=normal*ropes[j]->GetTotalLambdaPosition();
   }
   drag+=-mass*bodySettings.mLinearDamping*dt*(before+system.GetGravity()*dt);
  }
 }
 float weightImpulse=mass*system.GetGravity().Length()*.5f;
 Vec3 momentum=mass*(payload->GetLinearVelocity()-initialVelocity),gravity=mass*system.GetGravity()*.5f;
 float residual=(impulse+gravity+drag-momentum).Length()/weightImpulse,support=impulse.GetY()/weightImpulse;
 float oldResidual=(oldImpulse+gravity+drag-momentum).Length()/weightImpulse;
 Check(residual<.05f,"native full-rate rope/gravity/drag momentum balance");Check(support>.9f&&support<1.1f,"native rope support supplies weight");
 std::cout<<"{\"failures\":"<<failures<<",\"checks\":"<<checks<<",\"relativeResidual\":"<<residual<<",\"supportFraction\":"<<support<<",\"legacyRelativeResidual\":"<<oldResidual<<"}\n";
 for(auto &c:ropes)system.RemoveConstraint(c);ropes.clear();
 bi.RemoveBody(payload->GetID());bi.RemoveBody(anchor->GetID());bi.DestroyBody(payload->GetID());bi.DestroyBody(anchor->GetID());
 }
 UnregisterTypes();delete Factory::sInstance;Factory::sInstance=nullptr;return failures?1:0;
}
