// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-FileCopyrightText: 2026 Brickwright (fork addition, see BRICKWRIGHT.md)
// SPDX-License-Identifier: MIT

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Constraints/GearMeshConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#ifdef JPH_DEBUG_RENDERER
	#include <Jolt/Renderer/DebugRenderer.h>
#endif // JPH_DEBUG_RENDERER

JPH_NAMESPACE_BEGIN

JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(GearMeshConstraintSettings)
{
	JPH_ADD_BASE_CLASS(GearMeshConstraintSettings, TwoBodyConstraintSettings)

	JPH_ADD_ENUM_ATTRIBUTE(GearMeshConstraintSettings, mSpace)
	JPH_ADD_ATTRIBUTE(GearMeshConstraintSettings, mPoint1)
	JPH_ADD_ATTRIBUTE(GearMeshConstraintSettings, mHingeAxis1)
	JPH_ADD_ATTRIBUTE(GearMeshConstraintSettings, mPoint2)
	JPH_ADD_ATTRIBUTE(GearMeshConstraintSettings, mHingeAxis2)
	JPH_ADD_ATTRIBUTE(GearMeshConstraintSettings, mContactPoint)
	JPH_ADD_ATTRIBUTE(GearMeshConstraintSettings, mRatio)
}

void GearMeshConstraintSettings::SaveBinaryState(StreamOut &inStream) const
{
	ConstraintSettings::SaveBinaryState(inStream);

	inStream.Write(mSpace);
	inStream.Write(mPoint1);
	inStream.Write(mHingeAxis1);
	inStream.Write(mPoint2);
	inStream.Write(mHingeAxis2);
	inStream.Write(mContactPoint);
	inStream.Write(mRatio);
}

void GearMeshConstraintSettings::RestoreBinaryState(StreamIn &inStream)
{
	ConstraintSettings::RestoreBinaryState(inStream);

	inStream.Read(mSpace);
	inStream.Read(mPoint1);
	inStream.Read(mHingeAxis1);
	inStream.Read(mPoint2);
	inStream.Read(mHingeAxis2);
	inStream.Read(mContactPoint);
	inStream.Read(mRatio);
}

TwoBodyConstraint *GearMeshConstraintSettings::Create(Body &inBody1, Body &inBody2) const
{
	return new GearMeshConstraint(inBody1, inBody2, *this);
}

// Axes closer to parallel than this (|a1 x a2|, ~1.1 degrees) are treated as parallel
static constexpr float cParallelThreshold = 0.02f;

GearMeshConstraint::GearMeshConstraint(Body &inBody1, Body &inBody2, const GearMeshConstraintSettings &inSettings) :
	TwoBodyConstraint(inBody1, inBody2, inSettings),
	mRatio(inSettings.mRatio)
{
	RVec3 world_contact;
	if (inSettings.mSpace == EConstraintSpace::WorldSpace)
	{
		RMat44 inv1 = inBody1.GetInverseCenterOfMassTransform();
		RMat44 inv2 = inBody2.GetInverseCenterOfMassTransform();
		mLocalSpacePoint1 = Vec3(inv1 * inSettings.mPoint1);
		mLocalSpaceHingeAxis1 = inv1.Multiply3x3(inSettings.mHingeAxis1).Normalized();
		mLocalSpacePoint2 = Vec3(inv2 * inSettings.mPoint2);
		mLocalSpaceHingeAxis2 = inv2.Multiply3x3(inSettings.mHingeAxis2).Normalized();
		world_contact = inSettings.mContactPoint;
	}
	else
	{
		mLocalSpacePoint1 = Vec3(inSettings.mPoint1);
		mLocalSpaceHingeAxis1 = inSettings.mHingeAxis1.Normalized();
		mLocalSpacePoint2 = Vec3(inSettings.mPoint2);
		mLocalSpaceHingeAxis2 = inSettings.mHingeAxis2.Normalized();
		world_contact = inBody1.GetCenterOfMassTransform() * inSettings.mContactPoint;
	}

	// Classify the mesh and, for non-parallel axes, store the pitch point in the mesh frame (a1, u = a1 x a2, a1 x u) at c1
	RMat44 t1 = inBody1.GetCenterOfMassTransform();
	RMat44 t2 = inBody2.GetCenterOfMassTransform();
	Vec3 a1 = t1.Multiply3x3(mLocalSpaceHingeAxis1);
	Vec3 a2 = t2.Multiply3x3(mLocalSpaceHingeAxis2);
	Vec3 a1xa2 = a1.Cross(a2);
	mParallel = a1xa2.Length() < cParallelThreshold;
	mMeshFrameContactPoint = Vec3::sZero();
	if (!mParallel)
	{
		Vec3 u = a1xa2.Normalized();
		Vec3 v = a1.Cross(u);
		Vec3 d = Vec3(world_contact - t1 * mLocalSpacePoint1);
		mMeshFrameContactPoint = Vec3(d.Dot(a1), d.Dot(u), d.Dot(v));
	}

	CalculateGeometry();
}

void GearMeshConstraint::NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM)
{
	if (mBody1->GetID() == inBodyID)
		mLocalSpacePoint1 -= inDeltaCOM;
	else if (mBody2->GetID() == inBodyID)
		mLocalSpacePoint2 -= inDeltaCOM;
}

void GearMeshConstraint::SetConstraints(const Constraint *inGear1, const Constraint *inGear2)
{
	mGear1Constraint = inGear1;
	mGear2Constraint = inGear2;

	const Constraint *hinges[2] = { inGear1, inGear2 };
	const Body *gears[2] = { mBody1, mBody2 };
	const Vec3 local_axes[2] = { mLocalSpaceHingeAxis1, mLocalSpaceHingeAxis2 };
	for (int i = 0; i < 2; ++i)
	{
		mHingeSign[i] = 0.0f;
		mPrevAngle[i] = 0.0f;
		mAccumAngle[i] = 0.0f;
		if (hinges[i] == nullptr || hinges[i]->GetSubType() != EConstraintSubType::Hinge)
			continue;
		const HingeConstraint *hinge = StaticCast<HingeConstraint>(hinges[i]);

		// The hinge angle is the rotation of its body 2 relative to its body 1 about its world axis (body 1's)
		float body_sign;
		if (hinge->GetBody2() == gears[i])
			body_sign = 1.0f;
		else if (hinge->GetBody1() == gears[i])
			body_sign = -1.0f;
		else
		{
			JPH_ASSERT(false, "Hinge does not hold this gear");
			continue;
		}
		Vec3 hinge_axis = hinge->GetBody1()->GetRotation() * hinge->GetLocalSpaceHingeAxis1();
		Vec3 gear_axis = gears[i]->GetRotation() * local_axes[i];
		mHingeSign[i] = body_sign * (hinge_axis.Dot(gear_axis) >= 0.0f? 1.0f : -1.0f);
		mPrevAngle[i] = hinge->GetCurrentAngle();
	}
	if (mHingeSign[0] == 0.0f || mHingeSign[1] == 0.0f)
	{
		// Need both for the phase error
		mGear1Constraint = nullptr;
		mGear2Constraint = nullptr;
	}
}

float GearMeshConstraint::GetRelativeRotation(int inGear) const
{
	const Constraint *c = inGear == 0? mGear1Constraint.GetPtr() : mGear2Constraint.GetPtr();
	float angle = StaticCast<HingeConstraint>(c)->GetCurrentAngle();
	return mHingeSign[inGear] * (mAccumAngle[inGear] + CenterAngleAroundZero(angle - mPrevAngle[inGear]));
}

float GearMeshConstraint::GetPositionError() const
{
	if (mGear1Constraint == nullptr || mGear2Constraint == nullptr)
		return 0.0f;

	// Displacement of the material points at P (relative to the frame) since the hinges were set, along the line of
	// action; C = n . (p2 - p1) is the AxisConstraintPart convention
	return mWorldSpaceNormal.Dot(GetRelativeRotation(1) * mPitchVelocity2 - GetRelativeRotation(0) * mPitchVelocity1);
}

void GearMeshConstraint::CalculateGeometry()
{
	RMat44 t1 = mBody1->GetCenterOfMassTransform();
	RMat44 t2 = mBody2->GetCenterOfMassTransform();
	Vec3 a1 = t1.Multiply3x3(mLocalSpaceHingeAxis1).Normalized();
	Vec3 a2 = t2.Multiply3x3(mLocalSpaceHingeAxis2).Normalized();
	RVec3 c1 = t1 * mLocalSpacePoint1;
	RVec3 c2 = t2 * mLocalSpacePoint2;

	if (mParallel)
	{
		// Pitch point on the line of centres, dividing it so that the pitch radii give the ratio:
		// rho = -ratio * sign(a1 . a2) (> 0 external, < 0 internal), r1 = D rho / (1 + rho); axially halfway
		Vec3 d = Vec3(c2 - c1);
		float axial = d.Dot(a1);
		Vec3 perp = d - axial * a1;
		float dist = perp.Length();
		if (dist > 1.0e-9f)
		{
			Vec3 u = perp / dist;
			float rho = -mRatio * (a1.Dot(a2) >= 0.0f? 1.0f : -1.0f);
			float r1 = Square(1.0f + rho) > 1.0e-12f? dist * rho / (1.0f + rho) : 0.0f;
			mWorldSpaceContactPoint = c1 + r1 * u + (0.5f * axial) * a1;
		}
		else
			mWorldSpaceContactPoint = c1;
	}
	else
	{
		Vec3 a1xa2 = a1.Cross(a2);
		float len = a1xa2.Length();
		if (len > 1.0e-6f)
		{
			Vec3 u = a1xa2 / len;
			Vec3 v = a1.Cross(u);
			mWorldSpaceContactPoint = c1 + mMeshFrameContactPoint.GetX() * a1 + mMeshFrameContactPoint.GetY() * u + mMeshFrameContactPoint.GetZ() * v;
		}
		// else: axes became parallel (should be held by the hinges), keep the last contact point
	}

	// Velocity of the material point of each gear at P per unit rotation rate about its axis
	mPitchVelocity1 = a1.Cross(Vec3(mWorldSpaceContactPoint - c1));
	mPitchVelocity2 = a2.Cross(Vec3(mWorldSpaceContactPoint - c2));

	// Line of action n: in the plane of the two pitch velocities with n . e1 = ratio * n . e2 (then n . (e2 w2 - e1 w1) = 0 <=> w2 = ratio w1).
	// Parallel pitch velocities (spur, bevel): n = e1 / |e1|, the ratio follows from the pitch radii.
	float e1_len = mPitchVelocity1.Length();
	if (e1_len < 1.0e-9f)
		return; // Degenerate (pitch point on axis 1), keep the last normal
	Vec3 e1n = mPitchVelocity1 / e1_len;
	Vec3 n = e1n;
	float e2_len = mPitchVelocity2.Length();
	if (!mParallel && e2_len > 1.0e-9f && e1n.Cross(mPitchVelocity2 / e2_len).Length() > 1.0e-3f)
	{
		Vec3 w = mPitchVelocity1 - mRatio * mPitchVelocity2;
		float w_len_sq = w.LengthSq();
		if (w_len_sq > 1.0e-12f * Square(e1_len))
		{
			Vec3 m = mPitchVelocity1 - (mPitchVelocity1.Dot(w) / w_len_sq) * w;
			float m_len = m.Length();
			if (m_len > 1.0e-6f * e1_len)
				n = m / m_len;
		}
	}
	mWorldSpaceNormal = n;
}

void GearMeshConstraint::SetupVelocityConstraint(float inDeltaTime)
{
	// Track the hinge angles once per step (unwrapped)
	if (mGear1Constraint != nullptr && mGear2Constraint != nullptr)
	{
		const Constraint *hinges[2] = { mGear1Constraint.GetPtr(), mGear2Constraint.GetPtr() };
		for (int i = 0; i < 2; ++i)
		{
			float angle = StaticCast<HingeConstraint>(hinges[i])->GetCurrentAngle();
			mAccumAngle[i] += CenterAngleAroundZero(angle - mPrevAngle[i]);
			mPrevAngle[i] = angle;
		}
	}

	CalculateGeometry();
	Vec3 r1 = Vec3(mWorldSpaceContactPoint - mBody1->GetCenterOfMassPosition());
	Vec3 r2 = Vec3(mWorldSpaceContactPoint - mBody2->GetCenterOfMassPosition());
	mAxisConstraintPart.CalculateConstraintProperties(*mBody1, r1, *mBody2, r2, mWorldSpaceNormal);
}

void GearMeshConstraint::ResetWarmStart()
{
	mAxisConstraintPart.Deactivate();
}

void GearMeshConstraint::WarmStartVelocityConstraint(float inWarmStartImpulseRatio)
{
	mAxisConstraintPart.WarmStart(*mBody1, *mBody2, mWorldSpaceNormal, inWarmStartImpulseRatio);
}

bool GearMeshConstraint::SolveVelocityConstraint(float inDeltaTime)
{
	return mAxisConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2, mWorldSpaceNormal, -FLT_MAX, FLT_MAX);
}

bool GearMeshConstraint::SolvePositionConstraint(float inDeltaTime, float inBaumgarte)
{
	if (mGear1Constraint == nullptr || mGear2Constraint == nullptr)
		return false;

	// Bodies may have moved
	CalculateGeometry();
	float error = GetPositionError();
	if (error == 0.0f)
		return false;

	Vec3 r1 = Vec3(mWorldSpaceContactPoint - mBody1->GetCenterOfMassPosition());
	Vec3 r2 = Vec3(mWorldSpaceContactPoint - mBody2->GetCenterOfMassPosition());
	mAxisConstraintPart.CalculateConstraintProperties(*mBody1, r1, *mBody2, r2, mWorldSpaceNormal);
	return mAxisConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, mWorldSpaceNormal, error, inBaumgarte);
}

#ifdef JPH_DEBUG_RENDERER
void GearMeshConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	inRenderer->DrawMarker(mWorldSpaceContactPoint, Color::sWhite, 0.01f);
	inRenderer->DrawArrow(mWorldSpaceContactPoint, mWorldSpaceContactPoint + 0.02f * mWorldSpaceNormal, Color::sGreen, 0.005f);
}
#endif // JPH_DEBUG_RENDERER

void GearMeshConstraint::SaveState(StateRecorder &inStream) const
{
	TwoBodyConstraint::SaveState(inStream);

	mAxisConstraintPart.SaveState(inStream);
	inStream.Write(mWorldSpaceNormal);
	inStream.Write(mPrevAngle[0]);
	inStream.Write(mPrevAngle[1]);
	inStream.Write(mAccumAngle[0]);
	inStream.Write(mAccumAngle[1]);
}

void GearMeshConstraint::RestoreState(StateRecorder &inStream)
{
	TwoBodyConstraint::RestoreState(inStream);

	mAxisConstraintPart.RestoreState(inStream);
	inStream.Read(mWorldSpaceNormal);
	inStream.Read(mPrevAngle[0]);
	inStream.Read(mPrevAngle[1]);
	inStream.Read(mAccumAngle[0]);
	inStream.Read(mAccumAngle[1]);
}

Ref<ConstraintSettings> GearMeshConstraint::GetConstraintSettings() const
{
	GearMeshConstraintSettings *settings = new GearMeshConstraintSettings;
	ToConstraintSettings(*settings);
	settings->mSpace = EConstraintSpace::LocalToBodyCOM;
	settings->mPoint1 = RVec3(mLocalSpacePoint1);
	settings->mHingeAxis1 = mLocalSpaceHingeAxis1;
	settings->mPoint2 = RVec3(mLocalSpacePoint2);
	settings->mHingeAxis2 = mLocalSpaceHingeAxis2;
	settings->mContactPoint = mBody1->GetInverseCenterOfMassTransform() * mWorldSpaceContactPoint;
	settings->mRatio = mRatio;
	return settings;
}

Mat44 GearMeshConstraint::GetConstraintToBody1Matrix() const
{
	Vec3 perp = mLocalSpaceHingeAxis1.GetNormalizedPerpendicular();
	return Mat44(Vec4(mLocalSpaceHingeAxis1, 0), Vec4(perp, 0), Vec4(mLocalSpaceHingeAxis1.Cross(perp), 0), Vec4(mLocalSpacePoint1, 1));
}

Mat44 GearMeshConstraint::GetConstraintToBody2Matrix() const
{
	Vec3 perp = mLocalSpaceHingeAxis2.GetNormalizedPerpendicular();
	return Mat44(Vec4(mLocalSpaceHingeAxis2, 0), Vec4(perp, 0), Vec4(mLocalSpaceHingeAxis2.Cross(perp), 0), Vec4(mLocalSpacePoint2, 1));
}

JPH_NAMESPACE_END
