// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-FileCopyrightText: 2026 Brickwright (fork addition, see BRICKWRIGHT.md)
// SPDX-License-Identifier: MIT

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Constraints/RackMeshConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/LargeIslandSplitter.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#ifdef JPH_DEBUG_RENDERER
	#include <Jolt/Renderer/DebugRenderer.h>
#endif // JPH_DEBUG_RENDERER

JPH_NAMESPACE_BEGIN

JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(RackMeshConstraintSettings)
{
	JPH_ADD_BASE_CLASS(RackMeshConstraintSettings, TwoBodyConstraintSettings)

	JPH_ADD_ENUM_ATTRIBUTE(RackMeshConstraintSettings, mSpace)
	JPH_ADD_ATTRIBUTE(RackMeshConstraintSettings, mHingeAxis)
	JPH_ADD_ATTRIBUTE(RackMeshConstraintSettings, mSliderAxis)
	JPH_ADD_ATTRIBUTE(RackMeshConstraintSettings, mRatio)
}

void RackMeshConstraintSettings::SaveBinaryState(StreamOut &inStream) const
{
	ConstraintSettings::SaveBinaryState(inStream);

	inStream.Write(mSpace);
	inStream.Write(mHingeAxis);
	inStream.Write(mSliderAxis);
	inStream.Write(mRatio);
}

void RackMeshConstraintSettings::RestoreBinaryState(StreamIn &inStream)
{
	ConstraintSettings::RestoreBinaryState(inStream);

	inStream.Read(mSpace);
	inStream.Read(mHingeAxis);
	inStream.Read(mSliderAxis);
	inStream.Read(mRatio);
}

TwoBodyConstraint *RackMeshConstraintSettings::Create(Body &inBody1, Body &inBody2) const
{
	return new RackMeshConstraint(inBody1, inBody2, *this);
}

RackMeshConstraint::RackMeshConstraint(Body &inBody1, Body &inBody2, const RackMeshConstraintSettings &inSettings) :
	TwoBodyConstraint(inBody1, inBody2, inSettings),
	mLocalSpaceHingeAxis(inSettings.mHingeAxis),
	mLocalSpaceSliderAxis(inSettings.mSliderAxis),
	mRatio(inSettings.mRatio)
{
	if (inSettings.mSpace == EConstraintSpace::WorldSpace)
	{
		mLocalSpaceHingeAxis = inBody1.GetInverseCenterOfMassTransform().Multiply3x3(mLocalSpaceHingeAxis).Normalized();
		mLocalSpaceSliderAxis = inBody2.GetInverseCenterOfMassTransform().Multiply3x3(mLocalSpaceSliderAxis).Normalized();
	}
	else
	{
		mLocalSpaceHingeAxis = mLocalSpaceHingeAxis.Normalized();
		mLocalSpaceSliderAxis = mLocalSpaceSliderAxis.Normalized();
	}
}

void RackMeshConstraint::SetConstraints(const Constraint *inPinion, const Constraint *inRack)
{
	mPinionConstraint = nullptr;
	mRackConstraint = nullptr;
	mPinionFrame = nullptr;
	mRackFrame = nullptr;
	mHingeSign = 0.0f;
	mSliderSign = 0.0f;
	mPrevAngle = 0.0f;
	mAccumAngle = 0.0f;
	mBasePosition = 0.0f;
	if (inPinion == nullptr || inRack == nullptr
		|| inPinion->GetSubType() != EConstraintSubType::Hinge || inRack->GetSubType() != EConstraintSubType::Slider)
		return;
	const HingeConstraint *hinge = StaticCast<HingeConstraint>(inPinion);
	const SliderConstraint *slider = StaticCast<SliderConstraint>(inRack);

	// The hinge angle is the rotation of its body 2 relative to its body 1 about its axis (body 1's)
	float hinge_body_sign;
	if (hinge->GetBody2() == mBody1)
		hinge_body_sign = 1.0f;
	else if (hinge->GetBody1() == mBody1)
		hinge_body_sign = -1.0f;
	else
	{
		JPH_ASSERT(false, "Hinge does not hold the pinion");
		return;
	}
	Vec3 hinge_axis = hinge->GetBody1()->GetRotation() * hinge->GetLocalSpaceHingeAxis1();
	Vec3 pinion_axis = mBody1->GetRotation() * mLocalSpaceHingeAxis;

	// The slider position is the travel of its body 2 relative to its body 1 along its axis (body 1's)
	float slider_body_sign;
	if (slider->GetBody2() == mBody2)
		slider_body_sign = 1.0f;
	else if (slider->GetBody1() == mBody2)
		slider_body_sign = -1.0f;
	else
	{
		JPH_ASSERT(false, "Slider does not hold the rack");
		return;
	}
	Vec3 slider_axis = slider->GetBody1()->GetRotation() * Vec3(slider->GetConstraintToBody1Matrix().GetColumn3(0));
	Vec3 rack_axis = mBody2->GetRotation() * mLocalSpaceSliderAxis;

	mPinionConstraint = inPinion;
	mRackConstraint = inRack;
	mHingeSign = hinge_body_sign * (hinge_axis.Dot(pinion_axis) >= 0.0f? 1.0f : -1.0f);
	mSliderSign = slider_body_sign * (slider_axis.Dot(rack_axis) >= 0.0f? 1.0f : -1.0f);
	mPinionFrame = hinge_body_sign > 0.0f? hinge->GetBody1() : hinge->GetBody2();
	mRackFrame = slider_body_sign > 0.0f? slider->GetBody1() : slider->GetBody2();
	mPrevAngle = hinge->GetCurrentAngle();
	mBasePosition = slider->GetCurrentPosition();
}

float RackMeshConstraint::GetPinionRotation() const
{
	float angle = StaticCast<HingeConstraint>(mPinionConstraint.GetPtr())->GetCurrentAngle();
	return mHingeSign * (mAccumAngle + CenterAngleAroundZero(angle - mPrevAngle));
}

float RackMeshConstraint::GetRackTranslation() const
{
	return mSliderSign * (StaticCast<SliderConstraint>(mRackConstraint.GetPtr())->GetCurrentPosition() - mBasePosition);
}

float RackMeshConstraint::GetPositionError() const
{
	if (mPinionConstraint == nullptr || mRackConstraint == nullptr)
		return 0.0f;
	return GetPinionRotation() - mRatio * GetRackTranslation();
}

void RackMeshConstraint::CalculateConstraintProperties()
{
	mWorldSpaceHingeAxis = mBody1->GetRotation() * mLocalSpaceHingeAxis;
	mWorldSpaceSliderAxis = mBody2->GetRotation() * mLocalSpaceSliderAxis;
	const Vec3 a = mWorldSpaceHingeAxis;
	const Vec3 rb = mRatio * mWorldSpaceSliderAxis;

	// C' = (w1 - W_Fp) . a - r (v2 - v_Fr - W_Fr x (x2 - x_Fr)) . b
	mNumRows = 0;
	auto add_row = [this](Body *inBody, Vec3Arg inLinear, Vec3Arg inAngular) {
		for (int i = 0; i < mNumRows; ++i)
			if (mRows[i].mBody == inBody)
			{
				mRows[i].mLinear += inLinear;
				mRows[i].mAngular += inAngular;
				return;
			}
		Row &r = mRows[mNumRows++];
		r.mBody = inBody;
		r.mLinear = inLinear;
		r.mAngular = inAngular;
	};
	add_row(mBody1, Vec3::sZero(), a);
	add_row(mBody2, -rb, Vec3::sZero());
	if (mPinionFrame != nullptr)
		add_row(mPinionFrame, Vec3::sZero(), -a);
	if (mRackFrame != nullptr)
		add_row(mRackFrame, rb, Vec3(mBody2->GetCenterOfMassPosition() - mRackFrame->GetCenterOfMassPosition()).Cross(rb));

	// K^-1 = J M^-1 J^T over the dynamic bodies
	float inv_effective_mass = 0.0f;
	for (int i = 0; i < mNumRows; ++i)
	{
		Row &r = mRows[i];
		if (r.mBody->IsDynamic())
		{
			const MotionProperties *mp = r.mBody->GetMotionProperties();
			r.mInvMass = mp->GetInverseMass();
			r.mInvIAngular = mp->MultiplyWorldSpaceInverseInertiaByVector(r.mBody->GetRotation(), r.mAngular);
			inv_effective_mass += r.mInvMass * r.mLinear.LengthSq() + r.mAngular.Dot(r.mInvIAngular);
		}
		else
		{
			r.mInvMass = 0.0f;
			r.mInvIAngular = Vec3::sZero();
		}
	}
	if (inv_effective_mass >= FLT_MIN)
		mEffectiveMass = 1.0f / inv_effective_mass;
	else
	{
		mEffectiveMass = 0.0f;
		mTotalLambda = 0.0f;
	}
}

bool RackMeshConstraint::ApplyVelocityStep(float inLambda)
{
	if (inLambda == 0.0f)
		return false;
	for (int i = 0; i < mNumRows; ++i)
	{
		const Row &r = mRows[i];
		if (!r.mBody->IsDynamic())
			continue;
		MotionProperties *mp = r.mBody->GetMotionProperties();
		if (r.mInvMass != 0.0f && r.mLinear != Vec3::sZero())
			mp->AddLinearVelocityStep((inLambda * r.mInvMass) * r.mLinear);
		mp->AddAngularVelocityStep(inLambda * r.mInvIAngular);
	}
	return true;
}

bool RackMeshConstraint::ApplyPositionStep(float inLambda)
{
	if (inLambda == 0.0f)
		return false;
	for (int i = 0; i < mNumRows; ++i)
	{
		const Row &r = mRows[i];
		if (!r.mBody->IsDynamic())
			continue;
		if (r.mInvMass != 0.0f && r.mLinear != Vec3::sZero())
			r.mBody->AddPositionStep((inLambda * r.mInvMass) * r.mLinear);
		r.mBody->AddRotationStep(inLambda * r.mInvIAngular);
	}
	return true;
}

void RackMeshConstraint::SetupVelocityConstraint(float inDeltaTime)
{
	// Track the hinge angle once per step (unwrapped)
	if (mPinionConstraint != nullptr)
	{
		float angle = StaticCast<HingeConstraint>(mPinionConstraint.GetPtr())->GetCurrentAngle();
		mAccumAngle += CenterAngleAroundZero(angle - mPrevAngle);
		mPrevAngle = angle;
	}

	CalculateConstraintProperties();
}

void RackMeshConstraint::ResetWarmStart()
{
	mTotalLambda = 0.0f;
}

void RackMeshConstraint::WarmStartVelocityConstraint(float inWarmStartImpulseRatio)
{
	mTotalLambda *= inWarmStartImpulseRatio;
	ApplyVelocityStep(mTotalLambda);
}

bool RackMeshConstraint::SolveVelocityConstraint(float inDeltaTime)
{
	if (mEffectiveMass == 0.0f)
		return false;

	float jv = 0.0f;
	for (int i = 0; i < mNumRows; ++i)
	{
		const Row &r = mRows[i];
		if (r.mBody->IsStatic())
			continue;
		jv += r.mLinear.Dot(r.mBody->GetLinearVelocity()) + r.mAngular.Dot(r.mBody->GetAngularVelocity());
	}
	float lambda = -mEffectiveMass * jv;
	mTotalLambda += lambda;
	return ApplyVelocityStep(lambda);
}

bool RackMeshConstraint::SolvePositionConstraint(float inDeltaTime, float inBaumgarte)
{
	if (mPinionConstraint == nullptr || mRackConstraint == nullptr)
		return false;

	CalculateConstraintProperties();
	float error = GetPositionError();
	if (error == 0.0f || mEffectiveMass == 0.0f)
		return false;
	return ApplyPositionStep(-mEffectiveMass * inBaumgarte * error);
}

uint RackMeshConstraint::BuildIslandSplits(LargeIslandSplitter &ioSplitter) const
{
	// The frames are touched too: keep this constraint out of the parallel splits when one of them is dynamic
	if ((mPinionFrame != nullptr && mPinionFrame->IsDynamic()) || (mRackFrame != nullptr && mRackFrame->IsDynamic()))
		return ioSplitter.AssignToNonParallelSplit(mBody1);
	return TwoBodyConstraint::BuildIslandSplits(ioSplitter);
}

#ifdef JPH_DEBUG_RENDERER
void RackMeshConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	RMat44 transform1 = mBody1->GetCenterOfMassTransform();
	RMat44 transform2 = mBody2->GetCenterOfMassTransform();
	inRenderer->DrawArrow(transform1.GetTranslation(), transform1 * mLocalSpaceHingeAxis, Color::sGreen, 0.01f);
	inRenderer->DrawArrow(transform2.GetTranslation(), transform2 * mLocalSpaceSliderAxis, Color::sBlue, 0.01f);
}
#endif // JPH_DEBUG_RENDERER

void RackMeshConstraint::SaveState(StateRecorder &inStream) const
{
	TwoBodyConstraint::SaveState(inStream);

	inStream.Write(mTotalLambda);
	inStream.Write(mPrevAngle);
	inStream.Write(mAccumAngle);
	inStream.Write(mBasePosition);
}

void RackMeshConstraint::RestoreState(StateRecorder &inStream)
{
	TwoBodyConstraint::RestoreState(inStream);

	inStream.Read(mTotalLambda);
	inStream.Read(mPrevAngle);
	inStream.Read(mAccumAngle);
	inStream.Read(mBasePosition);
}

Ref<ConstraintSettings> RackMeshConstraint::GetConstraintSettings() const
{
	RackMeshConstraintSettings *settings = new RackMeshConstraintSettings;
	ToConstraintSettings(*settings);
	settings->mSpace = EConstraintSpace::LocalToBodyCOM;
	settings->mHingeAxis = mLocalSpaceHingeAxis;
	settings->mSliderAxis = mLocalSpaceSliderAxis;
	settings->mRatio = mRatio;
	return settings;
}

Mat44 RackMeshConstraint::GetConstraintToBody1Matrix() const
{
	Vec3 perp = mLocalSpaceHingeAxis.GetNormalizedPerpendicular();
	return Mat44(Vec4(mLocalSpaceHingeAxis, 0), Vec4(perp, 0), Vec4(mLocalSpaceHingeAxis.Cross(perp), 0), Vec4(0, 0, 0, 1));
}

Mat44 RackMeshConstraint::GetConstraintToBody2Matrix() const
{
	Vec3 perp = mLocalSpaceSliderAxis.GetNormalizedPerpendicular();
	return Mat44(Vec4(mLocalSpaceSliderAxis, 0), Vec4(perp, 0), Vec4(mLocalSpaceSliderAxis.Cross(perp), 0), Vec4(0, 0, 0, 1));
}

JPH_NAMESPACE_END
