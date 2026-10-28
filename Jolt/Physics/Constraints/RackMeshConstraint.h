// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-FileCopyrightText: 2026 Brickwright (fork addition, see BRICKWRIGHT.md)
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>

JPH_NAMESPACE_BEGIN

/// Rack mesh constraint settings (Brickwright fork addition, see BRICKWRIGHT.md section 4)
///
/// A rack and pinion whose relation holds RELATIVE TO THE FRAMES that carry them, as GearMeshConstraint does for
/// gears. RackAndPinionConstraint ties the pinion's world rotation rate to the rack's world velocity
/// (w1 . a = r v2 . b): that is only right while the frame holding both is at rest. On a turning or driving chassis
/// it drives the rack along with the chassis's own motion (a forklift's carriage pushed to its end stop as the
/// robot turned, a steering rack that twisted the car). Here
///
///   (w_pinion - W_Fp) . a = r (v_rack - v_Fr - W_Fr x (x_rack - x_Fr)) . b
///
/// with Fp the other body of the pinion's hinge and Fr the other body of the rack's slider (given by SetConstraints;
/// without them the frames count as at rest, which is RackAndPinionConstraint's equation). The impulse is a couple on
/// pinion/Fp and an equal and opposite force pair along b on rack/Fr (through the rack's centre of mass), so linear
/// and angular momentum are conserved and the reaction reaches the frames.
///
/// Conventions as RackAndPinionConstraintSettings: body 1 the pinion, body 2 the rack, mRatio in rad per metre.
class JPH_EXPORT RackMeshConstraintSettings final : public TwoBodyConstraintSettings
{
	JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, RackMeshConstraintSettings)

public:
	// See: ConstraintSettings::SaveBinaryState
	virtual void				SaveBinaryState(StreamOut &inStream) const override;

	/// Create an instance of this constraint.
	virtual TwoBodyConstraint *	Create(Body &inBody1, Body &inBody2) const override;

	/// This determines in which space the constraint is setup, specified properties below should be in the specified space
	EConstraintSpace			mSpace = EConstraintSpace::WorldSpace;

	/// Rotation axis of the pinion (body 1)
	Vec3						mHingeAxis = Vec3::sAxisX();

	/// Sliding axis of the rack (body 2)
	Vec3						mSliderAxis = Vec3::sAxisX();

	/// Ratio between the pinion's rotation and the rack's translation (rad per metre: 2 pi / (teeth * tooth pitch))
	float						mRatio = 1.0f;

protected:
	// See: ConstraintSettings::RestoreBinaryState
	virtual void				RestoreBinaryState(StreamIn &inStream) override;
};

/// A rack and pinion relative to their frames, see RackMeshConstraintSettings.
class JPH_EXPORT RackMeshConstraint final : public TwoBodyConstraint
{
public:
	JPH_OVERRIDE_NEW_DELETE

	/// Construct rack mesh constraint
								RackMeshConstraint(Body &inBody1, Body &inBody2, const RackMeshConstraintSettings &inSettings);

	// Generic interface of a constraint
	virtual EConstraintSubType	GetSubType() const override								{ return EConstraintSubType::RackMesh; }
	virtual void				NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM) override { /* Nothing */ }
	virtual void				SetupVelocityConstraint(float inDeltaTime) override;
	virtual void				ResetWarmStart() override;
	virtual void				WarmStartVelocityConstraint(float inWarmStartImpulseRatio) override;
	virtual bool				SolveVelocityConstraint(float inDeltaTime) override;
	virtual bool				SolvePositionConstraint(float inDeltaTime, float inBaumgarte) override;
#ifdef JPH_DEBUG_RENDERER
	virtual void				DrawConstraint(DebugRenderer *inRenderer) const override;
#endif // JPH_DEBUG_RENDERER
	virtual void				SaveState(StateRecorder &inStream) const override;
	virtual void				RestoreState(StateRecorder &inStream) override;
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override;

	// See: TwoBodyConstraint
	virtual Mat44				GetConstraintToBody1Matrix() const override;
	virtual Mat44				GetConstraintToBody2Matrix() const override;

	/// The hinge holding the pinion and the slider holding the rack (the pinion / rack may be either body of its joint;
	/// the joint's other body is the frame). Optional: when set, the frames' motion is taken out of the relation and the
	/// position solve removes the drift since this call (call it again to re-base).
	void						SetConstraints(const Constraint *inPinion, const Constraint *inRack);

	/// Ratio (rad per metre)
	float						GetRatio() const										{ return mRatio; }

	/// Drift since SetConstraints (rad, pinion rotation minus ratio times rack translation, both relative to their frames)
	float						GetPositionError() const;

	///@name Get Lagrange multiplier from last physics update (N m s, the impulse about the pinion axis)
	inline float				GetTotalLambda() const									{ return mTotalLambda; }

	// See: Constraint (the frames take part in the solve)
	virtual uint				BuildIslandSplits(LargeIslandSplitter &ioSplitter) const override;

private:
	void						CalculateConstraintProperties();
	bool						ApplyVelocityStep(float inLambda);
	bool						ApplyPositionStep(float inLambda);
	float						GetPinionRotation() const;
	float						GetRackTranslation() const;

	// CONFIGURATION PROPERTIES FOLLOW

	Vec3						mLocalSpaceHingeAxis;
	Vec3						mLocalSpaceSliderAxis;
	float						mRatio;

	RefConst<Constraint>		mPinionConstraint;
	RefConst<Constraint>		mRackConstraint;
	Body *						mPinionFrame = nullptr;			///< Other body of the pinion's hinge (nullptr: at rest)
	Body *						mRackFrame = nullptr;			///< Other body of the rack's slider (nullptr: at rest)
	float						mHingeSign = 0.0f;				///< Maps the hinge angle to the pinion's rotation about a
	float						mSliderSign = 0.0f;				///< Maps the slider position to the rack's travel along b
	float						mPrevAngle = 0.0f;
	float						mAccumAngle = 0.0f;
	float						mBasePosition = 0.0f;

	// RUN TIME PROPERTIES FOLLOW

	Vec3						mWorldSpaceHingeAxis = Vec3::sAxisX();
	Vec3						mWorldSpaceSliderAxis = Vec3::sAxisX();

	struct Row
	{
		Body *					mBody = nullptr;
		Vec3					mLinear = Vec3::sZero();
		Vec3					mAngular = Vec3::sZero();
		Vec3					mInvIAngular = Vec3::sZero();
		float					mInvMass = 0.0f;
	};
	Row							mRows[4];
	int							mNumRows = 0;
	float						mEffectiveMass = 0.0f;
	float						mTotalLambda = 0.0f;
};

JPH_NAMESPACE_END
