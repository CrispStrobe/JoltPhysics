// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-FileCopyrightText: 2026 Brickwright (fork addition, see BRICKWRIGHT.md)
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>

JPH_NAMESPACE_BEGIN

/// Gear mesh constraint settings (Brickwright fork addition, see BRICKWRIGHT.md)
///
/// A gear mesh modelled as what it physically is: a 1-D contact between the two gears at the pitch point P,
/// along the tooth's line of action n: n . (V2(P) - V1(P)) = 0 where Vi(P) is the velocity of gear i's material
/// point at P. Gear i turns relative to its frame Fi (the other body of its hinge) about ai, so
/// Vi(P) = vFi + WFi x (P - xFi) + ((wi - WFi) . ai) ei with ei = ai x (P - ci). The Jacobian is therefore
/// angular-only on the gears (ki ai, ki = n . ei: as well conditioned as GearConstraint) and linear + angular on the
/// frames. The impulse is an equal and opposite pair at P (the gear/frame terms of each side add up to the force n at
/// P), so linear and angular momentum are conserved, the reaction reaches the frame(s), and torque is multiplied by
/// the ratio. Both gears may hang on the same frame (gearbox) or on different ones (sun on the frame, planet on a
/// carrier). The frames are taken from the hinges given to SetConstraints; without them the frames are assumed at
/// rest (then it behaves like GearConstraint with the ratio applied correctly).
///
/// Ratio convention: mRatio = w2 / w1 where wi is the rotation rate of gear i RELATIVE TO THE FRAME holding both,
/// about mHingeAxisi. E.g. an external spur mesh 8T (gear 1) : 24T (gear 2) with both axes pointing the same way
/// has mRatio = -8/24; an internal mesh (gear 2 a ring gear) has a positive ratio.
///
/// Parallel axes (spur, internal): the pitch point is derived from the ratio and the axes every step
/// (it divides the centre distance D at r1 = D |ratio| / (1 + |ratio|) for external meshes), mContactPoint is ignored.
/// Non-parallel axes (worm, crossed helical, bevel): mContactPoint gives the pitch point; it is stored relative to
/// both axes (so it follows the frame). The line of action n is chosen in the plane spanned by the two pitch
/// velocities so that the ratio holds; for a worm that is the thread normal (lead angle). Friction (worm self-locking)
/// is NOT modelled.
class JPH_EXPORT GearMeshConstraintSettings final : public TwoBodyConstraintSettings
{
	JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, GearMeshConstraintSettings)

public:
	// See: ConstraintSettings::SaveBinaryState
	virtual void				SaveBinaryState(StreamOut &inStream) const override;

	/// Create an instance of this constraint.
	virtual TwoBodyConstraint *	Create(Body &inBody1, Body &inBody2) const override;

	/// This determines in which space the constraint is setup, all properties below should be in the specified space
	/// (for LocalToBodyCOM: mPoint1, mHingeAxis1 and mContactPoint local to body 1, mPoint2 and mHingeAxis2 local to body 2)
	EConstraintSpace			mSpace = EConstraintSpace::WorldSpace;

	/// A point on the rotation axis of gear 1 (e.g. its centre)
	RVec3						mPoint1 = RVec3::sZero();

	/// Rotation axis of gear 1
	Vec3						mHingeAxis1 = Vec3::sAxisX();

	/// A point on the rotation axis of gear 2 (e.g. its centre)
	RVec3						mPoint2 = RVec3::sZero();

	/// Rotation axis of gear 2
	Vec3						mHingeAxis2 = Vec3::sAxisX();

	/// Pitch point, only used when the axes are not parallel (worm, bevel, crossed axes)
	RVec3						mContactPoint = RVec3::sZero();

	/// Ratio w2 / w1 (rates relative to the frame, about mHingeAxis2 / mHingeAxis1)
	float						mRatio = -1.0f;

protected:
	// See: ConstraintSettings::RestoreBinaryState
	virtual void				RestoreBinaryState(StreamIn &inStream) override;
};

/// A gear mesh as a tooth contact at the pitch point (linear + angular Jacobian), see GearMeshConstraintSettings.
/// Use it together with two hinge constraints that hold the gears on a common frame body (or the world).
class JPH_EXPORT GearMeshConstraint final : public TwoBodyConstraint
{
public:
	JPH_OVERRIDE_NEW_DELETE

	/// Construct gear mesh constraint
								GearMeshConstraint(Body &inBody1, Body &inBody2, const GearMeshConstraintSettings &inSettings);

	// Generic interface of a constraint
	virtual EConstraintSubType	GetSubType() const override								{ return EConstraintSubType::GearMesh; }
	virtual void				NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM) override;
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

	/// The hinge constraints that hold both gears (gear 1 = body 1 of this constraint). Optional: when set, the
	/// accumulated hinge angles give the mesh's phase error and the position solve removes drift. The gear may be
	/// either body of its hinge; the hinges' other body is the frame.
	void						SetConstraints(const Constraint *inGear1, const Constraint *inGear2);

	/// Ratio w2 / w1
	float						GetRatio() const										{ return mRatio; }

	/// World space pitch point and line of action of the last step
	RVec3						GetContactPoint() const									{ return mWorldSpaceContactPoint; }
	Vec3						GetContactNormal() const								{ return mWorldSpaceNormal; }

	/// Phase error (m, along the line of action) from the hinge angles, 0 when no hinges are set
	float						GetPositionError() const;

	///@name Get Lagrange multiplier from last physics update (the linear impulse along the line of action, N s)
	inline float				GetTotalLambda() const									{ return mTotalLambda; }

	// See: Constraint (the frames take part in the solve)
	virtual uint				BuildIslandSplits(LargeIslandSplitter &ioSplitter) const override;

private:
	// Jacobian rows (gears and frames) and effective mass for the current geometry
	void						CalculateConstraintProperties();

	// Apply impulse lambda as a velocity change (or, for the position step, as a position change)
	bool						ApplyVelocityStep(float inLambda);
	bool						ApplyPositionStep(float inLambda);

	// Recompute pitch point, normal and pitch velocity directions from the bodies' current poses
	void						CalculateGeometry();

	// Signed rotation of gear i relative to its frame about its gear axis since SetConstraints (unwrapped)
	float						GetRelativeRotation(int inGear) const;

	// CONFIGURATION PROPERTIES FOLLOW

	Vec3						mLocalSpacePoint1;				///< Point on gear 1 axis, local to body 1 COM
	Vec3						mLocalSpaceHingeAxis1;			///< Gear 1 axis, local to body 1
	Vec3						mLocalSpacePoint2;				///< Point on gear 2 axis, local to body 2 COM
	Vec3						mLocalSpaceHingeAxis2;			///< Gear 2 axis, local to body 2
	Vec3						mMeshFrameContactPoint;			///< Pitch point in the mesh frame (a1, u, a1 x u at c1), non-parallel axes only
	float						mRatio;
	bool						mParallel;

	RefConst<Constraint>		mGear1Constraint;
	RefConst<Constraint>		mGear2Constraint;
	Body *						mFrame1 = nullptr;				///< Other body of gear 1's hinge (nullptr: frame at rest)
	Body *						mFrame2 = nullptr;				///< Other body of gear 2's hinge
	float						mHingeSign[2] = { 0.0f, 0.0f };	///< Maps hinge angle to rotation of the gear about its gear axis
	float						mPrevAngle[2] = { 0.0f, 0.0f };	///< Hinge angle at the last step
	float						mAccumAngle[2] = { 0.0f, 0.0f };	///< Unwrapped hinge angle since SetConstraints, up to mPrevAngle

	// RUN TIME PROPERTIES FOLLOW

	RVec3						mWorldSpaceContactPoint = RVec3::sZero();
	Vec3						mWorldSpaceNormal = Vec3::sAxisY();
	Vec3						mPitchVelocity1 = Vec3::sZero();	///< a1 x (P - c1)
	Vec3						mPitchVelocity2 = Vec3::sZero();	///< a2 x (P - c2)

	/// One body's part of the Jacobian
	struct Row
	{
		Body *					mBody = nullptr;
		Vec3					mLinear = Vec3::sZero();
		Vec3					mAngular = Vec3::sZero();
		Vec3					mInvIAngular = Vec3::sZero();	///< I^-1 * mAngular (dynamic bodies)
		float					mInvMass = 0.0f;				///< 0 for non-dynamic bodies
	};
	Row							mRows[4];
	int							mNumRows = 0;
	float						mEffectiveMass = 0.0f;
	float						mTotalLambda = 0.0f;
};

JPH_NAMESPACE_END
