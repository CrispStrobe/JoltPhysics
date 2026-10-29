// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2026 Brickwright contributors
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Constraints/ConstraintPart/AngleConstraintPart.h>
#include <Jolt/Physics/StateRecorder.h>
#include <cmath>

JPH_NAMESPACE_BEGIN

/// Ideal reflected axial inertia, without a hidden rigid body or modified body inertia.
/// Cdot = a . (w2 - w1) - wr; Jacobian [-a, +a, -1]. The scalar wr has inertia Jr.
/// It is solved alongside the hinge motor, point and transverse angular rows.
class RotationalInertiaConstraintPart
{
public:
	bool SetInertia(float inInertia, float inRelativeSpeed)
	{
		if (!std::isfinite(inInertia) || inInertia < 0.0f)
			return false;
		if (inInertia == mInertia)
			return true;
		// Enabling initializes the new coordinate at the current speed, without an
		// impulse. Changing Jr preserves speed (configuration may change energy).
		if (mInertia == 0.0f || inInertia == 0.0f)
			mRotorSpeed = inInertia > 0.0f? inRelativeSpeed : 0.0f;
		mInertia = inInertia;
		ResetWarmStart();
		return true;
	}
	float GetInertia() const { return mInertia; }
	float GetRotorSpeed() const { return mRotorSpeed; }
	float GetTotalLambda() const { return mTotalLambda; }
	void ResetWarmStart() { mTotalLambda = 0.0f; mEffectiveMass = 0.0; }

	void CalculateConstraintProperties(const Body &inBody1, const Body &inBody2, Vec3Arg inAxis)
	{
		mInvI1Axis = inBody1.IsDynamic()? inBody1.GetMotionProperties()->MultiplyWorldSpaceInverseInertiaByVector(inBody1.GetRotation(), inAxis) : Vec3::sZero();
		mInvI2Axis = inBody2.IsDynamic()? inBody2.GetMotionProperties()->MultiplyWorldSpaceInverseInertiaByVector(inBody2.GetRotation(), inAxis) : Vec3::sZero();
		if (mInertia == 0.0f || (!inBody1.IsDynamic() && !inBody2.IsDynamic()))
		{
			ResetWarmStart();
			return;
		}
		// Double intermediates avoid overflowing 1/Jr for a finite tiny Jr.
		mEffectiveMass = 1.0 / (double(inAxis.Dot(mInvI1Axis + mInvI2Axis)) + 1.0 / double(mInertia));
	}
	void WarmStart(Body &ioBody1, Body &ioBody2, float inRatio)
	{
		if (mEffectiveMass == 0.0)
			return;
		mTotalLambda *= inRatio;
		Apply(ioBody1, ioBody2, mTotalLambda);
	}
	bool SolveVelocityConstraint(Body &ioBody1, Body &ioBody2, Vec3Arg inAxis)
	{
		if (mEffectiveMass == 0.0)
			return false;
		float error = inAxis.Dot(ioBody2.GetAngularVelocity() - ioBody1.GetAngularVelocity()) - mRotorSpeed;
		float lambda = float(-mEffectiveMass * double(error));
		mTotalLambda += lambda;
		return Apply(ioBody1, ioBody2, lambda);
	}
	/// Preserve each scaled warmstart impulse, but avoid two cancelling body kicks.
	void WarmStartMotor(Body &ioBody1, Body &ioBody2, AngleConstraintPart &ioMotor, float inRatio)
	{
		if (mEffectiveMass == 0.0)
		{
			ioMotor.WarmStart(ioBody1, ioBody2, inRatio);
			return;
		}
		float motor = ioMotor.GetTotalLambda() * inRatio;
		mTotalLambda *= inRatio;
		mRotorSpeed -= float(double(mTotalLambda) / double(mInertia));
		ioMotor.ApplyCoupledVelocityStep(ioBody1, ioBody2, motor, float(double(motor) + double(mTotalLambda)));
	}

	/// Solve the common body Jacobian as a two-row block. Other hinge rows still
	/// participate in the ordinary island iterations; no body inertia is changed.
	bool SolveMotorVelocityConstraint(Body &ioBody1, Body &ioBody2, Vec3Arg inAxis,
		AngleConstraintPart &ioMotor, float inMinLambda, float inMaxLambda)
	{
		if (mEffectiveMass == 0.0)
			return ioMotor.SolveVelocityConstraint(ioBody1, ioBody2, inAxis, inMinLambda, inMaxLambda);
		double s = ioMotor.GetInverseEffectiveMass(inAxis);
		double r = 1.0 / double(mInertia);
		double sr = s + r;
		double gamma = ioMotor.GetSoftness();
		// Stable Schur complement: avoid subtracting two nearly equal bare mobilities.
		double denominator = gamma + s * (r / sr);
		if (!(denominator > 0.0))
			return false; // No dynamic body response and no soft motor row.
		double v = inAxis.Dot(ioBody2.GetAngularVelocity() - ioBody1.GetAngularVelocity());
		double rotor_error = v - double(mRotorSpeed);
		double motor_error = v + double(ioMotor.GetAccumulatedBias());
		double motor_delta = -(motor_error - (s / sr) * rotor_error) / denominator;
		float old_motor = ioMotor.GetTotalLambda();
		float new_motor = Clamp(float(double(old_motor) + motor_delta), inMinLambda, inMaxLambda);
		float actual_motor_delta = new_motor - old_motor;
		double rotor_delta = -(rotor_error + s * double(actual_motor_delta)) / sr;
		// Apply the physical double increment before rounding the diagnostic total.
		// Recovering it from a rounded accumulated lambda loses light-shaft response.
		mTotalLambda = float(double(mTotalLambda) + rotor_delta);
		mRotorSpeed = float(double(mRotorSpeed) - rotor_delta / double(mInertia));
		// Algebraically delta_motor + delta_rotor, without their cancellation.
		double body_delta = (r * double(actual_motor_delta) - rotor_error) / sr;
		bool changed = ioMotor.ApplyCoupledVelocityStep(ioBody1, ioBody2, new_motor, float(body_delta));
		return changed || rotor_delta != 0.0;
	}

	void SaveState(StateRecorder &inStream) const
	{
		inStream.Write(mInertia);
		inStream.Write(mRotorSpeed);
		inStream.Write(mTotalLambda);
	}
	void RestoreState(StateRecorder &inStream)
	{
		inStream.Read(mInertia);
		inStream.Read(mRotorSpeed);
		inStream.Read(mTotalLambda);
		mEffectiveMass = 0.0; // Derived again before warm starting the next step.
	}

private:
	bool Apply(Body &ioBody1, Body &ioBody2, float inLambda)
	{
		if (inLambda == 0.0f)
			return false;
		if (ioBody1.IsDynamic())
			ioBody1.GetMotionProperties()->SubAngularVelocityStep(inLambda * mInvI1Axis);
		if (ioBody2.IsDynamic())
			ioBody2.GetMotionProperties()->AddAngularVelocityStep(inLambda * mInvI2Axis);
		mRotorSpeed -= float(double(inLambda) / double(mInertia));
		return true;
	}
	float mInertia = 0.0f;
	float mRotorSpeed = 0.0f;
	float mTotalLambda = 0.0f;
	double mEffectiveMass = 0.0;
	Vec3 mInvI1Axis = Vec3::sZero();
	Vec3 mInvI2Axis = Vec3::sZero();
};

JPH_NAMESPACE_END
