// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2023 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include "UnitTestFramework.h"
#include "PhysicsTestContext.h"
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include "Layers.h"
#include <Jolt/Physics/StateRecorderImpl.h>
#include <limits>

TEST_SUITE("HingeConstraintTests")
{
	TEST_CASE("TestReflectedRotorConfigurationAndState")
	{
		PhysicsTestContext context;
		context.ZeroGravity();
		Body &body = context.CreateSphere(RVec3::sZero(), 0.1f, EMotionType::Dynamic, EMotionQuality::Discrete, Layers::MOVING);
		context.GetBodyInterface().SetAngularVelocity(body.GetID(), 3.0f * Vec3::sAxisY());
		HingeConstraintSettings settings;
		settings.mReflectedRotorInertia = 0.25f;
		HingeConstraint &hinge = context.CreateConstraint<HingeConstraint>(Body::sFixedToWorld, body, settings);
		CHECK(hinge.GetReflectedRotorInertia() == 0.25f);
		CHECK(hinge.GetRotorAngularVelocity() == 3.0f);
		CHECK(hinge.SetReflectedRotorInertia(0.25f));
		CHECK(hinge.GetRotorAngularVelocity() == 3.0f);
		CHECK_FALSE(hinge.SetReflectedRotorInertia(-1.0f));
		CHECK_FALSE(hinge.SetReflectedRotorInertia(std::numeric_limits<float>::infinity()));
		CHECK_FALSE(hinge.SetReflectedRotorInertia(std::numeric_limits<float>::quiet_NaN()));
		CHECK(hinge.GetReflectedRotorInertia() == 0.25f);
		StateRecorderImpl state;
		hinge.SaveState(state);
		CHECK(hinge.SetReflectedRotorInertia(0.0f));
		CHECK(hinge.GetRotorAngularVelocity() == 0.0f);
		state.Rewind();
		hinge.RestoreState(state);
		CHECK(hinge.GetReflectedRotorInertia() == 0.25f);
		CHECK(hinge.GetRotorAngularVelocity() == 3.0f);
		settings.mReflectedRotorInertia = -1.0f;
		CHECK(settings.Create(Body::sFixedToWorld, body) == nullptr);
	}

	TEST_CASE("TestReflectedRotorSettingsBinaryRoundTrip")
	{
		HingeConstraintSettings settings;
		CHECK(settings.mReflectedRotorInertia == 0.0f);
		settings.mReflectedRotorInertia = 0.125f;
		StateRecorderImpl stream;
		settings.SaveBinaryState(stream);
		stream.Rewind();
		auto restored = ConstraintSettings::sRestoreFromBinaryState(stream);
		REQUIRE_FALSE(restored.HasError());
		CHECK(static_cast<HingeConstraintSettings *>(restored.Get().GetPtr())->mReflectedRotorInertia == 0.125f);
	}

	TEST_CASE("TestCoupledRotorSoftMotorSingleIteration")
	{
		PhysicsTestContext context;
		context.ZeroGravity();
		Body &body = context.CreateSphere(RVec3::sZero(), 0.005f, EMotionType::Dynamic, EMotionQuality::Discrete, Layers::MOVING);
		context.GetBodyInterface().SetAngularVelocity(body.GetID(), Vec3::sAxisY());
		HingeConstraintSettings settings;
		settings.mReflectedRotorInertia = 0.0004529942571492882f;
		HingeConstraint &hinge = context.CreateConstraint<HingeConstraint>(Body::sFixedToWorld, body, settings);
		constexpr float dt = 0.005f, c = 0.007881959086455769f;
		auto &motor = hinge.GetMotorSettings();
		motor.mSpringSettings.mMode = ESpringMode::StiffnessAndDamping;
		motor.mSpringSettings.mStiffness = c / (2 * dt);
		motor.mSpringSettings.mDamping = c / 2;
		hinge.SetMotorState(EMotorState::Position);
		hinge.SetTargetAngle(hinge.GetCurrentAngle());
		hinge.SetupVelocityConstraint(dt);
		hinge.WarmStartVelocityConstraint(0.0f);
		hinge.SolveVelocityConstraint(dt);
		float velocity = body.GetAngularVelocity().GetY();
		CHECK(std::abs(velocity + hinge.GetTotalLambdaMotor() / (dt * c)) <= 1.0e-4f);
		CHECK(std::abs(velocity - hinge.GetRotorAngularVelocity()) <= 1.0e-4f);
		CHECK(hinge.GetTotalLambdaMotor() * velocity <= 0.0f);
	}

	// Test if the hinge constraint can be used to create a spring
	TEST_CASE("TestHingeSpring")
	{
		// Configuration of the spring
		const float cInitialAngle = DegreesToRadians(100.0f);
		const float cFrequency = 2.0f;
		const float cDamping = 0.1f;

		for (int mode = 0; mode < 2; ++mode)
		{
			// Create a sphere
			PhysicsTestContext context;
			Body &body = context.CreateBody(new SphereShapeSettings(0.5f), RVec3::sZero(), Quat::sIdentity(), EMotionType::Dynamic, EMotionQuality::Discrete, Layers::MOVING, EActivation::Activate);
			body.GetMotionProperties()->SetAngularDamping(0.0f);
			body.SetAllowSleeping(false);

			// Calculate stiffness and damping of spring
			float inertia = body.GetMotionProperties()->GetInverseInertiaForRotation(Mat44::sIdentity()).Inversed3x3().GetAxisY().Length();
			float omega = 2.0f * JPH_PI * cFrequency;
			float k = inertia * Square(omega);
			float c = 2.0f * inertia * cDamping * omega;

			// Create spring
			HingeConstraintSettings constraint;
			if (mode == 0)
			{
				// First iteration use stiffness and damping
				constraint.mLimitsSpringSettings.mMode = ESpringMode::StiffnessAndDamping;
				constraint.mLimitsSpringSettings.mStiffness = k;
				constraint.mLimitsSpringSettings.mDamping = c;
			}
			else
			{
				// Second iteration use frequency and damping
				constraint.mLimitsSpringSettings.mMode = ESpringMode::FrequencyAndDamping;
				constraint.mLimitsSpringSettings.mFrequency = cFrequency;
				constraint.mLimitsSpringSettings.mDamping = cDamping;
			}
			constraint.mLimitsMin = constraint.mLimitsMax = 0.0f;
			context.CreateConstraint<HingeConstraint>(Body::sFixedToWorld, body, constraint);

			// Rotate the body to the initial angle
			context.GetBodyInterface().SetRotation(body.GetID(), Quat::sRotation(Vec3::sAxisY(), cInitialAngle), EActivation::Activate);

			// Simulate angular spring
			float angle = cInitialAngle;
			float angular_v = 0.0f;
			float dt = context.GetDeltaTime();
			for (int i = 0; i < 120; ++i)
			{
				// Using the equations from page 32 of Soft Constraints: Reinventing The Spring - Erin Catto - GDC 2011 for an implicit euler spring damper
				angular_v = (angular_v - dt * k / inertia * angle) / (1.0f + dt * c / inertia + Square(dt) * k / inertia);
				angle += angular_v * dt;

				// Run physics simulation
				context.SimulateSingleStep();

				// Decompose body rotation
				Vec3 actual_axis;
				float actual_angle;
				body.GetRotation().GetAxisAngle(actual_axis, actual_angle);
				if (actual_axis.GetY() < 0.0f)
					actual_angle = -actual_angle;

				// Test if simulation matches prediction
				CHECK_APPROX_EQUAL(angle, actual_angle, DegreesToRadians(0.1f));
				CHECK_APPROX_EQUAL(actual_axis.GetX(), 0);
				CHECK_APPROX_EQUAL(actual_axis.GetZ(), 0);
			}
		}
	}
}
