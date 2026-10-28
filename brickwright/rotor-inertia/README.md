# Native reflected axial rotor controls

The optional hinge element stores scalar relative speed with inertia Jr. It
solves `a dot (omega2 - omega1) - rotorSpeed = 0` inside normal velocity
iterations. The same world axis as the hinge motor gives opposite body impulses;
the scalar receives `-lambda/Jr`. No rigid body, inflated inertia or post-step
kick is added. This is the ideal reflected axial model, not a hardware claim.

`SetReflectedRotorInertia` returns false for negative/non-finite input and
otherwise changes configuration outside `Update`. Equal values are exact
no-ops. Enabling initializes scalar speed to the current relative rate; positive
changes preserve speed, potentially changing configured energy. Disabling
clears scalar speed and accumulated impulse. Callers explicitly activate dynamic
bodies after configuration, as with existing motor setters. `Create` rejects
invalid settings with null. Direct constructor calls require valid settings.

Native body deactivation zeros body velocities without notifying constraints.
The internal scalar therefore retains its speed while asleep; waking can
redistribute that stored energy. No all-lifetime conservation claim spans that
body-energy discard. A caller choosing to discard rotor energy too must
explicitly set Jr to zero and back to its configured value while bodies are
stopped, then activate them. This initializes the scalar from their current
relative speed and clears the old warm start. Positive-to-positive changes
preserve scalar speed even while asleep. The probe records real deactivation,
configuration and explicit resynchronization/activation, then an idle step
which must have no stale impulse kick. It does not force sleeping off globally
or invent a deactivation callback. Natural auto-sleep/wake with retained
nonzero scalar energy is an explicit behavioral limit, not hidden acceptance.

The constraint owns state and lifetime. Recorder snapshots retain Jr, scalar
speed and warm-start impulse. Binary settings retain Jr. No cross-version
binary/recorder compatibility is promised. `GetTotalLambdaMotor` is unchanged;
the rotor's separate impulse is `GetTotalLambdaRotor` in N m s.

The new hosted workflow builds this probe and all existing hinge unit cases.
It retains candidate, disabled-row negative control and paired old/new zero
trajectories even on failure. Baseline native source is the exact released
bw.7 tree. Existing rope, gear, determinism and contact qualification stay
required. No solver iteration, material or timestep policy changes here.

The probe uses collision-disabled benches, at most 200 updates per named case.
It covers reduced inertia and bare momentum, variable dt, clamped and moving
kinematic orientations, anisotropic offset anchors and an ordinary transverse
neighbor, shared rotors, repeated/removal/invalid configuration, activation and
exact restore. Metrics retain unchanged `1e-4` neighbor angular and point-speed
limits. The new unit cases also cover binary settings and native configuration.
The analytic scalar is checked against independently known inertia/torque;
disabling the new row must fail those expectations.

Evidence lines have prefixes `ROTOR_META`, `ROTOR_CASE`, `ROTOR_CONFIG`,
`ROTOR_HINGE`, `ROTOR_EVENT` and `ROTOR_STEP`, followed by JSON. Step records
contain dt, external torque, before/after body mass/inverse-world-inertia/COM
poses/velocities/activation and hinge local anchors, common axis, Jr, scalar,
motor/rotor impulses, commands and limits. Matrix arrays store columns.
The separate source-runtime manifest binds actual checked-out commit/tree and
source hashes; event metadata is not silently relabeled as the branch head.

For constant positive Jr in an active step, scalar change is the negative final
accumulated rotor impulse divided by Jr, including warm starting and all solver
increments (within float roundoff). Do not apply that identity across config or
restore events, or assume a sleeping constraint's retained impulse was applied
again. These records support standalone readers; test PASS alone is not an
independent geometric or raw-state audit. Browser/model/release qualification
and original Sarrus/collision/hub endpoint assertions remain separate gates.
