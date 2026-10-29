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

Velocity-residual observers use the pre-update hinge axis and rotated local
anchor arms with the solved post-update velocities. Native velocity setup caches
these Jacobians before pose integration; position correction follows integration.
Post-pose geometric velocity residuals are reported separately and are not the
velocity solver residual. Momentum, energy and position observations retain their
actual post-update states. The original observer failure output remains evidence;
this correction changes no solver, fixture command, iteration or threshold.

## Coupled motor/rotor candidate

`--motor-block-only` adds seven bounded 100-step controls: driven/reversing then
zero-duty braking in both signs, an active motor impulse clamp, and both body orders for static and moving
kinematic anchors. Kinematic anchors have prescribed 0.2 rad/s axial motion. Parameters
come from the failed free-motor consumer's bare/reflected inertia ratio and DC
slope: `dt=0.005`, 30 velocity / 5 position steps. Existing probe iterations and
assertions are unchanged. A cached Jacobian axis is paired with after-solve
velocities; post-integration poses do not define the velocity residual frame.
The six unlimited cases use 30 positive-command steps, 30 negative-command
steps, then 40 zero-duty steps (the reversed case reverses command signs).
The limited case uses 30 positive, 20 negative, then 50 zero-duty steps. Its
asymmetric schedule preserves motion into braking: equal opposite saturated
impulses would cancel velocity before the brake phase and make the unchanged
moving-brake nonvacuity assertion impossible. An independent synthetic reference
identified that fixture flaw before the first native execution; the original
symmetric source checkpoint is retained. All seven cases still require actual
driven motion and actual moving-brake observations.

Motor KKT and rotor residual bounds are `1e-4` rad/s. Zero-duty passivity is
checked only when actual cached motor bias is zero, using `lambda_motor *
omega_end`; its declared float budget is 32 machine epsilons times absolute
motor impulse times `max(1, abs(omega_before), abs(omega_end))`. This is an
impulse-work numerical budget, not a fitted endpoint allowance or a full-scene
energy assertion. No-contact rows do not qualify contacts or all models.

The workflow compiles the same probe against the candidate and afdb with the
read-only `afdb-observer-only.patch`. The preparer verifies the exact afdb base,
all present tracked bytes/modes, unchanged full-index blob/modes, explicitly
unmaterialized sparse entries, and the three changed header hashes; no solve,
state or equation is patched in that baseline. Baseline failure and candidate
success are both required, alongside every previous native gate. The existing
bw.7 zero-inertia baseline is retained separately. All first outputs remain in
the artifact, including failures. A successful expanded artifact has the prior
11 files plus `afdb-observer-source.json`, `motor-block-candidate.log`,
`motor-block-afdb.log`, and `motor-block-status.json` (15 files total).

Source preparation is not runtime qualification. The original failed consumer
endpoint programmes and later built/packed bindings remain separate gates.

### Preserved first block-qualification failure

The first block candidate at `31c24b01dc6136a49db730e9d45627a0de482a32`
failed overall. Body creation replaced the new fixture's declared small first
inertia with its near-zero fallback (actual inverse inertia 25 instead of the
declared reciprocal). The static-body2 moving-brake nonvacuity assertion
correctly failed; no bound was changed. The intended inertia ratio was not
qualified. Independently, source admission refused the observational baseline
because its frozen `Build/CMakeLists.txt` input was not materialized; that
reader refusal precedes physical acceptance and is retained separately.

Only the new block fixtures now install the original declared inverse inertia
on their dynamic bodies before constructing the hinge, then assert exact
native inverse-inertia readback and unchanged inverse mass. This follows the
consumer's explicit small-inertia admission rather than changing Jolt's body
creation policy. The shared `BodyAt` helper, previous cases, unit tests, zero
trajectory, solver equations, parameters, iteration counts and all guards are
unchanged. The observational baseline checkout additionally materializes
`Build`; its frozen reader and source requirements are unchanged. This is an
unqualified source revision, not a retry or reinterpretation of the first run.
