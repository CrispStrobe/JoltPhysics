# Coupled hinge motor and reflected rotor scope

Status: CLAIM ONLY; implementation and qualification pending.
Owner: Codex, continuing the user-authorized Robot Lab physics work.
Base: `3663a39020866aae3b4a5311305fffdaa30ffb1e` on
`brickwright/gear-mesh-master` in the owned Jolt Physics fork.

The bounded reflected-rotor library controls passed, but the adopting
[Lite consumer](https://github.com/CrispStrobe/brickwright-lite/pull/1553)
fails unchanged positive full-turn endpoint tests. A failed-case diagnostic
observed initial angle zero, target 360 degrees and completion near 360.852
degrees, followed by actual body and rotor reversal during zero-duty braking
to about 353.906 degrees. Original programmes, endpoint tolerances, three-second
horizon and model inputs remain unchanged. Preserve the failure; library
controls do not qualify this consumer.

Sequential soft-motor and rotor equations are strongly coupled when bare body
inertia is small relative to reflected rotor inertia. A finite-iteration
residual is a supported hypothesis, not yet an experimentally established
cause. Qualify a simultaneous two-row solve against the previous native source
before claiming causation or a repair.

Own only `Jolt/Physics/Constraints/HingeConstraint.{h,cpp}`; narrowly needed
motor-row accessors in `Jolt/Physics/Constraints/ConstraintPart/AngleConstraintPart.h`
and `Jolt/Physics/Constraints/ConstraintPart/SpringPart.h`;
`Jolt/Physics/Constraints/ConstraintPart/RotationalInertiaConstraintPart.h`;
`UnitTests/Physics/HingeConstraintTests.cpp`; `brickwright/rotor-inertia/**`;
`.github/workflows/relative-rotor.yml`; this scope and affected `BRICKWRIGHT.md`
status prose. Preserve existing helper callers. No source implementation
precedes merging this claim.

Implement a coupled motor/rotor block with existing accumulated motor impulse
clamps and a consistent rotor impulse update. Preserve the exact zero-inertia
path, native warm starting, checkpoints, configuration, body mass, COM and bare
inertia. No added iterations, damping, timestep, material/contact/floor change,
fitted correction or target/encoder reset is included. Preserve all original
tests, thresholds, branch failures and evidence.

New controls retain paired old and candidate results at the same declared
inertia ratio, motor parameters, timestep and iteration count. Check zero-duty
brake passivity and both row residuals, driven/reversing and limited motors,
static/dynamic/kinematic cases, ordinary neighboring hinges, anchors, shared
rotors, exact restore and zero-inertia trajectory identity. A disabled block
or old-source negative control must expose this error class. Standalone readers
audit retained actual state and command records without replaying successful
originals. Existing rope/gear/contact checks remain gates.

Use hosted builds; no engine build is needed for this claim. Every enabled
exact-head check must succeed, with only declared skips, before a normal
matched-head merge and exact parent/tree proof. Separately qualify the built
and packed scalar/SIMD binding package before advancing Lite's exact package
pin through its owning lane. Original hub endpoints, promoted programmes and
rendered worker/save-reopen paths must pass on that combined consumer. This
scope makes no hardware accuracy, application release or deployment claim.
