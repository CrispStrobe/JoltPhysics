# Coupled hinge motor and reflected rotor scope

Status: BOUNDED NATIVE DONE; WebAssembly package and consumer qualification pending.
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
inertia is small relative to reflected rotor inertia. Paired native controls
now establish the finite-iteration error and its correction in the observed
high-ratio motor/brake scenes. They do not by themselves establish a repair of
the adopting application, its programmes or every connected mechanism.

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

## Bounded native implementation

The positive-inertia active-motor path solves the motor and rotor as a two-row
block, retaining the existing motor impulse bounds. A stable Schur complement
avoids subtracting nearly equal bare mobilities. Both solve and warmstart apply
the summed body impulse once; the motor and rotor keep their distinct native
accumulated impulses and scalar coordinate. No new checkpoint field is added.
The zero-inertia path retains the existing standalone motor solve.

Read-only row properties expose the last setup cache for bounded qualification;
they do not refresh a pose or change a constraint. The paired previous-engine
control is explicitly `afdb16ddd4f4d5f2ad16cc8814abe0c80114464e` plus a retained
three-header, read-only observer patch. It is not labeled pristine afdb bytes.
The patch receipt hashes both header versions and verifies every other present tracked
file and mode and the full Git index are unchanged; omitted sparse entries
are explicitly recorded by index blob/mode, without claiming their bytes were read. The older zero-inertia engine baseline is separate.

The new control records known commands, actual cached softness/bias/axis/bare
mobility, body velocities and both impulses. It judges motor clamp KKT signs,
rotor equality and zero-bias brake impulse work at unchanged iteration counts.
All existing probe checks and zero-trajectory comparison remained gates.
Application endpoint acceptance remains separate from these native results.

## Accepted native results and preserved failures

[PR 6](https://github.com/CrispStrobe/JoltPhysics/pull/6) normally merged as
[`0ff27c94898c910cb024b4e74218818358ee4852`](https://github.com/CrispStrobe/JoltPhysics/commit/0ff27c94898c910cb024b4e74218818358ee4852)
after all three enabled checks succeeded. The retained
[hosted native run](https://github.com/CrispStrobe/JoltPhysics/actions/runs/40812084515)
passed all 819 existing native probe checks and five hinge unit cases with
743 assertions. The new candidate passed 4,115 checks with zero failures;
the previous-engine observational baseline exposed 619 violations. A standalone
reader independently accepted the actual inputs and all seven 100-step cases,
including their motor KKT signs, rotor rows and zero-bias brake work. The
zero-inertia comparison retained exactly 200 byte-identical trajectory rows.

The first source `31c24b0` and its overall failure are preserved: its new small-inertia fixture
admitted a native fallback tensor, and its observational baseline lacked a
required materialized build input. Those setup errors were corrected only in
the new fixtures and checkout inputs; the solver and every physics assertion,
parameter, timestep and iteration count remained unchanged.

The initial `7fbfacb` tensor-observation refusal is also preserved. The
original validator compared a binary32 matrix observation with an ideal
isotropic matrix; at the admitted inverse-inertia scale, the actual pinned
SSE2 multiply/add sequence retains cancellation terms. A new independent
validator was derived from that source operation order, frozen, and then checked
against the retained observations. It matches exact binary32 observations, including dtype
and matrix layout, without an additional tolerance. This is a stronger
observation check, not a numerical physics budget change. The original physics
judges remained unchanged, and retained producer bytes were read without an
engine replay.

This completes only the bounded native scope. Built and packed scalar/SIMD
bw.9 qualification, affected Lite programmes/model guards, rendered worker and
save-reopen checks remain pending. No hardware, arbitrary sleeping lifetime,
all-lifetime energy conservation, full-model accuracy, package publication or
application deployment is qualified by this status. The overall goal stays open.
