# Distance-constraint impulse observation

`DistanceConstraint::GetLastVelocityImpulse()` returns the signed impulse from
the last active velocity solve, retained across position correction. Divide by
that collision step's duration to obtain its average constraint force. It is
an observation of the native solver, never a force inferred from motion.

`GetTotalLambdaPosition()` and all solver equations remain unchanged. A
unilateral limit becoming inactive during position correction clears the axis
constraint's live lambda; that getter can therefore omit an impulse already
applied to momentum. The new getter snapshots the accumulated velocity-solver
lambda after warm starting and each velocity iteration. Position correction
does not overwrite it. No snapshot participates in warm starting or solving.

Velocity setup, explicit warm-start reset and state restoration clear the
observation. It is deliberately excluded from serialized solver state, so
restore requires another native step before a fresh observation. A sleeping
constraint retains its last solve; consumers must not label that as a new
impulse. With multiple collision substeps the getter describes the latest
substep, not their sum. Brickwright's rope observations use one 5 ms substep.

Build the standalone native regression with CMake from this directory and run
`rope-impulse-probe`. It tests both unilateral limit directions, unchanged
legacy behavior and momentum, fresh setup/reset/restore, and the complete
100-step settled impulse balance of a four-rope suspension. Every test uses
explicit runtime checks in Release builds. `--legacy` deliberately exercises
the clearing readout and must fail. Optional second argument writes every
actual body position, orientation and velocity in hexadecimal float format.

Defining `BW_DISTANCE_IMPULSE_BASELINE` allows the same test to compile against
the previous producer. Its force checks must fail, while its recorded physical
trajectory must equal the repaired producer byte for byte. No material,
contact, timestep, gravity or constraint parameter changes are involved.
