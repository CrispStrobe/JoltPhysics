# Brickwright relative rotor inertia: native scope

Status: CLAIMED — native source candidate; hosted qualification pending.
Owner: Codex, authorized Robot Lab physics continuation.
Integration target: `brickwright/gear-mesh-master` at
`389854fb7912d814112ce46b5a3cda63490e70b3`. Its complete source tree equals
released native `84f66c4dd4cb18f4c7be1bfdedb3d9f6d86ac5ff`.
This explicitly retains the owned fork's release lineage.

Source branch: `lane/relative-rotor-native-20261028`, based on merged claim
`ca0e9f8f`. Optional native hinge scalar, API/settings/state serialization,
focused unit cases, bounded before/after-state probe and hosted workflow are
implemented for review. No native compilation or execution has occurred at
this source checkpoint. Zero-inertia baseline, original guard and all affected
qualification gates remain required. Bindings and release pins stay separate.

The [Lite rotor correction](https://github.com/CrispStrobe/brickwright-lite/tree/lane/robot-rotor-correction-successor-20261028)
remains rejected for general use. A clean, bounded Node 22 / released bw.7
native control measured an ordinary neighboring hinge's forbidden transverse
speed at 0.313091 rad/s with the post-step correction, versus 1.40e-8 rad/s
without it, against the same 1e-4 limit. Preserve that failure and all original
Sarrus, collision and hub endpoint assertions. Free-pair algebra is insufficient.

Implement an optional internal scalar rotor velocity/inertia element in the
existing hinge constraint. It must participate in normal velocity iterations,
warm starting, contacts, constraint islands and save/restore, without adding a
rigid body or changing body mass, centre of mass or bare inertia. Default zero
rotor inertia retains the original behavior. Use the hinge motor's common world
axis and native impulse sign; do not add JavaScript post-step angular kicks.
The intended model is ideal reflected axial inertia, not measured hardware.

Own `Jolt/Physics/Constraints/HingeConstraint.h` and `.cpp`, a focused new
constraint-part header if needed, `Jolt/Jolt.cmake` source registration,
focused hinge unit tests and their source inventory, a bounded native probe
under `brickwright/rotor-inertia/`, its hosted workflow, and this scope plus
`BRICKWRIGHT.md`. Keep existing rope, gear, determinism and material behavior
and all existing assertions/workflows. No solver iteration, timestep, material,
contact policy or unrelated constraint change is included.

Require analytic reduced inertia and bare angular momentum; static/dynamic/
kinematic orientations; ordinary neighboring hinge and anchor-point velocity
constraints; asymmetric anisotropic bodies; multiple rotors; zero-inertia
compatibility; repeated configuration/removal; sleeping and activation;
variable-step warm starting; lifetime and exact checkpoint restoration.
Use meaningful negative controls and preserve every failure. Heavy compilation
belongs on hosted CI. Admit every enabled exact-head check, only declared skips,
before a normal matched-head merge and exact parent/tree preservation proof.

Bindings and Lite adoption require their separate scopes. No native pin, npm
release or Lite change is made by this documentation-only claim. No upstream
posting, physical-accuracy, hardware, reference firmware or release claim.
