# Brickwright relative rotor inertia: native scope

Status: NATIVE SOURCE QUALIFIED AND MERGED — WebAssembly and Lite qualification pending.
Owner: Codex, authorized Robot Lab physics continuation.
Integration target: `brickwright/gear-mesh-master` at
`389854fb7912d814112ce46b5a3cda63490e70b3`. Its complete source tree equals
released native `84f66c4dd4cb18f4c7be1bfdedb3d9f6d86ac5ff`.
This explicitly retains the owned fork's release lineage.

Native source [PR 3](https://github.com/CrispStrobe/JoltPhysics/pull/3) merged
as `afdb16ddd4f4d5f2ad16cc8814abe0c80114464e`, with reviewed source tree
`5ed8cd85dac0f67323e52b56eb5c3e1216d3f6d5`. All three exact-head jobs passed:
[relative rotor](https://github.com/CrispStrobe/JoltPhysics/actions/runs/40771299072)
and [Linux/macOS rope controls](https://github.com/CrispStrobe/JoltPhysics/actions/runs/40771299078).

The retained native artifact was acquired once and independently audited without
producer imports or engine replay. Its actual CI checkout
`3521e262008e1426a231bce4fd5ec92b9201313d` has the reviewed full tree. The bounded
probe passed 819 checks; the focused hinge suite passed three cases and 736
assertions. The zero-rotor trace matched the previous native baseline byte for
byte, and disabling the new row produced the intended negative control failure.
Executable bytes were absent: reported binary digests were custody-bound,
not independently rehashed binaries.

The initial local probe failure remains recorded: the observer combined
post-integration poses with velocities solved against pre-integration constraint
frames. Retained-data analysis found this mismatch in baseline and rotor cases.
The qualified revision corrects observation only; engine code, thresholds and
iteration policy were unchanged by that correction.

These are bounded native no-contact scene and existing hinge/rope controls.
WebAssembly packages, Lite model programs, rendered save/reopen, hardware and
all-lifetime energy conservation remain unqualified. Recorder binary state is
same-build only. Configuration, deactivation and explicit resynchronization
remain separate state boundaries. Bindings and release pins require their own
qualification.

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

Bindings and Lite adoption require their separate scopes. This status-only update changes no native source, dependency pin, package
release or Lite code. No upstream
posting, physical-accuracy, hardware, reference firmware or release claim.
