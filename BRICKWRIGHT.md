# Brickwright fork of Jolt Physics

Branch `brickwright/gear-mesh-master` = upstream **master** at `01adafba0` (after v5.6.0; includes 64-bit
ObjectLayers #2163, `EstimateCollisionResponse` with `ContactSettings` #2166 and the denormal effective-mass guard
#2105) plus the same gear changes as `brickwright/gear-mesh` (which is v5.6.0 `e77f175` + the gear changes and is
the branch the released JoltPhysics.js `1.1.0-bw.1` builds). Since JoltPhysics.js `1.1.0-bw.2` this is the branch
the releases build; `1.1.0-bw.3` adds change 3 below.

Port notes (cherry-picks of `686dbe2`, `fdedbe6`, `c1143da`, `f878494`):

* `GearConstraintPart.h` conflicted with #2105: kept the fork's `K⁻¹ = a·I1⁻¹·a + r·b·mRatio_InvI2_B` and upstream's
  `inv_effective_mass < FLT_MIN` deactivation test.
* `GearMeshConstraint.cpp` uses the same `>= FLT_MIN` convention for its effective mass (was `> 0`).
* `Constraint.h` (GearMesh after Pulley), `Jolt.cmake`, `RegisterTypes.cpp`: applied cleanly. #2163 and #2166 do not
  touch the gear code (no object-layer or contact-response use); they matter only to the JoltPhysics.js bindings.

Branch `brickwright/gear-mesh` = upstream tag **v5.6.0** (`e77f175`, the version JoltPhysics.js 1.1.0 builds) plus
two gear changes needed by Brickwright Lite's Robot Lab. Nothing here is offered upstream (owner decision); this file
documents what the fork changes and why. Jolt Physics is MIT licensed (see `LICENSE`, unchanged); the new files carry
the upstream SPDX header plus a Brickwright line.

The npm package built from this fork is in the JoltPhysics.js fork:
<https://github.com/CrispStrobe/JoltPhysics.js/tree/brickwright/gear-mesh> (see its `BRICKWRIGHT.md` for the release
tarball and integrity hash).

## 1. What was wrong: `GearConstraint` passes torque 1:1

`GearConstraintPart.h` documents the constraint `C = θ1 + r·θ2`, Jacobian `J = [0, aᵀ, 0, r·bᵀ]`, and uses
`K = a·I1⁻¹·a + r²·b·I2⁻¹·b` (correct). But the impulse it applies is

```cpp
ioBody1.GetMotionProperties()->AddAngularVelocityStep(inLambda * mInvI1_A);
ioBody2.GetMotionProperties()->AddAngularVelocityStep(inLambda * mInvI2_B);   // should be inLambda * r * I2^-1 b
```

and the same in `SolvePositionConstraint` (`AddRotationStep(lambda * mInvI2_B)`). The ratio is missing from body 2's
part of `Jᵀλ`. Consequences (all measured with JoltPhysics.js 1.1.0, `brickwright/verify.mjs` in the JS fork):

* The kinematic ratio comes out right (the solver iterates the velocity error to zero), but the **torque crosses the
  mesh 1:1** instead of ×r. Two free discs 8:24 (I1 = 1e-4, I2 = 9e-4 kg·m², τ = 1e-3 N·m on the 8T):
  α1 = **2.50** rad/s² = τ/(I1 + I2/3) instead of τ/(I1 + I2·(1/3)²) = **5.00**; work 1.25e-3 J but kinetic energy
  6.25e-4 J (**−50 %** of the work vanishes).
* Static balance on a 1:3 train (load m1 on the 8T, m2 on the 24T, same arm): balances at **m2 = m1** (0.08°), and
  the physical m2 = 3·m1 swings away (112° after 2 s).
* One velocity iteration is not a Newton step any more (`K⁻¹` uses r², the impulse r¹), so the iteration contracts by
  `(A + r·B)/(A + r²·B)`: for r < 0 or small r the factor leaves (0, 2) and the solve **diverges** — which is why
  Brickwright's `jolt-core.js` had to order every gear pair so that r ≥ 1.

The bug is still present on upstream `master` (checked 2026-10-19). It is not a usage problem: Brickwright's mapping
(`r = 1/|ratio|`, signed axes) is consistent with the documented equation; the impulse is not.

**Fix** (`Jolt/Physics/Constraints/ConstraintPart/GearConstraintPart.h`): store `mRatio_InvI2_B = r·I2⁻¹·b`, use it in
the velocity step, the position step and `K⁻¹ = a·I1⁻¹·a + r·b·mRatio_InvI2_B`. API unchanged.

## 2. What was missing: a gear mesh that loads its axles

Even with the ratio fixed, `GearConstraint` is purely angular: it applies `λ·a` to gear 1 and `λ·r·b` to gear 2, whose
sum is not zero. A real mesh is a force pair at the pitch point: equal and opposite, so it has no net moment, it pushes
both axles (the hinges then carry the reaction into the frame) and total angular momentum is conserved. With the
angular-only constraint a mesh on a free frame creates angular momentum: a free-floating gearbox whose motor drives
an 8:24 pair turned **64°** in 2 s with |L| 0.68 × the gear's own angular momentum, and its relative ratio read
+0.54 instead of −1/3 (1.1.0). This is the "spinning excavator".

**New constraint: `GearMeshConstraint`** (`Jolt/Physics/Constraints/GearMeshConstraint.{h,cpp}`, subtype
`EConstraintSubType::GearMesh`, registered in `RegisterTypes.cpp`, listed in `Jolt/Jolt.cmake`).

* Physics: a 1-D bilateral contact at the pitch point P along the line of action n,
  `n · (V2(P) − V1(P)) = 0`, Vi(P) = velocity of gear i's material point at P. Gear i turns relative to its frame
  Fi (the other body of its hinge) about ai, so `Vi(P) = vFi + ΩFi × (P − xFi) + ((ωi − ΩFi)·ai)·ei` with
  `ei = ai × (P − ci)`. Written that way the Jacobian is angular-only on the gears (`±ki·ai`, `ki = n·ei`) and
  linear + angular on the frames (`±n`, `±((P − xFi) × n − ki·ai)`; merged into a pure couple `k1·a1 − k2·a2` when
  one frame holds both gears). The impulse on each side adds up to a force ∓λn at P, so linear and angular momentum
  are conserved exactly, the reaction reaches the frame(s) (the axles are loaded), and torque is multiplied by the
  pitch-radius ratio. Gears on different frames (a planet on a carrier meshing with a sun on the frame) work too.
  A first version used one `AxisConstraintPart` on the two gear bodies (the textbook contact): physically the same,
  but it converged badly when a gear's inertia is large next to m·r² (Brickwright floors gear inertia): 1 % ratio
  and energy error on two discs, and a motor-held gear crept 13° in 5 s. The frame form is conditioned like
  `GearConstraint`. Because the frames are written to, the constraint goes to the non-parallel island split
  when a frame is dynamic.
* Parallel axes (spur, internal): P is recomputed every step from the two axes and the ratio:
  `ρ = −ratio·sign(a1·a2)`, `r1 = D·ρ/(1+ρ)` from gear 1's axis towards gear 2's (external ρ > 0, internal ρ < 0),
  axially halfway; n = a1 × (P − c1) normalised. The ratio is exact whatever the centre distance D.
* Non-parallel axes (worm, crossed helical, bevel): `mContactPoint` gives P; it is stored in a frame built from both
  axes (a1, a1 × a2, …) so it follows the frame holding them. n lies in the plane of the two pitch velocities
  `ei = ai × (P − ci)` with `n·e1 = ratio·n·e2`; for a worm that is the thread normal (lead angle), so the wheel's
  load pushes the worm along its axis. For bevel gears (e1 ∥ e2) n = e1/|e1| and P must be on the pitch cones'
  contact line (`|P−axis1| / |P−axis2| = |ratio|`). **Friction is not modelled**, so a worm does not self-lock
  (Brickwright keeps its per-step hold).
* Drift: `SetConstraints(hinge1, hinge2)` (optional, same idea as `GearConstraint::SetConstraints`). The two hinge
  angles are unwrapped once per step; the phase error is `C = n·(Δφ2·e2 − Δφ1·e1)` (arc length at P), corrected
  by the position solve. Either body of a hinge may be the gear; its other body is the frame. Without hinges the
  frames are assumed at rest (it then behaves like a correct `GearConstraint`) and there is no drift correction.
* Ratio convention: `mRatio = ω2/ω1`, rates **relative to the frame**, about `mHingeAxis2` / `mHingeAxis1`
  (external spur 8T→24T, same axis direction: −1/3).

```cpp
GearMeshConstraintSettings s;
s.mSpace = EConstraintSpace::WorldSpace;
s.mPoint1 = c1; s.mHingeAxis1 = a1;   // gear 1 centre / axis
s.mPoint2 = c2; s.mHingeAxis2 = a2;   // gear 2 centre / axis
s.mRatio = -8.0f / 24.0f;             // w2 / w1 relative to the frame
s.mContactPoint = p;                  // only for non-parallel axes
GearMeshConstraint *c = static_cast<GearMeshConstraint *>(s.Create(gear1, gear2));
c->SetConstraints(hinge1, hinge2);    // optional drift correction
physics_system.AddConstraint(c);
```

Queries: `GetContactPoint()`, `GetContactNormal()`, `GetPositionError()` (m), `GetTotalLambda()` (N·s along n).

## 3. A bounce across two bodies: continuous collision resolved it off-centre (2026-10-20, `brickwright/gear-mesh-master`)

`PhysicsSystem::JobFindCCDContacts` finds the FIRST body a `LinearCast` body hits and the average point of that one
manifold; `JobResolveCCDContacts` then applies the whole impact there, restitution included. A body landing across
two others at the same time (Brickwright's validation scene: a 2 × 4 brick dropped 20 cm to 1 m across two loose
beams, restitution 0.4) got its bounce impulse at the first beam's contact, off its centre of mass, and turned over
(measured with JoltPhysics.js 1.1.0-bw.2; with CCD disabled the same 20 cm drop bounces straight up). With two LOOSE
beams the first one also took the whole reaction: the brick left at ~0.96 of e·v and rose to 4 % of the drop instead
of e² = 16 % (`brickwright/verify.mjs` scene F, upstream vs fork).

The change (`PhysicsSystem.cpp`, `JobResolveCCDContacts` and the new `sTouchesAnotherBody`): when a CCD impact would
BOUNCE (combined restitution > 0, approach faster than `mMinVelocityForRestitution`) and the body, posed at the time of
impact, is within the speculative contact distance of ANOTHER body it can collide with (layers, collision group, not a
sensor), the CCD step does not resolve it: the body is moved to the time of impact (`mFraction`, touching; not the
`mFractionPlusSlop` pose, which is up to `mLinearCastMaxPenetration` inside) with its velocity unchanged, and the
next step's discrete contact solve, which has the manifolds with every body it touches, applies the restitution. It
cannot tunnel: from that pose it is in contact. Everything else is upstream's, bit for bit: a bounce off a single body
(a part dropped on the floor), every impact that does not bounce (`verify.mjs` scene E's hash is unchanged), soft
bodies.

Limit (not changed here): the discrete solve resolves a flat multi-point restitution impact by sequential impulses,
and that solution is not unique (a point may end with zero impulse while the body spins): a flat box bouncing off two
static beams leaves with ~0.1–0.3 rad/s of spin (more velocity steps reduce it slowly: 0.26 → 0.17 rad/s from 10 to
100). It does not turn the body over on the first bounce, but a tall drop's second, slightly tilted landing can.

## 4. A rack and pinion on a moving frame: `RackMeshConstraint` (2026-10-20, released in JoltPhysics.js 1.1.0-bw.4)

`RackAndPinionConstraint` ties the pinion's WORLD rotation rate to the rack's WORLD velocity (w1 . a = r v2 . b). That
holds only while the frame carrying both is at rest. On a robot that turns or drives, the frame's own motion is read as
pinion rotation or rack travel: Brickwright's forklift carriage was pushed to its end stop and tipped the machine as
the robot turned, and a car's steering rack twisted the car (so the differential car steered by a direct kingpin).

The new `RackMeshConstraint` (`Physics/Constraints/RackMeshConstraint.{h,cpp}`) does for a rack what
`GearMeshConstraint` does for gears: the relation holds relative to the frames, taken from the pinion's hinge and the
rack's slider (`SetConstraints(hinge, slider)`; each joint's other body is that part's frame):

  (w_pinion - W_Fp) . a = r (v_rack - v_Fr - W_Fr x (x_rack - x_Fr)) . b

Jacobian rows: pinion angular a; pinion frame angular -a; rack linear -r b; rack frame linear r b and angular
(x_rack - x_Fr) x r b (one row when both frames are the same body). The impulse is a couple on pinion and frame and an
equal and opposite force pair along b through the rack's centre of mass: linear and angular momentum are conserved
and the reaction reaches the frame. The position solve removes the drift of the joints' relative angle/travel since
the last `SetConstraints` (Brickwright re-bases it before every step, as for the gear meshes). Without
`SetConstraints` the frames count as at rest: RackAndPinionConstraint's equation. `RackAndPinionConstraint` itself is
unchanged (the regression scene's hash is unchanged); settings and conventions are the same (body 1 the pinion,
body 2 the rack, mRatio rad per metre). Measured (JoltPhysics.js `brickwright/verify.mjs` scene H, 60 Hz, 1 s): a
chassis spinning at 3 rad/s or driving at 0.3 m/s with the pinion held keeps the rack where it was (|slide| < 1e-8 m;
RackAndPinionConstraint: 38 mm and -261 mm); with the pinion turned 2 rad by its motor the rack travels r·θ (16.0 mm).

## Results (JoltPhysics.js wasm-compat, 60 Hz, `brickwright/verify.mjs`)

| Scene | 1.1.0 `GearConstraint` | fork `GearConstraint` | fork `GearMeshConstraint` |
|---|---|---|---|
| Free discs 8:24, τ on 8T: α1 (expected τ/(I1 + I2·(1/3)²) = 5.00 rad/s²) | 2.50 | 5.00 | 5.00 |
| same: (KE − work)/work | −50 % | 5e-7 | 7e-7 |
| Floating gearbox, internal motor drives 8:24: max abs(L) / gear's abs(I1·ω1) | 0.68 | 0.58 | 5.6e-7 |
| same: relative ratio (expected −0.3333) | +0.54 | +0.41 | −0.3333 |
| same: frame turn in 2 s (inertias chosen so I1ω1 + I2ω2 = 0: physically 0°) | 64° | 159° | 0.000° |
| Worm 1:24 in a floating frame: abs(L) / gear's; ratio (expected 0.04167) | n/a | n/a | 0.0089; 0.04164 |
| Balance 1:3, drum weights m2 = 3·m1: angle after 2 s | 175° | 0° | 0° |
| Balance 1:3, m2 = m1 (must run away) | 0° (balanced: the bug) | 146° | 146° |
| 8T held by its motor, 0.05 N·m on 24T for 5 s: 8T drift / mesh phase error | 0.0003° / 0.0005° | 0° / 0° | 0° / 0° |
| Non-gear regression scene (contacts, hinge, rack and pinion; state hash after 2 s) | 582ce7fb… | 582ce7fb… (identical) | — |
| Rack and pinion travel per pinion radian (expected −0.008 m) | −0.008 | −0.008 | — |

Gear bodies: 8T I = 1e-4 / 3e-5 kg·m², 24T 9e-4 / 9e-5 kg·m² (Brickwright floors gear inertia at 3e-5),
30 velocity / 10 position steps on the gear constraints, 60 Hz. The worm's 0.9 % falls to 0.1 % when the frame's inertia is
made isotropic: most of it is the tumbling frame (Jolt integrates without a gyroscopic term), not the mesh.

## Build

No local build was used: the C++ is compiled by the JoltPhysics.js fork's GitHub Actions workflow
(`.github/workflows/brickwright-release.yml`, emsdk 6.0.2 as upstream), which fetches this branch at a pinned commit.
A native build works as upstream (`cd Build && ./cmake_linux_clang_gcc.sh`). The GearMesh enum value is inserted
after `Pulley`, shifting `User1..4` by one (nothing in Jolt or JoltPhysics.js depends on their numeric values).
