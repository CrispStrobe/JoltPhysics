# Explicit internal-edge pairs

`PhysicsSystem::SetSimInternalEdgeRemovalPair(source, surface, true)` enables
enhanced internal-edge removal for one live rigid-body pair, with `surface` as
the second shape in the narrow-phase query. The unordered full BodyID key finds
the pair independently of creation order; its value preserves the requested
orientation. Native solver order and simulation shape/part-pair filters remain
in effect. No body flags are changed, and a new system has no selected pairs.

Configure at an externally serialized idle-world point, before stepping. This
API is not a general synchronization mechanism for concurrent world operations.
The boolean APIs refuse calls during Update, callback reentry, or contention.
Changing orientation, enabling, disabling, or clearing invalidates both contact
caches without waking sleeping bodies. Destroying or unassigning a body removes
its selections before ID reuse and invalidates each surviving partner.

A custom `SetSimCollideBodyVsBody` callback clears selections and prevents new
selections. `ResetSimCollideBodyVsBody` explicitly restores the default callback
and clears selections; enabling a pair never silently replaces a callback.
Callback replacement and pair/cache changes share the existing body-list lock;
user functor destruction occurs after releasing it.

Selections are session configuration, like the simulation shape filter and
custom collision callback. They are not included in SaveState. Save/restore is
supported only with unchanged configuration and body identity. Applications must
discard saved checkpoints when reconfiguring the policy or recreating bodies;
restoring an old checkpoint does not restore an old selection map. A fresh
PhysicsSystem starts with no selections.

The native controls exercise the engine, asymmetric part-pair allow/deny lists,
both creation orders, selected and unselected pairs, cache transitions, exact ID
reuse including sequence wrap, callback exclusion/reentry, and unchanged-session
save/restore. Configure this directory with CMake, build
`internal-edge-pair-controls`, and run that executable. These synthetic controls
do not establish actual model, browser, hardware, or performance qualification.
