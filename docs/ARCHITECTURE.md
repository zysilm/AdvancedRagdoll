# AdvancedRagdoll: how the library is put together

The source comments point here for the explanations that do not fit next to the code. The user-facing description
(how to use the public API) is the [README](../README.md).

## 1. Layout

- `src/AdvancedRagdoll/` is the library (`AdvancedRagdoll.csproj`, the NuGet package).
  - Folders named after a namespace below `AdvancedRagdoll` hold that namespace: `Hosting` (host interfaces),
    `Bepu` (extension points), `SkeletonProfiles`, `SurfaceProfiles`, `CollapseProfiles`, `MeshVolume` (mesh
    collision), `Hair`, `Attachment` (retained garments), `Recording`, `WeaponDrop`.
  - The other folders only group files of the root namespace `AdvancedRagdoll` by topic: `Ragdoll` (the partial
    files of the `Ragdoll` class, one per concern), `Rig` (bone table, tuning, hull fitting), `Activation` (the start
    trigger), `World` (terrain triangles, water), `Rendering` (rendered geometry and pose bookkeeping) and `Gear`
    (dropped equipment and weapon geometry).
- `tests/` holds the sample and the offline tests, each its own project. Most compile the library source files they
  check directly (so they see internal types); `tests/LibrarySample` references the library project and sees only the
  public API.
- `docs/` holds this file.

## 2. Host interfaces

A ragdoll never touches the game itself: everything it needs from the game goes through host interfaces. There are
two layers.

- **Public** (`Hosting/RagdollHost.cs`): `RagdollHost`, `ISkeletonPose`, `IWorldQuery`, `IRagdollActor`,
  `IRagdollSettingsSource`, `IRagdollLog`. Characters are an opaque `ActorId`; poses are read and written in batches.
  This is what another game implements.
- **Internal host services** (`Hosting/RagdollHostInterfaces.cs`): `IHostSkeleton`, `IHostSkeletonSource`,
  `IHostWorldQuery`, `IHostActors`, `IHostColliders`, `IHostSettingsSource`, `IMeshSource`, and the injectable clock,
  random source and scheduler, collected in `RagdollHostServices`. The ragdoll core is written against these.
  Characters are an opaque native-sized id, and a skeleton hands out its model pose as a writable span over the live
  pose, so every memory access and every float operation stays exactly where it was when the core was written.
  Trusted hosts (the assemblies named in `InternalsVisibleTo`) implement these directly.

The public facade (`Ragdoll/Ragdoll.Facade.cs`, class `FacadeHost`) adapts the public interfaces onto the internal
ones: an `ActorId` becomes an opaque id, the skeleton is read into a buffer once per `Step` and the core reads and
writes that buffer in place, `WritePose` writes back only the bones that changed, the `dt` passed to `Step` becomes the
frame time, and the settings are taken once per activation and once per step.

### 2.1 Conventions

World Y up, metres, seconds, radians, kilograms, `System.Numerics`. Poses are in model space (relative to the
skeleton root); the root transform maps model space to world space. The character's forward axis comes from the
skeleton profile. The library does no handedness conversion: a host with another convention converts in its adapter.

### 2.2 Threading

- A `Ragdoll` is not thread-safe. `Activate`, `Step`, `WritePose`, `Deactivate`, `Dispose` and every other call that
  changes the simulation run on one thread, or the host serialises them.
- Each ragdoll has its own BepuPhysics simulation. With mesh collision on, the narrow phase runs on a small shared
  pool of worker threads (`MeshVolume/MeshPhysicsDispatcher.cs`); the contact callbacks there read only a copy of the
  settings taken at the start of the frame.
- Background work (hull fitting, loading hair samples, reduced collision surfaces) only computes: its results are
  installed on the simulation thread at fixed points. Whether a result is ready in time is asked of the scheduler
  (`IRagdollScheduler.TryWait`), and every wall-clock decision goes through `IRagdollClock`, so a recording can keep
  the decision and a replay can give the same one back.

## 3. Skeleton profile

The ragdoll code never names a bone. It asks the `SkeletonProfile` for "the pelvis", "the hand bones", "the left
thigh". A profile carries:

- landmarks (named bones with an anatomical part: pelvis, spine, head, the limb segments of each side) and bone
  classes (cloth, soft tissue, weapon, the limb classes) as name rules;
- the bone table (parent, shape, radius, half length, mass, joint type, swing and twist limits), the ranges of motion
  and the mass budget;
- the signature bones a skeleton is recognised by, the forward axis, hull-fit and hinge-frame targets.

`SkeletonProfile.Auto()` stands for the rig generated from a skeleton's own topology. A skeleton that has all of a
profile's signature bones gets the profile's rig; any other skeleton gets the generated one.

`tests/BoneNameLiterals` keeps the library free of bone-name literals and of host-specific names.

## 4. Settings

The host's saved settings stay with the host; the library has its own copies of the few setting types it needs and
never serialises them.

- `RagdollSettings` (public) is taken once per activation (`IRagdollSettingsSource.Capture`) and does not change until
  the next activation: ranges of motion, the knee buckle, the exact ground.
- `RagdollLiveSettings` (public) is read at the start of every step: water, verbose logging.
- `HostSettings` (internal) is the full set the core reads. The internal host refreshes it once at the start of every
  render frame (`IHostSettingsSource.Refresh`); everything in that frame, worker threads included, reads this copy,
  so a setting changed in the middle of a frame takes effect at the next one. Its property names are written into
  recordings by name, so they do not change.

## 5. Extension points

`Bepu/RagdollExtensions.cs` defines two internal extension points that reach into the simulation:

- `IRagdollStepExtension`: once per fixed step on the simulation thread, after the water forces and before the
  world's `Timestep`. Setting `handled` means the extension stepped the world itself (or chose not to), and the
  ragdoll skips its own `Timestep`.
- `IRagdollContactExtension`: once per contact manifold, on a physics worker thread, as the first thing the contact
  callback does. Setting `handled` makes the ragdoll leave the manifold as the extension configured it.

They are registered with `Ragdoll.AddExtension` and called in registration order; the first one that sets `handled`
ends the call. With none registered each call site costs one length check. A `RagdollExtensionContext` gives an
extension the ragdoll's simulation and the state it needs. In 0.1 all of this is internal and carries no
compatibility promise.

## 6. Recording and replay

A trusted host can record ragdoll runs: while a recording session is on (`Recording/RagdollRecordingSession.cs`),
each activation writes one file. The recorder only observes:

- the host interfaces it can watch from outside are wrapped in decorators (`Hosting/RecordingDecorators.cs`): world
  queries, mesh products, clock readings, random draws, scheduler decisions;
- the observation points inside the simulation (skeleton captures, frames, steps, the public calls) are written by
  `Ragdoll/Ragdoll.Recording.cs`, at the point where they happen.

The file format is described in `Recording/RagdollRecordingFormat.cs`. An offline replay host implements every host
interface from a recording: each call takes the next matching record, checks the arguments, and answers what was
recorded; the replay then compares every step's bodies and every written pose with the recorded ones bit for bit.
Results are bit-exact only on the same machine, .NET runtime and BepuPhysics version.

## 7. Dropped weapons

`WeaponDrop/WeaponDropSimulation.cs` simulates the weapons a character drops when it dies, in one BepuPhysics
simulation of its own, shared by all dropped weapons. Its host (`Hosting/WeaponDropHost.cs`) reuses the ragdoll's host
interfaces and adds `IWeaponDrawSource` for the drawn weapons. The ragdoll recordings do not cover it;
`tests/WeaponDropGolden` replays fixed synthetic scenarios and compares the complete output with committed golden
files byte for byte.

## 8. Components and the simulation context

The `Ragdoll` class is being split, one subsystem at a time, into components that share one simulation context. Each
step moves code without changing what it does: every read, write, random draw, clock reading, recording call, log line
and physics call happens in the same order as before, and the ragdoll recordings replay bit for bit.

- **`SimulationContext`** (`Ragdoll/SimulationContext.cs`) holds the state many components read and write: the host
  services they use (skeletons, world queries, mesh source, actors, clock, log, the settings of this frame), the
  skeleton profile and its bone configurations, the simulation and its buffer pool, the ragdoll's bodies, the target
  character, whether the rig is generated, and the skeleton root transform with its model/world conversions. It is a
  plain object with fixed fields, made once with the ragdoll; it is not a service locator. Only state an extracted
  component needs moves into it.
- **A component** is a sealed class per subsystem. It owns that subsystem's fields, gets the context in its
  constructor, and keeps no reference to the ragdoll. The ragdoll creates its components in its constructor, keeps the
  same members it had (internal and public) and forwards them to the component.
- **The rest of the class**, not split yet, reaches the moved fields through private forwarding properties under their
  old names (`simulation`, `ragdollBones`, `settings`, `skelWorldPos`, ...), so the code that is not split yet reads
  and writes the same storage.
- **`CoreCalls`**: what a component still needs from the part that is not split yet (recording, the mesh caches,
  constraint removal, the character scale, the render interpolation of the stepping loop) goes through this one small
  object. Each member forwards to the ragdoll's own method unchanged. It lists exactly where the components still
  depend on the rest, and shrinks as that becomes components too.
- For now the context, `CoreCalls` and the components are private types nested in `Ragdoll`, in the partial file of
  their subsystem: they use the ragdoll's private body table and helpers without widening anything a trusted host can
  see. The compiler keeps a nested component honest: it has no `Ragdoll` instance, so any use of the ragdoll's own
  state that did not go through the context fails to compile.

Components so far:

| Component | File | Owns |
| --- | --- | --- |
| `Water` | `Ragdoll.Water.cs` | the water level over the corpse, the probe timer and position, the wave clock, whether a body is in the water |
| `WeaponContactSurfaces` | `Ragdoll.WeaponContactMesh.cs` | the kinematic surface twins a weapon rests on, and the three contact filter sets for them |
| `HairRig` | `Ragdoll.HairRig.cs`, `Ragdoll.HairSurfaces.cs` | the hair guide chains, live (animated) hair, the body surfaces and ground samples the hair rests on |
| `MeshVolumes` | `Ragdoll.MeshVolume.cs`, `Ragdoll.MeshCache.cs`, `Ragdoll.MeshDeformation.cs`, `Ragdoll.MeshExecution.cs` | the frozen body surfaces the controller carries over activations (its target's, and other actors' for weapon contacts) with their LOD settings, the mesh bodies built from them, the collision LODs swapped in while stepping, the shared-worker lease and the mesh step timings, the mesh status text |
| `StructuralFit` | `Ragdoll.StructuralMeshFit.cs` | the hull fit started when the ragdoll is armed, and the hull shapes made from it |
| `SurfaceRuntime` | `Ragdoll.Surface.cs` | the query copy of the accepted collision proxy (traversal, grabbing, debug drawing) |

<!-- a6-2: the three rows above. The capture of a body surface (bone definitions, mod soft-tissue bones, severed
limbs) stays in the ragdoll; the components reach it through CoreCalls. The nested type is `MeshVolumes`, not
`MeshVolume`, so that it does not hide the namespace AdvancedRagdoll.MeshVolume inside Ragdoll. -->
| `Ground` | `Ragdoll.Ground.cs` | the ground height (physics and real), the ground patches laid so far and their size, the scratch list for the exact ground triangles |
| `StandingSupport` | `Ragdoll.StandingSupport.cs` | the standing hold's constraints, anchor body and target, whether it is up; the wrist constraints and their force and frequency |
| `Grab` | `Ragdoll.Grab.cs` | the grab slots and the next slot id, the grabber whose collision is parked during a grab |
| `Impacts` | `Ragdoll.Impacts.cs` | the hitstop and cooldown timers, the last torso descent speed, the torso body indices, the jitter random state |
| `WeaponStrike` | `Ragdoll.Strikes.cs` | the strike window (timer, power, striking colliders), the blade's last sample, the bodies struck this swing |

<!-- a6-3 -->
Since A6-3 the context also holds whether the ragdoll is active, the rig's total mass, the active class profile and a
body lookup by bone name; `CoreCalls` also carries the start of the settle window (stepping loop) and the carrying of
worn external rigs with the standing hold (external bodies). Where a component would need another subsystem's state or a
recorded API call, the ragdoll's own method does that step and passes the value in (for example whether a grab or the
standing support holds the rig, or the attachment a grab resolved), so the components do not reference each other.
The ragdoll's direct pokes (impulse, angular kick, punt, shake), the joint release after a recoil or a landing (joint
state), the carrying of external rigs and the grab-attachment queries stay with the ragdoll for now.

<!-- a6-4 -->
| Component | File | Owns |
| --- | --- | --- |
| `KneePowerLoss` | `Ragdoll.KneePowerLoss.cs` | the knee power-loss pattern: whether it is armed or running, its phase and clocks, its six servos, the start poses, foot proxies and knee flex targets |
| `GuidedCollapse` | `Ragdoll.GuidedCollapse.cs` | the directed collapse (kneel-pitch drive or a collapse profile's phases: what is armed for the next death, the five servos, targets, clocks) and the collapse entry conditioning (armed or running, the measured stance and knee angles, the continuation) |

Since A6-4 the context also holds whether the rig rests asleep (the stepping loop decides it), waking every body, the
character's flattened facing and a bone's origin from its body. The two collapse components call recorded API methods
(stopping the patterns, beginning the knee power-loss) at fixed points of their work, so the ragdoll's own method makes
those calls between the component's steps: beginning is split into a check (rig ready, bodies found) and the start,
and a step that stops the pattern returns that to the ragdoll, which walks a collapse profile's controllers itself so a
controller that releases the rig stops it at the same point. The entry conditioning's continuation is the ragdoll's
recorded begin, handed in. The collapse profile book and the knee power-loss tuning stay with the ragdoll.

<!-- a6-5 -->
| Component | File | Owns |
| --- | --- | --- |
| `ExternalBodies` | `Ragdoll.ExternalBodies.cs` | the bodies and rigs added from outside (dropped gear, garments) with their generation stamp, the four contact filter sets the narrow phase reads for them, the rigs carried along with the standing hold |
| `SoftTissue` | `Ragdoll.SoftTissue.cs` | the squash-and-stretch springs and the base scales they multiply, the soft-body leashes and their log budget |

Since A6-5 the context also removes the constraints on a body before it is removed (`RemoveConstraintsOnBody`, used by
the external bodies and the weapon-contact surfaces), and `CoreCalls` no longer carries constraint removal or the
carrying of external rigs. The standing hold's anchor correction is a ragdoll method again: the component reports the
correction, the ragdoll collects the worn rigs (a recorded decision), the component moves the rig, and the external
bodies move the collected rigs by the same delta. What needs the settle window, the recorded external-body calls (a rig
body's shape, removing a rig's bodies) or the weapon-contact surfaces stays in the ragdoll's own `*Core` method; the soft
tissue gets the bone definitions and the frame counter passed in. The rig build still fills the leashes and resets the
squash state through forwarding properties.

<!-- a6-6 -->
| Component | File | Owns |
| --- | --- | --- |
| `CharacterColliders` | `Ragdoll.Colliders.cs` | the colliders of the other characters (per character: bone capsules, a skinned mesh or one fallback capsule), the lightweight set, the two kinematic body sets the narrow phase reads for them, the corpse bounds and wake centre, whether a collider moved near the corpse this frame |
| `CorpseTraversal` | `Ragdoll.CorpseTraversal.cs` | the retained support pairs of the NPCs standing on the corpse and the capsule query buffer, the face landmarks, the traversal mass multiplier, the finite-mass carriers of autonomous followers with their requests, lock and body set |

Since A6-6 the context also holds whether physics has started and the settle window (`BeginBiomechanicalSettle`, with
its duration), and reads the character scale (`GetCharacterScale`, `TryGetVisualScale`); `CoreCalls` no longer carries
either. The colliders component builds the bone capsules and owns the kinematic moves (parking, rescaling, the wake
note) and the lightweight and traversal-ghost switches. What reads and records the host stays in the ragdoll: the
skeleton read at build, the fallback capsule (its actor position is read between the shape and the body), the mesh
collider (its triangles are recorded), and the per-frame tracking loop of the stepping, which also drives the strike
window and the grab's parked grabber; adding and removing a live collider stays too (strike colliders, recorded calls).
A support query is split the same way: the ragdoll finds the NPC's collider state, the component collects its capsules,
the ragdoll reads and records the NPC's root, the component resolves the height. The character surface the queries use
belongs to `SurfaceRuntime`, so the ragdoll hands it in.

<!-- a6-7 -->
| Component | File | Owns |
| --- | --- | --- |
| `JointModel` | `Ragdoll.Joints.cs` | the character's anatomical facing, the joint frames (bind-pose hinge frames, anatomical hinge axes and fold directions, ball twist references), the anatomical additions to a joint (fold stop, signed flexion limit, passive hinge rest, directional and kneecap-facing swing caps), the joint friction, the swing-limit stress monitors, each joint's motor and swing limit as built, and the brief joint release after a recoil or a landing |

Since A6-7 the context also holds the target's entity id, whether the host runs the live hair on this ragdoll and the
settle window (starting it, the target's liveness check and the character scale are context methods), so `CoreCalls`
no longer carries the settle window, the liveness check, the live-hair flag or the character scale. The rig build still
creates each joint's own constraints (ball sockets, hinges, swivel hinges, cones, twist limits, servos, motors) inline,
in the same order, and calls the joint model for the anatomical additions at the same points; it records the motors
and swing limits for the release through forwarding properties. Moving that loop belongs to the rig build stage.
