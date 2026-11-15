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

The `Ragdoll` class was split (stages A6-1 to A6-13), one subsystem at a time, into components that share one
simulation context. Each stage moved code without changing what it does: every read, write, random draw, clock reading,
recording call, log line and physics call happens in the same order as before, and the ragdoll recordings replay bit
for bit. What the split left on the ragdoll itself is described at the end ([State after A6](#state-after-a6)).

- **`SimulationContext`** (`Ragdoll/SimulationContext.cs`) holds the state many components read and write: the host
  services they use (skeletons, world queries, mesh source, actors, clock, log, the settings of this frame), the
  skeleton profile and its bone configurations, the simulation and its buffer pool, the ragdoll's bodies, the target
  character, whether the rig is generated, and the skeleton root transform with its model/world conversions. It is a
  plain object with fixed fields, made once with the ragdoll; it is not a service locator. Only state a component
  needs is in it.
- **A component** is a sealed class per subsystem. It owns that subsystem's fields, gets the context in its
  constructor, and keeps no reference to the ragdoll. The ragdoll creates its components in its constructor, keeps the
  same members it had (internal and public) and forwards them to the component. A component changes only its own
  fields; the ragdoll changes a component's state only through that component's methods.
- **The ragdoll's own code** reaches the context's fields through private forwarding properties under their old names
  (`simulation`, `ragdollBones`, `settings`, `skelWorldPos`, ...). Forwarders to a component's fields are read-only;
  forwarders to the context's vector and quaternion fields return them by reference, so a member write or a mutating
  call reaches the field and not a copy.
- **No calls back into the ragdoll.** Until A6-12 a component reached the code that was not split yet through one small
  object, `CoreCalls`; the last three members (the two mesh-cache reads and the frozen mesh capture) went in A6-13 and
  the class is gone. A component that needs another component is handed it in its constructor; these are the explicit
  dependencies:

  | Component | Is handed | For |
  | --- | --- | --- |
  | `MeshVolumes` | `SkeletonReader` | the bone definitions and the severed-limb selection a body surface is captured with |
  | `HairRig` | `MeshVolumes` | the cached body surface the hair rests on (`CachedSurfaceFor`) |
  | `RigBuilder` | `MeshVolumes`, `StructuralFit`, `JointModel`, `SoftTissue` | the shapes they make, the anatomical additions to a joint, the soft tissue's reset and leashes |

  A component never makes a recorded API call itself: where its work needs one, it returns what is needed (an enum when
  there is more than one outcome) and the ragdoll's own method makes the recorded call between the component's steps.
- The context and the components are private types nested in `Ragdoll`, in the partial file of their subsystem: they
  use the ragdoll's private body table and helpers without widening anything a trusted host can see. The compiler keeps
  a nested component honest: it has no `Ragdoll` instance, so any use of the ragdoll's own state that did not go
  through the context fails to compile.

### Components by stage

| Component | File | Owns |
| --- | --- | --- |
| `Water` | `Ragdoll.Water.cs` | the water level over the corpse, the probe timer and position, the wave clock, whether a body is in the water |
| `WeaponContactSurfaces` | `Ragdoll.WeaponContactMesh.cs` | the kinematic surface twins a weapon rests on, and the three contact filter sets for them |
| `HairRig` | `Ragdoll.HairRig.cs`, `Ragdoll.HairSurfaces.cs` | the hair guide chains, live (animated) hair, the body surfaces and ground samples the hair rests on |
| `MeshVolumes` | `Ragdoll.MeshVolume.cs`, `Ragdoll.MeshCache.cs`, `Ragdoll.MeshDeformation.cs`, `Ragdoll.MeshExecution.cs` | the frozen body surfaces the controller carries over activations (its target's, and other actors' for weapon contacts) with their LOD settings and their capture from the rendered body, the mesh bodies built from them, the collision LODs swapped in while stepping, the shared-worker lease and the mesh step timings, whether the active rig has mesh volumes, the mesh status text |
| `StructuralFit` | `Ragdoll.StructuralMeshFit.cs` | the hull fit started when the ragdoll is armed, and the hull shapes made from it |
| `SurfaceRuntime` | `Ragdoll.Surface.cs` | the query copy of the accepted collision proxy (traversal, grabbing, debug drawing) |

<!-- a6-2: the six rows above (A6-1 made the first three). The nested type is `MeshVolumes`, not `MeshVolume`, so that
it does not hide the namespace AdvancedRagdoll.MeshVolume inside Ragdoll. The capture of a body surface (bone
definitions, mod soft-tissue bones, severed limbs) stayed in the ragdoll until A6-13, which moved it into
`MeshVolumes`. -->
| `Ground` | `Ragdoll.Ground.cs` | the ground height (physics and real), the ground patches laid so far and their size, the scratch list for the exact ground triangles |
| `StandingSupport` | `Ragdoll.StandingSupport.cs` | the standing hold's constraints, anchor body and target, whether it is up; the wrist constraints and their force and frequency |
| `Grab` | `Ragdoll.Grab.cs` | the grab slots and the next slot id, the grabber whose collision is parked during a grab |
| `Impacts` | `Ragdoll.Impacts.cs` | the hitstop and cooldown timers, the last torso descent speed, the torso body indices, the jitter random state |
| `WeaponStrike` | `Ragdoll.Strikes.cs` | the strike window (timer, power, striking colliders), the blade's last sample, the bodies struck this swing |

<!-- a6-3 -->
Since A6-3 the context also holds whether the ragdoll is active, the rig's total mass, the active class profile and a
body lookup by bone name. Where a component would need another subsystem's state or a recorded API call, the ragdoll's
own method does that step and passes the value in (for example whether a grab or the standing support holds the rig, or
the attachment a grab resolved), so the components do not reference each other. The ragdoll's direct pokes (impulse,
angular kick, punt, shake) and the grab-attachment queries stay with the ragdoll.

<!-- a6-4 -->
| Component | File | Owns |
| --- | --- | --- |
| `KneePowerLoss` | `Ragdoll.KneePowerLoss.cs` | the knee power-loss pattern: whether it is armed or running, its phase and clocks, its six servos, the start poses, foot proxies and knee flex targets |
| `GuidedCollapse` | `Ragdoll.GuidedCollapse.cs` | the directed collapse (kneel-pitch drive or a collapse profile's phases: what is armed for the next death, the five servos, targets, clocks) and the collapse entry conditioning (armed or running, the measured stance and knee angles, the continuation) |

Since A6-4 the context also holds whether the rig rests asleep, waking every body, the character's flattened facing
and a bone's origin from its body. The two collapse components call recorded API methods (stopping the patterns,
beginning the knee power-loss) at fixed points of their work, so the ragdoll's own method makes those calls between the
component's steps: beginning is split into a check (rig ready, bodies found) and the start, and a step that stops the
pattern returns that to the ragdoll, which walks a collapse profile's controllers itself so a controller that releases
the rig stops it at the same point. The entry conditioning's continuation is the ragdoll's recorded begin, handed in.
The collapse profile book and the knee power-loss tuning stay with the ragdoll.

<!-- a6-5 -->
| Component | File | Owns |
| --- | --- | --- |
| `ExternalBodies` | `Ragdoll.ExternalBodies.cs` | the bodies and rigs added from outside (dropped gear, garments) with their generation stamp, the four contact filter sets the narrow phase reads for them, the rigs carried along with the standing hold |
| `SoftTissue` | `Ragdoll.SoftTissue.cs` | the squash-and-stretch springs and the base scales they multiply, the soft-body leashes and their log budget |

Since A6-5 the context also removes the constraints on a body before it is removed (`RemoveConstraintsOnBody`, used by
the external bodies and the weapon-contact surfaces). The standing hold's anchor correction is a ragdoll method: the
component reports the correction, the ragdoll collects the worn rigs (a recorded decision; the external bodies keep the
list), the component moves the rig, and the external bodies move the collected rigs by the same delta. What needs the
settle window, the recorded external-body calls (a rig body's shape, removing a rig's bodies) or the weapon-contact
surfaces stays in the ragdoll's own `*Core` method; the soft tissue gets the bone definitions and the frame counter
passed in. The rig build resets the squash state and fills the leashes through the soft tissue's own methods (since
A6-10).

<!-- a6-6 -->
| Component | File | Owns |
| --- | --- | --- |
| `CharacterColliders` | `Ragdoll.Colliders.cs` | the colliders of the other characters (per character: bone capsules, a skinned mesh or one fallback capsule), the lightweight set, the two kinematic body sets the narrow phase reads for them, the corpse bounds and wake centre, whether a collider moved near the corpse this frame |
| `CorpseTraversal` | `Ragdoll.CorpseTraversal.cs` | the retained support pairs of the NPCs standing on the corpse and the capsule query buffer, the face landmarks, the traversal mass multiplier, the finite-mass carriers of autonomous followers with their requests, lock and body set |

Since A6-6 the context also holds whether physics has started and the settle window (`BeginBiomechanicalSettle`, with
its duration), and reads the character scale (`GetCharacterScale`, `TryGetVisualScale`). The colliders component builds
the bone capsules and owns the kinematic moves (parking, rescaling, the wake note) and the lightweight and
traversal-ghost switches. What reads and records the host stays in the ragdoll: the skeleton read at build, the
fallback capsule (its actor position is read between the shape and the body), the mesh collider (its triangles are
recorded), and the per-frame tracking loop of the stepping, which also drives the strike window and the grab's parked
grabber; adding and removing a live collider stays too (strike colliders, recorded calls). Those ragdoll methods change
the collider list, the kinematic body set and the strike colliders through the components' methods (since A6-13). A
support query is split the same way: the ragdoll finds the NPC's collider state, the component collects its capsules,
the ragdoll reads and records the NPC's root, the component resolves the height. The character surface the queries use
belongs to `SurfaceRuntime`, so the ragdoll hands it in.

<!-- a6-7 -->
| Component | File | Owns |
| --- | --- | --- |
| `JointModel` | `Ragdoll.Joints.cs` | the character's anatomical facing, the joint frames (bind-pose hinge frames, anatomical hinge axes and fold directions, ball twist references), the anatomical additions to a joint (fold stop, signed flexion limit, passive hinge rest, directional and kneecap-facing swing caps), the joint friction, the swing-limit stress monitors, each joint's motor and swing limit as built, and the brief joint release after a recoil or a landing |

Since A6-7 the context also holds the target's entity id, whether the host runs the live hair on this ragdoll and the
settle window (starting it, the target's liveness check and the character scale are context methods). Each joint's own
constraints (ball sockets, hinges, swivel hinges, cones, twist limits, servos, motors) are created by the rig builder
(since A6-10), in the same order, which calls the joint model for the anatomical additions at the same points and
records the motors and swing limits for the release through the joint model's own methods.

<!-- a6-8 -->
| Component | File | Owns |
| --- | --- | --- |
| `SkeletonReader` | `Ragdoll.Skeleton.cs` | reading the host skeleton into the rig's bone definitions (the profile's humanoid set, the generated set for a non-humanoid skeleton, the soft-tissue bones discovered on it, the bodies left out for severed limbs); the definitions the active rig was built from; the severed-limb selection |
| `PoseWriter` | `Ragdoll.PoseWriting.cs` | the per-frame pose write-back: the bone buffers (this tick's body poses, the previous tick's for render interpolation, the originals and deltas of the frame), the rig's bone index set and the head bone index; reading the bodies, writing the simulated bones, carrying their motion to the bones without a body and to the partial skeletons on the head, collapsing severed limbs |

The stepping loop calls the pose writer at the old points and makes everything else between its steps: it decides when
the previous tick is captured (and whether that snapshot is still valid) and how far to blend (the stepper, since
A6-11; the interpolation flag is in the context); a body that is not finite is reported back (`BodyReadOutcome`) and the
ragdoll deactivates itself; the external-body and hair wake checks, the terrain patches, the frame summary log, the
soft-tissue squash, the hair read-back, the written-pose record and the follow transform stay in the stepping loop. The
rig build sets the active definitions, the bone index set and the head index through the components' own methods
(`SetActiveDefs`, `CollectBoneIndices`, `SetHeadBoneIndex`; since A6-13). The stateless parts stay static on the
ragdoll: the bone-class tests, the profile defaults and the bone-definition builder from the configurations (the host
reads them as `Ragdoll` members), bone lookup by name, and the model/world conversions (`Ragdoll.Transforms.cs`).

<!-- a6-9 -->
| Component | File | Owns |
| --- | --- | --- |
| `Recorder` | `Ragdoll.Recording.cs` | the recording session of an activation (its file and writer, the nesting depth of recorded calls, the render frame, physics frame and fixed step counters, the structural fit outcome, the configuration watches, the skeleton topologies, last poses and mesh caches already written, the body-state buffers) and the controller's recording id |

Since A6-9 the context holds the recorder (`SimulationContext.Recorder`): a component asks it whether the activation is
recorded and writes its records through it (mesh caches, mesh LODs and uses, the hair mesh, the structural fit
outcome). The recorder writes the records that need only their arguments and the context: the header and
configuration, the target's identity, skeleton poses and topologies, actor positions, decisions and events, trigger
inputs, the step body states, bone definitions, structural fits, meshes and triangles, and the argument helpers of
recorded calls; it also runs the checks at the start of a render frame (session stopped, limits). The records that read
other subsystems' state stay in the ragdoll's own `Rec*` methods, which write them through the recorder's writer at the
same moments as before: the Activate record and the carried body surfaces, the render frame, frame begin and end, the
start and end of the rig build, and the selected NPCs (whether the host has a selector). Every `Rec*` method keeps its
name on the ragdoll (most forward in one line), and so do the recorded API calls; a recorded call enters and leaves the
recorder's nesting (`RecEnter`, `RecLeave`). The hair mesh record gets whether the hair is animated from the hair rig,
which passes its own value. The debug capsules (`Ragdoll.Diagnostics.cs`) read the rig and the frame cost counters are
process-wide, so they stay on the ragdoll; the replay's carried-state restore (`Ragdoll.Replay.cs`) stays too, and calls
each component's own `RestoreCarried` (since A6-12).

<!-- a6-10 -->
| Component | File | Owns |
| --- | --- | --- |
| `RigBuilder` | `Ragdoll.RigBuild.cs` | the rig build of one activation: the solver budget, the simulation with its safety ground, the bone poses the rig is built in (with the handoff velocities), the bodies with their shapes, mass, inertia and starting velocity, the startup lift, each joint's own constraints with the connected pairs and the soft-tissue leashes; it owns the set of soft-tissue bodies |

The rig builder is constructed with the components it calls as explicit dependencies (the table above). It creates
bodies and constraints in exactly the old order. The ragdoll's `InitializePhysics` runs its steps and does everything
between them that reads or records the host or another component's state: the skeleton read and the bone definitions,
the ground ray and the terrain patches (recorded NPC lists and positions), the contact callbacks (they collect the
tables of seven components), the mesh volumes and hull fits (recorded), the mesh status text, the surface cache, the
torso bodies, the bone index set, the animation freeze, the colliders of the other characters, the hair rig and the
start and end records; it changes the other components' state through their own methods (since A6-13). The handoff
sample is activation state; the ragdoll hands it in. One host read stays inside the rig builder, as it was before the
stage: `CreateJoints` reads the target's facing (`IHostActors.GetFacing`) for the anatomical frame. It is not recorded
there; the replay host answers it from the facing in the target's Identity record, which the rig build writes at its
start.

<!-- a6-11 -->
| Component | File | Owns |
| --- | --- | --- |
| `Stepper` | `Ragdoll.Stepping.cs` | the stepping loop's rules: the frame clock (the last frame stamp, the frame delta and its limits), the frame counter, the start of physics (counter, settle window, a clean accumulator), the fixed-step accumulator and the substep budget (the substep cap and the slow-frame limit), the previous-tick flag and the render blend, the settle countdown, the resting rule (the rig rests when BEPU has it asleep and nothing blocks the rest), the fall gravity and the velocity clamp |

Since A6-11 the context holds the fixed-step accumulator and the previous-tick flag (`PhysicsAccumulator`,
`HasPrevPhysicsState`) and the render interpolation the hair reads (`PhysicsInterpolation`). The ragdoll's step method
is a short ordered sequence: the start of the frame (liveness, skeleton read and record, rebuild check, animation
freeze), the skeleton root, the traversal carriers, the resting decision, the collapse controllers, the tracking of the
other characters' colliders, the physics steps, the containment and the settle countdown, the frame-end record, the
pose write-back and the follow target. The substep loop stays in the ragdoll because every step calls other components
and recorded methods (grab drive, mesh LOD, water, extensions, the solver step, the standing anchor, the surface, the
hair, impacts, the step record, the mesh early out); the stepper decides when a substep is due, captures and drops the
backlog and counts the substeps against the cap. The resting decision (`DecideResting`) stays in the ragdoll too: it
gathers what blocks the rest from eight components (the water, the external bodies, the hair, the traversal carriers,
the colliders, the grab, the guided collapse and the knee power-loss) and wakes and probes on the way, and only the rule
itself (`Stepper.IsResting`) is the stepper's. The fall gravity gets the water, the hold (a grab or the standing
support) and the ground height from the ragdoll. The collider tracking loop reads and records the host, so it stays in
the ragdoll as its own method.

<!-- a6-12 -->
Since A6-12 the replay's carried-state restore (`Ragdoll.Replay.cs`) and the activation's resets and teardown
(`Ragdoll.Activation.cs`) no longer write other components' fields through forwarding properties. Each component whose
fields the replay restores has its own `RestoreCarried` (the collapse, the knee power-loss, the skeleton reader, the
character colliders, the impacts, the grab, the external bodies, the weapon strike, the corpse traversal, the mesh
volumes; the hair rig already had `RestoreLiveHairState`), and the ragdoll calls them in the order of the old writes.
Activation, deactivation and the simulation teardown call small reset and teardown methods on the components
(`ResetForActivation`, `Disarm*`, `Clear*`, `Teardown`, `BumpGeneration`, `ResetLodInstall`), split where the old code
interleaved one component's writes with another's or with a recorded call (stopping a collapse pattern, then dropping
what was armed), so every write keeps its place. Known gap kept as it was: the teardown does not clear the external
bodies' carried rigs.

<!-- a6-13 -->
A6-13 finished the split:

- The ragdoll's own code writes no component field through a forwarding property any more: every write is a method of
  the owning component, called at the same point (the frame clock and the frame counter, the strike window and the
  blade sample, the armed collapse patterns, the hitstop, the ground heights and patch size, the corpse wake centre and
  the moved-collider note, the collider list, the mesh step timings, the mesh status and the rig's mesh-volume state,
  the active bone definitions, the bone index set and the head bone, the torso bodies, the soft-tissue body set, the
  carried rigs, the recorder's nesting depth and fit outcome). The component forwarders that are left are read-only.
- `CoreCalls` is gone. The hair rig reads the cached body surface from the mesh volumes it is handed; the frozen mesh
  capture moved into the mesh volumes, with the skeleton reader handed in. The weapon-contact surface stays a recorded
  call on the ragdoll (`GetWeaponContactMesh`, asked from outside by the weapon-drop simulation too): attaching a weapon
  is split into `WeaponContactSurfaces.PrepareAttach` (an `AttachStep`), the recorded call when the twins must be made,
  `CreateProxies` with its result, and `AddWeapon`, so the recorded call and its records come at the same point as
  before.
- The mesh volumes' carried fields are private with read-only properties (the recording reads them; the replay
  restores them through `RestoreCarried`).
- The frame-rate limits (`MaxSubstepsPerFrame`, `SlowFrameSeconds`, `MaxFrameDelta`) moved into the stepper, their only
  user.
- Unused forwarders and accessors were deleted (the impact cooldown and descent speed, the recoil relaxers, the
  standing support's three, the carried rigs, the traversal anchors and face landmarks, the bone index set, the bone
  configurations). `tests/ContactGolden` reads the watched state at its component path instead of through the ragdoll's
  forwarders; its output is unchanged.

### State after A6

The components (23) and what each owns are in the tables above; the shared state is in `SimulationContext`. What stays
on `Ragdoll`, and why:

- **The recorded layer** (`Ragdoll.Recording.cs` and the entry points in the other partial files, with the public
  facade over them in `Ragdoll.Facade.cs`): every internal and public entry point a host calls, and every recorded API
  call, with its records, around a `*Core` method. The records
  are the replay's input, so where a call is recorded, and what it records, is fixed; a component never makes one.
  The `Rec*` methods that write records from several components' state stay here too.
- **The activation flow** (`Ragdoll.Activation.cs`, `Ragdoll.Replay.cs`): activation and the start
  trigger, the handoff sample of the death animation, the animation freeze and its restore, deactivation and the
  simulation teardown, the deferred follow transform, the replay's carried-state restore. Each calls the components'
  reset, teardown and restore methods in the order of the old writes, between host reads and recorded calls.
- **The step sequence** (`Ragdoll.Stepping.cs`, `InitializePhysics` in `Ragdoll.RigBuild.cs`): the order in which a
  frame and a rig build call the components, the host and the recorded methods, with the steps that read or record the
  host or gather several components' state (the skeleton root and its ground ray, the resting decision, the collider
  tracking loop, the substep loop, the frame-pose write-back around the pose writer, the follow target; in the rig
  build, the contact callbacks, the terrain patches and the colliders of the other characters).
- **Stateless helpers and shared types**: the bone-class tests, profile defaults and bone-definition builder, bone
  lookup, the model/world conversions, the narrow-phase and pose-integrator callbacks, and the data types the
  components share.

Measured with the partial files' lines inside `Ragdoll` but outside the components, the context and `CoreCalls`: 6419
lines (4516 not blank or comment) before A6-13, 6234 (4363) after; the components hold 11161.
