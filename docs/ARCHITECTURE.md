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
