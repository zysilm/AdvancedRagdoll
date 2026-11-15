// This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
using System.Collections.Generic;
using System.Numerics;
using AdvancedRagdoll.Hosting;
using AdvancedRagdoll.MeshVolume;
using AdvancedRagdoll.SkeletonProfiles;
using BepuPhysics;
using BepuUtilities.Memory;
using BepuSimulation = BepuPhysics.Simulation;

namespace AdvancedRagdoll;

// The shared simulation context and the seam to the part of the class not yet split into components
// (docs/ARCHITECTURE.md §8).
public unsafe partial class Ragdoll
{
    /// <summary>The fixed physics step. One value for the stepping loop and every component.</summary>
    private const float FixedTimestep = 1f / 60f;

    /// <summary>
    /// The state many components read and write: the host services they use, the simulation, the ragdoll's bodies and
    /// the skeleton root transform. A plain object with fixed fields, created with the ragdoll and handed to each
    /// component in its constructor. The ragdoll's own code reaches the same fields through forwarding properties
    /// (Ragdoll.cs) until it is split into components too. Only what an extracted component needs lives here.
    /// </summary>
    private sealed class SimulationContext
    {
        internal SimulationContext(CoreCalls core, SkeletonProfile profile, RagdollBoneConfig[] boneDefaults,
            IHostSkeletonSource skeletons, IHostWorldQuery world, IMeshSource meshes, IHostActors actors,
            IRagdollClock clock, IRagdollLog log)
        {
            Core = core;
            Profile = profile;
            BoneDefaults = boneDefaults;
            Skeletons = skeletons;
            World = world;
            Meshes = meshes;
            Actors = actors;
            Clock = clock;
            Log = log;
        }

        /// <summary>Calls into the part of the ragdoll that is not a component yet.</summary>
        internal readonly CoreCalls Core;

        // Host services (the recording decorators where the ragdoll records them) and the settings of this frame.
        internal readonly HostSettings Settings = new();
        internal readonly IHostSkeletonSource Skeletons;
        internal readonly IHostWorldQuery World;
        internal readonly IMeshSource Meshes;
        internal readonly IHostActors Actors;
        internal readonly IRagdollClock Clock;
        internal readonly IRagdollLog Log;

        // This ragdoll's skeleton profile and the bone configurations built from it.
        internal readonly SkeletonProfile Profile;
        internal readonly RagdollBoneConfig[] BoneDefaults;

        // The simulation (null before physics is built and after it is torn down) and its buffer pool.
        internal BepuSimulation? Simulation;
        internal BufferPool? BufferPool;
        // The ragdoll's bodies, one per simulated bone.
        internal readonly List<RagdollBone> Bodies = new();
        // The character this ragdoll is for (an opaque host id; 0 = none).
        internal nint TargetCharacterAddress;
        // True while the active ragdoll was built by the generic (non-humanoid) path.
        internal bool IsGeneric;

        // Skeleton root transform: model space (the skeleton's pose space) to world space.
        internal Vector3 RootPosition;
        internal Quaternion RootRotation;
        internal Quaternion RootRotationInverse;
        internal Vector3 RootScale = Vector3.One;

        internal Vector3 ModelToWorld(Vector3 modelPos)
            => RootPosition + Vector3.Transform(modelPos * RootScale, RootRotation);

        internal Vector3 WorldToModel(Vector3 worldPos)
            => DivideComponents(Vector3.Transform(worldPos - RootPosition, RootRotationInverse), RootScale);

        internal Quaternion ModelRotToWorld(Quaternion modelRot)
            => Quaternion.Normalize(RootRotation * modelRot);

        internal Quaternion WorldRotToModel(Quaternion worldRot)
            => Quaternion.Normalize(RootRotationInverse * worldRot);

        /// <summary>A body's bone origin and rotation: the same body → bone-origin reconstruction as skeleton
        /// write-back.</summary>
        internal void GetBonePoseFromBody(RagdollBone rb, out Vector3 origin, out Quaternion rotation,
            out Vector3 bodyPosition, out BodyVelocity velocity)
        {
            var body = Simulation!.Bodies.GetBodyReference(rb.BodyHandle);
            bodyPosition = body.Pose.Position;
            velocity = body.Velocity;
            rotation = Quaternion.Normalize(body.Pose.Orientation * rb.CapsuleToBoneOffset);
            origin = bodyPosition - Vector3.Transform(rb.ShapeCenterOffset, body.Pose.Orientation);
            if (rb.SegmentHalfLength > 0)
                origin -= rb.SegmentHalfLength * Vector3.Transform(Vector3.UnitY, body.Pose.Orientation);
        }

        // a6-3
        // True from Activate until Deactivate (or until the ragdoll deactivates itself).
        internal bool IsActive;
        // What the rig actually weighs, summed when the rig is built.
        internal float TotalMass;

        /// <summary>The profile whose class tuning applies: the generated rig's, or this ragdoll's profile.</summary>
        internal SkeletonProfile ActiveClassProfile => IsGeneric ? SkeletonProfile.Auto() : Profile;

        /// <summary>The body of a bone, by name; null when the bone has none.</summary>
        internal BodyHandle? FindBodyHandle(string boneName)
        {
            foreach (var rb in Bodies)
                if (rb.Name == boneName) return rb.BodyHandle;
            return null;
        }

        // a6-4
        // True once BEPU has put the complete ragdoll constraint island to sleep. The engine owns
        // the decision; no independent ground test or forced velocity zeroing is layered on top.
        internal bool PrevAllAsleep;

        /// <summary>Wakes every body of the rig (the biomechanical settle keeps them awake).</summary>
        internal void WakeBodies()
        {
            if (Simulation == null) return;

            for (int i = 0; i < Bodies.Count; i++)
            {
                try
                {
                    var bodyRef = Simulation.Bodies.GetBodyReference(Bodies[i].BodyHandle);
                    bodyRef.Awake = true;
                }
                catch { }
            }
        }

        /// <summary>The character's facing (the profile's forward axis), flattened onto the ground plane.</summary>
        internal Vector3 ResolveForwardDirection()
            => FlatNormalize(Vector3.Transform(Profile.Forward, RootRotation), Profile.Forward);

        /// <summary>A bone's origin from its body's pose; false when the bone has no body.</summary>
        internal bool TryGetBoneOriginPosition(string boneName, out Vector3 position)
        {
            position = Vector3.Zero;
            if (Simulation == null) return false;
            RagdollBone? bone = null;
            foreach (var rb in Bodies)
            {
                if (rb.Name == boneName)
                {
                    bone = rb;
                    break;
                }
            }
            if (!bone.HasValue) return false;

            var body = Simulation.Bodies.GetBodyReference(bone.Value.BodyHandle);
            var yAxis = Vector3.Transform(Vector3.UnitY, body.Pose.Orientation);
            var profileOrigin = body.Pose.Position -
                                Vector3.Transform(bone.Value.ShapeCenterOffset, body.Pose.Orientation);
            position = profileOrigin - yAxis * bone.Value.SegmentHalfLength;
            return true;
        }

        // a6-5
        /// <summary>
        /// Remove every constraint still attached to <paramref name="body"/>. BEPU corrupts the solver if a
        /// body is removed while constraints reference it: the constraint keeps a now-dangling body index,
        /// and the next island-sleeper traversal (Solver.EnumerateConnectedBodyReferences) faults on it —
        /// far from the real cause. The debug assert that would catch this is compiled out in release, so
        /// every body removal must run this first. Idempotent: a body whose constraints are already gone
        /// enumerates an empty list.
        /// </summary>
        internal void RemoveConstraintsOnBody(BodyHandle body)
        {
            var simulation = Simulation;
            if (simulation == null) return;
            try
            {
                var bodyRef = simulation.Bodies.GetBodyReference(body);
                ref var attached = ref bodyRef.Constraints;
                if (attached.Count == 0) return;

                // Snapshot first — Solver.Remove mutates the body's constraint list as we go.
                var handles = new ConstraintHandle[attached.Count];
                for (var i = 0; i < attached.Count; i++)
                    handles[i] = attached[i].ConnectingConstraintHandle;

                foreach (var h in handles)
                {
                    try { simulation.Solver.Remove(h); }
                    catch (System.Exception ex)
                    {
                        Log.Warning(ex, $"Ragdoll: failed to remove constraint {h.Value} on body {body.Value}");
                    }
                }
            }
            catch (System.Exception ex)
            {
                Log.Warning(ex, $"Ragdoll: failed to enumerate constraints on body {body.Value}");
            }
        }

        // a6-6
        // True once physics has taken the body over from the death animation.
        internal bool PhysicsStarted;
        // Short post-wake window that keeps the island awake long enough for passive constraints and
        // contacts to resolve after grab/release or an external collision.
        internal float BiomechanicalSettleRemaining;

        /// <summary>The rig is no longer resting: keep it stepping for at least <paramref name="duration"/> seconds
        /// (the stepping loop counts the window down).</summary>
        internal void BeginBiomechanicalSettle(float duration = BiomechanicalSettleDuration)
        {
            BiomechanicalSettleRemaining = System.MathF.Max(BiomechanicalSettleRemaining, duration);
            PrevAllAsleep = false;
        }

        /// <summary>
        /// Read the scale that is actually applied to the rendered character. The NPC Scale effect
        /// writes the render scale directly, while ordinary actors normally expose the same
        /// value through Skeleton.Transform.Scale. Prefer the draw object so collision follows live
        /// live scale changes, and retain the skeleton value as a safe fallback.
        /// </summary>
        internal Vector3 GetCharacterScale(nint address, IHostSkeleton ns)
        {
            if (TryGetVisualScale(address, out var drawScale))
                return drawScale;

            return GetSkeletonScale(ns);
        }

        internal bool TryGetVisualScale(nint address, out Vector3 scale)
        {
            scale = Vector3.One;
            if (address != nint.Zero)
            {
                if (Actors.TryGetDrawScale(address, out scale))
                {
                    if (IsFinite(scale) && System.MathF.Abs(scale.X) >= 1e-6f && System.MathF.Abs(scale.Y) >= 1e-6f && System.MathF.Abs(scale.Z) >= 1e-6f)
                        return true;
                }
            }

            scale = Vector3.One;
            return false;
        }
    }

    /// <summary>
    /// What the components call in the part of the ragdoll that is not a component yet: recording, the mesh caches,
    /// constraint removal, the character scale and the stepping clock. Every member forwards to the ragdoll's own
    /// method unchanged; the list shrinks as those parts become components.
    /// </summary>
    private sealed class CoreCalls
    {
        private readonly Ragdoll owner;
        internal CoreCalls(Ragdoll owner) { this.owner = owner; }

        /// <summary>True while a recording session writes this activation.</summary>
        internal bool Recording => owner.rec != null;
        internal void RecMeshCache(FrozenMeshGeometryCache? cache, string use) => owner.RecMeshCache(cache, use);
        internal void RecMeshUse(string site, string bone, BodyMeshGeometry geometry) => owner.RecMeshUse(site, bone, geometry);
        internal void RecHairMesh(Hair.HairMeshBinding? binding) => owner.RecHairMesh(binding);

        internal FrozenMeshGeometryCache? GetWeaponContactMesh(nint address) => owner.GetWeaponContactMesh(address);
        internal FrozenMeshGeometryCache? CachedSurfaceFor(nint address) => owner.CachedSurfaceFor(address);

        /// <summary>How far the render frame is between the last two physics steps (1 before the first step).</summary>
        internal float PhysicsInterpolation =>
            owner.hasPrevPhysicsState ? System.Math.Clamp(owner.physicsAccumulator / FixedTimestep, 0, 1) : 1;

        // a6-2
        internal void RecMeshLod(string site, string bone, MeshCollisionLodResult lod) => owner.RecMeshLod(site, bone, lod);
        /// <summary>The recorded outcome of the structural fit (1 none, 2 not ready, 3 fitted, 4 failed).</summary>
        internal void RecFitOutcome(int outcome) => owner.recFitOutcome = outcome;
        internal FrozenMeshGeometryCache? CaptureFrozenMeshCache(nint address) => owner.CaptureFrozenMeshCache(address);
        internal bool TargetObjectAlive() => owner.TargetObjectAlive();
        internal bool RunAnimatedHair => owner.RunAnimatedHair;
    }
}
