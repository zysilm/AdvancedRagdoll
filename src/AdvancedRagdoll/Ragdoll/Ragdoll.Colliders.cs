// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using AdvancedRagdoll.Recording;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using AdvancedRagdoll.Hosting;
using BepuSimulation = BepuPhysics.Simulation;

namespace AdvancedRagdoll;

public unsafe partial class Ragdoll
{
    // Per-bone kinematic collision body for an NPC (dynamically created from skeleton)
    private struct NpcBoneStatic
    {
        public BodyHandle Handle;
        public TypedIndex ShapeIndex;
        public int BoneIndex;           // this bone's skeleton index
        public int ParentBoneIndex;     // parent bone for segment direction
        public float BaseHalfLength;    // unscaled half-length of the capsule body segment
        public float BaseRadius;        // unscaled capsule radius
        public float CenterFactor;      // parent->child fraction where the capsule is centered
        public string BoneName;
        public Vector3 ShapeCenterOffset;
        public Vector3 PreviousPosition;
        public Quaternion PreviousOrientation;
        public bool HasPreviousPose;
        public float AppliedCollisionScale;
    }

    // Default capsule radius for dynamically discovered bones
    private const float NpcDefaultBoneRadius = 0.04f;
    private const float NpcAutoMinRadius = 0.025f;
    private const float NpcAutoMaxRadius = 0.55f;
    // Minimum segment length to create a collision capsule. Raised from 0.02 to drop the
    // dense cluster of tiny face/finger/hair segments — they don't meaningfully block a
    // falling ragdoll but each one costs a per-frame reposition + UpdateBounds.
    private const float NpcMinSegmentLength = 0.05f;
    // Hard cap on collision capsules per character: keep the longest segments (torso,
    // limbs, head) and drop the rest. Bounds the per-frame static-update cost so a wave
    // of high-bone-count enemies can't add tens of thousands of UpdateBounds calls.
    private const int NpcMaxCollisionSegments = 24;
    private const int NpcLightweightCollisionSegments = 3;
    private const int MeshCollisionMaxTriangles = 8000;
    // A character whose root is farther than this (metres) from the corpse is skipped in
    // the per-frame static update — it cannot contact the ragdoll, so tracking its bones
    // is wasted work. It still resumes updating if it moves back into range.
    private const float NpcCollisionUpdateRadius = 8f;
    // Within that radius, a character is tracked only while its collision could touch the corpse:
    // its root no farther from the corpse's bounds than the reach its capsules have shown, times
    // this (a swing reaches farther than a stance), plus the margin. Everyone else is parked.
    private const float NpcReachAllowance = 1.5f;
    private const float NpcReachMargin = 1.0f;

    // Other characters' colliders (Ragdoll.Colliders.cs).
    private readonly CharacterColliders characterColliders;

    // NPC bone collision — per-bone static capsules for active targets
    private List<NpcCollisionState> npcCollisionStates => characterColliders.States;
    // Swarm followers keep their allocated bone proxies so detail can be restored without removing
    // bodies from a live BEPU simulation, but only a small representative subset is active.
    private HashSet<nint> lightweightNpcCollisionAddresses => characterColliders.LightweightAddresses;
    // Kinematic proxies driven by living NPC/player skeletons (the narrow phase's soft NPC material).
    private HashSet<int> npcCollisionKinematicBodyHandles => characterColliders.KinematicBodyHandles;
    // Proxies of auto-traversing swarm actors: live geometric probes, no contacts with this ragdoll.
    private HashSet<int> npcTraversalGhostKinematicBodyHandles => characterColliders.TraversalGhostBodyHandles;
    // Set when an NPC collider genuinely moved near the corpse this frame; consumed by the NEXT
    // frame's resting decision.
    private bool npcColliderMovedNearCorpse => characterColliders.MovedNearCorpse;

    private void RefreshCorpseBounds() => characterColliders.RefreshCorpseBounds();

    private bool NpcMayTouchCorpse(in NpcCollisionState state, Vector3 root)
        => characterColliders.NpcMayTouchCorpse(state, root);

    // Per-NPC collision state (bone-based, skinned mesh, or single-capsule fallback)
    private struct NpcCollisionState
    {
        public nint NpcAddress;
        public List<NpcBoneStatic> BoneStatics;   // populated when skeleton readable
        public BodyHandle FallbackHandle;          // used when skeleton unreadable
        public bool IsFallback;
        public bool IsMesh;
        public BodyHandle MeshHandle;
        public TypedIndex MeshShapeIndex;
        public Vector3 PreviousPosition;
        public Quaternion PreviousOrientation;
        public bool HasPreviousPose;
        public float AppliedCollisionScale;
        // Root transform that produced the currently stored proxy poses. Traversal queries run
        // from Framework.Update while proxy poses are refreshed from the render hook, so the live
        // GameObject root can already belong to the next frame. Keeping the matching sample root
        // prevents that phase difference from being interpreted as a change in corpse height.
        public Vector3 TraversalSampleRoot;
        public bool HasTraversalSampleRoot;
        public bool Lightweight;
        public bool LightweightExtrasParked;
        // The follow set occupies the first NpcLightweightCollisionSegments entries. A distinct
        // hand/claw/head proxy may temporarily replace the last follow proxy during a real strike.
        public int LightweightAttackProxyIndex;
        public bool LightweightAttackMode;
        // True while this character's statics have been parked far away because it left the
        // update radius. Prevents leaving them frozen on the corpse ("ghost" capsules) and
        // avoids re-parking every frame; cleared when it re-enters range.
        public bool Parked;
        // How far from its root this character's collision has been seen to reach (metres, the
        // largest over the frames it was tracked). 0 until it has been tracked once.
        public float Reach;
        public TypedIndex FallbackShapeIndex;
        public float FallbackBaseRadius;
        public float FallbackBaseLength;
    }

    /// <summary>
    /// Build collision proxies for a live character (enemy, companion, or player).
    /// Discovers bones from the character's skeleton and creates one kinematic capsule
    /// per parent→child segment. Falls back to a single capsule when the skeleton
    /// can't be read. Works for any model (humanoid, monster, dragon).
    /// </summary>
    private void BuildCharacterCollision(nint address, string label, float scale, float capsuleRadius)
    {
        var ns = skeletons.TryGetSkeleton(address, PosePurpose.ColliderBuild);
        if (rec != null) RecSkeleton(RecordType.SkeletonPose, PosePurpose.ColliderBuild, address, ns, false);
        if (ns == null)
        {
            CreateFallbackCharacterCollision(label, address);
            return;
        }

        if (!ns.HasRoot)
        {
            CreateFallbackCharacterCollision(label, address);
            return;
        }

        var npcRoot = ns.Root;
        var npcSkelPos = npcRoot.Position;
        var npcSkelRot = npcRoot.Rotation;
        var npcVisualScale = GetCharacterScale(address, ns);
        var npcShapeScale = CollisionScale(npcVisualScale);

        if (settings.RagdollNpcCollisionMode == RagdollNpcCollisionMode.Mesh &&
            BuildMeshCollision(address, ns, npcSkelPos, npcSkelRot, label))
            return;

        if (!characterColliders.BuildBoneCollision(address, label, scale, capsuleRadius, ns,
                npcSkelPos, npcSkelRot, npcVisualScale, npcShapeScale))
        {
            CreateFallbackCharacterCollision(label, address);
        }
    }

    private static bool IsNpcBoneProxyActive(in NpcCollisionState state, int boneIndex, bool attackMode)
    {
        if (!state.Lightweight)
            return true;
        if (boneIndex < 0 || boneIndex >= state.BoneStatics.Count)
            return false;

        var followCount = Math.Min(NpcLightweightCollisionSegments, state.BoneStatics.Count);
        if (!attackMode || state.LightweightAttackProxyIndex < followCount ||
            state.LightweightAttackProxyIndex >= state.BoneStatics.Count)
            return boneIndex < followCount;

        // Preserve the two structural/support proxies and replace only the third follow proxy.
        return boneIndex < Math.Max(1, followCount - 1) || boneIndex == state.LightweightAttackProxyIndex;
    }

    private struct NpcAutoCollisionContext
    {
        public float SkeletonRadius;
        public float[] NearestBoneDistances;
    }

    private bool BuildMeshCollision(nint address, IHostSkeleton ns, Vector3 npcSkelPos, Quaternion npcSkelRot, string label)
    {
        if (simulation == null || bufferPool == null)
            return false;

        try
        {
            // The character's skinned models as drawn (the mesh source reads them).
            var triangles = new List<Triangle>(Math.Min(MeshCollisionMaxTriangles, 1024));
            if (!meshes.TryCollectActorTriangles(address, ns, label, settings.RagdollVerboseLog, triangles,
                    out var loadedModels, out var processedMeshes))
            {
                log.Warning($"Ragdoll: {label} mesh collision skipped; skin transforms unavailable");
                return false;
            }

            if (triangles.Count < 4)
            {
                log.Warning($"Ragdoll: {label} mesh collision produced {triangles.Count} triangles from {loadedModels} models; using bone capsules");
                return false;
            }

            if (TryCreateMeshShape(triangles, GetCharacterScale(address, ns), out var shapeIndex))
            {
                var rootRot = Quaternion.Normalize(npcSkelRot);
                var bodyHandle = simulation.Bodies.Add(BodyDescription.CreateKinematic(
                    new RigidPose(npcSkelPos, rootRot),
                    default(BodyVelocity),
                    new CollidableDescription(shapeIndex, 0.04f),
                    new BodyActivityDescription(0.01f)));
                characterColliders.AddKinematicBody(bodyHandle.Value);

                characterColliders.AddState(new NpcCollisionState
                {
                    NpcAddress = address,
                    BoneStatics = new List<NpcBoneStatic>(),
                    IsFallback = false,
                    IsMesh = true,
                    MeshHandle = bodyHandle,
                    MeshShapeIndex = shapeIndex,
                    PreviousPosition = npcSkelPos,
                    PreviousOrientation = rootRot,
                    HasPreviousPose = true,
                });

                var meshScale = GetCharacterScale(address, ns);
                log.Info($"Ragdoll: {label} mesh collision - {triangles.Count} triangles from {loadedModels} models/{processedMeshes} meshes, rootScale=({meshScale.X:F3},{meshScale.Y:F3},{meshScale.Z:F3})");
                return true;
            }

            log.Warning($"Ragdoll: {label} mesh collision failed to create BEPU mesh shape; using bone capsules");
            return false;
        }
        catch (Exception ex)
        {
            log.Warning(ex, $"Ragdoll: {label} mesh collision build failed; using bone capsules");
            return false;
        }
    }

    private bool TryCreateMeshShape(List<Triangle> triangles, Vector3 scale, out TypedIndex shapeIndex)
    {
        shapeIndex = default;
        if (simulation == null || bufferPool == null || triangles.Count < 4)
            return false;
        if (rec != null) RecTriangles("actorMesh", triangles, scale);

        Buffer<Triangle> triangleBuffer = default;
        var ownsTriangleBuffer = false;
        try
        {
            bufferPool.Take<Triangle>(triangles.Count, out triangleBuffer);
            ownsTriangleBuffer = true;
            for (int i = 0; i < triangles.Count; i++)
                triangleBuffer[i] = triangles[i];

            var meshShape = new BepuPhysics.Collidables.Mesh(triangleBuffer, scale, bufferPool);
            ownsTriangleBuffer = false;
            shapeIndex = simulation.Shapes.Add(meshShape);
            return true;
        }
        catch
        {
            return false;
        }
        finally
        {
            if (ownsTriangleBuffer)
                bufferPool.Return(ref triangleBuffer);
        }
    }

    private void CreateFallbackCharacterCollision(string label, nint address)
    {
        var visualScale = TryGetVisualScale(address, out var drawScale) ? drawScale : Vector3.One;
        var collisionScale = CollisionScale(visualScale);
        var baseRadius = settings.RagdollNpcCollisionAutoSize ? 0.35f : 0.3f * settings.RagdollNpcCollisionScale;
        var baseLength = settings.RagdollNpcCollisionAutoSize ? 1.2f : MathF.Max(0.2f, 1.6f - baseRadius * 2f);
        var shapeIndex = simulation!.Shapes.Add(new Capsule(baseRadius * collisionScale, baseLength * collisionScale));
        var goPosition = actors.GetPosition(address, ActorSite.ColliderFallbackBuild);
        var npcPos = new Vector3(goPosition.X, goPosition.Y + 0.8f * visualScale.Y, goPosition.Z);
        if (rec != null) RecActorTransform(ActorSite.ColliderFallbackBuild, address, goPosition, Quaternion.Identity, true, visualScale);
        var handle = simulation!.Bodies.Add(BodyDescription.CreateKinematic(
            new RigidPose(npcPos, Quaternion.Identity),
            default(BodyVelocity),
            new CollidableDescription(shapeIndex, 0.04f),
            new BodyActivityDescription(0.01f)));
        characterColliders.AddKinematicBody(handle.Value);
        characterColliders.AddState(new NpcCollisionState
        {
            NpcAddress = address,
            BoneStatics = new List<NpcBoneStatic>(),
            FallbackHandle = handle,
            IsFallback = true,
            FallbackShapeIndex = shapeIndex,
            FallbackBaseRadius = baseRadius,
            FallbackBaseLength = baseLength,
            AppliedCollisionScale = collisionScale,
            TraversalSampleRoot = goPosition,
            HasTraversalSampleRoot = true,
            PreviousPosition = npcPos,
            PreviousOrientation = Quaternion.Identity,
            HasPreviousPose = true,
        });
        log.Info($"Ragdoll: {label} using fallback single capsule");
    }

    /// <summary>Move all of a character's collision proxies to a far-away park position
    /// so they stop colliding with the ragdoll.</summary>
    private void ParkNpcStatics(ref NpcCollisionState npcState, Vector3 parkPos, float dt = FixedTimestep)
        => characterColliders.ParkNpcStatics(ref npcState, parkPos, dt);

    // Parking: a collider that is out of play is sent far below the world. Each one goes to its own
    // spot there, spaced apart: parked at one shared point, every parked capsule of every character
    // overlapped every other, and the broad phase produced (and the callback rejected) each of those
    // pairs on every substep. Measured offline on one corpse with 40 characters' capsules: 32.7 ms a
    // frame at one point, 0.3 ms spread (scratchpad parkbench); in game 20 such corpses ran at 1-2 fps.
    private const float ParkingDepth = -9000f;
    private const float ParkingSpacing = 8f;

    /// <summary>The target as given, or, when it is the parking depth, this body's own spot there.</summary>
    private static Vector3 ParkingSpot(Vector3 target, int key)
        => target.Y > ParkingDepth
            ? target
            : new Vector3((key & 255) * ParkingSpacing, target.Y, ((key >> 8) & 1023) * ParkingSpacing);

    private void MoveNpcKinematic(ref NpcCollisionState state, BodyHandle handle,
        Vector3 targetPosition, Quaternion targetOrientation, float dt)
        => characterColliders.MoveNpcKinematic(ref state, handle, targetPosition, targetOrientation, dt);

    private void MoveNpcBoneKinematic(ref NpcBoneStatic bone, Vector3 targetPosition, Quaternion targetOrientation, float dt)
        => characterColliders.MoveNpcBoneKinematic(ref bone, targetPosition, targetOrientation, dt);

    private void RescaleNpcBoneCollision(ref NpcCollisionState state, float collisionScale)
        => characterColliders.RescaleNpcBoneCollision(ref state, collisionScale);

    private void RescaleNpcFallbackCollision(ref NpcCollisionState state, float collisionScale)
        => characterColliders.RescaleNpcFallbackCollision(ref state, collisionScale);

    private static void SoftenAttackContactVelocity(BodyReference body)
    {
        body.Velocity.Linear = ClampVectorLength(body.Velocity.Linear, 0.55f);
        body.Velocity.Angular = ClampVectorLength(body.Velocity.Angular, 2f);
    }

    /// <summary>
    /// Register a live character as a moving collider in the ragdoll simulation AFTER activation
    /// (the per-bone collision built at init only captures party/NPC actors that existed then).
    /// Used by Enemy Control so the creature physically pushes the ragdoll when it walks into it,
    /// not just on an explicit attack. Returns false if the sim isn't ready yet — callers should
    /// retry until it succeeds. Pair with <see cref="RemoveLiveCollider"/> before the actor despawns.
    /// </summary>
    private bool AddLiveColliderCore(nint address)
    {
        if (simulation == null || !isActive || !physicsStarted) return false;
        if (address == nint.Zero || address == targetCharacterAddress) return false;
        // Already a collision actor (e.g. the controlled killer was a selected NPC captured when the
        // ragdoll activated) — its bone capsules already track it; just mark it as the strike collider
        // so the bone/weapon strike activates. Without this, the killer's strike never fired.
        foreach (var s in npcCollisionStates)
            if (s.NpcAddress == address) { strike.SetColliderAddress(address); return true; }
        // Need a readable skeleton so we take the bone-capsule (or hull) path, not the fallback.
        if (skeletons.TryGetSkeleton(address) == null) return false;

        var scale = settings.RagdollNpcCollisionScale;
        var capsuleRadius = settings.RagdollNpcCollisionAutoSize ? NpcDefaultBoneRadius : NpcDefaultBoneRadius * scale;
        var before = npcCollisionStates.Count;
        BuildCharacterCollision(address, "monster", scale, capsuleRadius);
        if (npcCollisionStates.Count <= before) return false;
        strike.SetColliderAddress(address); // this collider's bones drive the attack strike
        return true;
    }

    /// <summary>Use only a representative prefix of an actor's already allocated bone proxies.
    /// Intended for large swarms where full-detail collision would create hundreds of simultaneous
    /// contacts. Changing the mode never adds or removes a BEPU body from the running simulation.</summary>
    private void SetLiveColliderLightweightCore(nint address, bool lightweight)
        => characterColliders.SetLightweight(address, lightweight);

    /// <summary>
    /// Keep a live NPC's proxy bodies available to traversal queries while excluding their
    /// dynamic-ragdoll contacts. This is the one-way coupling used by auto-navigation: the corpse
    /// determines the follower root, but a root correction cannot feed an infinite-mass kinematic
    /// impulse back into that same corpse. No bodies are added/removed from the running simulation.
    /// </summary>
    private void SetLiveColliderTraversalGhostCore(nint address, bool ghost)
        => characterColliders.SetTraversalGhost(address, ghost);

    /// <summary>Remove a live collider registered via <see cref="AddLiveCollider"/> (parks its
    /// statics far away and drops the per-frame tracking entry so a despawned actor isn't read).</summary>
    private void RemoveLiveColliderCore(nint address)
    {
        characterColliders.RemoveLightweight(address);
        SetLiveColliderTraversalGhost(address, false);
        ClearNpcTraversalSupport(address);
        if (simulation == null) return;
        strike.RemoveColliderAddress(address);
        for (int i = npcCollisionStates.Count - 1; i >= 0; i--)
        {
            if (npcCollisionStates[i].NpcAddress != address) continue;
            var npcState = npcCollisionStates[i];
            ParkNpcStatics(ref npcState, new Vector3(0, -9999, 0));
            characterColliders.RemoveStateAt(i);
        }
    }

    /// <summary>
    /// The colliders of the other characters the corpse collides with: their per-character state (bone capsules, a
    /// skinned mesh or one fallback capsule), the kinematic body sets the narrow phase reads for them, the corpse
    /// bounds that decide who is tracked, and whether a collider moved near the corpse this frame. The bone-capsule
    /// build, the kinematic moves (parking, rescaling) and the lightweight and traversal-ghost switches live here.
    /// The recorded reads (the skeleton at build, the actor position of a fallback capsule, the triangles of a mesh
    /// collider) and the per-frame tracking loop of the stepping stay in the ragdoll. A component
    /// (docs/ARCHITECTURE.md §8).
    /// </summary>
    private sealed class CharacterColliders
    {
        private readonly SimulationContext ctx;
        internal CharacterColliders(SimulationContext ctx) { this.ctx = ctx; }

        private BepuSimulation? simulation => ctx.Simulation;
        private BufferPool? bufferPool => ctx.BufferPool;
        private List<RagdollBone> ragdollBones => ctx.Bodies;
        private HostSettings settings => ctx.Settings;
        private IRagdollLog log => ctx.Log;
        private Vector3 skelWorldPos => ctx.RootPosition;

        // NPC bone collision — per-bone static capsules for active targets
        private readonly List<NpcCollisionState> npcCollisionStates = new();
        // Swarm followers keep their allocated bone proxies so detail can be restored without removing
        // bodies from a live BEPU simulation, but only a small representative subset is active. This
        // bounds the number of constraints that can converge on one corpse body when a large pack crowds
        // around it (and avoids BEPU 2.5 beta's SequentialFallbackBatch removal bug).
        private readonly HashSet<nint> lightweightNpcCollisionAddresses = new();
        // Kinematic proxies driven by living NPC/player skeletons. When corpse traversal is enabled,
        // the narrow phase gives only these contacts the soft, low-friction material that prevents a
        // walking character from punting the corpse sideways.
        private readonly HashSet<int> npcCollisionKinematicBodyHandles = new();
        // Auto-traversing swarm actors are root-driven from corpse support queries. Their proxy poses
        // remain live as geometric probes, but contacts with this ragdoll are disabled to break the
        // unstable kinematic-NPC -> dynamic-corpse -> root-correction feedback loop.
        private readonly HashSet<int> npcTraversalGhostKinematicBodyHandles = new();

        // Set when an NPC collider genuinely moved (beyond the kinematic deadband) near the
        // corpse this frame; consumed by the NEXT frame's resting decision (one frame of latency
        // is fine — stepping resumes before the contact needs solving).
        private bool npcColliderMovedNearCorpse;
        // Rough corpse centre for the collider proximity gate (the stepping loop sets it).
        private Vector3 corpseWakeCenter;

        // World bounds of this corpse's own bodies, refreshed once per frame before colliders move.
        private Vector3 corpseBoundsMin;
        private Vector3 corpseBoundsMax;
        private bool hasCorpseBounds;

        internal List<NpcCollisionState> States => npcCollisionStates;
        internal HashSet<nint> LightweightAddresses => lightweightNpcCollisionAddresses;
        internal HashSet<int> KinematicBodyHandles => npcCollisionKinematicBodyHandles;
        internal HashSet<int> TraversalGhostBodyHandles => npcTraversalGhostKinematicBodyHandles;
        internal bool MovedNearCorpse => npcColliderMovedNearCorpse;

        /// <summary>A new collider: its kinematic body (the narrow phase's soft NPC material), then its state.</summary>
        internal void AddKinematicBody(int bodyHandle) => npcCollisionKinematicBodyHandles.Add(bodyHandle);
        internal void AddState(NpcCollisionState state) => npcCollisionStates.Add(state);
        /// <summary>The tracking loop writes a collider's state back.</summary>
        internal void SetState(int index, NpcCollisionState state) => npcCollisionStates[index] = state;
        internal void RemoveStateAt(int index) => npcCollisionStates.RemoveAt(index);
        internal void RemoveLightweight(nint address) => lightweightNpcCollisionAddresses.Remove(address);
        /// <summary>The stepping loop's rough corpse centre for the proximity gate.</summary>
        internal void SetWakeCenter(Vector3 center) => corpseWakeCenter = center;
        /// <summary>This frame's resting decision has read it; this frame's collider updates re-arm it.</summary>
        internal void ClearMovedNearCorpse() => npcColliderMovedNearCorpse = false;

        internal void RefreshCorpseBounds()
        {
            hasCorpseBounds = false;
            if (simulation == null || ragdollBones.Count == 0)
                return;

            var min = new Vector3(float.MaxValue);
            var max = new Vector3(float.MinValue);
            foreach (var rb in ragdollBones)
            {
                var box = simulation.Bodies.GetBodyReference(rb.BodyHandle).BoundingBox;
                min = Vector3.Min(min, box.Min);
                max = Vector3.Max(max, box.Max);
            }
            if (!(min.X <= max.X) || !float.IsFinite(min.X + min.Y + min.Z + max.X + max.Y + max.Z))
                return;

            corpseBoundsMin = min;
            corpseBoundsMax = max;
            hasCorpseBounds = true;
        }

        /// <summary>How far a point lies outside the corpse's bounds (0 inside).</summary>
        private float DistanceToCorpseBounds(Vector3 point)
        {
            var outside = Vector3.Max(Vector3.Zero, Vector3.Max(corpseBoundsMin - point, point - corpseBoundsMax));
            return outside.Length();
        }

        /// <summary>
        /// Whether this character's collision could touch the corpse this frame, so is worth tracking.
        /// Unknown reach (never tracked) falls back to the plain update radius.
        /// </summary>
        internal bool NpcMayTouchCorpse(in NpcCollisionState state, Vector3 root)
        {
            if ((root - skelWorldPos).LengthSquared() > NpcCollisionUpdateRadius * NpcCollisionUpdateRadius)
                return false;
            if (!hasCorpseBounds || state.Reach <= 0f || state.IsMesh || state.IsFallback)
                return true;
            return DistanceToCorpseBounds(root) <= state.Reach * NpcReachAllowance + NpcReachMargin;
        }

        /// <summary>
        /// The bone-capsule collision of a character whose skeleton was read (and recorded) by the ragdoll: one
        /// kinematic capsule per kept parent→child segment. False when no segment qualified (the ragdoll then
        /// builds the fallback capsule).
        /// </summary>
        internal bool BuildBoneCollision(nint address, string label, float scale, float capsuleRadius,
            IHostSkeleton ns, Vector3 npcSkelPos, Quaternion npcSkelRot, Vector3 npcVisualScale, float npcShapeScale)
        {
            var boneStatics = new List<NpcBoneStatic>();
            var autoSize = settings.RagdollNpcCollisionAutoSize;
            var profileRadii = autoSize ? BuildHumanoidCollisionRadiusMap(ns) : null;
            var autoContext = autoSize ? BuildNpcAutoCollisionContext(ns) : default;

            // Pass 1: collect every qualifying parent→child segment with its length so we can
            // keep only the longest NpcMaxCollisionSegments (the big body-blocking segments)
            // rather than building a static for every twig.
            var candidates = new List<(float SegLen, int BoneIdx, int ParentIdx, string Name, string Role,
                float HalfLen, float Radius, float CenterFactor, Vector3 Center, Quaternion Rot)>();
            var boneCount = Math.Min(ns.BoneCount, ns.ParentCount);
            for (int i = 1; i < boneCount; i++) // skip root (index 0, no parent)
            {
                var parentIdx = ns.Parent(i);
                if (parentIdx < 0 || parentIdx >= ns.BoneCount) continue;

                // ModelPose translations are unscaled. Apply the live draw-object scale both to
                // their world positions and to the capsule dimensions built from the segment.
                ref var parentMt = ref ns.ModelPose[parentIdx];
                var parentModelPos = new Vector3(parentMt.Translation.X, parentMt.Translation.Y, parentMt.Translation.Z);
                var parentWorldPos = NpcModelToWorld(parentModelPos, npcSkelPos, npcSkelRot, npcVisualScale);

                ref var childMt = ref ns.ModelPose[i];
                var childModelPos = new Vector3(childMt.Translation.X, childMt.Translation.Y, childMt.Translation.Z);
                var childWorldPos = NpcModelToWorld(childModelPos, npcSkelPos, npcSkelRot, npcVisualScale);

                var segment = childWorldPos - parentWorldPos;
                var segLen = segment.Length();
                var modelSegLen = Vector3.Distance(parentModelPos, childModelPos);
                if (modelSegLen < NpcMinSegmentLength) continue; // skip tiny segments (face, fingers)

                var baseRadius = autoSize
                    ? EstimateNpcCollisionRadius(i, parentIdx, modelSegLen, profileRadii, autoContext)
                    : capsuleRadius;
                var baseHalfLen = autoSize ? MathF.Max(0.01f, (modelSegLen * 0.5f) - baseRadius) : modelSegLen * 0.45f * scale;
                var centerFactor = autoSize ? 0.5f : baseHalfLen / modelSegLen;
                var segDir = segment / segLen;
                var boneName = ns.BoneName(i) ?? $"bone_{i}";
                var role = InferAnatomicalRole(ctx.Profile, boneName, null, false).ToString();
                candidates.Add((modelSegLen, i, parentIdx, boneName, role, baseHalfLen,
                    baseRadius, centerFactor, parentWorldPos + (segLen * centerFactor) * segDir, RotationFromYToDirection(segment)));
            }

            // Put three stable locomotion proxies first: the longest body axis, the lowest support
            // segment, and the best torso/second-longest segment. Keeping a hand, claw, mouth or head in
            // the always-on follow set made a dense swarm lead with that endpoint and shove the corpse
            // sideways. The best attack endpoint is retained immediately after the follow prefix and is
            // swapped in only during the short physical-strike window.
            var lightweightAttackProxyIndex = -1;
            if (candidates.Count > 0)
            {
                var selectedIndices = new List<int>();
                var longestIndex = 0;
                var attackIndex = 0;
                var supportIndex = 0;
                var torsoIndex = 0;
                for (var i = 1; i < candidates.Count; i++)
                {
                    if (candidates[i].SegLen > candidates[longestIndex].SegLen)
                        longestIndex = i;
                    var attackScore = LightweightCollisionRoleScore(candidates[i].Name);
                    var bestAttackScore = LightweightCollisionRoleScore(candidates[attackIndex].Name);
                    if (attackScore > bestAttackScore ||
                        (attackScore == bestAttackScore && candidates[i].SegLen > candidates[attackIndex].SegLen))
                        attackIndex = i;
                    if (candidates[i].Center.Y < candidates[supportIndex].Center.Y)
                        supportIndex = i;
                    var torsoScore = LightweightFollowRoleScore(candidates[i].Name);
                    var bestTorsoScore = LightweightFollowRoleScore(candidates[torsoIndex].Name);
                    if (torsoScore > bestTorsoScore ||
                        (torsoScore == bestTorsoScore && candidates[i].SegLen > candidates[torsoIndex].SegLen))
                        torsoIndex = i;
                }

                selectedIndices.Add(longestIndex);
                if (!selectedIndices.Contains(supportIndex))
                    selectedIndices.Add(supportIndex);
                if (!selectedIndices.Contains(torsoIndex))
                    selectedIndices.Add(torsoIndex);
                foreach (var indexed in candidates.Select((candidate, index) => (candidate, index))
                             .OrderByDescending(x => x.candidate.SegLen))
                {
                    if (selectedIndices.Count >= NpcLightweightCollisionSegments)
                        break;
                    if (!selectedIndices.Contains(indexed.index))
                        selectedIndices.Add(indexed.index);
                }

                var selectedSet = selectedIndices.ToHashSet();
                var ordered = new List<(float SegLen, int BoneIdx, int ParentIdx, string Name, string Role,
                    float HalfLen, float Radius, float CenterFactor, Vector3 Center, Quaternion Rot)>();
                foreach (var index in selectedIndices)
                    ordered.Add(candidates[index]);
                if (!selectedSet.Contains(attackIndex))
                {
                    lightweightAttackProxyIndex = ordered.Count;
                    ordered.Add(candidates[attackIndex]);
                    selectedSet.Add(attackIndex);
                }
                else
                {
                    lightweightAttackProxyIndex = selectedIndices.IndexOf(attackIndex);
                }
                ordered.AddRange(candidates.Select((candidate, index) => (candidate, index))
                    .Where(x => !selectedSet.Contains(x.index))
                    .OrderByDescending(x => x.candidate.SegLen)
                    .Select(x => x.candidate));
                candidates.Clear();
                candidates.AddRange(ordered.Take(NpcMaxCollisionSegments));
            }

            // Pass 2: create the kinematic capsule for each kept segment. When lightweight mode was
            // requested before construction, create inactive proxies parked from the outset so there is
            // never even a one-frame full-detail contact burst in an already running simulation.
            var lightweight = lightweightNpcCollisionAddresses.Contains(address);
            for (var candidateIndex = 0; candidateIndex < candidates.Count; candidateIndex++)
            {
                var c = candidates[candidateIndex];
                var shapeCenterOffset = Vector3.Zero;
                var shapeIndex = simulation!.Shapes.Add(new Capsule(
                    c.Radius * npcShapeScale, c.HalfLen * npcShapeScale * 2f));
                var inactiveLightweightProxy = lightweight && candidateIndex >= NpcLightweightCollisionSegments;
                var bodyCenter = inactiveLightweightProxy
                    ? new Vector3(0, -9999, 0)
                    : c.Center + Vector3.Transform(shapeCenterOffset, c.Rot);
                var bodyRotation = inactiveLightweightProxy ? Quaternion.Identity : c.Rot;
                var bodyHandle = simulation!.Bodies.Add(BodyDescription.CreateKinematic(
                    new RigidPose(bodyCenter, bodyRotation),
                    default(BodyVelocity),
                    new CollidableDescription(shapeIndex, 0.04f),
                    new BodyActivityDescription(0.01f)));
                npcCollisionKinematicBodyHandles.Add(bodyHandle.Value);
                if (inactiveLightweightProxy)
                {
                    // Born parked: at its own spot, not on top of every other parked capsule.
                    bodyCenter = ParkingSpot(bodyCenter, bodyHandle.Value);
                    simulation.Bodies.GetBodyReference(bodyHandle).Pose.Position = bodyCenter;
                }
                boneStatics.Add(new NpcBoneStatic
                {
                    Handle = bodyHandle,
                    ShapeIndex = shapeIndex,
                    BoneIndex = c.BoneIdx,
                    ParentBoneIndex = c.ParentIdx,
                    BaseHalfLength = c.HalfLen,
                    BaseRadius = c.Radius,
                    CenterFactor = c.CenterFactor,
                    BoneName = c.Name,
                    ShapeCenterOffset = shapeCenterOffset,
                    PreviousPosition = bodyCenter,
                    PreviousOrientation = bodyRotation,
                    HasPreviousPose = true,
                    AppliedCollisionScale = npcShapeScale,
                });
            }

            if (boneStatics.Count > 0)
            {
                npcCollisionStates.Add(new NpcCollisionState
                {
                    NpcAddress = address,
                    BoneStatics = boneStatics,
                    IsFallback = false,
                    TraversalSampleRoot = npcSkelPos,
                    HasTraversalSampleRoot = true,
                    Lightweight = lightweight,
                    LightweightExtrasParked = lightweight,
                    LightweightAttackProxyIndex = lightweightAttackProxyIndex,
                });
                var activeSegments = lightweight
                    ? Math.Min(NpcLightweightCollisionSegments, boneStatics.Count)
                    : boneStatics.Count;
                var detail = activeSegments == boneStatics.Count ? string.Empty : $", {activeSegments} active";
                log.Info($"Ragdoll: {label} bone collision — {boneStatics.Count} segments{detail} from {boneCount} bones");
                return true;
            }
            return false;
        }

        // Which of another character's bones it attacks with, and which its light collision body follows:
        // scored by name fragments, profile data (SkeletonProfile.LightCollisionAttackScores / FollowScores).
        private int LightweightCollisionRoleScore(string boneName)
            => ctx.Profile.LightCollisionAttackScores.Score(boneName);

        private int LightweightFollowRoleScore(string boneName)
            => ctx.Profile.LightCollisionFollowScores.Score(boneName);

        private Dictionary<int, float> BuildHumanoidCollisionRadiusMap(IHostSkeleton skel)
        {
            var result = new Dictionary<int, float>();
            foreach (var signatureBone in ctx.Profile.SignatureBones)
                if (ResolveBoneIndex(skel, signatureBone) < 0)
                    return result;

            foreach (var def in BuildBoneDefsFromConfigs(ctx.Profile, ctx.BoneDefaults, ctx.BoneDefaults))
            {
                var index = ResolveBoneIndex(skel, def.Name);
                if (index < 0)
                    continue;

                var radius = def.CapsuleRadius;
                if (def.ColliderShape == RagdollColliderShape.Box)
                {
                    var extents = ResolveBoxHalfExtents(def, ResolveBodyHalfLength(def));
                    radius = MathF.Max(extents.X, extents.Z);
                }

                result[index] = Math.Clamp(radius, NpcAutoMinRadius, NpcAutoMaxRadius);
            }

            return result;
        }

        private static NpcAutoCollisionContext BuildNpcAutoCollisionContext(IHostSkeleton skel)
        {
            var boneCount = Math.Min(skel.BoneCount, skel.ParentCount);
            var positions = new Vector3[boneCount];
            var min = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
            var max = new Vector3(float.MinValue, float.MinValue, float.MinValue);

            for (int i = 0; i < boneCount; i++)
            {
                ref var mt = ref skel.ModelPose[i];
                var model = new Vector3(mt.Translation.X, mt.Translation.Y, mt.Translation.Z);
                positions[i] = model;
                min = Vector3.Min(min, model);
                max = Vector3.Max(max, model);
            }

            var nearest = new float[boneCount];
            Array.Fill(nearest, float.MaxValue);
            for (int i = 0; i < boneCount; i++)
            {
                for (int j = i + 1; j < boneCount; j++)
                {
                    var d = Vector3.Distance(positions[i], positions[j]);
                    if (d <= 0.001f)
                        continue;
                    if (d < nearest[i]) nearest[i] = d;
                    if (d < nearest[j]) nearest[j] = d;
                }
            }

            var extents = max - min;
            var skeletonRadius = Math.Clamp(MathF.Max(extents.X, extents.Z) * 0.08f, NpcAutoMinRadius, NpcAutoMaxRadius);
            return new NpcAutoCollisionContext
            {
                SkeletonRadius = skeletonRadius,
                NearestBoneDistances = nearest,
            };
        }

        private static float EstimateNpcCollisionRadius(
            int boneIndex,
            int parentIndex,
            float segmentLength,
            Dictionary<int, float>? profileRadii,
            NpcAutoCollisionContext context)
        {
            if (profileRadii != null && profileRadii.TryGetValue(boneIndex, out var profileRadius))
                return Math.Clamp(MathF.Min(profileRadius, segmentLength * 0.45f), NpcAutoMinRadius, NpcAutoMaxRadius);

            var childNearest = boneIndex >= 0 && boneIndex < context.NearestBoneDistances.Length
                ? context.NearestBoneDistances[boneIndex]
                : float.MaxValue;
            var parentNearest = parentIndex >= 0 && parentIndex < context.NearestBoneDistances.Length
                ? context.NearestBoneDistances[parentIndex]
                : float.MaxValue;
            var nearest = MathF.Min(childNearest, parentNearest);
            var densityRadius = float.IsFinite(nearest) ? nearest * 0.42f : context.SkeletonRadius;
            var segmentRadius = segmentLength * 0.22f;
            var radius = MathF.Max(context.SkeletonRadius, MathF.Max(densityRadius, segmentRadius));
            return Math.Clamp(MathF.Min(radius, segmentLength * 0.7f), NpcAutoMinRadius, NpcAutoMaxRadius);
        }

        /// <summary>Move all of a character's collision proxies to a far-away park position
        /// so they stop colliding with the ragdoll.</summary>
        internal void ParkNpcStatics(ref NpcCollisionState npcState, Vector3 parkPos, float dt)
        {
            if (simulation == null) return;
            npcState.HasTraversalSampleRoot = false;
            if (npcState.IsMesh)
            {
                MoveNpcKinematic(ref npcState, npcState.MeshHandle, parkPos, Quaternion.Identity, dt);
                return;
            }
            if (npcState.IsFallback)
            {
                MoveNpcKinematic(ref npcState, npcState.FallbackHandle, parkPos, Quaternion.Identity, dt);
                return;
            }
            for (int b = 0; b < npcState.BoneStatics.Count; b++)
            {
                var bs = npcState.BoneStatics[b];
                MoveNpcBoneKinematic(ref bs, parkPos, Quaternion.Identity, dt);
                npcState.BoneStatics[b] = bs;
            }
        }

        internal void MoveNpcKinematic(ref NpcCollisionState state, BodyHandle handle,
            Vector3 targetPosition, Quaternion targetOrientation, float dt)
        {
            if (simulation == null) return;
            targetPosition = ParkingSpot(targetPosition, handle.Value);
            var body = simulation.Bodies.GetBodyReference(handle);
            if (MoveKinematicBody(body, targetPosition, targetOrientation,
                    ref state.PreviousPosition, ref state.PreviousOrientation, ref state.HasPreviousPose, dt))
            {
                SoftenNpcTraversalKinematicVelocity(body);
                NoteNpcColliderMoved(targetPosition);
            }
        }

        // A collider that genuinely moved only matters to the corpse's rest if it moved NEAR the
        // corpse — a monster breathing five yalms away must not hold fifty bodies awake. Within
        // range, real collider motion blocks the resting fast-path, so the next contact is stepped.
        private const float NpcColliderWakeRadiusSq = 2.5f * 2.5f;

        // A collider within this of the corpse's bounds (beyond its own extent) could touch it now.
        private const float NpcWakeContactMargin = 0.1f;

        /// <summary>
        /// A collider moved. It keeps the corpse awake only if it could touch it: with the corpse's
        /// bounds known and the collider's extent given, only when it is within that extent of them. A
        /// crowd fighting over a body no longer holds it awake while nobody is touching it.
        /// </summary>
        private void NoteNpcColliderMoved(Vector3 colliderPos, float extent)
        {
            if (hasCorpseBounds && extent > 0f)
            {
                if (DistanceToCorpseBounds(colliderPos) <= extent + NpcWakeContactMargin)
                    npcColliderMovedNearCorpse = true;
                return;
            }
            NoteNpcColliderMoved(colliderPos);
        }

        private void NoteNpcColliderMoved(Vector3 colliderPos)
        {
            if (Vector3.DistanceSquared(colliderPos, corpseWakeCenter) < NpcColliderWakeRadiusSq)
                npcColliderMovedNearCorpse = true;
        }

        internal void MoveNpcBoneKinematic(ref NpcBoneStatic bone, Vector3 targetPosition, Quaternion targetOrientation, float dt)
        {
            if (simulation == null) return;
            targetPosition = ParkingSpot(targetPosition, bone.Handle.Value);
            var body = simulation.Bodies.GetBodyReference(bone.Handle);
            if (MoveKinematicBody(body, targetPosition, targetOrientation,
                    ref bone.PreviousPosition, ref bone.PreviousOrientation, ref bone.HasPreviousPose, dt))
            {
                SoftenNpcTraversalKinematicVelocity(body);
                var scale = bone.AppliedCollisionScale > 0f ? bone.AppliedCollisionScale : 1f;
                NoteNpcColliderMoved(targetPosition, (bone.BaseHalfLength + bone.BaseRadius) * scale);
            }
        }

        internal void RescaleNpcBoneCollision(ref NpcCollisionState state, float collisionScale)
        {
            if (simulation == null || bufferPool == null)
                return;

            for (var i = 0; i < state.BoneStatics.Count; i++)
            {
                var bone = state.BoneStatics[i];
                if (MathF.Abs(bone.AppliedCollisionScale - collisionScale) <= 0.0001f)
                    continue;

                var newCenterOffset = Vector3.Zero;
                var newShape = simulation.Shapes.Add(new Capsule(
                    MathF.Max(0.0001f, bone.BaseRadius * collisionScale),
                    MathF.Max(0.0001f, bone.BaseHalfLength * collisionScale * 2f)));
                var oldShape = bone.ShapeIndex;
                var body = simulation.Bodies.GetBodyReference(bone.Handle);
                var logicalCenter = body.Pose.Position -
                                    Vector3.Transform(bone.ShapeCenterOffset, body.Pose.Orientation);
                simulation.Bodies.SetShape(bone.Handle, newShape);
                body.Pose.Position = logicalCenter +
                                     Vector3.Transform(newCenterOffset, body.Pose.Orientation);
                bone.ShapeIndex = newShape;
                bone.ShapeCenterOffset = newCenterOffset;
                bone.AppliedCollisionScale = collisionScale;
                bone.PreviousPosition = body.Pose.Position;
                state.BoneStatics[i] = bone;
                try { simulation.Shapes.RemoveAndDispose(oldShape, bufferPool); } catch { }
            }
        }

        internal void RescaleNpcFallbackCollision(ref NpcCollisionState state, float collisionScale)
        {
            if (simulation == null || bufferPool == null ||
                MathF.Abs(state.AppliedCollisionScale - collisionScale) <= 0.0001f)
                return;

            var newShape = simulation.Shapes.Add(new Capsule(
                MathF.Max(0.0001f, state.FallbackBaseRadius * collisionScale),
                MathF.Max(0.0001f, state.FallbackBaseLength * collisionScale)));
            var oldShape = state.FallbackShapeIndex;
            simulation.Bodies.SetShape(state.FallbackHandle, newShape);
            state.FallbackShapeIndex = newShape;
            state.AppliedCollisionScale = collisionScale;
            try { simulation.Shapes.RemoveAndDispose(oldShape, bufferPool); } catch { }
        }

        /// <summary>
        /// Kinematics have infinite mass, so their full locomotion velocity would be imposed on a
        /// corpse even with a soft contact spring. The visual proxy still follows the living actor's
        /// exact pose; only the velocity presented to the contact solver is capped. Explicit monster
        /// attacks remain forceful because BeginAttackStrike applies their measured limb velocity via
        /// the separate strike impulse path after this update.
        /// </summary>
        private static void SoftenNpcTraversalKinematicVelocity(BodyReference body)
        {
            const float maxHorizontalSpeed = 0.85f;
            const float maxDownwardSpeed = 0.75f;
            const float maxUpwardSpeed = 2.5f;
            const float maxAngularSpeed = 3f;

            var linear = body.Velocity.Linear;
            var horizontal = new Vector2(linear.X, linear.Z);
            var horizontalLength = horizontal.Length();
            if (horizontalLength > maxHorizontalSpeed && horizontalLength > 1e-5f)
                horizontal *= maxHorizontalSpeed / horizontalLength;

            body.Velocity.Linear = new Vector3(
                horizontal.X,
                Math.Clamp(linear.Y, -maxDownwardSpeed, maxUpwardSpeed),
                horizontal.Y);
            body.Velocity.Angular = ClampVectorLength(body.Velocity.Angular, maxAngularSpeed);
        }

        /// <summary>
        /// Speed past which a kinematic's move is a teleport, not motion.
        ///
        /// Collision proxies get parked kilometres below the world and snapped back again — the grab
        /// suspends the grabber's proxies, and out-of-range culling parks anyone who walks away. Finite-
        /// differencing THAT into a velocity gives six figures of m/s, which the solver then faithfully
        /// applies to whatever the proxy reappears inside, firing the corpse into the sky. The velocity
        /// clamp downstream cannot save it: that runs after Timestep, by which point the position has
        /// already been integrated across the substeps.
        ///
        /// No real bone travels anywhere near this fast, so anything above it is a teleport: place the
        /// body and give it no velocity at all.
        /// </summary>
        private const float KinematicTeleportSpeed = 60f; // m/s

        // Movement deadband for kinematic drives. Below these, a collider is "not moving": pose
        // snaps but velocity stays zero and the body is not woken — an idle NPC's breathing sway
        // stops pumping energy into everything it touches, which is what let the corpse island
        // finish its sleep countdown. The previous pose is deliberately NOT advanced inside the
        // band, so a slow creep accumulates against the anchor and eventually registers as real
        // motion instead of hiding under the band one frame at a time.
        private const float KinematicDeadbandPosSq = 1e-6f;              // (1 mm)²
        private const float KinematicDeadbandOrientationDot = 0.9999875f; // ≈0.6° of rotation

        /// <summary>Returns true when the body actually moved (beyond the deadband) this call.</summary>
        private static bool MoveKinematicBody(BodyReference body, Vector3 targetPosition, Quaternion targetOrientation,
            ref Vector3 previousPosition, ref Quaternion previousOrientation, ref bool hasPreviousPose, float dt)
        {
            targetOrientation = Quaternion.Normalize(targetOrientation);

            if (hasPreviousPose && dt > 1e-5f &&
                Vector3.DistanceSquared(targetPosition, previousPosition) < KinematicDeadbandPosSq &&
                MathF.Abs(Quaternion.Dot(targetOrientation, previousOrientation)) > KinematicDeadbandOrientationDot)
            {
                body.Pose.Position = targetPosition;
                body.Pose.Orientation = targetOrientation;
                body.Velocity.Linear = Vector3.Zero;
                body.Velocity.Angular = Vector3.Zero;
                // Awake untouched, anchor pose untouched (see the deadband comment above).
                return false;
            }

            var teleported = hasPreviousPose && dt > 1e-5f &&
                             Vector3.Distance(targetPosition, previousPosition) > KinematicTeleportSpeed * dt;

            var linearVelocity = hasPreviousPose && !teleported && dt > 1e-5f
                ? (targetPosition - previousPosition) / dt
                : Vector3.Zero;
            var angularVelocity = hasPreviousPose && !teleported && dt > 1e-5f
                ? AngularVelocityFromQuats(previousOrientation, targetOrientation, dt)
                : Vector3.Zero;

            body.Pose.Position = targetPosition;
            body.Pose.Orientation = targetOrientation;
            body.Velocity.Linear = linearVelocity;
            body.Velocity.Angular = angularVelocity;
            body.Awake = true;

            previousPosition = targetPosition;
            previousOrientation = targetOrientation;
            hasPreviousPose = true;
            return true;
        }

        /// <summary>Use only a representative prefix of an actor's already allocated bone proxies.
        /// Changing the mode never adds or removes a BEPU body from the running simulation.</summary>
        internal void SetLightweight(nint address, bool lightweight)
        {
            if (address == nint.Zero) return;
            if (lightweight)
                lightweightNpcCollisionAddresses.Add(address);
            else
                lightweightNpcCollisionAddresses.Remove(address);

            for (var i = 0; i < npcCollisionStates.Count; i++)
            {
                if (npcCollisionStates[i].NpcAddress != address) continue;
                var state = npcCollisionStates[i];
                state.Lightweight = lightweight;
                // The render-frame update parks or restores the correct range on its next safe pass.
                state.LightweightExtrasParked = false;
                npcCollisionStates[i] = state;
            }
        }

        /// <summary>Keep a live NPC's proxy bodies available to traversal queries while excluding their
        /// dynamic-ragdoll contacts. No bodies are added/removed from the running simulation.</summary>
        internal void SetTraversalGhost(nint address, bool ghost)
        {
            if (address == nint.Zero)
                return;

            foreach (var state in npcCollisionStates)
            {
                if (state.NpcAddress != address)
                    continue;

                void SetHandle(BodyHandle handle)
                {
                    if (ghost)
                        npcTraversalGhostKinematicBodyHandles.Add(handle.Value);
                    else
                        npcTraversalGhostKinematicBodyHandles.Remove(handle.Value);
                }

                if (state.IsMesh)
                    SetHandle(state.MeshHandle);
                else if (state.IsFallback)
                    SetHandle(state.FallbackHandle);
                else
                    foreach (var bone in state.BoneStatics)
                        SetHandle(bone.Handle);
            }
        }

        // a6-12
        /// <summary>The replay puts back the lightweight set earlier activations left.</summary>
        internal void RestoreCarried(List<nint> lightweightColliders)
        {
            lightweightNpcCollisionAddresses.Clear();
            foreach (var address in lightweightColliders) lightweightNpcCollisionAddresses.Add(address);
        }

        /// <summary>Activation: no collider has moved near the corpse yet.</summary>
        internal void ResetForActivation() => npcColliderMovedNearCorpse = false;

        /// <summary>Simulation teardown: the two kinematic body sets the narrow phase reads.</summary>
        internal void ClearKinematicBodySets()
        {
            npcCollisionKinematicBodyHandles.Clear();
            npcTraversalGhostKinematicBodyHandles.Clear();
        }

        /// <summary>Simulation teardown: the colliders of the other characters.</summary>
        internal void ClearStates() => npcCollisionStates.Clear();
    }
}
