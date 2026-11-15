// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System;
using System.Collections.Generic;
using System.Numerics;
using AdvancedRagdoll.SkeletonProfiles;
using AdvancedRagdoll.SurfaceProfiles;
using AdvancedRagdoll.Hosting;

namespace AdvancedRagdoll;

public unsafe partial class Ragdoll
{
    // The final collision-proxy surface (Ragdoll.Surface.cs).
    private readonly SurfaceRuntime surfaceRuntime;
    private CharacterSurfaceRuntime? characterSurfaceRuntime { get => surfaceRuntime.Runtime; set => surfaceRuntime.Runtime = value; }

    /// <summary>The resolved race/body profile used by this corpse, if it has a supported humanoid identity.</summary>
    internal string? ActiveCharacterSurfaceProfileName => characterSurfaceRuntime?.Profile.Name;

    /// <summary>Returns the lightweight profile surface nearest to a point on one named ragdoll bone.</summary>
    internal bool TryGetCharacterSurface(
        string boneName,
        Vector3 worldPoint,
        CharacterSurfaceUsage usage,
        out CharacterSurfaceHit hit)
    {
        hit = default;
        return characterSurfaceRuntime != null &&
               characterSurfaceRuntime.TryGetNearestSurface(boneName, worldPoint, usage, out hit);
    }

    /// <summary>Returns the lightweight profile surface nearest to a point on the whole ragdoll.</summary>
    internal bool TryGetCharacterSurface(
        Vector3 worldPoint,
        CharacterSurfaceUsage usage,
        float maxDistance,
        out CharacterSurfaceHit hit)
    {
        hit = default;
        return characterSurfaceRuntime != null &&
               characterSurfaceRuntime.TryGetNearestSurface(worldPoint, usage, maxDistance, out hit);
    }

    /// <summary>World-space profile triangles for developer visualization.</summary>
    internal IEnumerable<CharacterSurfaceDebugBone> GetCharacterSurfaceDebugBones(CharacterSurfaceUsage usage,
        int maximumFacesPerBone = 0, int maximumFacesTotal = 0)
        => characterSurfaceRuntime?.GetDebugBones(usage, maximumFacesPerBone, maximumFacesTotal) ?? Array.Empty<CharacterSurfaceDebugBone>();

    private CharacterSurfaceIdentity ReadCharacterSurfaceIdentity()
        => ReadCharacterSurfaceIdentity(targetCharacterAddress);

    private CharacterSurfaceIdentity ReadCharacterSurfaceIdentity(nint characterAddress)
        => ReadCharacterSurfaceIdentity(actors, characterAddress);

    /// <summary>The surface identity (race, body type, ...) of a character, read through the host.</summary>
    internal static CharacterSurfaceIdentity ReadCharacterSurfaceIdentity(IHostActors actors, nint characterAddress)
    {
        if (characterAddress == nint.Zero)
            return default;

        Span<byte> customize = stackalloc byte[0x1A];
        actors.ReadAppearance(characterAddress, customize, out var modelCharaId);
        return new CharacterSurfaceIdentity(
            customize[0x00], // race
            customize[0x01], // gender
            customize[0x04], // tribe
            customize[0x02], // body type
            customize[0x03], // height
            customize[0x15], // bust / muscle tone
            modelCharaId);
    }

    /// <summary>
    /// The lightweight query copy of the collision proxy the rig actually accepted, shared by traversal, grabbing and
    /// debug drawing; it follows the ragdoll's bodies. A component (docs/ARCHITECTURE.md §8).
    /// </summary>
    private sealed class SurfaceRuntime
    {
        private readonly SimulationContext ctx;
        internal SurfaceRuntime(SimulationContext ctx) { this.ctx = ctx; }

        // Lightweight final-collision surface cache shared by traversal, grabbing, debug drawing,
        // and the actual accepted per-bone collision shapes. It follows the
        // already-existing rigid bodies, so updating it is allocation-free and never touches model file data.
        internal CharacterSurfaceRuntime? Runtime;

        /// <summary>Builds the surface from the bodies just made; <paramref name="activeDefByName"/> is the rig's
        /// bone definitions by name.</summary>
        internal void Build(bool genericSkeleton, Dictionary<string, RagdollBoneDef> activeDefByName)
        {
            Runtime = null;
            if (genericSkeleton || ctx.Simulation == null || ctx.Bodies.Count == 0)
                return;

            var identity = ReadCharacterSurfaceIdentity(ctx.Actors, ctx.TargetCharacterAddress);
            if (!identity.IsHumanoid)
                return;

            // The runtime surface is a lightweight query copy of the collision proxy that was
            // actually accepted. Do not re-apply the old JSON race profile here: doing so would make
            // traversal/grab/debug disagree with physics even after the mesh measurement was valid.
            var profile = new CharacterSurfaceProfile
            {
                Id = "mesh-collision-runtime",
                Name = "Mesh-Derived Collision Proxy",
                Gender = byte.MaxValue,
                MinHeightScale = 1f,
                MaxHeightScale = 1f,
                Margins = new CharacterSurfaceMargins
                {
                    Physics = 1f,
                    Traversal = 1f,
                    Grab = 1f,
                    Ground = 1f,
                },
                DefaultScale = new CharacterSurfaceRoleScales
                {
                    Width = 1f,
                    Depth = 1f,
                    Length = 1f,
                    EndOverlap = 0f,
                    TaperStart = 1f,
                    TaperMiddle = 1f,
                    TaperEnd = 1f,
                },
            };

            var seeds = new CharacterSurfaceBoneSeed[ctx.Bodies.Count];
            for (var i = 0; i < ctx.Bodies.Count; i++)
            {
                var rb = ctx.Bodies[i];
                var radiusX = rb.ColliderShape == RagdollColliderShape.Box
                    ? rb.BoxHalfExtents.X
                    : rb.CapsuleRadius;
                var radiusZ = rb.ColliderShape == RagdollColliderShape.Box
                    ? rb.BoxHalfExtents.Z
                    : rb.CapsuleRadius;
                var halfLength = rb.ColliderShape == RagdollColliderShape.Box
                    ? rb.BoxHalfExtents.Y
                    : rb.CapsuleHalfLength;
                var role = activeDefByName.TryGetValue(rb.Name, out var def)
                    ? def.AnatomicalRole.ToString()
                    : SkeletonBoneRole.Generic.ToString();
                seeds[i] = new CharacterSurfaceBoneSeed(
                    rb.Name, role, halfLength, radiusX, radiusZ,
                    rb.ColliderShape == RagdollColliderShape.Box,
                    rb.MeshGeometry);
            }

            Runtime = new CharacterSurfaceRuntime(profile, identity, seeds);
            Update();
            ctx.Log.Info($"Ragdoll: final collision-proxy surface cached for {ctx.Bodies.Count} bones.");
        }

        internal void Update()
        {
            if (Runtime == null || ctx.Simulation == null)
                return;

            foreach (var rb in ctx.Bodies)
            {
                var body = ctx.Simulation.Bodies.GetBodyReference(rb.BodyHandle);
                Runtime.UpdateBonePose(
                    rb.Name, body.Pose.Position, body.Pose.Orientation);
            }
        }
    }
}
