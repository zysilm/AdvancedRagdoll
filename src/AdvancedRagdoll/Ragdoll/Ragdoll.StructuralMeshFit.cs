// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;
using BepuPhysics.Collidables;
using AdvancedRagdoll.Hosting;

namespace AdvancedRagdoll;

public unsafe partial class Ragdoll
{
    // The torso and thighs collide with a hull fitted to the character's own surface: the body and
    // gear models it is actually drawn with, mods included, in their reference pose. The fit owns
    // collision geometry only; skeleton topology and joint anchors stay on the bone pivots.
    //
    // Reading a dense body takes tens of milliseconds, so the fit is started when the ragdoll is
    // armed and computed on a worker; the death animation's delay is ample time. One appearance is
    // fitted once and shared by every ragdoll that wears it.

    internal sealed record StructuralFitResult(Dictionary<string, StructuralMeshFitter.Fit> Fits, string Summary);

    private static readonly Dictionary<string, StructuralMeshFitter.Fit> NoStructuralFits = new(StringComparer.Ordinal);

    // The fitted collision hulls (Ragdoll.StructuralMeshFit.cs).
    private readonly StructuralFit structuralFit;

    /// <summary>
    /// The collision hulls fitted to the target's own models: the fit started when the ragdoll is armed (the mesh
    /// source reads the models and fits them on a worker), collected when the rig is built, and the hull shapes made
    /// from it. A component (docs/ARCHITECTURE.md §8).
    /// </summary>
    private sealed class StructuralFit
    {
        private readonly SimulationContext ctx;
        private Task<StructuralFitResult>? structuralFitTask;

        internal StructuralFit(SimulationContext ctx) { this.ctx = ctx; }

        /// <summary>Starts fitting the target's collision hulls, if that has not been done for its appearance.</summary>
        internal void Prewarm()
        {
            structuralFitTask = null;
            try
            {
                var skel = ctx.Skeletons.TryGetSkeleton(ctx.TargetCharacterAddress);
                if (skel != null)
                    structuralFitTask = Start(skel);
            }
            catch (Exception ex)
            {
                ctx.Log.Warning(ex, "Ragdoll mesh geometry: could not start the fit; built-in volumes will be used.");
            }
        }

        /// <summary>The fitted volumes for the target, or none: every body then keeps its built-in volume.
        /// <paramref name="scheduler"/> is the ragdoll's scheduler, which waits for the fit.</summary>
        internal Dictionary<string, StructuralMeshFitter.Fit> Resolve(IHostSkeleton skel, IRagdollScheduler scheduler)
        {
            try
            {
                var task = structuralFitTask ?? Start(skel);
                structuralFitTask = null;
                if (task == null)
                {
                    if (ctx.Core.Recording) ctx.Core.RecFitOutcome(1);
                    return NoStructuralFits;
                }

                // Normally long finished. With no activation delay the first death of an appearance
                // waits here once, briefly; past that the built-in volumes are better than a stall.
                if (!scheduler.TryWait(task, 150, out var result))
                {
                    ctx.Log.Info("Ragdoll mesh geometry: fit not ready yet; built-in volumes used for this ragdoll.");
                    if (ctx.Core.Recording) ctx.Core.RecFitOutcome(2);
                    return NoStructuralFits;
                }
                if (ctx.Core.Recording) ctx.Core.RecFitOutcome(3);
                ctx.Log.Info($"Ragdoll mesh geometry: {result.Summary}");
                return result.Fits;
            }
            catch (Exception ex)
            {
                ctx.Log.Warning(ex, "Ragdoll mesh geometry: fit failed; built-in volumes used.");
                if (ctx.Core.Recording) ctx.Core.RecFitOutcome(4);
                return NoStructuralFits;
            }
        }

        /// <summary>Starts the hull fit: the mesh source reads the target's models now and fits them on a worker.</summary>
        private Task<StructuralFitResult>? Start(IHostSkeleton skel)
            => ctx.Meshes.StartStructuralFit(ctx.TargetCharacterAddress, skel);

        internal bool TryCreateHull(Vector3[] hullPoints, float scale, out ConvexHull hull, out Vector3 center,
            out Vector3[] corners, out Vector3[] edges)
        {
            hull = default;
            center = default;
            corners = Array.Empty<Vector3>();
            edges = Array.Empty<Vector3>();
            if (ctx.BufferPool == null)
                return false;
            try
            {
                return StructuralHullShape.TryCreate(hullPoints, scale, ctx.BufferPool, out hull, out center, out corners, out edges);
            }
            catch (Exception ex)
            {
                ctx.Log.Warning(ex, "Ragdoll mesh geometry: hull construction failed; envelope used.");
                return false;
            }
        }
    }

    internal static bool TryBuildReferenceModelTransforms(
        IHostSkeleton skel,
        out Matrix4x4[] referenceModel)
    {
        referenceModel = Array.Empty<Matrix4x4>();
        var reference = skel.ReferencePose;
        if (reference.IsEmpty)
            return false;

        var count = Math.Min(skel.BoneCount,
            Math.Min(skel.ParentCount, reference.Length));
        if (count <= 0)
            return false;

        referenceModel = new Matrix4x4[count];
        for (var i = 0; i < count; i++)
        {
            var local = QsToMatrix(reference[i]);
            var parent = skel.Parent(i);
            referenceModel[i] = parent >= 0 && parent < i
                ? local * referenceModel[parent]
                : local;
        }
        return true;
    }
}
