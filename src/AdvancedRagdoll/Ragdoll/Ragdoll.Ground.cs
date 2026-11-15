// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System;
using System.Collections.Generic;
using System.Numerics;
using AdvancedRagdoll.MeshVolume;
using AdvancedRagdoll.Recording;
using BepuPhysics;
using BepuPhysics.Collidables;

namespace AdvancedRagdoll;

public unsafe partial class Ragdoll
{
    // The ground under the corpse (Ragdoll.Ground.cs).
    private readonly Ground ground;

    private float terrainPatchRadius { get => ground.PatchRadius; set => ground.PatchRadius = value; }
    private float terrainPatchStep { get => ground.PatchStep; set => ground.PatchStep = value; }
    private float terrainPatchRefreshDistance { get => ground.PatchRefreshDistance; set => ground.PatchRefreshDistance = value; }
    // Ground height (physics ground may be lowered by floor offset)
    private float groundY { get => ground.GroundY; set => ground.GroundY = value; }
    // Real terrain ground (before floor offset), used for visual correction
    private float realGroundY { get => ground.RealGroundY; set => ground.RealGroundY = value; }

    private void AddTerrainPatch(float centerX, float centerZ, float defaultGroundY)
        => ground.AddTerrainPatch(centerX, centerZ, defaultGroundY, targetIsLocalPlayer, activeRagdollMeshVolumes);

    private void EnsureTerrainPatchCoverage(Vector3[] worldPositions, bool[] boneValid, int boneCount)
        => ground.EnsureCoverage(worldPositions, boneValid, boneCount, nHaraIndex, targetIsLocalPlayer, activeRagdollMeshVolumes);

    /// <summary>
    /// The ground under the corpse: its height, the sampled ground patches (and the game's own collision triangles under
    /// the player's body) added as statics, and more patches as the body travels. A component (docs/ARCHITECTURE.md §8).
    /// </summary>
    private sealed class Ground
    {
        private const float ExactGroundSafetyNetDrop = 0.25f; // how far the sampled grid sits below exact ground
        private const float ExactGroundVerticalReach = 8f;    // triangles this far above/below the patch height are taken
        private const int ExactGroundMaxTriangles = 20000;

        private readonly SimulationContext ctx;
        private readonly List<Vector3> exactGroundScratch = new();

        internal Ground(SimulationContext ctx) { this.ctx = ctx; }

        // Ground patch size. Set per ragdoll from how far its body reaches (CollisionMeshGeometry.PatchFor):
        // 6 m / 1 m samples for a human, larger for a large monster.
        internal float PatchRadius = 6.0f;
        internal float PatchStep = 1.0f;
        internal float PatchRefreshDistance = 4.0f;

        // Ground height (physics ground may be lowered by floor offset)
        internal float GroundY;
        // Real terrain ground (before floor offset), used for visual correction
        internal float RealGroundY;
        internal readonly List<Vector2> PatchCenters = new();

        /// <summary>
        /// Add the game's collision triangles around (centerX, centerZ) as static ground. Both windings of
        /// every triangle are added, since the game's meshes do not promise which way a face points. False
        /// when the scene could not be read, had nothing there, or had too much.
        /// </summary>
        private bool TryAddExactGround(float centerX, float centerZ, float patchGroundY)
        {
            var simulation = ctx.Simulation;
            var bufferPool = ctx.BufferPool;
            if (simulation == null || bufferPool == null) return false;

            exactGroundScratch.Clear();
            var min = new Vector3(centerX - PatchRadius, patchGroundY - ExactGroundVerticalReach, centerZ - PatchRadius);
            var max = new Vector3(centerX + PatchRadius, patchGroundY + ExactGroundVerticalReach, centerZ + PatchRadius);
            // A larger patch may hold proportionally more triangles, up to four times the base budget.
            var budget = (int)MathF.Min(ExactGroundMaxTriangles * 4f,
                ExactGroundMaxTriangles * (PatchRadius / 6f) * (PatchRadius / 6f));
            if (!ctx.World.TryCollectGroundTriangles(min, max, exactGroundScratch, budget) ||
                exactGroundScratch.Count < 3)
            {
                ctx.Log.Info("Ragdoll: exact ground unavailable for this patch; using the sampled grid");
                return false;
            }

            var count = exactGroundScratch.Count / 3;
            bufferPool.Take<Triangle>(count * 2, out var triangles);
            var written = 0;
            for (var i = 0; i < count; i++)
            {
                var a = exactGroundScratch[i * 3];
                var b = exactGroundScratch[i * 3 + 1];
                var c = exactGroundScratch[i * 3 + 2];
                if (!float.IsFinite(a.LengthSquared() + b.LengthSquared() + c.LengthSquared())) continue;
                if (Vector3.Cross(b - a, c - a).LengthSquared() < 1e-10f) continue; // degenerate
                triangles[written++] = new Triangle(a, b, c);
                triangles[written++] = new Triangle(a, c, b);
            }
            exactGroundScratch.Clear();
            if (written == 0)
            {
                bufferPool.Return(ref triangles);
                return false;
            }

            var mesh = new BepuPhysics.Collidables.Mesh(triangles.Slice(written), Vector3.One, bufferPool);
            simulation.Statics.Add(new StaticDescription(Vector3.Zero, Quaternion.Identity, simulation.Shapes.Add(mesh)));
            ctx.Log.Info($"Ragdoll: exact ground patch with {written / 2} triangles, radius {PatchRadius:F1} m");
            return true;
        }

        /// <summary>
        /// Lay one terrain collision patch centered on (centerX, centerZ): raycast
        /// a grid of ground heights and add it to the simulation as a static mesh. Used
        /// for both the player's death spot and nearby enemies so a grabbed-and-released
        /// ragdoll always has ground beneath it. <paramref name="defaultGroundY"/> is the
        /// fallback height for the center raycast and any grid ray that misses.
        /// <paramref name="targetIsLocalPlayer"/> and <paramref name="meshVolumes"/> are the ragdoll's own
        /// (the local player's body; the active rig collides through mesh volumes).
        /// </summary>
        internal void AddTerrainPatch(float centerX, float centerZ, float defaultGroundY, bool targetIsLocalPlayer,
            bool meshVolumes)
        {
            var simulation = ctx.Simulation;
            var bufferPool = ctx.BufferPool;
            if (simulation == null || bufferPool == null)
                return;

            var center = new Vector2(centerX, centerZ);
            foreach (var existing in PatchCenters)
                if (Vector2.DistanceSquared(existing, center) < 1.0f)
                    return;

            if (PatchCenters.Count >= MaxTerrainPatches)
                return;

            int gridSize = (int)(PatchRadius * 2 / PatchStep) + 1;

            // Ground estimate at the patch center — used as the ray origin height and the
            // miss fallback, so a patch under an enemy on different ground still works.
            float patchGroundY = defaultGroundY;
            if (ctx.World.Raycast(RaycastSite.PatchCenter,
                    new Vector3(centerX, defaultGroundY + TerrainRaycastStartYOffset, centerZ),
                    new Vector3(0, -1, 0), TerrainRaycastDistance, out var centerHit))
                patchGroundY = centerHit.Point.Y;

            // Exact ground: the game's own collision triangles for this patch. The sampled grid below is then
            // only a safety net under it, lowered so it never holds a body up where the grid is coarser than
            // the real ground. Only the player's own body gets it: building the mesh costs 2-17 ms a patch,
            // every ragdoll has its own simulation, and a wave of dying enemies would pay that once each.
            var gridDrop = ctx.Settings.RagdollExactGroundCollision && targetIsLocalPlayer &&
                           !(meshVolumes && !ctx.IsGeneric) &&
                           TryAddExactGround(centerX, centerZ, patchGroundY)
                ? ExactGroundSafetyNetDrop
                : 0f;

            var heights = new float[gridSize, gridSize];
            var ox = centerX - PatchRadius;
            var oz = centerZ - PatchRadius;
            for (int gz = 0; gz < gridSize; gz++)
            for (int gx = 0; gx < gridSize; gx++)
            {
                var wx = ox + gx * PatchStep;
                var wz = oz + gz * PatchStep;
                if (ctx.World.Raycast(RaycastSite.PatchGrid,
                        new Vector3(wx, patchGroundY + TerrainRaycastStartYOffset, wz),
                        new Vector3(0, -1, 0), TerrainRaycastDistance, out var gridHit))
                    heights[gx, gz] = gridHit.Point.Y - gridDrop;
                else
                    heights[gx, gz] = patchGroundY - gridDrop;
            }

            var triCount = (gridSize - 1) * (gridSize - 1) * 2;
            bufferPool.Take<Triangle>(triCount, out var triangles);
            int ti = 0;
            for (int gz = 0; gz < gridSize - 1; gz++)
            for (int gx = 0; gx < gridSize - 1; gx++)
            {
                var x0 = ox + gx * PatchStep;
                var x1 = x0 + PatchStep;
                var z0 = oz + gz * PatchStep;
                var z1 = z0 + PatchStep;
                var v00 = new Vector3(x0, heights[gx, gz], z0);
                var v10 = new Vector3(x1, heights[gx + 1, gz], z0);
                var v01 = new Vector3(x0, heights[gx, gz + 1], z1);
                var v11 = new Vector3(x1, heights[gx + 1, gz + 1], z1);

                // CW winding from above → front face points UP for collision from above
                triangles[ti++] = new Triangle(v00, v10, v01);
                triangles[ti++] = new Triangle(v10, v11, v01);
            }

            if (meshVolumes && !ctx.IsGeneric)
            {
                // Keep the exact sampled top surface, with solid ground below it.
                // Native convex/mesh queries bypass body/body contact sampling.
                TerrainCollisionSurface.Add(simulation, bufferPool, new ReadOnlySpan<Triangle>(triangles.Memory, triCount));
                bufferPool.Return(ref triangles);
            }
            else
            {
                var terrainMesh = new BepuPhysics.Collidables.Mesh(triangles, Vector3.One, bufferPool);
                var terrainIndex = simulation.Shapes.Add(terrainMesh);
                simulation.Statics.Add(new StaticDescription(
                    Vector3.Zero, Quaternion.Identity, terrainIndex));
            }
            PatchCenters.Add(center);
        }

        /// <summary>Add a patch where the body (its belly bone, or the middle of its bodies) has travelled off the
        /// patches it has.</summary>
        internal void EnsureCoverage(Vector3[] worldPositions, bool[] boneValid, int boneCount, int nHaraIndex,
            bool targetIsLocalPlayer, bool meshVolumes)
        {
            if (PatchCenters.Count >= MaxTerrainPatches)
                return;

            var ragdollBones = ctx.Bodies;
            Vector3 sample = default;
            var hasSample = false;
            if (nHaraIndex >= 0)
            {
                for (int i = 0; i < boneCount; i++)
                {
                    if (!boneValid[i] || ragdollBones[i].BoneIndex != nHaraIndex)
                        continue;

                    sample = worldPositions[i];
                    hasSample = true;
                    break;
                }
            }

            if (!hasSample)
            {
                var sum = Vector3.Zero;
                var count = 0;
                for (int i = 0; i < boneCount; i++)
                {
                    if (!boneValid[i])
                        continue;

                    sum += worldPositions[i];
                    count++;
                }

                if (count == 0)
                    return;

                sample = sum / count;
            }

            var sample2 = new Vector2(sample.X, sample.Z);
            foreach (var center in PatchCenters)
                if (Vector2.DistanceSquared(center, sample2) <= PatchRefreshDistance * PatchRefreshDistance)
                    return;

            AddTerrainPatch(sample.X, sample.Z, RealGroundY, targetIsLocalPlayer, meshVolumes);
            if (ctx.Settings.RagdollVerboseLog)
                ctx.Log.Info($"Ragdoll: added dynamic terrain patch at ({sample.X:F2}, {sample.Z:F2}); total={PatchCenters.Count}");
        }

        // a6-12
        /// <summary>Deactivation: the ground patches laid so far are forgotten.</summary>
        internal void ClearPatchCenters() => PatchCenters.Clear();
    }
}
