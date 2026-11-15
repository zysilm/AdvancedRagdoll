// This Source Code Form is subject to the Mozilla Public License, v. 2.0.
using System;
using BepuPhysics.Collidables;

namespace AdvancedRagdoll;

public unsafe partial class Ragdoll
{
    private long lastMeshLodInstall { get => meshVolumes.lastMeshLodInstall; set => meshVolumes.lastMeshLodInstall = value; }

    private sealed partial class MeshVolumes
    {
        private long lastMeshBudgetReport;
        internal long lastMeshLodInstall;

        // Frozen mesh mode never skins vertices or refits a native mesh during simulation.
        // Only newly completed cached LODs can replace a shape, once, outside worker steps.
        internal void ApplyCachedMeshLod()
        {
            if (ctx.Simulation == null || ctx.BufferPool == null || frozenMeshCache == null || !ctx.Settings.RagdollMeshCollisionLod) return;
            var now = ctx.Clock.GetTimestamp(Hosting.ClockSite.MeshLodInstall);
            if (lastMeshLodInstall != 0 && ctx.Clock.GetElapsedTime(Hosting.ClockSite.MeshLodInstall, lastMeshLodInstall, now).TotalMilliseconds < 16.7) return;
            var ragdollBones = ctx.Bodies;
            for (int index = 0; index < ragdollBones.Count; index++)
            {
                var bone = ragdollBones[index];
                if (bone.MeshGeometry is not { } full || bone.MeshLod != null ||
                    !ctx.Meshes.TryCreateLod(frozenMeshCache, Hosting.CachedLodUse.Step, bone.Name, full, out var lod)) continue;
                var body = ctx.Simulation.Bodies.GetBodyReference(bone.BodyHandle);
                var replacement = CreateBodyTriangleMesh(lod.Geometry);
                TypedIndex replacementIndex;
                try { replacementIndex = ctx.Simulation.Shapes.Add(replacement); }
                catch { replacement.Dispose(ctx.BufferPool); throw; }
                var previous = body.Collidable.Shape;
                body.SetShape(replacementIndex);
                ctx.Simulation.Shapes.RemoveAndDispose(previous, ctx.BufferPool);
                bone.MeshLod = lod;
                if (ctx.Recorder.IsRecording) ctx.Recorder.MeshLod("step", bone.Name, lod);
                ragdollBones[index] = bone;
                body.Awake = true; body.UpdateBounds();
                lastMeshLodInstall = now;
                return;
            }
        }

        internal void ReportMeshPhysicsBudget(long start, int steps)
        {
            if (start == 0 || steps == 0) return;
            var now = System.Diagnostics.Stopwatch.GetTimestamp();
            var milliseconds = System.Diagnostics.Stopwatch.GetElapsedTime(start, now).TotalMilliseconds;
            if (milliseconds < FixedTimestep * 1000 ||
                (lastMeshBudgetReport != 0 && System.Diagnostics.Stopwatch.GetElapsedTime(lastMeshBudgetReport, now).TotalSeconds < 5)) return;
            lastMeshBudgetReport = now;
            var ragdollBones = ctx.Bodies;
            int meshes = 0, faces = 0, collisionFaces = 0;
            foreach (var bone in ragdollBones)
                if (bone.MeshGeometry is { } geometry)
                {
                    meshes++; faces += geometry.Indices.Length / 3;
                    collisionFaces += bone.MeshLod?.CollisionFaceCount ?? geometry.Indices.Length / 3;
                }
            double ticksToMs = 1000d / System.Diagnostics.Stopwatch.Frequency;
            double collisionMs = meshCollisionTicks * ticksToMs, solveMs = meshSolveTicks * ticksToMs;
            ctx.Log.Warning($"Ragdoll mesh physics exceeded render budget: {milliseconds:F1} ms, {steps} fixed ticks, " +
                        $"{meshes}/{ragdollBones.Count} mesh/total bodies, {collisionFaces}/{faces} collision/full surface faces; catch-up limited. " +
                        $"Workers={meshDispatcherLease?.WorkerCount ?? 1}, solver={ctx.Simulation?.Solver.VelocityIterationCount}x{ctx.Simulation?.Solver.SubstepCount}, " +
                        $"frozen cache; skin/refit=0, collision={collisionMs:F1}, solve={solveMs:F1}, other={Math.Max(0, milliseconds - collisionMs - solveMs):F1} ms, " +
                        $"main native pool={(ctx.BufferPool?.GetTotalAllocatedByteCount() ?? 0) / 1048576d:F1} MiB.");
        }
    }
}
