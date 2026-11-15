// This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
using AdvancedRagdoll.MeshVolume;
using System.Diagnostics;
using BepuPhysics;

namespace AdvancedRagdoll;

public unsafe partial class Ragdoll
{
    private long meshCollisionTicks { get => meshVolumes.meshCollisionTicks; set => meshVolumes.meshCollisionTicks = value; }
    private long meshSolveTicks { get => meshVolumes.meshSolveTicks; set => meshVolumes.meshSolveTicks = value; }

    /// <summary>Workers finish before skin refit, native bone writeback or game access.</summary>
    private void StepPhysicsSimulation(float timestep)
    {
        if (simulation == null) return;
        startOverlap?.BeginStep();
        weaponContact.Sync();
        meshVolumes.Step(timestep);
    }

    private sealed partial class MeshVolumes
    {
        private MeshPhysicsDispatcher.Lease? meshDispatcherLease;
        private long meshStageStart;
        internal long meshCollisionTicks, meshSolveTicks;

        internal void InitializeMeshTiming()
        {
            if (ctx.Simulation?.Timestepper is not DefaultTimestepper timestepper) return;
            timestepper.BeforeCollisionDetection += (_, _) => meshStageStart = Stopwatch.GetTimestamp();
            timestepper.CollisionsDetected += (_, _) =>
            {
                var now = Stopwatch.GetTimestamp();
                meshCollisionTicks += now - meshStageStart;
                meshStageStart = now;
            };
            timestepper.ConstraintsSolved += (_, _) => meshSolveTicks += Stopwatch.GetTimestamp() - meshStageStart;
        }

        /// <summary>One simulation step: on the shared mesh workers while the active rig has mesh bodies, otherwise on
        /// this thread. The caller has checked that the simulation exists.</summary>
        internal void Step(float timestep)
        {
            if (!Active || ctx.IsGeneric)
            {
                ReleaseMeshDispatcher();
                ctx.Simulation!.Timestep(timestep);
                return;
            }
            meshDispatcherLease ??= MeshPhysicsDispatcher.Acquire(2);
            meshDispatcherLease.Run(ctx.Simulation!, timestep);
        }

        internal void ReleaseMeshDispatcher()
        {
            meshDispatcherLease?.Dispose();
            meshDispatcherLease = null;
        }
    }
}
