// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System.Numerics;

namespace AdvancedRagdoll;

public unsafe partial class Ragdoll
{
    /// <summary>Water under the corpse: finds the surface over it and gives the bodies in the water lift and drag.
    /// A component (docs/ARCHITECTURE.md §8).</summary>
    private sealed class Water
    {
        private const float ProbeInterval = 0.5f;
        private const float ProbeMoveDistance = 1.5f;

        private readonly SimulationContext ctx;
        // Still level of the surface over the corpse (NaN when there is none), probed now and then.
        private float level = float.NaN;
        private float probeCountdown;
        private Vector3 probePosition;
        private float clock;
        private bool inWater;

        internal Water(SimulationContext ctx) { this.ctx = ctx; }

        /// <summary>True while some body was in the water at the last step: moving water never lets the corpse
        /// rest.</summary>
        internal bool InWater => inWater;

        /// <summary>A new activation: no surface known, probe at once.</summary>
        internal void Reset()
        {
            level = float.NaN;
            probeCountdown = 0f;
            clock = 0f;
            inWater = false;
        }

        // Look for a water surface over the corpse: at once after activation or a long move, otherwise twice a second.
        internal void UpdateProbe(float dt)
        {
            var simulation = ctx.Simulation;
            var ragdollBones = ctx.Bodies;
            if (!ctx.Settings.RagdollWaterBuoyancy || simulation == null || ragdollBones.Count == 0)
            {
                level = float.NaN;
                inWater = false;
                return;
            }

            var position = simulation.Bodies.GetBodyReference(ragdollBones[0].BodyHandle).Pose.Position;
            probeCountdown -= dt;
            if (probeCountdown > 0f && Vector3.DistanceSquared(position, probePosition) < ProbeMoveDistance * ProbeMoveDistance)
                return;

            probeCountdown = ProbeInterval;
            probePosition = position;
            level = ctx.World.TryFindWaterSurface(position, 30f, 60f, out var surface) ? surface : float.NaN;
            if (float.IsNaN(level)) inWater = false;
        }

        // Lift and drag on every body that is in the water (see WaterBuoyancy). Bodies are only given forces;
        // how the corpse floats, rolls and rides the swell is left to the solver.
        internal void ApplyForces(float dt)
        {
            var simulation = ctx.Simulation;
            var ragdollBones = ctx.Bodies;
            if (simulation == null || float.IsNaN(level) || ragdollBones.Count == 0)
            {
                inWater = false;
                return;
            }

            clock += dt;
            var liquidDensity = WaterBuoyancy.LiquidDensity(ctx.Settings.RagdollWaterLiquid);
            var any = false;
            var reach = level + WaterBuoyancy.MaxWaveHeight;
            foreach (var rb in ragdollBones)
            {
                var body = simulation.Bodies.GetBodyReference(rb.BodyHandle);
                if (body.Kinematic) continue;
                var position = body.Pose.Position;
                var bottom = BodyBottom(rb, body);
                if (bottom >= reach) continue;

                var (surface, surfaceSpeed) = WaterBuoyancy.Surface(level, position.X, position.Z, clock, RagdollTuning.Gravity);
                var submerged = WaterBuoyancy.Submerged(bottom, 2f * position.Y - bottom, surface);
                if (submerged <= 0f) continue;

                any = true;
                var linear = body.Velocity.Linear;
                var angular = body.Velocity.Angular;
                WaterBuoyancy.Apply(ref linear, ref angular, submerged, WaterBuoyancy.RelativeDensity(ctx.Profile, rb.Name, liquidDensity),
                    surfaceSpeed, RagdollTuning.Gravity, dt);
                body.Velocity.Linear = linear;
                body.Velocity.Angular = angular;
                body.Awake = true; // moving water never lets a floating body rest
            }
            inWater = any;
        }
    }
}
