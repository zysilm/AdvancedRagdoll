// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System.Numerics;
using System.Collections.Generic;
using BepuPhysics;
using BepuPhysics.Constraints;

namespace AdvancedRagdoll;

public unsafe partial class Ragdoll
{
    // The standing hold and the wrist constraints (Ragdoll.StandingSupport.cs).
    private readonly StandingSupport standing;

    internal bool IsStandingSupportActive => standingActive;

    private bool standingActive { get => standing.Active; set => standing.Active = value; }
    private List<ConstraintHandle> standingConstraints => standing.Constraints;
    private List<ConstraintHandle> wristConstraints => standing.WristConstraints;

    private bool CreateStandingSupportCore(Vector3 anchorWorldPos, Quaternion uprightRot, string anchorBoneName)
        => standing.Create(anchorWorldPos, uprightRot, anchorBoneName, grabSlots.Count > 0);

    /// <summary>Swap the anchor bone/position while the support is already active, without re-waking bodies.</summary>
    private bool UpdateStandingSupportCore(Vector3 anchorWorldPos, Quaternion uprightRot, string anchorBoneName)
        => standing.Update(anchorWorldPos, uprightRot, anchorBoneName);

    private void ApplyStandingAnchorCorrection() => standing.ApplyAnchorCorrection();

    private void RemoveStandingSupportCore() => standing.Remove();

    private bool CreateWristConstraintsCore(Vector3 leftTarget, Vector3 rightTarget,
        float force = 500f, float freq = 80f)
        => standing.CreateWrists(leftTarget, rightTarget, force, freq, grabSlots.Count > 0);

    private bool UpdateWristConstraintsCore(Vector3 leftTarget, Vector3 rightTarget)
        => standing.UpdateWrists(leftTarget, rightTarget);

    private void RemoveWristConstraintsCore() => standing.RemoveWrists();

    /// <summary>
    /// The standing support (execution mode) and the wrist constraints. Pelvis: LinearServo to a computed standing
    /// height + AngularServo upright. Spine chain: progressively weaker AngularServos. Legs/arms/head: fully dynamic —
    /// gravity + joints handle them naturally. A component (docs/ARCHITECTURE.md §8).
    /// </summary>
    private sealed class StandingSupport
    {
        private readonly SimulationContext ctx;

        // --- Standing support constraint API (execution mode) ---
        private readonly List<ConstraintHandle> standingConstraints = new();
        private bool standingActive;
        private BodyHandle? standingAnchorHandle;
        private Vector3 standingAnchorTarget;

        // ─── Wrist constraints ───────────────────────────────────────────────────

        private readonly List<ConstraintHandle> wristConstraints = new();
        private float wristForce = 500f;
        private float wristFreq  = 80f;

        internal StandingSupport(SimulationContext ctx) { this.ctx = ctx; }

        /// <summary>True while the standing support holds the rig.</summary>
        internal bool Active { get => standingActive; set => standingActive = value; }
        internal List<ConstraintHandle> Constraints => standingConstraints;
        internal List<ConstraintHandle> WristConstraints => wristConstraints;

        /// <summary><paramref name="grabbing"/>: a grab holds the rig (it owns it; no support then).</summary>
        internal bool Create(Vector3 anchorWorldPos, Quaternion uprightRot, string anchorBoneName, bool grabbing)
        {
            var simulation = ctx.Simulation;
            var ragdollBones = ctx.Bodies;
            if (simulation == null || !ctx.IsActive || grabbing) return false;

            for (int i = 0; i < ragdollBones.Count; i++)
            {
                var bodyRef = simulation.Bodies.GetBodyReference(ragdollBones[i].BodyHandle);
                bodyRef.Activity.SleepThreshold = -1f;
                bodyRef.Awake = true;
            }
            ctx.Core.BeginBiomechanicalSettle();

            BuildStandingConstraints(anchorWorldPos, uprightRot, anchorBoneName);
            standingActive = true;
            ApplyAnchorCorrection();
            ctx.Log.Info($"Ragdoll: Standing support created — {standingConstraints.Count} constraints, anchor={anchorBoneName} ({anchorWorldPos.X:F2},{anchorWorldPos.Y:F2},{anchorWorldPos.Z:F2})");
            return true;
        }

        /// <summary>Swap the anchor bone/position while the support is already active, without re-waking bodies.</summary>
        internal bool Update(Vector3 anchorWorldPos, Quaternion uprightRot, string anchorBoneName)
        {
            var simulation = ctx.Simulation;
            if (!standingActive || simulation == null) return false;

            foreach (var h in standingConstraints)
                try { simulation.Solver.Remove(h); } catch { }
            standingConstraints.Clear();
            standingAnchorHandle = null;

            BuildStandingConstraints(anchorWorldPos, uprightRot, anchorBoneName);
            ApplyAnchorCorrection();
            return true;
        }

        private void BuildStandingConstraints(Vector3 anchorWorldPos, Quaternion uprightRot, string anchorBoneName)
        {
            var sim = ctx.Simulation;
            if (sim == null) return;

            var anchorHandle = ctx.FindBodyHandle(anchorBoneName);
            if (anchorHandle.HasValue)
            {
                standingAnchorHandle = anchorHandle.Value;
                standingAnchorTarget = anchorWorldPos;

                standingConstraints.Add(sim.Solver.Add(anchorHandle.Value,
                    new OneBodyLinearServo
                    {
                        LocalOffset    = Vector3.Zero,
                        Target         = anchorWorldPos,
                        ServoSettings  = new ServoSettings(8f, 1f, 3000f),
                        SpringSettings = new SpringSettings(120f, 1f),
                    }));

                standingConstraints.Add(sim.Solver.Add(anchorHandle.Value,
                    new OneBodyAngularServo
                    {
                        TargetOrientation = uprightRot,
                        ServoSettings     = new ServoSettings(8f, 1f, 600f),
                        SpringSettings    = new SpringSettings(40f, 1f),
                    }));
            }

            float spineForce = 350f;
            float spineFreq  = 25f;
            foreach (var name in ctx.Profile.StandingSupportSpine)
            {
                var h = ctx.FindBodyHandle(name);
                if (h.HasValue)
                {
                    standingConstraints.Add(sim.Solver.Add(h.Value,
                        new OneBodyAngularServo
                        {
                            TargetOrientation = uprightRot,
                            ServoSettings     = new ServoSettings(10f, 1f, spineForce),
                            SpringSettings    = new SpringSettings(spineFreq, 1f),
                        }));
                }
                spineForce *= 0.7f;
                spineFreq  *= 0.8f;
            }
        }

        internal void ApplyAnchorCorrection()
        {
            var simulation = ctx.Simulation;
            if (!standingActive || simulation == null || !standingAnchorHandle.HasValue)
                return;

            var anchor = simulation.Bodies.GetBodyReference(standingAnchorHandle.Value);
            var correction = standingAnchorTarget - anchor.Pose.Position;
            var anchorVelocity = anchor.Velocity.Linear;

            // Clothes still on the corpse come with it; the hold left them behind on the floor for the same
            // reason the grab did. Decide first, move second — see CollectCarriedExternalRigs.
            ctx.Core.CollectCarriedExternalRigs();

            // Keep the held point fixed without disabling contact response. If collision pushes the
            // ragdoll as a whole, translate every body back by the same delta and remove the global
            // anchor velocity; relative limb motion from the contact is preserved.
            foreach (var rb in ctx.Bodies)
            {
                var body = ctx.Simulation!.Bodies.GetBodyReference(rb.BodyHandle);
                body.Pose.Position += correction;
                body.Velocity.Linear -= anchorVelocity;
                body.Awake = true;
            }

            ctx.Core.TranslateCarriedExternalRigs(correction, anchorVelocity);
        }

        internal void Remove()
        {
            var simulation = ctx.Simulation;
            var ragdollBones = ctx.Bodies;
            if (!standingActive || simulation == null) return;

            foreach (var h in standingConstraints)
            {
                try { simulation.Solver.Remove(h); } catch { }
            }
            standingConstraints.Clear();
            standingAnchorHandle = null;

            var normalThreshold = RagdollSleepThreshold;
            for (int i = 0; i < ragdollBones.Count; i++)
            {
                try
                {
                    var bodyRef = simulation.Bodies.GetBodyReference(ragdollBones[i].BodyHandle);
                    bodyRef.Activity.SleepThreshold = normalThreshold;
                    bodyRef.Awake = true;
                }
                catch { }
            }

            standingActive = false;
            ctx.Core.BeginBiomechanicalSettle();
            ctx.Log.Info("Ragdoll: Standing support removed");
        }

        /// <summary><paramref name="grabbing"/>: a grab holds the rig (it owns it; no wrist constraints then).</summary>
        internal bool CreateWrists(Vector3 leftTarget, Vector3 rightTarget, float force, float freq, bool grabbing)
        {
            var simulation = ctx.Simulation;
            if (simulation == null || !ctx.IsActive || grabbing) return false;

            foreach (var h in wristConstraints)
                try { simulation.Solver.Remove(h); } catch { }
            wristConstraints.Clear();

            wristForce = force;
            wristFreq  = freq;
            BuildWristConstraint(ctx.Profile.Landmarks.Hand.Left, leftTarget);
            BuildWristConstraint(ctx.Profile.Landmarks.Hand.Right, rightTarget);
            ctx.Core.BeginBiomechanicalSettle();
            return wristConstraints.Count > 0;
        }

        internal bool UpdateWrists(Vector3 leftTarget, Vector3 rightTarget)
        {
            var simulation = ctx.Simulation;
            if (simulation == null || wristConstraints.Count == 0) return false;

            foreach (var h in wristConstraints)
                try { simulation.Solver.Remove(h); } catch { }
            wristConstraints.Clear();

            BuildWristConstraint(ctx.Profile.Landmarks.Hand.Left, leftTarget);
            BuildWristConstraint(ctx.Profile.Landmarks.Hand.Right, rightTarget);
            return true;
        }

        internal void RemoveWrists()
        {
            var simulation = ctx.Simulation;
            if (simulation == null) return;
            foreach (var h in wristConstraints)
                try { simulation.Solver.Remove(h); } catch { }
            wristConstraints.Clear();
        }

        private void BuildWristConstraint(string boneName, Vector3 target)
        {
            var handle = ctx.FindBodyHandle(boneName);
            if (!handle.HasValue) return;
            wristConstraints.Add(ctx.Simulation!.Solver.Add(handle.Value,
                new OneBodyLinearServo
                {
                    LocalOffset    = Vector3.Zero,
                    Target         = target,
                    ServoSettings  = new ServoSettings(8f, 1f, wristForce),
                    SpringSettings = new SpringSettings(wristFreq, 1f),
                }));
        }
    }
}
