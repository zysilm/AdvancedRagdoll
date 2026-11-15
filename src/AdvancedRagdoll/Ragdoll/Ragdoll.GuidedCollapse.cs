// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System;
using System.Numerics;
using AdvancedRagdoll.CollapseProfiles;
using BepuPhysics;
using BepuPhysics.Constraints;
using AdvancedRagdoll.Hosting;
using BepuSimulation = BepuPhysics.Simulation;

namespace AdvancedRagdoll;

public unsafe partial class Ragdoll
{
    // The directed collapse and the collapse entry conditioning (Ragdoll.GuidedCollapse.cs).
    private readonly GuidedCollapse collapse;

    private bool pendingDirectedCollapseSpike { get => collapse.PendingDirected; set => collapse.PendingDirected = value; }
    private string pendingDirectedProfileId { get => collapse.PendingProfileId; set => collapse.PendingProfileId = value; }
    private float pendingDirectedDuration { get => collapse.PendingDuration; set => collapse.PendingDuration = value; }
    private float pendingDirectedFootForce { get => collapse.PendingFootForce; set => collapse.PendingFootForce = value; }
    private float pendingDirectedPelvisForce { get => collapse.PendingPelvisForce; set => collapse.PendingPelvisForce = value; }
    private float pendingDirectedDrop { get => collapse.PendingDrop; set => collapse.PendingDrop = value; }
    private float pendingDirectedForward { get => collapse.PendingForward; set => collapse.PendingForward = value; }
    private float pendingDirectedPitchDegrees { get => collapse.PendingPitchDegrees; set => collapse.PendingPitchDegrees = value; }
    private bool directedCollapseActive => collapse.DirectedActive;
    private bool pendingEntryConditionedKneePowerLoss
    {
        get => collapse.PendingEntryConditionedKneePowerLoss;
        set => collapse.PendingEntryConditionedKneePowerLoss = value;
    }
    private bool entryConditioningActive => collapse.EntryConditioningActive;

    // --- Death-collapse spike (experimental proof-of-concept) ---
    // Active-ragdoll spike: instead of letting the corpse go instantly limp, snapshot the
    // current ("death-instant") relative orientation of every joint and pin it with an
    // AngularServo ("muscle tone"), then fade each joint's strength toward zero so gravity
    // drives a controlled collapse. The per-tier fade order/curve decides the collapse style.
    // NOTE: this reuses the AngularServo *mechanism* only and is tuned from scratch — it
    // inherits nothing from (and proves nothing about) the poor soft-body bones.
    // Knee buckle on death (Ragdoll page > Guided Collapse): the knee power-loss pattern, entered
    // through the stance-conditioning pre-pass when the death pose needs it.
    private void ArmConfiguredGuidedCollapse()
    {
        if (settings.GuidedCollapse.Enabled)
            RequestEntryConditionedKneePowerLossOnReady();
    }

    internal bool DirectedCollapseSpikeActive => directedCollapseActive;

    private void RequestDirectedKneelPitchOnReadyCore(float duration, float footForce, float pelvisForce,
        float drop, float forward, float pitchDegrees)
    {
        if (IsSimulationReady)
        {
            BeginDirectedKneelPitchSpike(duration, footForce, pelvisForce, drop, forward, pitchDegrees);
            return;
        }

        collapse.ArmKneelPitch(duration, footForce, pelvisForce, drop, forward, pitchDegrees);
    }

    private void RequestProfileDirectedCollapseOnReadyCore(string profileId)
    {
        if (IsSimulationReady)
        {
            BeginProfileDirectedCollapseSpike(profileId);
            return;
        }

        collapse.ArmProfile(profileId);
    }

    private bool BeginProfileDirectedCollapseSpikeCore(string profileId)
    {
        var profile = collapseProfileBook.FindById(profileId);
        if (profile == null)
        {
            log.Warning($"Ragdoll: directed collapse profile not found - profile={profileId}");
            return false;
        }

        return BeginProfileDirectedCollapseSpike(profile);
    }

    private bool BeginProfileDirectedCollapseSpike(CollapseProfile profile)
    {
        if (!collapse.CanBeginProfile(profile)) return false;

        StopDirectedCollapseSpike();

        collapse.BeginProfile(profile);
        return true;
    }

    private bool BeginDirectedKneelPitchSpikeCore(float duration, float footForce, float pelvisForce,
        float drop, float forward, float pitchDegrees)
    {
        if (!collapse.TryFindKneelPitchBodies(out var bodies)) return false;

        StopDirectedCollapseSpike();

        collapse.BeginKneelPitch(bodies, duration, footForce, pelvisForce, drop, forward, pitchDegrees);
        return true;
    }

    private void StopDirectedCollapseSpikeCore() => collapse.StopDirected();

    private void UpdateDirectedCollapseSpike(float dt)
    {
        if (!directedCollapseActive || simulation == null) return;
        if (collapse.HasProfile)
        {
            UpdateProfileDirectedCollapseSpike(dt);
            return;
        }

        if (collapse.UpdateKneelPitch(dt))
            StopDirectedCollapseSpike();
    }

    // The profile's phases and controllers are walked here, so that a controller that releases the rig makes the
    // recorded stop at the same point of the walk.
    private void UpdateProfileDirectedCollapseSpike(float dt)
    {
        if (!collapse.HasProfile || simulation == null) return;
        if (collapse.ProfilePhasesDone)
        {
            StopDirectedCollapseSpike();
            return;
        }

        var phase = collapse.AdvanceProfilePhase(dt, out var phaseDuration, out var phaseT);

        foreach (var controller in phase.Controllers)
            if (collapse.ApplyProfileController(controller, phaseT))
                StopDirectedCollapseSpike();

        if (collapse.EndProfilePhase(phaseDuration))
            StopDirectedCollapseSpike();
    }

    private static OneBodyLinearServo DirectedLinearServo(Vector3 target, float force, float freq)
        => DirectedLinearServo(target, force, freq, Vector3.Zero);

    private static OneBodyLinearServo DirectedLinearServo(Vector3 target, float force, float freq, Vector3 localOffset) => new()
    {
        LocalOffset = localOffset,
        Target = target,
        ServoSettings = new ServoSettings(12f, 1f, MathF.Max(0f, force)),
        SpringSettings = new SpringSettings(freq, 1f),
    };

    private static OneBodyAngularServo DirectedAngularServo(Quaternion target, float force, float freq) => new()
    {
        TargetOrientation = target,
        ServoSettings = new ServoSettings(12f, 1f, MathF.Max(0f, force)),
        SpringSettings = new SpringSettings(freq, 1f),
    };

    private static AngularServo DirectedRelativeAngularServo(Quaternion target, float force, float freq) => new()
    {
        TargetRelativeRotationLocalA = target,
        ServoSettings = new ServoSettings(12f, 1f, MathF.Max(0f, force)),
        SpringSettings = new SpringSettings(freq, 1f),
    };

    private static void ApplyDirectedLinear(BepuSimulation? simulation, ConstraintHandle? handle, Vector3 target, float force, float freq)
        => ApplyDirectedLinear(simulation, handle, target, force, freq, Vector3.Zero);

    private static void ApplyDirectedLinear(BepuSimulation? simulation, ConstraintHandle? handle, Vector3 target, float force, float freq, Vector3 localOffset)
    {
        if (!handle.HasValue || simulation == null) return;
        try { simulation.Solver.ApplyDescription(handle.Value, DirectedLinearServo(target, force, freq, localOffset)); } catch { }
    }

    private static void ApplyDirectedAngular(BepuSimulation? simulation, ConstraintHandle? handle, Quaternion target, float force, float freq)
    {
        if (!handle.HasValue || simulation == null) return;
        try { simulation.Solver.ApplyDescription(handle.Value, DirectedAngularServo(target, force, freq)); } catch { }
    }

    private static void ApplyDirectedRelativeAngular(BepuSimulation? simulation, ConstraintHandle? handle, Quaternion target, float force, float freq)
    {
        if (!handle.HasValue || simulation == null) return;
        try { simulation.Solver.ApplyDescription(handle.Value, DirectedRelativeAngularServo(target, force, freq)); } catch { }
    }

    private static void RemoveDirectedConstraint(BepuSimulation? simulation, ref ConstraintHandle? handle)
    {
        if (!handle.HasValue || simulation == null) { handle = null; return; }
        try { simulation.Solver.Remove(handle.Value); } catch { }
        handle = null;
    }

    private static float SmoothStep(float t)
    {
        t = Math.Clamp(t, 0f, 1f);
        return t * t * (3f - 2f * t);
    }

    private static Vector3 FlatNormalize(Vector3 v, Vector3 fallback)
    {
        v.Y = 0f;
        return v.LengthSquared() > 0.0001f ? Vector3.Normalize(v) : fallback;
    }

    /// <summary>The hip–knee–ankle bend in degrees from the bodies' bone origins; -1 when a bone has no body.</summary>
    private static float MeasureLegBendAngle(SimulationContext ctx, string hipBone, string kneeBone, string ankleBone, out Vector3 knee)
    {
        knee = Vector3.Zero;
        if (!ctx.TryGetBoneOriginPosition(hipBone, out var hip) ||
            !ctx.TryGetBoneOriginPosition(kneeBone, out knee) ||
            !ctx.TryGetBoneOriginPosition(ankleBone, out var ankle))
            return -1f;

        var thigh = NormalizeOrFallback(knee - hip, Vector3.UnitY);
        var shin = NormalizeOrFallback(ankle - knee, Vector3.UnitY);
        return MathF.Acos(Math.Clamp(Vector3.Dot(thigh, shin), -1f, 1f)) * 180f / MathF.PI;
    }

    // --- Collapse entry conditioning ---
    // Common pre-pass for narrow upright death poses. When the stance is too closed or the knees
    // are locked nearly straight, downstream collapse profiles inherit an unstable support base
    // and tend to fold the knees inward. This pre-pass briefly opens the support geometry and
    // starts a small pelvis drop before handing off to the requested collapse pattern.
    private void RequestEntryConditionedKneePowerLossOnReadyCore()
    {
        var kneeSettings = EffectiveKneePowerLossSettings();
        if (!kneeSettings.EntryConditioningEnabled)
        {
            RequestKneePowerLossForwardOnReady();
            return;
        }

        if (IsSimulationReady)
        {
            if (!BeginCollapseEntryConditioning(BeginKneePowerLossForwardPattern))
                BeginKneePowerLossForwardPattern();
            return;
        }

        collapse.ArmEntryConditionedKneePowerLoss();
    }

    private bool BeginCollapseEntryConditioning(Func<bool> continuation)
    {
        if (!collapse.EntryConditioningNeeded()) return false;

        StopDirectedCollapseSpike();
        StopKneePowerLossPattern();

        collapse.BeginEntryConditioning(continuation);
        return true;
    }

    private void StopCollapseEntryConditioning() => collapse.StopEntryConditioning();

    private void UpdateCollapseEntryConditioning(float dt) => collapse.UpdateEntryConditioning(dt);

    /// <summary>
    /// The directed collapse (a kneel-pitch drive or a collapse profile's phases: goal "foot plant + staged body drive"
    /// for a forward kneel/pitch death, separate from the relaxation-family Death Collapse) and the collapse entry
    /// conditioning that opens a narrow stance before the knee power-loss pattern. A component
    /// (docs/ARCHITECTURE.md §8). The recorded calls (stopping the patterns, beginning the knee power-loss) are the
    /// ragdoll's: it does them between this component's steps, and the entry conditioning's continuation is the
    /// ragdoll's recorded begin, handed in.
    /// </summary>
    private sealed class GuidedCollapse
    {
        private readonly SimulationContext ctx;
        private readonly IRagdollRandom random;

        // --- Directed collapse spike (experimental proof-of-concept) ---
        private ConstraintHandle? directedLeftFootConstraint;
        private ConstraintHandle? directedRightFootConstraint;
        private ConstraintHandle? directedPelvisLinearConstraint;
        private ConstraintHandle? directedPelvisAngularConstraint;
        private ConstraintHandle? directedChestAngularConstraint;
        private bool directedCollapseActive;
        private bool pendingDirectedCollapseSpike;
        private CollapseProfile? directedProfile;
        private int directedPhaseIndex;
        private float directedPhaseElapsed;
        private string pendingDirectedProfileId = string.Empty;
        private float directedElapsed;
        private float directedDuration;
        private float directedFootForce;
        private float directedPelvisForce;
        private float directedDrop;
        private float directedForwardDistance;
        private float directedPitchRadians;
        private float pendingDirectedDuration;
        private float pendingDirectedFootForce;
        private float pendingDirectedPelvisForce;
        private float pendingDirectedDrop;
        private float pendingDirectedForward;
        private float pendingDirectedPitchDegrees;
        private Vector3 directedForward;
        private Vector3 directedRight;
        private Vector3 directedLeftFootTarget;
        private Vector3 directedRightFootTarget;
        private Vector3 directedPelvisStart;
        private Quaternion directedPelvisStartRot;
        private Quaternion directedChestStartRot;

        // --- Collapse entry conditioning ---
        private bool pendingEntryConditionedKneePowerLoss;
        private bool entryConditioningActive;
        private float entryConditioningElapsed;
        private Func<bool>? entryConditioningContinuation;
        private Vector3 entryForward;
        private Vector3 entryRight;
        private Vector3 entryLeftKneeStart;
        private Vector3 entryRightKneeStart;
        private float entryStanceWidth;
        private float entryLeftKneeAngle;
        private float entryRightKneeAngle;
        private bool entryWasNeeded;

        internal GuidedCollapse(SimulationContext ctx, IRagdollRandom random)
        {
            this.ctx = ctx;
            this.random = random;
        }

        // What a request armed for the next death (begun when the simulation is ready).
        internal bool PendingDirected { get => pendingDirectedCollapseSpike; set => pendingDirectedCollapseSpike = value; }
        internal string PendingProfileId { get => pendingDirectedProfileId; set => pendingDirectedProfileId = value; }
        internal float PendingDuration { get => pendingDirectedDuration; set => pendingDirectedDuration = value; }
        internal float PendingFootForce { get => pendingDirectedFootForce; set => pendingDirectedFootForce = value; }
        internal float PendingPelvisForce { get => pendingDirectedPelvisForce; set => pendingDirectedPelvisForce = value; }
        internal float PendingDrop { get => pendingDirectedDrop; set => pendingDirectedDrop = value; }
        internal float PendingForward { get => pendingDirectedForward; set => pendingDirectedForward = value; }
        internal float PendingPitchDegrees { get => pendingDirectedPitchDegrees; set => pendingDirectedPitchDegrees = value; }
        internal bool PendingEntryConditionedKneePowerLoss
        {
            get => pendingEntryConditionedKneePowerLoss;
            set => pendingEntryConditionedKneePowerLoss = value;
        }

        /// <summary>True while the directed collapse drives the rig.</summary>
        internal bool DirectedActive => directedCollapseActive;
        /// <summary>True while the directed collapse plays a collapse profile (not the kneel-pitch drive).</summary>
        internal bool HasProfile => directedProfile != null;
        /// <summary>True while the entry conditioning opens the stance.</summary>
        internal bool EntryConditioningActive => entryConditioningActive;

        /// <summary>The kneel-pitch bodies: both feet, the pelvis and the chest.</summary>
        internal readonly record struct KneelPitchBodies(BodyHandle LeftFoot, BodyHandle RightFoot, BodyHandle Pelvis,
            BodyHandle Chest);

        internal void ArmKneelPitch(float duration, float footForce, float pelvisForce, float drop, float forward,
            float pitchDegrees)
        {
            pendingDirectedDuration = duration;
            pendingDirectedFootForce = footForce;
            pendingDirectedPelvisForce = pelvisForce;
            pendingDirectedDrop = drop;
            pendingDirectedForward = forward;
            pendingDirectedPitchDegrees = pitchDegrees;
            pendingDirectedProfileId = string.Empty;
            pendingDirectedCollapseSpike = true;
            ctx.Log.Info("Ragdoll: directed kneel-pitch armed for next death");
        }

        internal void ArmProfile(string profileId)
        {
            pendingDirectedProfileId = profileId;
            pendingDirectedCollapseSpike = true;
            ctx.Log.Info($"Ragdoll: directed collapse profile armed for next death - profile={profileId}");
        }

        /// <summary>The first half of beginning a profile: the rig is ready and the profile has phases.</summary>
        internal bool CanBeginProfile(CollapseProfile profile)
            => !(ctx.Simulation == null || !ctx.IsActive || profile.Phases.Count == 0);

        /// <summary>The second half of beginning a profile, after the ragdoll stopped the directed collapse.</summary>
        internal void BeginProfile(CollapseProfile profile)
        {
            var simulation = ctx.Simulation!;
            var landmarks = ctx.Profile.Landmarks;

            directedProfile = profile;
            directedPhaseIndex = 0;
            directedPhaseElapsed = 0f;
            directedElapsed = 0f;
            directedDuration = 0f;
            foreach (var phase in profile.Phases)
                directedDuration += MathF.Max(0.01f, phase.Duration);

            directedForward = ctx.ResolveForwardDirection();
            directedRight = FlatNormalize(Vector3.Transform(Vector3.UnitX, ctx.RootRotation), Vector3.UnitX);

            var pelvis = ctx.FindBodyHandle(landmarks.Pelvis);
            if (pelvis.HasValue)
            {
                var pelvisBody = simulation.Bodies.GetBodyReference(pelvis.Value);
                directedPelvisStart = pelvisBody.Pose.Position;
                directedPelvisStartRot = pelvisBody.Pose.Orientation;
            }

            var chest = ctx.FindBodyHandle(landmarks.Chest) ?? ctx.FindBodyHandle(landmarks.MidSpine);
            if (chest.HasValue)
                directedChestStartRot = simulation.Bodies.GetBodyReference(chest.Value).Pose.Orientation;

            var leftFoot = ctx.FindBodyHandle(landmarks.Foot.Left);
            if (leftFoot.HasValue)
                directedLeftFootTarget = simulation.Bodies.GetBodyReference(leftFoot.Value).Pose.Position;
            var rightFoot = ctx.FindBodyHandle(landmarks.Foot.Right);
            if (rightFoot.HasValue)
                directedRightFootTarget = simulation.Bodies.GetBodyReference(rightFoot.Value).Pose.Position;

            ctx.WakeBodies();
            EnsureProfileConstraints(profile);

            directedCollapseActive = true;
            ctx.PrevAllAsleep = false;
            ctx.BeginBiomechanicalSettle();
            ctx.Log.Info($"Ragdoll: directed collapse profile begun - profile={profile.Id} phases={profile.Phases.Count} duration={directedDuration:F2}");
        }

        /// <summary>The first half of beginning the kneel-pitch drive: the rig is ready and has the bodies.</summary>
        internal bool TryFindKneelPitchBodies(out KneelPitchBodies bodies)
        {
            bodies = default;
            if (ctx.Simulation == null || !ctx.IsActive) return false;

            var landmarks = ctx.Profile.Landmarks;
            var leftFoot = ctx.FindBodyHandle(landmarks.Foot.Left);
            var rightFoot = ctx.FindBodyHandle(landmarks.Foot.Right);
            var pelvis = ctx.FindBodyHandle(landmarks.Pelvis);
            var chest = ctx.FindBodyHandle(landmarks.Chest) ?? ctx.FindBodyHandle(landmarks.MidSpine);
            if (!leftFoot.HasValue || !rightFoot.HasValue || !pelvis.HasValue || !chest.HasValue)
            {
                ctx.Log.Warning("Ragdoll: directed kneel-pitch failed - missing foot/pelvis/chest body");
                return false;
            }

            bodies = new KneelPitchBodies(leftFoot.Value, rightFoot.Value, pelvis.Value, chest.Value);
            return true;
        }

        /// <summary>The second half of beginning the kneel-pitch drive, after the ragdoll stopped the directed
        /// collapse.</summary>
        internal void BeginKneelPitch(KneelPitchBodies b, float duration, float footForce, float pelvisForce,
            float drop, float forward, float pitchDegrees)
        {
            var simulation = ctx.Simulation!;

            directedProfile = null;
            directedPhaseIndex = 0;
            directedPhaseElapsed = 0f;
            directedDuration = Math.Clamp(duration, 0.25f, 4f);
            directedFootForce = Math.Clamp(footForce, 50f, 10000f);
            directedPelvisForce = Math.Clamp(pelvisForce, 50f, 10000f);
            directedDrop = Math.Clamp(drop, 0.05f, 1.5f);
            directedForwardDistance = Math.Clamp(forward, 0f, 2f);
            directedPitchRadians = Math.Clamp(pitchDegrees, -90f, 90f) * (MathF.PI / 180f);
            directedElapsed = 0f;

            directedForward = ctx.ResolveForwardDirection();
            directedRight = FlatNormalize(Vector3.Transform(Vector3.UnitX, ctx.RootRotation), Vector3.UnitX);

            var leftBody = simulation.Bodies.GetBodyReference(b.LeftFoot);
            var rightBody = simulation.Bodies.GetBodyReference(b.RightFoot);
            var pelvisBody = simulation.Bodies.GetBodyReference(b.Pelvis);
            var chestBody = simulation.Bodies.GetBodyReference(b.Chest);

            directedLeftFootTarget = leftBody.Pose.Position;
            directedRightFootTarget = rightBody.Pose.Position;
            directedPelvisStart = pelvisBody.Pose.Position;
            directedPelvisStartRot = pelvisBody.Pose.Orientation;
            directedChestStartRot = chestBody.Pose.Orientation;

            ctx.WakeBodies();

            directedLeftFootConstraint = simulation.Solver.Add(b.LeftFoot, DirectedLinearServo(directedLeftFootTarget, directedFootForce, 90f));
            directedRightFootConstraint = simulation.Solver.Add(b.RightFoot, DirectedLinearServo(directedRightFootTarget, directedFootForce, 90f));
            directedPelvisLinearConstraint = simulation.Solver.Add(b.Pelvis, DirectedLinearServo(directedPelvisStart, directedPelvisForce, 45f));
            directedPelvisAngularConstraint = simulation.Solver.Add(b.Pelvis, DirectedAngularServo(directedPelvisStartRot, directedPelvisForce * 0.25f, 20f));
            directedChestAngularConstraint = simulation.Solver.Add(b.Chest, DirectedAngularServo(directedChestStartRot, directedPelvisForce * 0.35f, 25f));

            directedCollapseActive = true;
            ctx.PrevAllAsleep = false;
            ctx.BeginBiomechanicalSettle();
            ctx.Log.Info($"Ragdoll: directed kneel-pitch begun - duration={directedDuration:F2} footForce={directedFootForce:F0} pelvisForce={directedPelvisForce:F0} drop={directedDrop:F2} forward={directedForwardDistance:F2} pitch={pitchDegrees:F0}");
        }

        internal void StopDirected()
        {
            var simulation = ctx.Simulation;
            if (simulation != null)
            {
                RemoveDirectedConstraint(simulation, ref directedLeftFootConstraint);
                RemoveDirectedConstraint(simulation, ref directedRightFootConstraint);
                RemoveDirectedConstraint(simulation, ref directedPelvisLinearConstraint);
                RemoveDirectedConstraint(simulation, ref directedPelvisAngularConstraint);
                RemoveDirectedConstraint(simulation, ref directedChestAngularConstraint);
            }
            else
            {
                directedLeftFootConstraint = null;
                directedRightFootConstraint = null;
                directedPelvisLinearConstraint = null;
                directedPelvisAngularConstraint = null;
                directedChestAngularConstraint = null;
            }

            if (directedCollapseActive)
            {
                directedCollapseActive = false;
                directedProfile = null;
                directedPhaseIndex = 0;
                directedPhaseElapsed = 0f;
                ctx.BeginBiomechanicalSettle();
                ctx.Log.Info("Ragdoll: directed kneel-pitch stopped");
            }
        }

        /// <summary>One step of the kneel-pitch drive. True when it has run its course (the ragdoll then makes the
        /// recorded stop); nothing follows that in the step.</summary>
        internal bool UpdateKneelPitch(float dt)
        {
            var simulation = ctx.Simulation;
            directedElapsed += dt;

            var t = Math.Clamp(directedElapsed / directedDuration, 0f, 1f);
            var plantGain = 1f - SmoothStep(Math.Clamp((t - 0.70f) / 0.25f, 0f, 1f));
            var driveGain = 1f - SmoothStep(Math.Clamp((t - 0.78f) / 0.20f, 0f, 1f));
            var buckle = SmoothStep(Math.Clamp(t / 0.62f, 0f, 1f));
            var pitch = SmoothStep(Math.Clamp((t - 0.32f) / 0.46f, 0f, 1f));

            var pelvisTarget = directedPelvisStart
                + directedForward * (directedForwardDistance * buckle)
                - Vector3.UnitY * (directedDrop * buckle);

            var pelvisTargetRot = Quaternion.Normalize(
                Quaternion.CreateFromAxisAngle(directedRight, directedPitchRadians * 0.35f * pitch) * directedPelvisStartRot);
            var chestTargetRot = Quaternion.Normalize(
                Quaternion.CreateFromAxisAngle(directedRight, directedPitchRadians * pitch) * directedChestStartRot);

            ApplyDirectedLinear(simulation, directedLeftFootConstraint, directedLeftFootTarget, directedFootForce * plantGain, 90f);
            ApplyDirectedLinear(simulation, directedRightFootConstraint, directedRightFootTarget, directedFootForce * plantGain, 90f);
            ApplyDirectedLinear(simulation, directedPelvisLinearConstraint, pelvisTarget, directedPelvisForce * driveGain, 45f);
            ApplyDirectedAngular(simulation, directedPelvisAngularConstraint, pelvisTargetRot, directedPelvisForce * 0.25f * driveGain, 20f);
            ApplyDirectedAngular(simulation, directedChestAngularConstraint, chestTargetRot, directedPelvisForce * 0.35f * driveGain, 25f);

            return directedElapsed >= directedDuration;
        }

        /// <summary>True when the profile has no phase left to play.</summary>
        internal bool ProfilePhasesDone => directedPhaseIndex >= directedProfile!.Phases.Count;

        /// <summary>Advances the profile clocks by <paramref name="dt"/> and returns the current phase, its duration
        /// and how far into it the profile is (0..1).</summary>
        internal CollapsePhaseProfile AdvanceProfilePhase(float dt, out float phaseDuration, out float phaseT)
        {
            directedElapsed += dt;
            directedPhaseElapsed += dt;

            var phase = directedProfile!.Phases[directedPhaseIndex];
            phaseDuration = MathF.Max(0.01f, phase.Duration);
            phaseT = Math.Clamp(directedPhaseElapsed / phaseDuration, 0f, 1f);
            return phase;
        }

        /// <summary>Moves to the next phase when the current one has run its course. True when that was the last
        /// phase (the ragdoll then makes the recorded stop).</summary>
        internal bool EndProfilePhase(float phaseDuration)
        {
            if (directedPhaseElapsed >= phaseDuration)
            {
                directedPhaseIndex++;
                directedPhaseElapsed = 0f;
                if (directedPhaseIndex >= directedProfile!.Phases.Count)
                    return true;
            }
            return false;
        }

        private void EnsureProfileConstraints(CollapseProfile profile)
        {
            foreach (var phase in profile.Phases)
            foreach (var controller in phase.Controllers)
            {
                switch (controller.Type)
                {
                    case "footPlant":
                        EnsureFootPlantConstraints(controller);
                        break;
                    case "pelvisDrop":
                        EnsurePelvisLinearConstraint(controller);
                        EnsurePelvisAngularConstraint(controller);
                        break;
                    case "torsoPitch":
                        EnsureChestAngularConstraint(controller);
                        EnsurePelvisAngularConstraint(controller);
                        break;
                }
            }
        }

        /// <summary>Applies one controller of the current phase. True when the controller releases the rig to
        /// passive (the ragdoll then makes the recorded stop at once).</summary>
        internal bool ApplyProfileController(CollapseControllerProfile controller, float phaseT)
        {
            var simulation = ctx.Simulation;
            var localT = ControllerLocalT(controller, phaseT);
            var eased = SmoothStep(localT);
            var strength = Lerp(controller.StrengthFrom, controller.StrengthTo, eased);

            switch (controller.Type)
            {
                case "footPlant":
                    ApplyDirectedLinear(simulation, directedLeftFootConstraint, directedLeftFootTarget,
                        controller.Force * strength, controller.Frequency > 0 ? controller.Frequency : 90f);
                    ApplyDirectedLinear(simulation, directedRightFootConstraint, directedRightFootTarget,
                        controller.Force * strength, controller.Frequency > 0 ? controller.Frequency : 90f);
                    break;

                case "pelvisDrop":
                {
                    var direction = ResolveProfileDirection(controller.Direction);
                    var drop = Lerp(controller.DropFrom, controller.Drop, eased);
                    var distance = Lerp(controller.DistanceFrom, controller.Distance, eased);
                    var target = directedPelvisStart
                        + direction * distance
                        - Vector3.UnitY * drop;
                    ApplyDirectedLinear(simulation, directedPelvisLinearConstraint, target,
                        controller.Force * strength, controller.Frequency > 0 ? controller.Frequency : 45f);

                    var pelvisPitch = Lerp(controller.PitchDegreesFrom, controller.PitchDegrees, eased)
                        * controller.PelvisPitchScale * (MathF.PI / 180f);
                    var pelvisTargetRot = Quaternion.Normalize(
                        Quaternion.CreateFromAxisAngle(directedRight, pelvisPitch) * directedPelvisStartRot);
                    ApplyDirectedAngular(simulation, directedPelvisAngularConstraint, pelvisTargetRot,
                        controller.Force * 0.25f * strength, 20f);
                    break;
                }

                case "torsoPitch":
                {
                    var pitchDegrees = Lerp(controller.PitchDegreesFrom, controller.PitchDegrees, eased);
                    var pitch = pitchDegrees * (MathF.PI / 180f);
                    var chestTargetRot = Quaternion.Normalize(
                        Quaternion.CreateFromAxisAngle(directedRight, pitch) * directedChestStartRot);
                    ApplyDirectedAngular(simulation, directedChestAngularConstraint, chestTargetRot,
                        controller.Force * strength, controller.Frequency > 0 ? controller.Frequency : 25f);

                    var pelvisPitch = pitchDegrees * controller.PelvisPitchScale * (MathF.PI / 180f);
                    var pelvisTargetRot = Quaternion.Normalize(
                        Quaternion.CreateFromAxisAngle(directedRight, pelvisPitch) * directedPelvisStartRot);
                    ApplyDirectedAngular(simulation, directedPelvisAngularConstraint, pelvisTargetRot,
                        controller.Force * 0.45f * strength, 20f);
                    break;
                }

                case "releaseToPassive":
                    return true;
            }
            return false;
        }

        private void EnsureFootPlantConstraints(CollapseControllerProfile controller)
        {
            var simulation = ctx.Simulation;
            if (simulation == null) return;

            foreach (var bone in controller.Bones)
            {
                var handle = ctx.FindBodyHandle(bone);
                if (!handle.HasValue) continue;

                if (bone.EndsWith(ctx.Profile.SideSuffixes[0], StringComparison.Ordinal)) // the left side
                {
                    if (!directedLeftFootConstraint.HasValue)
                    {
                        directedLeftFootTarget = simulation.Bodies.GetBodyReference(handle.Value).Pose.Position;
                        directedLeftFootConstraint = simulation.Solver.Add(handle.Value,
                            DirectedLinearServo(directedLeftFootTarget, 0f, controller.Frequency > 0 ? controller.Frequency : 90f));
                    }
                }
                else if (bone.EndsWith(ctx.Profile.SideSuffixes[1], StringComparison.Ordinal)) // the right side
                {
                    if (!directedRightFootConstraint.HasValue)
                    {
                        directedRightFootTarget = simulation.Bodies.GetBodyReference(handle.Value).Pose.Position;
                        directedRightFootConstraint = simulation.Solver.Add(handle.Value,
                            DirectedLinearServo(directedRightFootTarget, 0f, controller.Frequency > 0 ? controller.Frequency : 90f));
                    }
                }
            }
        }

        private void EnsurePelvisLinearConstraint(CollapseControllerProfile controller)
        {
            var simulation = ctx.Simulation;
            if (simulation == null || directedPelvisLinearConstraint.HasValue) return;
            var handle = ctx.FindBodyHandle(string.IsNullOrWhiteSpace(controller.Bone) ? ctx.Profile.Landmarks.Pelvis : controller.Bone);
            if (!handle.HasValue) return;
            directedPelvisStart = simulation.Bodies.GetBodyReference(handle.Value).Pose.Position;
            directedPelvisLinearConstraint = simulation.Solver.Add(handle.Value,
                DirectedLinearServo(directedPelvisStart, 0f, controller.Frequency > 0 ? controller.Frequency : 45f));
        }

        private void EnsurePelvisAngularConstraint(CollapseControllerProfile controller)
        {
            var simulation = ctx.Simulation;
            if (simulation == null || directedPelvisAngularConstraint.HasValue) return;
            var handle = ctx.FindBodyHandle(ctx.Profile.Landmarks.Pelvis);
            if (!handle.HasValue) return;
            directedPelvisStartRot = simulation.Bodies.GetBodyReference(handle.Value).Pose.Orientation;
            directedPelvisAngularConstraint = simulation.Solver.Add(handle.Value,
                DirectedAngularServo(directedPelvisStartRot, 0f, 20f));
        }

        private void EnsureChestAngularConstraint(CollapseControllerProfile controller)
        {
            var simulation = ctx.Simulation;
            if (simulation == null || directedChestAngularConstraint.HasValue) return;
            var handle = ctx.FindBodyHandle(string.IsNullOrWhiteSpace(controller.Bone) ? ctx.Profile.Landmarks.Chest : controller.Bone)
                ?? ctx.FindBodyHandle(ctx.Profile.Landmarks.MidSpine);
            if (!handle.HasValue) return;
            directedChestStartRot = simulation.Bodies.GetBodyReference(handle.Value).Pose.Orientation;
            directedChestAngularConstraint = simulation.Solver.Add(handle.Value,
                DirectedAngularServo(directedChestStartRot, 0f, controller.Frequency > 0 ? controller.Frequency : 25f));
        }

        private Vector3 ResolveProfileDirection(string direction)
        {
            return direction.Equals("backward", StringComparison.OrdinalIgnoreCase) ? -directedForward :
                   direction.Equals("left", StringComparison.OrdinalIgnoreCase) ? -directedRight :
                   direction.Equals("right", StringComparison.OrdinalIgnoreCase) ? directedRight :
                   direction.Equals("sideways", StringComparison.OrdinalIgnoreCase) ? (random.Next(RandomSite.DirectedCollapseSideways, 2) == 0 ? -directedRight : directedRight) :
                   directedForward;
        }

        private static float ControllerLocalT(CollapseControllerProfile controller, float phaseT)
        {
            var start = Math.Clamp(controller.StartTime, 0f, 1f);
            var end = Math.Clamp(controller.EndTime, 0f, 1f);
            if (end <= start) end = start + 0.0001f;
            return Math.Clamp((phaseT - start) / (end - start), 0f, 1f);
        }

        private static float Lerp(float a, float b, float t)
            => a + (b - a) * Math.Clamp(t, 0f, 1f);

        private bool TryGetSupportPoint(bool left, out Vector3 point)
        {
            point = Vector3.Zero;
            var simulation = ctx.Simulation;
            if (simulation == null) return false;

            var names = new[]
            {
                ctx.Profile.Landmarks.Toes.For(left),
                ctx.Profile.Landmarks.Foot.For(left),
                ctx.Profile.Landmarks.Calf.For(left),
            };

            var found = false;
            var bestY = float.PositiveInfinity;
            foreach (var name in names)
            {
                var handle = ctx.FindBodyHandle(name);
                if (!handle.HasValue) continue;
                var pos = simulation.Bodies.GetBodyReference(handle.Value).Pose.Position;
                if (!found || pos.Y < bestY)
                {
                    found = true;
                    bestY = pos.Y;
                    point = pos;
                }
            }
            return found;
        }

        private void ApplyCoreAxisVelocity(string boneName, Vector3 axis, float targetSpeed, float maxDelta)
        {
            var simulation = ctx.Simulation;
            if (simulation == null || axis.LengthSquared() < 0.0001f) return;
            var handle = ctx.FindBodyHandle(boneName);
            if (!handle.HasValue) return;
            var body = simulation.Bodies.GetBodyReference(handle.Value);
            var n = Vector3.Normalize(axis);
            var current = Vector3.Dot(body.Velocity.Linear, n);
            var delta = Math.Clamp(targetSpeed - current, -maxDelta, maxDelta);
            body.Velocity.Linear += n * delta;
            body.Awake = true;
        }

        internal void ArmEntryConditionedKneePowerLoss()
        {
            pendingEntryConditionedKneePowerLoss = true;
            ctx.Log.Info("Ragdoll: entry-conditioned knee power-loss armed for next death");
        }

        /// <summary>The first half of beginning the entry conditioning: measures the death pose and decides whether the
        /// stance needs opening (false: it does not, or the pose could not be measured).</summary>
        internal bool EntryConditioningNeeded()
        {
            if (ctx.Simulation == null || !ctx.IsActive) return false;

            if (!TryMeasureEntryPose(out entryStanceWidth, out entryLeftKneeAngle, out entryRightKneeAngle,
                    out entryLeftKneeStart, out entryRightKneeStart))
            {
                ctx.Log.Warning("Ragdoll: entry conditioning skipped - missing leg pose data");
                return false;
            }

            entryForward = ctx.ResolveForwardDirection();
            entryRight = FlatNormalize(Vector3.Transform(Vector3.UnitX, ctx.RootRotation), Vector3.UnitX);
            var knee = EffectiveKneePowerLossSettings();
            var straightestKnee = MathF.Min(entryLeftKneeAngle, entryRightKneeAngle);
            entryWasNeeded = entryStanceWidth < Math.Clamp(knee.EntryStanceThreshold, 0.05f, 1.0f) ||
                             straightestKnee < Math.Clamp(knee.EntryReadyKneeAngle, 1f, 60f);
            if (!entryWasNeeded)
            {
                ctx.Log.Info($"Ragdoll: entry conditioning skipped - stance={entryStanceWidth:F2} kneeAngles=({entryLeftKneeAngle:F1},{entryRightKneeAngle:F1})");
                return false;
            }
            return true;
        }

        /// <summary>The second half of beginning the entry conditioning, after the ragdoll stopped the directed collapse
        /// and the knee power-loss. <paramref name="continuation"/> runs when the stance is open.</summary>
        internal void BeginEntryConditioning(Func<bool> continuation)
        {
            StopEntryConditioning();

            entryConditioningContinuation = continuation;
            entryConditioningElapsed = 0f;
            entryConditioningActive = true;
            ctx.WakeBodies();
            ctx.BeginBiomechanicalSettle();
            ctx.Log.Info($"Ragdoll: entry conditioning begun - stance={entryStanceWidth:F2} kneeAngles=({entryLeftKneeAngle:F1},{entryRightKneeAngle:F1})");
        }

        internal void StopEntryConditioning()
        {
            if (!entryConditioningActive) return;
            entryConditioningActive = false;
            entryConditioningContinuation = null;
            ctx.BeginBiomechanicalSettle();
            ctx.Log.Info("Ragdoll: entry conditioning stopped");
        }

        internal void UpdateEntryConditioning(float dt)
        {
            if (!entryConditioningActive || ctx.Simulation == null) return;

            entryConditioningElapsed += dt;
            var knee = EffectiveKneePowerLossSettings();
            var minDuration = Math.Clamp(knee.EntryMinDuration, 0.05f, 1.0f);
            var maxDuration = Math.Clamp(MathF.Max(knee.EntryMaxDuration, minDuration), minDuration, 1.5f);
            var t = SmoothStep(Math.Clamp(entryConditioningElapsed / maxDuration, 0f, 1f));
            var stepScale = Math.Clamp(dt * 60f, 0.35f, 2.0f);

            var targetStance = Math.Clamp(knee.EntryTargetStanceStart, 0.05f, 1.2f)
                + (Math.Clamp(knee.EntryTargetStanceEnd, 0.05f, 1.2f) - Math.Clamp(knee.EntryTargetStanceStart, 0.05f, 1.2f)) * t;
            var pelvisDown = Math.Clamp(knee.EntryPelvisDownStart, 0f, 2f)
                + (Math.Clamp(knee.EntryPelvisDownEnd, 0f, 2f) - Math.Clamp(knee.EntryPelvisDownStart, 0f, 2f)) * t;

            ApplyEntryKneeSeparation(targetStance, 0.10f * stepScale);
            ApplyCoreAxisVelocity(ctx.Profile.Landmarks.Pelvis, -Vector3.UnitY, pelvisDown, 0.075f * stepScale);
            ApplyCoreAxisVelocity(ctx.Profile.Landmarks.Pelvis, entryForward, 0.20f + 0.18f * t, 0.055f * stepScale);
            ApplyCoreAxisVelocity(ctx.Profile.Landmarks.MidSpine, entryForward, 0.10f + 0.10f * t, 0.040f * stepScale);

            var ready = TryMeasureEntryPose(out var stance, out var leftAngle, out var rightAngle, out _, out _) &&
                        stance >= Math.Clamp(knee.EntryReadyStance, 0.05f, 1.2f) &&
                        MathF.Min(leftAngle, rightAngle) >= Math.Clamp(knee.EntryReadyKneeAngle, 1f, 60f);
            if (entryConditioningElapsed < minDuration || (!ready && entryConditioningElapsed < maxDuration))
                return;

            var continuation = entryConditioningContinuation;
            entryConditioningActive = false;
            entryConditioningContinuation = null;
            ctx.Log.Info($"Ragdoll: entry conditioning complete - stance={stance:F2} kneeAngles=({leftAngle:F1},{rightAngle:F1}) ready={ready}");
            _ = continuation?.Invoke();
        }

        private bool TryMeasureEntryPose(out float stanceWidth, out float leftKneeAngle, out float rightKneeAngle,
            out Vector3 leftKnee, out Vector3 rightKnee)
        {
            stanceWidth = 0f;
            leftKneeAngle = 0f;
            rightKneeAngle = 0f;
            leftKnee = Vector3.Zero;
            rightKnee = Vector3.Zero;

            if (!TryGetSupportPoint(true, out var leftSupport) || !TryGetSupportPoint(false, out var rightSupport))
                return false;
            var rightAxis = FlatNormalize(Vector3.Transform(Vector3.UnitX, ctx.RootRotation), Vector3.UnitX);
            stanceWidth = MathF.Abs(Vector3.Dot(rightSupport - leftSupport, rightAxis));

            var landmarks = ctx.Profile.Landmarks;
            leftKneeAngle = MeasureLegBendAngle(ctx, landmarks.Thigh.Left, landmarks.Shin.Left, landmarks.Foot.Left, out leftKnee);
            rightKneeAngle = MeasureLegBendAngle(ctx, landmarks.Thigh.Right, landmarks.Shin.Right, landmarks.Foot.Right, out rightKnee);
            return leftKneeAngle >= 0f && rightKneeAngle >= 0f;
        }

        private void ApplyEntryKneeSeparation(float targetWidth, float maxDelta)
        {
            var simulation = ctx.Simulation;
            if (simulation == null) return;
            var leftHandle = ctx.FindBodyHandle(ctx.Profile.Landmarks.Shin.Left);
            var rightHandle = ctx.FindBodyHandle(ctx.Profile.Landmarks.Shin.Right);
            if (!leftHandle.HasValue || !rightHandle.HasValue) return;

            var lateral = FlatNormalize(entryRight, Vector3.UnitX);
            var left = simulation.Bodies.GetBodyReference(leftHandle.Value);
            var right = simulation.Bodies.GetBodyReference(rightHandle.Value);
            var currentWidth = MathF.Abs(Vector3.Dot(right.Pose.Position - left.Pose.Position, lateral));
            var error = MathF.Max(0f, targetWidth - currentWidth);
            if (error <= 0.001f) return;

            var leftSign = Vector3.Dot(left.Pose.Position - right.Pose.Position, lateral) >= 0f ? 1f : -1f;
            var leftDir = lateral * leftSign;
            var rightDir = -leftDir;
            var speed = Math.Clamp(error * 3.2f, 0f, 0.55f);
            ApplyBodyVelocityAlong(left, leftDir, speed, maxDelta);
            ApplyBodyVelocityAlong(right, rightDir, speed, maxDelta);
        }

        private static void ApplyBodyVelocityAlong(BodyReference body, Vector3 axis, float targetSpeed, float maxDelta)
        {
            if (axis.LengthSquared() < 0.0001f) return;
            var n = Vector3.Normalize(axis);
            var current = Vector3.Dot(body.Velocity.Linear, n);
            var delta = Math.Clamp(targetSpeed - current, -maxDelta, maxDelta);
            body.Velocity.Linear += n * delta;
            body.Awake = true;
        }
    }
}
