// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System;
using System.Collections.Generic;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Constraints;

namespace AdvancedRagdoll;

public unsafe partial class Ragdoll
{
    // --- Grab constraint API (for cinematic victory sequence, bone-hold test mode, and
    // Enemy Control's grab-as-attack) ---
    //
    // Multiple grabs can be live at once (e.g. a body held by both the neck and the pelvis for
    // a carry pose), so each grab gets its own slot keyed by an opaque id handed back from
    // CreateGrabConstraint. A caller that only ever holds one at a time just keeps that one id.
    private sealed class GrabSlot
    {
        public ConstraintHandle ConstraintHandle;
        public BodyHandle BodyHandle;
        // Servo/spring tuning captured at CreateGrabConstraint and reused by every
        // UpdateGrabTarget — otherwise the per-frame update would overwrite the caller's
        // configured force/speed/stiffness with hardcoded defaults after a single frame.
        public ServoSettings ServoSettings;
        public SpringSettings SpringSettings;
        public bool IsRigid;          // selects a firmer external servo, never a different body model
        public bool ServoActive;
        // Where the caught BONE sits on the body that carries it. Zero when the bone has a body of
        // its own; non-zero when it is an ear, a tail, a finger — see TryResolveGrabAttachment.
        public Vector3 LocalOffset;
        public Vector3 TargetPosition;   // the grabber's grip, live
        public Vector3 CapturePosition;  // where the caught bone lay at the moment it was caught
        public float MaxSpeed;
        public float ReelElapsed;
    }

    // The grab slots (Ragdoll.Grab.cs).
    private readonly Grab grab;

    private Dictionary<int, GrabSlot> grabSlots => grab.Slots;
    private int nextGrabSlotId { get => grab.NextSlotId; set => grab.NextSlotId = value; }
    // Address of the grabbing NPC whose collision is parked during grab (0 = none). Shared across
    // every slot — every pair a single grab session holds belongs to the same grabber.
    private nint suspendedNpcAddress { get => grab.SuspendedNpcAddress; set => grab.SuspendedNpcAddress = value; }

    /// <summary>Request a firmer finite-force external Grab servo. This never makes a body
    /// kinematic and never changes internal joint stiffness or inertia.</summary>
    internal bool GrabRigid { get; set; }

    /// <summary>
    /// Create a OneBodyLinearServo constraint that pins a ragdoll bone to a world-space
    /// target position. The target is updated each frame via UpdateGrabTarget().
    /// Also ensures all ragdoll bodies stay awake (SleepThreshold = -1).
    /// </summary>
    private int? CreateGrabConstraintCore(string boneName, Vector3 initialTarget, nint grabbingNpcAddress = 0, float maxForce = 1000f, float maxSpeed = 50f, float springFreq = 120f)
    {
        if (simulation == null || !isActive) return null;

        if (!TryResolveGrabAttachment(boneName, out var targetBody, out var localOffset))
        {
            log.Warning($"Ragdoll: nothing on the rig carries bone '{boneName}'");
            return null;
        }

        // External controllers have exclusive ownership. A standing/posed support left alive here
        // would apply a second world-space target to the same dynamic island.
        RemoveStandingSupport();
        RemoveWristConstraints();

        // Guided-collapse servos are a short death-entry effect. Once an external hand takes
        // control, retire them instead of making the Grab servo fight pose targets inside the rig.
        StopDirectedCollapseSpike();
        pendingDirectedCollapseSpike = false;
        StopCollapseEntryConditioning();
        pendingEntryConditionedKneePowerLoss = false;
        StopKneePowerLossPattern();
        pendingKneePowerLossPattern = false;

        return grab.Create(boneName, targetBody, localOffset, initialTarget, grabbingNpcAddress, maxForce, maxSpeed,
            springFreq, GrabRigid);
    }

    /// <summary>Blend grab entry, then follow the live grip with finite-force position servos.</summary>
    private void DriveGrabConstraints(float dt) => grab.Drive(dt, standingActive);

    /// <summary>
    /// A compatibility bounding capsule for a ragdoll bone. Mesh bodies use actual triangle
    /// surfaces for contact; use TryGetCharacterSurface for final conform/support decisions.
    /// False if the bone has no body
    /// or the ragdoll isn't simulating yet.
    /// </summary>
    internal bool TryGetBoneCapsule(string boneName, out BoneCapsule capsule)
    {
        capsule = default;
        if (simulation == null || !isActive || !physicsStarted) return false;

        foreach (var rb in ragdollBones)
        {
            if (rb.Name != boneName) continue;
            var body = simulation.Bodies.GetBodyReference(rb.BodyHandle);
            capsule = new BoneCapsule(
                body.Pose.Position, body.Pose.Orientation, rb.CapsuleRadius, rb.CapsuleHalfLength);
            return true;
        }
        return false;
    }

    /// <summary>
    /// The bones the ragdoll currently has bodies of their own for, in build order (root outwards).
    /// Empty while nothing is simulating.
    ///
    /// This is NOT the set a grab can take hold of — that set is every bone on the skeleton, because
    /// <see cref="TryResolveGrabAttachment"/> will carry a bone that has no body by whichever ancestor
    /// does.
    /// </summary>
    internal IReadOnlyList<string> GetSimulatedBoneNames()
    {
        if (!isActive || !physicsStarted) return Array.Empty<string>();

        var names = new List<string>(ragdollBones.Count);
        foreach (var rb in ragdollBones)
            names.Add(rb.Name);
        return names;
    }

    /// <summary>
    /// Every bone a grab can take hold of on a character: the whole skeleton, ears and all. A bone with
    /// no body of its own is not out of reach — it has no body precisely because something that does is
    /// carrying it rigidly, and the grab takes that carrier at the right offset. The rig defines what
    /// grabbing means, so it is the rig that answers what can be grabbed.
    /// </summary>
    internal IReadOnlyList<string> GetGrabbableBoneNames(nint characterAddress)
    {
        // Every bone on the skeleton, in skeleton order (what BoneTransformService.GetBoneNames lists).
        if (characterAddress == nint.Zero) return Array.Empty<string>();
        var skel = skeletons.TryGetSkeleton(characterAddress);
        if (skel == null) return Array.Empty<string>();
        var count = Math.Min(skel.BoneCount, skel.BoneNameCount);
        var names = new List<string>(count);
        for (int i = 0; i < count; i++)
        {
            var name = skel.BoneName(i);
            if (!string.IsNullOrEmpty(name)) names.Add(name);
        }
        return names;
    }

    /// <summary>True while this bone has a physics body of its very own.</summary>
    internal bool IsBoneSimulated(string boneName)
    {
        foreach (var rb in ragdollBones)
            if (rb.Name == boneName) return true;
        return false;
    }

    /// <summary>
    /// Work out what a grab on this bone should actually take hold of.
    ///
    /// The rig only builds bodies for about forty bones. A skeleton has well over a hundred: ears,
    /// horns, tails, whiskers, fingers, toes. None of those carry physics — but that is precisely
    /// BECAUSE they are rigidly carried by something that does. An ear does not move independently of
    /// the head; it IS the head, at an offset.
    ///
    /// So grabbing an ear is not a special case at all. It is grabbing the head, at the point where the
    /// ear happens to sit — and the servo already had the field for that (LocalOffset) sitting unused at
    /// Vector3.Zero. Walk up the skeleton until a bone with a body turns up, hold THAT, and offset the
    /// grip to where the bone the caller actually asked for is. Pull a corpse by the ear and the head
    /// comes round by its ear, rather than being towed from the middle of the skull.
    /// </summary>
    private bool TryResolveGrabAttachmentCore(string boneName, out BodyHandle body, out Vector3 localOffset)
    {
        body = default;
        localOffset = Vector3.Zero;
        if (simulation == null || !isActive) return false;

        // A bone with a body of its own: nothing to work out.
        foreach (var rb in ragdollBones)
        {
            if (rb.Name != boneName) continue;
            body = rb.BodyHandle;
            return true;
        }

        var skel = skeletons.TryGetSkeleton(targetCharacterAddress);
        if (skel == null) return false;

        var index = ResolveBoneIndex(skel, boneName);
        if (index < 0) return false;

        var requested = skeletons.GetBoneWorldPosition(targetCharacterAddress, boneName);
        if (rec != null) RecBoneWorldPosition("TryResolveGrabAttachment", targetCharacterAddress, boneName, requested);
        if (requested == null) return false;

        // Up the parent chain until something with a body turns up.
        var count = Math.Min(skel.BoneCount, skel.ParentCount);
        var current = index;
        for (int guard = 0; guard < count; guard++)
        {
            var parent = skel.Parent(current);
            if (parent < 0 || parent >= count) break;

            var parentName = skel.BoneName(parent);
            if (!string.IsNullOrEmpty(parentName))
            {
                foreach (var rb in ragdollBones)
                {
                    if (rb.Name != parentName) continue;

                    var carrier = simulation.Bodies.GetBodyReference(rb.BodyHandle);
                    body = rb.BodyHandle;
                    // Where the requested bone sits, in the carrying body's own frame.
                    localOffset = Vector3.Transform(
                        requested.Value - carrier.Pose.Position,
                        Quaternion.Inverse(carrier.Pose.Orientation));

                    log.Info($"Ragdoll: '{boneName}' has no body — carried by '{rb.Name}', " +
                             $"gripping it {localOffset.Length():F3}m off that body's centre");
                    return true;
                }
            }

            current = parent;
        }

        return false;
    }

    /// <summary>
    /// Retune a live grab. <see cref="UpdateGrabTarget"/> re-applies the stored servo/spring settings
    /// every frame, so a caller driving the grab from sliders needs this to make them take effect
    /// without tearing the constraint down and rebuilding it.
    /// </summary>
    private void SetGrabTuningCore(int slotId, float maxForce, float maxSpeed, float springFreq)
        => grab.SetTuning(slotId, maxForce, maxSpeed, springFreq);

    /// <summary>
    /// Update the grab constraint's target position (call each frame with NPC hand world pos).
    /// </summary>
    private void UpdateGrabTargetCore(int slotId, Vector3 worldTarget) => grab.UpdateTarget(slotId, worldTarget);

    /// <summary>Remove one Grab constraint. Shared sleep state is restored after the last slot.</summary>
    private void RemoveGrabConstraintCore(int slotId) => grab.Remove(slotId);

    /// <summary>
    /// The grab slots: a finite-force servo per caught bone that reels it from where it lay to the grabber's grip and
    /// then follows the grip, and the grabber whose collision is parked meanwhile. A component
    /// (docs/ARCHITECTURE.md §8).
    /// </summary>
    private sealed class Grab
    {
        /// <summary>
        /// Headroom over the rig's own weight that a spring grab is given.
        ///
        /// A servo's force ceiling was a flat 1000 N. That is 1.5x the weight of a default 70 kg body — it
        /// lifts, but with so little left over that the pull feels like a winch — and at 200 kg the body
        /// weighs 1960 N, so the servo could not have held it up, let alone raised it. A ceiling that does
        /// not scale with the thing it is lifting is just a hidden weight limit.
        ///
        /// So the floor is derived from the rig's real mass: enough to carry it, plus enough again to
        /// accelerate it and to chase a walking grabber.
        /// </summary>
        private const float GrabServoWeightHeadroom = 3.5f;

        /// <summary>
        /// How long the caught bone takes to travel from where it was caught to the grabber's grip.
        ///
        /// A grab has to be instant in the sense that matters — the bone is HELD from the frame the grab
        /// lands — without the body teleporting to make that true. So the hold is established where the
        /// bone already lies, and the bone is then reeled to the grip over this long. Long enough to read
        /// as a pull rather than a cut, short enough that nobody would call it slow.
        /// </summary>
        private const float GrabReelSeconds = 0.18f;

        // Limit physical body motion in the servo, rather than delaying the grip target.
        private const float GrabMaxSpeed = 12f;

        private readonly SimulationContext ctx;
        private readonly Dictionary<int, GrabSlot> grabSlots = new();

        internal Grab(SimulationContext ctx) { this.ctx = ctx; }

        internal Dictionary<int, GrabSlot> Slots => grabSlots;
        internal int NextSlotId = 1;
        // Address of the grabbing NPC whose collision is parked during grab (0 = none).
        internal nint SuspendedNpcAddress;

        /// <summary>Force floor for a spring grab: whatever the caller asked for, or enough to actually
        /// pick this body up, whichever is greater.</summary>
        private float ResolveGrabServoForce(float requested)
        {
            if (ctx.TotalMass <= 0f) return requested;
            var needed = ctx.TotalMass * RagdollTuning.Gravity * GrabServoWeightHeadroom;
            return MathF.Max(requested, needed);
        }

        /// <summary>The servo on the resolved attachment (<paramref name="targetBody"/> at
        /// <paramref name="localOffset"/>), once the caller has taken the rig over from every other
        /// controller.</summary>
        internal int Create(string boneName, BodyHandle targetBody, Vector3 localOffset, Vector3 initialTarget,
            nint grabbingNpcAddress, float maxForce, float maxSpeed, float springFreq, bool rigid)
        {
            var slot = new GrabSlot { BodyHandle = targetBody, LocalOffset = localOffset };
            var ragdollBones = ctx.Bodies;

            // Wake every body AND keep it awake for the duration of the grab. SleepThreshold=-1
            // alone only prevents *future* sleep — a corpse that has already settled has all its
            // bodies asleep, so without an explicit wake the servo lifts the pinned bone while the
            // sleeping leg bodies stay frozen in the settled pose. The joints then get dragged into
            // violation and the knees snap into a bend. Waking them up front lets the whole rig
            // follow the lift smoothly.
            for (int i = 0; i < ragdollBones.Count; i++)
            {
                var bodyRef = ctx.Simulation!.Bodies.GetBodyReference(ragdollBones[i].BodyHandle);
                bodyRef.Activity.SleepThreshold = -1f;
                bodyRef.Awake = true;
            }
            // The rig is no longer resting; force a full physics step next frame.
            ctx.Core.BeginBiomechanicalSettle();

            slot.IsRigid = rigid;

            // The same finite-force tuning path is used at creation and during live retuning.
            var servoForce = ConfigureGrabTuning(slot, maxForce, maxSpeed, springFreq);

            var simulation = ctx.Simulation!;
            var caught = simulation.Bodies.GetBodyReference(slot.BodyHandle);
            SuspendedNpcAddress = grabbingNpcAddress;
            slot.TargetPosition = initialTarget;
            slot.CapturePosition = GrabGripPoint(caught, slot.LocalOffset);
            slot.ReelElapsed = 0f;
            slot.ConstraintHandle = simulation.Solver.Add(slot.BodyHandle,
                new OneBodyLinearServo
                {
                    LocalOffset = slot.LocalOffset,
                    Target = slot.CapturePosition,
                    ServoSettings = slot.ServoSettings,
                    SpringSettings = slot.SpringSettings,
                });
            slot.ServoActive = true;
            var slotId = NextSlotId++;
            grabSlots[slotId] = slot;

            // Caught where it lies. Nothing moves on this frame — the bone is simply held from now on, and
            // the reel walks it to the grip from here. The capture is the GRIP POINT, which is the bone the
            // caller asked for, not the centre of whatever body happens to be carrying it.
            ctx.Log.Info($"Ragdoll: Grab #{slotId} created on '{boneName}' — caught at ({slot.CapturePosition.X:F2},{slot.CapturePosition.Y:F2},{slot.CapturePosition.Z:F2}), " +
                     $"reeling to ({initialTarget.X:F2},{initialTarget.Y:F2},{initialTarget.Z:F2}), firm={slot.IsRigid}, " +
                     $"rig weighs {ctx.TotalMass:F1}kg, servo={slot.ServoSettings.MaximumForce:F0}N " +
                     $"(base floor {servoForce:F0}N, asked for {maxForce:F0}N)" +
                     $", suspend NPC 0x{grabbingNpcAddress:X}");
            return slotId;
        }

        /// <summary>Where the grip actually is in the world: the caught bone, which for an ear or a tail is
        /// an offset away from the centre of the body carrying it.</summary>
        private static Vector3 GrabGripPoint(BodyReference body, Vector3 localOffset)
            => body.Pose.Position + Vector3.Transform(localOffset, body.Pose.Orientation);

        private Vector3 AdvanceGrabTarget(GrabSlot slot, float dt)
        {
            slot.ReelElapsed = MathF.Min(slot.ReelElapsed + dt, GrabReelSeconds);
            // Once the initial pull-in finishes, submit the current grip directly.
            // The finite-force linear servo still governs how the body follows it;
            // neither body position nor orientation is assigned here.
            if (slot.ReelElapsed >= GrabReelSeconds)
                return slot.TargetPosition;

            var reel = GrabReelSeconds <= 1e-4f ? 1f : slot.ReelElapsed / GrabReelSeconds;
            reel = reel * reel * (3f - 2f * reel);
            return Vector3.Lerp(slot.CapturePosition, slot.TargetPosition, reel);
        }

        /// <summary>Blend grab entry, then follow the live grip with finite-force position servos.
        /// <paramref name="standingActive"/>: the standing support holds the rig.</summary>
        internal void Drive(float dt, bool standingActive)
        {
            var simulation = ctx.Simulation;
            if (grabSlots.Count == 0 || simulation == null) return;

            // The standing support owns the whole rig when it is up; leave the body to it.
            if (standingActive) return;

            foreach (var slot in grabSlots.Values)
            {
                var target = AdvanceGrabTarget(slot, dt);

                if (slot.ServoActive)
                {
                    try
                    {
                        simulation.Solver.ApplyDescription(slot.ConstraintHandle,
                            new OneBodyLinearServo
                            {
                                LocalOffset = slot.LocalOffset,
                                Target = target,
                                ServoSettings = slot.ServoSettings,
                                SpringSettings = slot.SpringSettings,
                            });
                    }
                    catch (Exception ex)
                    {
                        ctx.Log.Warning(ex, "Ragdoll: Failed to drive grab target");
                    }
                }
            }
        }

        internal void SetTuning(int slotId, float maxForce, float maxSpeed, float springFreq)
        {
            if (!grabSlots.TryGetValue(slotId, out var slot)) return;
            ConfigureGrabTuning(slot, maxForce, maxSpeed, springFreq);
        }

        private float ConfigureGrabTuning(GrabSlot slot, float maxForce, float maxSpeed, float springFreq)
        {
            var bodySpeedLimit = ctx.ActiveClassProfile.ClassConstants.MaxLinearVelocity;
            slot.MaxSpeed = Math.Clamp(float.IsFinite(maxSpeed) ? maxSpeed : GrabMaxSpeed, 0.1f,
                MathF.Min(GrabMaxSpeed, bodySpeedLimit));
            var servoForce = ResolveGrabServoForce(float.IsFinite(maxForce) ? maxForce : 1000f);
            var resolvedFrequency = Math.Clamp(float.IsFinite(springFreq) ? springFreq : 120f, 1f, 180f);
            if (slot.IsRigid)
            {
                var firmForce = MathF.Min(
                    servoForce * 2f,
                    MathF.Max(servoForce, ctx.TotalMass * 120f));
                slot.ServoSettings = new ServoSettings(slot.MaxSpeed, 1f, firmForce);
                slot.SpringSettings = new SpringSettings(MathF.Max(60f, resolvedFrequency), 1.5f);
            }
            else
            {
                slot.ServoSettings = new ServoSettings(slot.MaxSpeed, 1f, servoForce);
                slot.SpringSettings = new SpringSettings(resolvedFrequency, 1f);
            }
            return servoForce;
        }

        internal void UpdateTarget(int slotId, Vector3 worldTarget)
        {
            if (ctx.Simulation == null || !grabSlots.TryGetValue(slotId, out var slot)) return;

            slot.TargetPosition = worldTarget;

        }

        internal void Remove(int slotId)
        {
            var simulation = ctx.Simulation;
            var ragdollBones = ctx.Bodies;
            if (simulation == null || !grabSlots.TryGetValue(slotId, out var slot)) return;
            grabSlots.Remove(slotId);

            if (slot.ServoActive)
            {
                try
                {
                    simulation.Solver.Remove(slot.ConstraintHandle);
                }
                catch { }
            }
            if (grabSlots.Count > 0)
            {
                ctx.Log.Info($"Ragdoll: Grab #{slotId} released ({grabSlots.Count} still held)");
                return;
            }

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

            SuspendedNpcAddress = nint.Zero;
            ctx.Core.BeginBiomechanicalSettle();

            ctx.Log.Info("Ragdoll: Grab constraint removed");
        }
    }
}
