// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System;
using System.Collections.Generic;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;

namespace AdvancedRagdoll;

public unsafe partial class Ragdoll
{
    internal readonly struct ExternalShapePart
    {
        public readonly Vector3 HalfExtents;
        public readonly Vector3 LocalPosition;
        public readonly Quaternion LocalOrientation;

        public ExternalShapePart(Vector3 halfExtents, Vector3 localPosition, Quaternion localOrientation)
        {
            HalfExtents = halfExtents;
            LocalPosition = localPosition;
            LocalOrientation = localOrientation;
        }
    }

    internal sealed class ExternalBodyHandle
    {
        internal BodyHandle Body;
        internal TypedIndex[] Shapes = Array.Empty<TypedIndex>();
        internal int Generation;
        internal bool Removed;
    }

    internal readonly struct ExternalRigBodySpec
    {
        public readonly string Name;
        public readonly IReadOnlyList<ExternalShapePart> Parts;
        public readonly float Mass;
        public readonly Vector3 Position;
        public readonly Quaternion Orientation;
        public readonly Vector3 LinearVelocity;
        public readonly Vector3 AngularVelocity;

        public ExternalRigBodySpec(
            string name,
            IReadOnlyList<ExternalShapePart> parts,
            float mass,
            Vector3 position,
            Quaternion orientation,
            Vector3 linearVelocity,
            Vector3 angularVelocity)
        {
            Name = name;
            Parts = parts;
            Mass = mass;
            Position = position;
            Orientation = orientation;
            LinearVelocity = linearVelocity;
            AngularVelocity = angularVelocity;
        }
    }

    internal enum ExternalRigJointKind
    {
        // BallSocket + optional SwingLimit + AngularMotor + optional pose-guide servo (chain garments/limbs).
        BallSocketSwing = 0,
        // A single DistanceLimit between two anchor points: inextensible above MaxDistance, compressible to
        // MinDistance. The edge primitive of the garment tube (ring-adjacent + ring-to-ring + diagonals).
        DistanceLimit = 1,
    }

    internal readonly struct ExternalRigJointSpec
    {
        public readonly ExternalRigJointKind Kind;
        public readonly int ParentIndex;
        public readonly int ChildIndex;
        // Ball-socket fields.
        public readonly Vector3 AnchorWorld;
        public readonly float SwingLimit;
        public readonly float InitialSwingFactor;
        public readonly float PoseGuideMaxForce;
        public readonly float PoseGuideFrequency;
        // Distance-limit fields (Kind == DistanceLimit). Offsets are in each body's local frame.
        public readonly Vector3 LocalOffsetChild;
        public readonly Vector3 LocalOffsetParent;
        public readonly float MinDistance;
        public readonly float MaxDistance;
        public readonly float DistanceFrequency;
        public readonly float DistanceDamping;
        /// <summary>Torque the joint resists bending with. 0 keeps the stiff rig default (0.65).</summary>
        public float JointFriction { get; init; }

        public ExternalRigJointSpec(int parentIndex, int childIndex, Vector3 anchorWorld, float swingLimit,
            float initialSwingFactor = 0.28f, float poseGuideMaxForce = 0f, float poseGuideFrequency = 5f)
        {
            Kind = ExternalRigJointKind.BallSocketSwing;
            ParentIndex = parentIndex;
            ChildIndex = childIndex;
            AnchorWorld = anchorWorld;
            SwingLimit = swingLimit;
            InitialSwingFactor = Math.Clamp(initialSwingFactor, 0.05f, 1f);
            PoseGuideMaxForce = MathF.Max(0f, poseGuideMaxForce);
            PoseGuideFrequency = MathF.Max(0.1f, poseGuideFrequency);
            LocalOffsetChild = Vector3.Zero;
            LocalOffsetParent = Vector3.Zero;
            MinDistance = 0f;
            MaxDistance = 0f;
            DistanceFrequency = 0f;
            DistanceDamping = 0f;
        }

        private ExternalRigJointSpec(int parentIndex, int childIndex,
            Vector3 localOffsetChild, Vector3 localOffsetParent,
            float minDistance, float maxDistance, float frequency, float damping)
        {
            Kind = ExternalRigJointKind.DistanceLimit;
            ParentIndex = parentIndex;
            ChildIndex = childIndex;
            AnchorWorld = Vector3.Zero;
            SwingLimit = 0f;
            InitialSwingFactor = 1f;
            PoseGuideMaxForce = 0f;
            PoseGuideFrequency = 5f;
            LocalOffsetChild = localOffsetChild;
            LocalOffsetParent = localOffsetParent;
            MinDistance = MathF.Max(0f, minDistance);
            MaxDistance = MathF.Max(MinDistance, maxDistance);
            DistanceFrequency = MathF.Max(0.1f, frequency);
            DistanceDamping = Math.Clamp(damping, 0f, 8f);
        }

        /// <summary>A center-to-center (or offset) distance limit edge, e.g. a garment tube seam.</summary>
        public static ExternalRigJointSpec MakeDistanceLimit(int childIndex, int parentIndex,
            float minDistance, float maxDistance, float frequency = 15f, float damping = 1.5f)
            => new(parentIndex, childIndex, Vector3.Zero, Vector3.Zero, minDistance, maxDistance, frequency, damping);
    }

    internal sealed class ExternalRigHandle
    {
        internal readonly List<ExternalBodyHandle> Bodies = new();
        internal readonly List<ConstraintHandle> Constraints = new();
        internal readonly List<(int, int)> ConnectedPairs = new();
        // Swing-limit constraints that start tight (hold garment shape at spawn) and relax to their full
        // ROM over the first ~second. Kept separately from Constraints so the target ROM can be re-applied
        // each frame without re-scanning the mixed constraint list.
        internal readonly List<RigSwingConstraint> SwingConstraints = new();
        internal readonly List<RigPoseGuideConstraint> PoseGuideConstraints = new();
        internal int Generation;
        internal bool Removed;
    }

    internal struct RigSwingConstraint
    {
        public ConstraintHandle Handle;
        public Vector3 AxisLocalA;
        public Vector3 AxisLocalB;
        public SpringSettings Spring;
        public float TargetSwing;
    }

    internal struct RigPoseGuideConstraint
    {
        public ConstraintHandle Handle;
        public Quaternion TargetRelativeRotationLocalA;
        public float Frequency;
        public float MaxForce;
    }

    /// <summary>Fraction of a garment joint's full swing ROM used at spawn, before it relaxes to full over
    /// ~1s. Shared with the DismembermentController local-sim rig so both hosts spawn equally stiff.</summary>
    internal const float GarmentRigInitialSwingFactor = 0.28f;

    // The bodies and rigs added from outside (Ragdoll.ExternalBodies.cs).
    private readonly ExternalBodies external;

    private int externalBodyGeneration { get => external.Generation; set => external.Generation = value; }
    private List<ExternalBodyHandle> externalBodies => external.BodyList;
    private List<ExternalRigHandle> externalRigs => external.Rigs;
    private HashSet<int> externalDynamicBodyHandles => external.DynamicBodyHandles;
    private HashSet<int> externalRigDynamicBodyHandles => external.RigDynamicBodyHandles;
    private HashSet<int> externalRigNoRagdollContactBodyHandles => external.RigNoRagdollContactBodyHandles;
    private HashSet<(int, int)> externalRigConnectedPairs => external.RigConnectedPairs;
    private List<ExternalRigHandle> carriedRigs => external.CarriedRigs;

    private bool TryCreateExternalDynamicBodyCore(IReadOnlyList<ExternalShapePart> parts, float mass,
        Vector3 position, Quaternion orientation, Vector3 linearVelocity, Vector3 angularVelocity,
        out ExternalBodyHandle? handle)
    {
        handle = null;
        if (!isActive || simulation == null || bufferPool == null || parts.Count == 0)
            return false;

        try
        {
            var bodyHandle = external.AddDynamicBody(parts, mass, position, orientation, linearVelocity,
                angularVelocity, out var shapes);
            WakeRagdollBodiesForBiomechanicalSettle();

            handle = external.TrackDynamicBody(bodyHandle, shapes);
            prevAllAsleep = false;
            BeginBiomechanicalSettle(0.35f);
            return true;
        }
        catch (Exception ex)
        {
            log.Warning(ex, "Ragdoll: failed to create external body");
            return false;
        }
    }

    private bool TryCreateExternalRigCore(
        IReadOnlyList<ExternalRigBodySpec> bodySpecs,
        IReadOnlyList<ExternalRigJointSpec> jointSpecs,
        out ExternalRigHandle? handle,
        bool collideWithRagdoll = true)
    {
        handle = null;
        if (!isActive || simulation == null || bufferPool == null || bodySpecs.Count == 0)
            return false;

        var rig = new ExternalRigHandle { Generation = externalBodyGeneration };
        try
        {
            external.BuildRig(rig, bodySpecs, jointSpecs, collideWithRagdoll);
            handle = rig;
            prevAllAsleep = false;
            WakeRagdollBodiesForBiomechanicalSettle();
            BeginBiomechanicalSettle(0.45f);
            return true;
        }
        catch (Exception ex)
        {
            log.Warning(ex, "Ragdoll: failed to create external rig");
            RemoveExternalRig(rig);
            return false;
        }
    }

    private bool TrySetExternalBodyShapeCore(ExternalBodyHandle? handle,
        IReadOnlyList<ExternalShapePart> parts, float mass)
    {
        if (!external.TrySetBodyShape(handle, parts, mass))
            return false;

        WakeRagdollBodiesForBiomechanicalSettle();
        prevAllAsleep = false;
        BeginBiomechanicalSettle(0.25f);
        return true;
    }

    internal bool TryGetExternalBodyPose(ExternalBodyHandle? handle, out Vector3 position, out Quaternion orientation,
        out Vector3 linearVelocity, out Vector3 angularVelocity)
        => external.TryGetBodyPose(handle, out position, out orientation, out linearVelocity, out angularVelocity);

    internal bool TryGetExternalRigBodyPose(ExternalRigHandle? handle, int bodyIndex,
        out Vector3 position, out Quaternion orientation, out Vector3 linearVelocity, out Vector3 angularVelocity)
        => external.TryGetRigBodyPose(handle, bodyIndex, out position, out orientation, out linearVelocity,
            out angularVelocity);

    /// <summary>
    /// Carry one rig body to a pose (a garment segment that stays on the body). It remains a dynamic body
    /// and is simply re-placed every frame, with the velocity that takes it there.
    /// </summary>
    private bool TrySetExternalRigBodyPoseCore(ExternalRigHandle? handle, int bodyIndex,
        Vector3 position, Quaternion orientation, Vector3 linearVelocity, Vector3 angularVelocity)
        => external.TrySetRigBodyPose(handle, bodyIndex, position, orientation, linearVelocity, angularVelocity);

    /// <summary>Let one rig body start (or stop) colliding with the ragdoll's own bodies.</summary>
    private bool SetExternalRigBodyRagdollContactCore(ExternalRigHandle? handle, int bodyIndex, bool collide)
        => external.SetRigBodyRagdollContact(handle, bodyIndex, collide);

    /// <summary>Replace one rig body's collision shape (a garment segment collapsing flat). Goes through the
    /// recorded <see cref="TrySetExternalBodyShape"/>, so it stays with the ragdoll.</summary>
    private bool TrySetExternalRigBodyShapeCore(ExternalRigHandle? handle, int bodyIndex,
        IReadOnlyList<ExternalShapePart> parts, float mass)
    {
        if (handle == null || handle.Removed || handle.Generation != externalBodyGeneration ||
            simulation == null || bodyIndex < 0 || bodyIndex >= handle.Bodies.Count)
            return false;

        return TrySetExternalBodyShape(handle.Bodies[bodyIndex], parts, mass);
    }

    private bool TryApplyExternalRigVelocityDeltaCore(ExternalRigHandle? handle, Vector3 velocityDelta)
        => external.TryApplyRigVelocityDelta(handle, velocityDelta);

    private bool TryDampenExternalRigVelocityCore(ExternalRigHandle? handle,
        float horizontalScale, float verticalScale, float angularScale)
        => external.TryDampenRigVelocity(handle, horizontalScale, verticalScale, angularScale);

    /// <summary>Re-apply each garment swing constraint's ROM as <paramref name="factor"/> (0..1) of its
    /// stored target, widening the joints from their tight spawn ROM to full as the garment settles.</summary>
    private bool RelaxExternalRigSwingLimitsCore(ExternalRigHandle? handle, float factor)
        => external.RelaxRigSwingLimits(handle, factor);

    /// <summary>Fade temporary garment pose-guide servos from their captured handoff pose toward zero force.</summary>
    private bool ApplyExternalRigPoseGuidanceCore(ExternalRigHandle? handle, float strength)
        => external.ApplyRigPoseGuidance(handle, strength);

    private bool TrySetExternalBodyRotationCore(ExternalBodyHandle? handle, Quaternion orientation, float angularVelocityScale)
        => external.TrySetBodyRotation(handle, orientation, angularVelocityScale);

    private bool TryApplyExternalVelocityDeltaCore(ExternalBodyHandle? handle, Vector3 velocityDelta)
        => external.TryApplyVelocityDelta(handle, velocityDelta);

    private bool TryDampenExternalBodyVelocityCore(ExternalBodyHandle? handle,
        float horizontalScale, float verticalScale, float angularScale)
        => external.TryDampenBodyVelocity(handle, horizontalScale, verticalScale, angularScale);

    private void RemoveExternalBodyCore(ExternalBodyHandle? handle) => external.RemoveBody(handle);

    /// <summary>Removing a rig detaches each body from the weapon-contact surfaces (a recorded call) and removes it
    /// through the recorded <see cref="RemoveExternalBody"/>, so the ragdoll walks the bodies; the component does the
    /// rest.</summary>
    private void RemoveExternalRigCore(ExternalRigHandle? handle)
    {
        if (handle == null || handle.Removed)
            return;

        external.RemoveRigConstraints(handle);

        foreach (var bodyHandle in handle.Bodies)
        {
            if (handle.Generation == externalBodyGeneration) DetachWeaponContactBody(bodyHandle.Body);
            external.ForgetRigBody(bodyHandle);
            RemoveExternalBody(bodyHandle);
        }

        external.ForgetRig(handle);
    }

    private bool AnyExternalBodyAwake() => external.AnyAwake();

    /// <summary>
    /// Owner-supplied test: should this external rig ride along when the ragdoll is projected onto an
    /// anchor (a grab, or the standing hold)?
    ///
    /// A garment rig is held to the corpse by nothing but contact and friction — there is no joint
    /// tying it on. So when the standing support relocates the WHOLE rig in one step, the clothes are
    /// simply left standing on the ground: the body vanishes out of them. Carrying them by the same
    /// delta keeps them exactly where they sat on the body, drape and all.
    ///
    /// (A grab needs none of this. It holds only the bone it caught and lets the joints haul the rest,
    /// so the body travels continuously and its clothes come along by the contact that was holding them
    /// on in the first place. It is a relocation that leaves them behind, not motion.)
    ///
    /// But only garments still BEING WORN may come along; one that already slid off and is puddled on
    /// the floor must stay there, even though the corpse is very likely lying right on top of it. Only
    /// the rig's owner can tell those apart, so it supplies the test. Null = carry nothing.
    /// </summary>
    internal Func<ExternalRigHandle, bool>? CarryExternalRigOnAnchor { get; set; }

    /// <summary>
    /// Work out which rigs come along — BEFORE the body is moved.
    ///
    /// The test asks whether a garment is still on the corpse, so it has to be asked while the corpse
    /// is still under it. Ask afterwards and the answer is always no: the body is already in the
    /// grabber's fist and the clothes are still on the floor a metre below. That answer then locks
    /// itself in, because a garment that is never carried never closes the gap.
    ///
    /// The answer is a recorded decision, so this step stays with the ragdoll.
    /// </summary>
    private void CollectCarriedExternalRigs()
    {
        carriedRigs.Clear();
        if (CarryExternalRigOnAnchor == null || simulation == null) return;

        foreach (var rig in externalRigs)
        {
            if (rig.Removed || rig.Generation != externalBodyGeneration) continue;
            if (RecDecision("CarryExternalRigOnAnchor", CarryExternalRigOnAnchor(rig))) carriedRigs.Add(rig);
        }
    }

    /// <summary>
    /// The bodies and rigs added from outside (dropped gear, worn and dropped garments): their creation, shape, pose
    /// and velocity edits, removal, the contact filter sets the narrow phase reads for them, and carrying the worn
    /// rigs when the standing hold moves the ragdoll. A component (docs/ARCHITECTURE.md §8).
    /// </summary>
    private sealed class ExternalBodies
    {
        private readonly SimulationContext ctx;

        private int externalBodyGeneration;
        private readonly List<ExternalBodyHandle> externalBodies = new();
        private readonly List<ExternalRigHandle> externalRigs = new();
        private readonly HashSet<int> externalDynamicBodyHandles = new();
        private readonly HashSet<int> externalRigDynamicBodyHandles = new();
        private readonly HashSet<int> externalRigNoRagdollContactBodyHandles = new();
        private readonly HashSet<(int, int)> externalRigConnectedPairs = new();
        private readonly List<ExternalRigHandle> carriedRigs = new();

        internal ExternalBodies(SimulationContext ctx) { this.ctx = ctx; }

        /// <summary>Bumped when the simulation is torn down; handles stamped with an older value are dead.</summary>
        internal int Generation { get => externalBodyGeneration; set => externalBodyGeneration = value; }
        internal List<ExternalBodyHandle> BodyList => externalBodies;
        internal List<ExternalRigHandle> Rigs => externalRigs;
        internal HashSet<int> DynamicBodyHandles => externalDynamicBodyHandles;
        internal HashSet<int> RigDynamicBodyHandles => externalRigDynamicBodyHandles;
        internal HashSet<int> RigNoRagdollContactBodyHandles => externalRigNoRagdollContactBodyHandles;
        internal HashSet<(int, int)> RigConnectedPairs => externalRigConnectedPairs;
        internal List<ExternalRigHandle> CarriedRigs => carriedRigs;

        /// <summary>Add one dynamic body (the caller checked the simulation is up). Throws on failure.</summary>
        internal BodyHandle AddDynamicBody(IReadOnlyList<ExternalShapePart> parts, float mass,
            Vector3 position, Quaternion orientation, Vector3 linearVelocity, Vector3 angularVelocity,
            out List<TypedIndex> shapes)
        {
            var simulation = ctx.Simulation!;
            var shapeIndex = CreateExternalBodyShape(parts, out shapes, out var inertia, mass);
            var bodyHandle = simulation.Bodies.Add(BodyDescription.CreateDynamic(
                new RigidPose(position, Quaternion.Normalize(orientation)),
                default(BodyVelocity),
                inertia,
                new CollidableDescription(shapeIndex, 0.016f),
                new BodyActivityDescription(0.01f)));

            var body = simulation.Bodies.GetBodyReference(bodyHandle);
            body.Velocity.Linear = linearVelocity;
            body.Velocity.Angular = angularVelocity;
            body.Awake = true;
            externalDynamicBodyHandles.Add(bodyHandle.Value);
            return bodyHandle;
        }

        /// <summary>The handle of a body <see cref="AddDynamicBody"/> made, now tracked.</summary>
        internal ExternalBodyHandle TrackDynamicBody(BodyHandle bodyHandle, List<TypedIndex> shapes)
        {
            var handle = new ExternalBodyHandle
            {
                Body = bodyHandle,
                Shapes = shapes.ToArray(),
                Generation = externalBodyGeneration,
            };
            externalBodies.Add(handle);
            return handle;
        }

        /// <summary>Add a rig's bodies and joints and track it (the caller checked the simulation is up). Throws on
        /// failure, leaving what was added in <paramref name="rig"/> for the caller to remove.</summary>
        internal void BuildRig(ExternalRigHandle rig, IReadOnlyList<ExternalRigBodySpec> bodySpecs,
            IReadOnlyList<ExternalRigJointSpec> jointSpecs, bool collideWithRagdoll)
        {
            var simulation = ctx.Simulation!;
            for (var i = 0; i < bodySpecs.Count; i++)
            {
                var spec = bodySpecs[i];
                if (spec.Parts.Count == 0)
                    throw new InvalidOperationException($"External rig body '{spec.Name}' has no shapes.");

                var shapeIndex = CreateExternalBodyShape(spec.Parts, out var shapes, out var inertia, spec.Mass);
                var bodyHandle = simulation.Bodies.Add(BodyDescription.CreateDynamic(
                    new RigidPose(spec.Position, Quaternion.Normalize(spec.Orientation)),
                    default(BodyVelocity),
                    inertia,
                    new CollidableDescription(shapeIndex, 0.016f),
                    new BodyActivityDescription(0.01f)));

                var body = simulation.Bodies.GetBodyReference(bodyHandle);
                body.Velocity.Linear = spec.LinearVelocity;
                body.Velocity.Angular = spec.AngularVelocity;
                body.Awake = true;

                var bodyHandleWrapper = new ExternalBodyHandle
                {
                    Body = bodyHandle,
                    Shapes = shapes.ToArray(),
                    Generation = externalBodyGeneration,
                };
                rig.Bodies.Add(bodyHandleWrapper);
                externalBodies.Add(bodyHandleWrapper);
                externalDynamicBodyHandles.Add(bodyHandle.Value);
                externalRigDynamicBodyHandles.Add(bodyHandle.Value);
                if (!collideWithRagdoll)
                    externalRigNoRagdollContactBodyHandles.Add(bodyHandle.Value);
            }

            foreach (var joint in jointSpecs)
            {
                if (joint.ParentIndex < 0 || joint.ParentIndex >= rig.Bodies.Count ||
                    joint.ChildIndex < 0 || joint.ChildIndex >= rig.Bodies.Count ||
                    joint.ParentIndex == joint.ChildIndex)
                    continue;

                AddExternalRigJoint(rig, joint);
            }

            externalRigs.Add(rig);
        }

        private void AddExternalRigJoint(ExternalRigHandle rig, ExternalRigJointSpec joint)
        {
            var simulation = ctx.Simulation;
            if (simulation == null)
                return;

            var parent = rig.Bodies[joint.ParentIndex];
            var child = rig.Bodies[joint.ChildIndex];

            if (joint.Kind == ExternalRigJointKind.DistanceLimit)
            {
                rig.Constraints.Add(simulation.Solver.Add(child.Body, parent.Body, new DistanceLimit
                {
                    LocalOffsetA = joint.LocalOffsetChild,
                    LocalOffsetB = joint.LocalOffsetParent,
                    MinimumDistance = joint.MinDistance,
                    MaximumDistance = joint.MaxDistance,
                    SpringSettings = new SpringSettings(joint.DistanceFrequency, joint.DistanceDamping),
                }));

                var loD = Math.Min(parent.Body.Value, child.Body.Value);
                var hiD = Math.Max(parent.Body.Value, child.Body.Value);
                var pairD = (loD, hiD);
                rig.ConnectedPairs.Add(pairD);
                externalRigConnectedPairs.Add(pairD);
                return;
            }

            var parentBody = simulation.Bodies.GetBodyReference(parent.Body);
            var childBody = simulation.Bodies.GetBodyReference(child.Body);

            var anchorWorld = joint.AnchorWorld;
            var childLocalAnchor = Vector3.Transform(anchorWorld - childBody.Pose.Position, Quaternion.Inverse(childBody.Pose.Orientation));
            var parentLocalAnchor = Vector3.Transform(anchorWorld - parentBody.Pose.Position, Quaternion.Inverse(parentBody.Pose.Orientation));
            var jointSpring = new SpringSettings(9f, 1.4f);
            var limitSpring = new SpringSettings(5f, 1.6f);

            rig.Constraints.Add(simulation.Solver.Add(child.Body, parent.Body, new BallSocket
            {
                LocalOffsetA = childLocalAnchor,
                LocalOffsetB = parentLocalAnchor,
                SpringSettings = jointSpring,
            }));

            var childAxisWorld = NormalizeOrFallback(Vector3.Transform(Vector3.UnitY, childBody.Pose.Orientation), Vector3.UnitY);
            if (joint.SwingLimit > 0f)
            {
                var axisLocalA = Vector3.Normalize(Vector3.Transform(childAxisWorld, Quaternion.Inverse(childBody.Pose.Orientation)));
                var axisLocalB = Vector3.Normalize(Vector3.Transform(childAxisWorld, Quaternion.Inverse(parentBody.Pose.Orientation)));
                // Spawn at a fraction of the target ROM so the garment holds its worn shape on handoff, then
                // relax to full via RelaxExternalRigSwingLimits over ~1s.
                var target = Math.Clamp(joint.SwingLimit, 0.05f, MathF.PI - 0.05f);
                var handle = simulation.Solver.Add(child.Body, parent.Body, new SwingLimit
                {
                    AxisLocalA = axisLocalA,
                    AxisLocalB = axisLocalB,
                    MaximumSwingAngle = Math.Clamp(target * joint.InitialSwingFactor, 0.05f, MathF.PI - 0.05f),
                    SpringSettings = limitSpring,
                });
                rig.Constraints.Add(handle);
                rig.SwingConstraints.Add(new RigSwingConstraint
                {
                    Handle = handle,
                    AxisLocalA = axisLocalA,
                    AxisLocalB = axisLocalB,
                    Spring = limitSpring,
                    TargetSwing = target,
                });
            }

            rig.Constraints.Add(simulation.Solver.Add(child.Body, parent.Body, new AngularMotor
            {
                TargetVelocityLocalA = Vector3.Zero,
                Settings = new MotorSettings(joint.JointFriction > 0f ? joint.JointFriction : 0.65f, 0.45f),
            }));

            if (joint.PoseGuideMaxForce > 0f)
            {
                var target = Quaternion.Normalize(Quaternion.Inverse(childBody.Pose.Orientation) * parentBody.Pose.Orientation);
                var handle = simulation.Solver.Add(child.Body, parent.Body, new AngularServo
                {
                    TargetRelativeRotationLocalA = target,
                    SpringSettings = new SpringSettings(joint.PoseGuideFrequency, 1.15f),
                    ServoSettings = new ServoSettings(6f, 0f, joint.PoseGuideMaxForce),
                });
                rig.Constraints.Add(handle);
                rig.PoseGuideConstraints.Add(new RigPoseGuideConstraint
                {
                    Handle = handle,
                    TargetRelativeRotationLocalA = target,
                    Frequency = joint.PoseGuideFrequency,
                    MaxForce = joint.PoseGuideMaxForce,
                });
            }

            var lo = Math.Min(parent.Body.Value, child.Body.Value);
            var hi = Math.Max(parent.Body.Value, child.Body.Value);
            var pair = (lo, hi);
            rig.ConnectedPairs.Add(pair);
            externalRigConnectedPairs.Add(pair);
        }

        /// <summary>Replace a body's collision shape. The caller starts the settle window when this succeeds.</summary>
        internal bool TrySetBodyShape(ExternalBodyHandle? handle, IReadOnlyList<ExternalShapePart> parts, float mass)
        {
            var simulation = ctx.Simulation;
            var bufferPool = ctx.BufferPool;
            if (handle == null || handle.Removed || handle.Generation != externalBodyGeneration ||
                simulation == null || bufferPool == null || parts.Count == 0)
                return false;

            TypedIndex[]? newShapes = null;
            try
            {
                var newShape = CreateExternalBodyShape(parts, out var shapeList, out var inertia, mass);
                newShapes = shapeList.ToArray();
                var oldShapes = handle.Shapes;

                simulation.Bodies.SetShape(handle.Body, newShape);
                simulation.Bodies.SetLocalInertia(handle.Body, in inertia);
                var body = simulation.Bodies.GetBodyReference(handle.Body);
                body.Awake = true;

                handle.Shapes = newShapes;
                foreach (var shape in oldShapes)
                    try { simulation.Shapes.RemoveAndDispose(shape, bufferPool); } catch { }

                return true;
            }
            catch (Exception ex)
            {
                if (newShapes != null)
                    foreach (var shape in newShapes)
                        try { ctx.Simulation?.Shapes.RemoveAndDispose(shape, bufferPool); } catch { }
                ctx.Log.Warning(ex, "Ragdoll: failed to replace external body shape");
                return false;
            }
        }

        private TypedIndex CreateExternalBodyShape(IReadOnlyList<ExternalShapePart> parts,
            out List<TypedIndex> shapes, out BodyInertia inertia, float mass)
        {
            var simulation = ctx.Simulation;
            var bufferPool = ctx.BufferPool;
            shapes = new List<TypedIndex>(parts.Count + 1);
            TypedIndex shapeIndex;
            if (parts.Count == 1 &&
                parts[0].LocalPosition.LengthSquared() < 1e-6f &&
                IsIdentity(parts[0].LocalOrientation))
            {
                var h = parts[0].HalfExtents;
                shapeIndex = simulation!.Shapes.Add(new Box(h.X * 2f, h.Y * 2f, h.Z * 2f));
                shapes.Add(shapeIndex);
            }
            else
            {
                bufferPool!.Take<CompoundChild>(parts.Count, out var children);
                for (int i = 0; i < parts.Count; i++)
                {
                    var p = parts[i];
                    var h = p.HalfExtents;
                    var childShape = simulation!.Shapes.Add(new Box(h.X * 2f, h.Y * 2f, h.Z * 2f));
                    shapes.Add(childShape);
                    children[i] = new CompoundChild
                    {
                        ShapeIndex = childShape,
                        LocalPosition = p.LocalPosition,
                        LocalOrientation = p.LocalOrientation,
                    };
                }

                shapeIndex = simulation!.Shapes.Add(new Compound(children));
                shapes.Add(shapeIndex);
            }

            var half = ComputeExternalShapeHalf(parts);
            inertia = new Box(half.X * 2f, half.Y * 2f, half.Z * 2f)
                .ComputeInertia(MathF.Max(0.01f, mass));
            return shapeIndex;
        }

        internal bool TryGetBodyPose(ExternalBodyHandle? handle, out Vector3 position, out Quaternion orientation,
            out Vector3 linearVelocity, out Vector3 angularVelocity)
        {
            position = Vector3.Zero;
            orientation = Quaternion.Identity;
            linearVelocity = Vector3.Zero;
            angularVelocity = Vector3.Zero;

            var simulation = ctx.Simulation;
            if (handle == null || handle.Removed || handle.Generation != externalBodyGeneration || simulation == null)
                return false;

            try
            {
                var body = simulation.Bodies.GetBodyReference(handle.Body);
                position = body.Pose.Position;
                orientation = body.Pose.Orientation;
                linearVelocity = body.Velocity.Linear;
                angularVelocity = body.Velocity.Angular;
                return true;
            }
            catch
            {
                return false;
            }
        }

        internal bool TryGetRigBodyPose(ExternalRigHandle? handle, int bodyIndex,
            out Vector3 position, out Quaternion orientation, out Vector3 linearVelocity, out Vector3 angularVelocity)
        {
            position = Vector3.Zero;
            orientation = Quaternion.Identity;
            linearVelocity = Vector3.Zero;
            angularVelocity = Vector3.Zero;

            if (handle == null || handle.Removed || handle.Generation != externalBodyGeneration ||
                ctx.Simulation == null || bodyIndex < 0 || bodyIndex >= handle.Bodies.Count)
                return false;

            return TryGetBodyPose(handle.Bodies[bodyIndex], out position, out orientation,
                out linearVelocity, out angularVelocity);
        }

        internal bool TrySetRigBodyPose(ExternalRigHandle? handle, int bodyIndex,
            Vector3 position, Quaternion orientation, Vector3 linearVelocity, Vector3 angularVelocity)
        {
            var simulation = ctx.Simulation;
            if (handle == null || handle.Removed || handle.Generation != externalBodyGeneration ||
                simulation == null || bodyIndex < 0 || bodyIndex >= handle.Bodies.Count ||
                !float.IsFinite(position.LengthSquared()) || !float.IsFinite(orientation.LengthSquared()))
                return false;

            try
            {
                var body = simulation.Bodies.GetBodyReference(handle.Bodies[bodyIndex].Body);
                body.Pose.Position = position;
                body.Pose.Orientation = Quaternion.Normalize(orientation);
                body.Velocity.Linear = linearVelocity;
                body.Velocity.Angular = angularVelocity;
                body.Awake = true;
                return true;
            }
            catch
            {
                return false;
            }
        }

        internal bool SetRigBodyRagdollContact(ExternalRigHandle? handle, int bodyIndex, bool collide)
        {
            if (handle == null || handle.Removed || handle.Generation != externalBodyGeneration ||
                ctx.Simulation == null || bodyIndex < 0 || bodyIndex >= handle.Bodies.Count)
                return false;

            var body = handle.Bodies[bodyIndex].Body.Value;
            if (collide) externalRigNoRagdollContactBodyHandles.Remove(body);
            else externalRigNoRagdollContactBodyHandles.Add(body);
            return true;
        }

        internal bool TryApplyRigVelocityDelta(ExternalRigHandle? handle, Vector3 velocityDelta)
        {
            var simulation = ctx.Simulation;
            if (handle == null || handle.Removed || handle.Generation != externalBodyGeneration || simulation == null)
                return false;

            foreach (var bodyHandle in handle.Bodies)
            {
                try
                {
                    var body = simulation.Bodies.GetBodyReference(bodyHandle.Body);
                    body.Velocity.Linear += velocityDelta;
                    body.Awake = true;
                }
                catch
                {
                    return false;
                }
            }

            return true;
        }

        internal bool TryDampenRigVelocity(ExternalRigHandle? handle,
            float horizontalScale, float verticalScale, float angularScale)
        {
            var simulation = ctx.Simulation;
            if (handle == null || handle.Removed || handle.Generation != externalBodyGeneration || simulation == null)
                return false;

            horizontalScale = Math.Clamp(horizontalScale, 0f, 1f);
            verticalScale = Math.Clamp(verticalScale, 0f, 1f);
            angularScale = Math.Clamp(angularScale, 0f, 1f);

            foreach (var bodyHandle in handle.Bodies)
            {
                try
                {
                    var body = simulation.Bodies.GetBodyReference(bodyHandle.Body);
                    var lin = body.Velocity.Linear;
                    body.Velocity.Linear = new Vector3(lin.X * horizontalScale, lin.Y * verticalScale, lin.Z * horizontalScale);
                    body.Velocity.Angular *= angularScale;
                }
                catch
                {
                    return false;
                }
            }

            return true;
        }

        internal bool RelaxRigSwingLimits(ExternalRigHandle? handle, float factor)
        {
            var simulation = ctx.Simulation;
            if (handle == null || handle.Removed || handle.Generation != externalBodyGeneration || simulation == null)
                return false;

            factor = Math.Clamp(factor, 0f, 1f);
            foreach (var s in handle.SwingConstraints)
            {
                try
                {
                    simulation.Solver.ApplyDescription(s.Handle, new SwingLimit
                    {
                        AxisLocalA = s.AxisLocalA,
                        AxisLocalB = s.AxisLocalB,
                        MaximumSwingAngle = Math.Clamp(s.TargetSwing * factor, 0.05f, MathF.PI - 0.05f),
                        SpringSettings = s.Spring,
                    });
                }
                catch
                {
                    return false;
                }
            }

            return true;
        }

        internal bool ApplyRigPoseGuidance(ExternalRigHandle? handle, float strength)
        {
            var simulation = ctx.Simulation;
            if (handle == null || handle.Removed || handle.Generation != externalBodyGeneration || simulation == null)
                return false;

            strength = Math.Clamp(strength, 0f, 1f);
            foreach (var s in handle.PoseGuideConstraints)
            {
                try
                {
                    simulation.Solver.ApplyDescription(s.Handle, new AngularServo
                    {
                        TargetRelativeRotationLocalA = s.TargetRelativeRotationLocalA,
                        SpringSettings = new SpringSettings(s.Frequency, 1.15f),
                        ServoSettings = new ServoSettings(6f, 0f, s.MaxForce * strength),
                    });
                }
                catch
                {
                    return false;
                }
            }

            return true;
        }

        internal bool TrySetBodyRotation(ExternalBodyHandle? handle, Quaternion orientation, float angularVelocityScale)
        {
            var simulation = ctx.Simulation;
            if (handle == null || handle.Removed || handle.Generation != externalBodyGeneration || simulation == null)
                return false;

            try
            {
                var body = simulation.Bodies.GetBodyReference(handle.Body);
                body.Pose.Orientation = Quaternion.Normalize(orientation);
                body.Velocity.Angular *= Math.Clamp(angularVelocityScale, 0f, 1f);
                body.Awake = true;
                return true;
            }
            catch
            {
                return false;
            }
        }

        internal bool TryApplyVelocityDelta(ExternalBodyHandle? handle, Vector3 velocityDelta)
        {
            var simulation = ctx.Simulation;
            if (handle == null || handle.Removed || handle.Generation != externalBodyGeneration || simulation == null)
                return false;

            try
            {
                var body = simulation.Bodies.GetBodyReference(handle.Body);
                body.Velocity.Linear += velocityDelta;
                body.Awake = true;
                return true;
            }
            catch
            {
                return false;
            }
        }

        internal bool TryDampenBodyVelocity(ExternalBodyHandle? handle,
            float horizontalScale, float verticalScale, float angularScale)
        {
            var simulation = ctx.Simulation;
            if (handle == null || handle.Removed || handle.Generation != externalBodyGeneration || simulation == null)
                return false;

            try
            {
                horizontalScale = Math.Clamp(horizontalScale, 0f, 1f);
                verticalScale = Math.Clamp(verticalScale, 0f, 1f);
                angularScale = Math.Clamp(angularScale, 0f, 1f);

                var body = simulation.Bodies.GetBodyReference(handle.Body);
                var lin = body.Velocity.Linear;
                body.Velocity.Linear = new Vector3(lin.X * horizontalScale, lin.Y * verticalScale, lin.Z * horizontalScale);
                body.Velocity.Angular *= angularScale;
                return true;
            }
            catch
            {
                return false;
            }
        }

        internal void RemoveBody(ExternalBodyHandle? handle)
        {
            if (handle == null || handle.Removed)
                return;

            var simulation = ctx.Simulation;
            var bufferPool = ctx.BufferPool;
            if (simulation != null && bufferPool != null && handle.Generation == externalBodyGeneration)
            {
                ctx.RemoveConstraintsOnBody(handle.Body);
                try { simulation.Bodies.Remove(handle.Body); }
                catch (Exception ex) { ctx.Log.Warning(ex, $"Ragdoll: failed to remove body {handle.Body.Value}"); }
                foreach (var shape in handle.Shapes)
                    try { simulation.Shapes.RemoveAndDispose(shape, bufferPool); } catch { }
            }

            externalDynamicBodyHandles.Remove(handle.Body.Value);
            handle.Removed = true;
            externalBodies.Remove(handle);
        }

        /// <summary>First step of removing a rig: its constraints and its connected pairs.</summary>
        internal void RemoveRigConstraints(ExternalRigHandle handle)
        {
            var simulation = ctx.Simulation;
            if (simulation != null && handle.Generation == externalBodyGeneration)
            {
                // Never swallow this silently: a constraint that survives here is exactly what turns the
                // body removal below into solver corruption. RemoveExternalBody sweeps any leftovers, but
                // a failure here means our bookkeeping is wrong and we want to see it.
                foreach (var constraint in handle.Constraints)
                {
                    try { simulation.Solver.Remove(constraint); }
                    catch (Exception ex)
                    {
                        ctx.Log.Warning(ex, $"Ragdoll: failed to remove rig constraint {constraint.Value}");
                    }
                }
            }

            foreach (var pair in handle.ConnectedPairs)
                externalRigConnectedPairs.Remove(pair);
            handle.Constraints.Clear();
            handle.ConnectedPairs.Clear();
        }

        /// <summary>A rig body leaves the rig's contact filter sets (before the body itself is removed).</summary>
        internal void ForgetRigBody(ExternalBodyHandle bodyHandle)
        {
            externalRigNoRagdollContactBodyHandles.Remove(bodyHandle.Body.Value);
            externalRigDynamicBodyHandles.Remove(bodyHandle.Body.Value);
        }

        /// <summary>Last step of removing a rig, once its bodies are gone.</summary>
        internal void ForgetRig(ExternalRigHandle handle)
        {
            handle.Bodies.Clear();
            handle.Removed = true;
            externalRigs.Remove(handle);
        }

        internal bool AnyAwake()
        {
            var simulation = ctx.Simulation;
            if (simulation == null || externalBodies.Count == 0)
                return false;

            var anyAwake = false;
            for (int i = externalBodies.Count - 1; i >= 0; i--)
            {
                var handle = externalBodies[i];
                if (handle.Removed || handle.Generation != externalBodyGeneration)
                {
                    externalDynamicBodyHandles.Remove(handle.Body.Value);
                    externalBodies.RemoveAt(i);
                    continue;
                }

                try
                {
                    if (simulation.Bodies.GetBodyReference(handle.Body).Awake)
                        anyAwake = true;
                }
                catch
                {
                    externalDynamicBodyHandles.Remove(handle.Body.Value);
                    handle.Removed = true;
                    externalBodies.RemoveAt(i);
                }
            }

            return anyAwake;
        }

        // A carried garment's panels weigh a fraction of a limb, so an impulse a corpse would barely
        // register throws them clear across the room. The ragdoll's own bodies have had a per-substep
        // ceiling for exactly this reason (see ClampVelocities); external rigs never did, and being
        // teleported around a live scene is precisely when they need one.
        private const float CarriedRigMaxLinearVelocity = 6f;   // m/s
        private const float CarriedRigMaxAngularVelocity = 20f; // rad/s

        /// <summary>Move the rigs collected by <see cref="Ragdoll.CollectCarriedExternalRigs"/> by the same delta the
        /// ragdoll just took. Uniform translation, so a garment's shape and its position ON the body both survive.</summary>
        internal void TranslateCarried(Vector3 correction, Vector3 anchorVelocity)
        {
            var simulation = ctx.Simulation;
            if (simulation == null) return;

            foreach (var rig in carriedRigs)
            {
                if (rig.Removed || rig.Generation != externalBodyGeneration) continue;

                foreach (var handle in rig.Bodies)
                {
                    if (handle.Removed || handle.Generation != externalBodyGeneration) continue;
                    // Kinematic rig bodies are driven from elsewhere; moving them here would fight that.
                    if (!externalRigDynamicBodyHandles.Contains(handle.Body.Value)) continue;

                    var body = simulation.Bodies.GetBodyReference(handle.Body);
                    body.Pose.Position += correction;
                    body.Velocity.Linear = ClampVectorLength(
                        body.Velocity.Linear - anchorVelocity, CarriedRigMaxLinearVelocity);
                    body.Velocity.Angular = ClampVectorLength(
                        body.Velocity.Angular, CarriedRigMaxAngularVelocity);
                    body.Awake = true;
                }
            }
        }

        private static bool IsIdentity(Quaternion q)
            => MathF.Abs(q.X) < 1e-5f && MathF.Abs(q.Y) < 1e-5f &&
               MathF.Abs(q.Z) < 1e-5f && MathF.Abs(q.W - 1f) < 1e-5f;

        private static Vector3 ComputeExternalShapeHalf(IReadOnlyList<ExternalShapePart> parts)
        {
            var min = new Vector3(float.MaxValue);
            var max = new Vector3(float.MinValue);

            foreach (var p in parts)
            {
                var half = RotatedLocalHalfExtent(p.LocalOrientation, p.HalfExtents);
                min = Vector3.Min(min, p.LocalPosition - half);
                max = Vector3.Max(max, p.LocalPosition + half);
            }

            return new Vector3(
                MathF.Max(MathF.Abs(min.X), MathF.Abs(max.X)),
                MathF.Max(MathF.Abs(min.Y), MathF.Abs(max.Y)),
                MathF.Max(MathF.Abs(min.Z), MathF.Abs(max.Z)));
        }

        private static Vector3 RotatedLocalHalfExtent(Quaternion rot, Vector3 half)
        {
            var x = Vector3.Transform(Vector3.UnitX, rot);
            var y = Vector3.Transform(Vector3.UnitY, rot);
            var z = Vector3.Transform(Vector3.UnitZ, rot);
            return new Vector3(
                MathF.Abs(x.X) * half.X + MathF.Abs(y.X) * half.Y + MathF.Abs(z.X) * half.Z,
                MathF.Abs(x.Y) * half.X + MathF.Abs(y.Y) * half.Y + MathF.Abs(z.Y) * half.Z,
                MathF.Abs(x.Z) * half.X + MathF.Abs(y.Z) * half.Y + MathF.Abs(z.Z) * half.Z);
        }
    }
}
