// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System;
using System.Collections.Generic;
using System.Numerics;
using AdvancedRagdoll.Hosting;
using AdvancedRagdoll.SkeletonProfiles;
using BepuSimulation = BepuPhysics.Simulation;

namespace AdvancedRagdoll;

public unsafe partial class Ragdoll
{
    // Writing the simulated pose back onto the host skeleton every frame (Ragdoll.PoseWriting.cs).
    private readonly PoseWriter poseWriter;

    // The head bone index (the pose writer's; the rig build sets it): the hair rig is built on it.
    private int kaoBodyBoneIndex => poseWriter.HeadBoneIndex;

    private static bool IsDescendantOrSelf(IHostSkeleton skel, int bone, int root)
    {
        var guard = 0;
        while (bone >= 0 && guard++ < 256)
        {
            if (bone == root) return true;
            bone = skel.Parent(bone);
        }
        return false;
    }

    /// <summary>Index of a body bone by name, or -1 (what BoneTransformService.ResolveBoneIndex does).</summary>
    internal static int ResolveBoneIndex(IHostSkeleton skel, string boneName)
    {
        var count = Math.Min(skel.BoneCount, skel.BoneNameCount);
        for (int i = 0; i < count; i++)
        {
            var name = skel.BoneName(i);
            if (name == boneName) return i;
        }
        return -1;
    }

    /// <summary>
    /// Writes the simulated pose back onto the host skeleton every frame: the per-frame bone buffers (the bodies' bone
    /// poses this tick and the tick before, for render interpolation), the write of every simulated bone, the carrying
    /// of that motion to the bones without a body and to the partial skeletons on the head, and the collapsing of
    /// severed limbs. A component (docs/ARCHITECTURE.md §8). The stepping loop decides when the previous tick is
    /// captured and how far to blend; the ragdoll makes the recorded calls (deactivating on a non-finite body, the
    /// written-pose record) and the other subsystems' steps between the pose writer's steps.
    /// </summary>
    private sealed class PoseWriter
    {
        private readonly SimulationContext ctx;

        internal PoseWriter(SimulationContext ctx) { this.ctx = ctx; }

        private BepuSimulation? simulation => ctx.Simulation;
        private List<RagdollBone> ragdollBones => ctx.Bodies;
        private IRagdollLog log => ctx.Log;
        private SkeletonProfile skeletonProfile => ctx.Profile;

        // Reused per-frame buffers for StepAndApply (avoid per-frame GC pressure)
        private BoneModificationResult? cachedResult;
        private Vector3[]? cachedWorldPositions;
        private Quaternion[]? cachedWorldRotations;
        private bool[]? cachedBoneValid;

        // Render interpolation: physics advances in fixed 60Hz ticks but we render every frame
        // (often >60fps). Without interpolation the ragdoll shows the same pose for several
        // frames then jumps on each tick — visible micro-judder. We snapshot the bone world
        // pose just before each physics tick (prev) and after (cur, in cachedWorld*), then blend
        // by the leftover-accumulator fraction so the rendered pose advances smoothly every frame.
        private Vector3[]? prevWorldPositions;
        private Quaternion[]? prevWorldRotations;
        private readonly HashSet<int> ragdollBoneIndices = new();
        // Head bone index (the profile's head) — used for hair physics and partial skeleton propagation
        private int kaoBodyBoneIndex = -1;

        internal int HeadBoneIndex => kaoBodyBoneIndex;
        /// <summary>The bodies' bone world positions read this frame (<see cref="ReadBodies"/>).</summary>
        internal Vector3[] WorldPositions => cachedWorldPositions!;
        /// <summary>Which bodies were read this frame (<see cref="ReadBodies"/>).</summary>
        internal bool[] BoneValid => cachedBoneValid!;

        /// <summary>How the read of the bodies ended.</summary>
        internal enum BodyReadOutcome
        {
            /// <summary>Every body was read.</summary>
            Read,
            /// <summary>A body's pose was not finite (logged); the ragdoll deactivates and writes nothing.</summary>
            NotFinite,
        }

        /// <summary>Makes sure the per-frame buffers hold <paramref name="boneCount"/> bones; true when they were
        /// reallocated (the previous tick's snapshot is then stale).</summary>
        internal bool EnsureBuffers(int boneCount)
        {
            if (cachedWorldPositions == null || cachedWorldPositions.Length < boneCount)
            {
                cachedWorldPositions = new Vector3[boneCount];
                cachedWorldRotations = new Quaternion[boneCount];
                cachedBoneValid = new bool[boneCount];
                prevWorldPositions = new Vector3[boneCount];
                prevWorldRotations = new Quaternion[boneCount];
                return true;
            }
            return false;
        }

        // Snapshot the current reconstructed bone world pose into the interpolation prev buffers,
        // using the same capsule-center → bone-origin reconstruction as Pass 1. Called just before
        // each physics tick so prev holds the pre-tick state to blend from.
        internal void CapturePrevBodyPoses(int boneCount)
        {
            for (int i = 0; i < boneCount; i++)
            {
                var rb = ragdollBones[i];
                if (rb.BoneIndex < 0) continue;
                var b = simulation!.Bodies.GetBodyReference(rb.BodyHandle);
                prevWorldRotations![i] = Quaternion.Normalize(b.Pose.Orientation * rb.CapsuleToBoneOffset);
                var profileOrigin = b.Pose.Position - Vector3.Transform(rb.ShapeCenterOffset, b.Pose.Orientation);
                if (rb.SegmentHalfLength > 0)
                {
                    var y = Vector3.Transform(Vector3.UnitY, b.Pose.Orientation);
                    prevWorldPositions![i] = profileOrigin - rb.SegmentHalfLength * y;
                }
                else
                {
                    prevWorldPositions![i] = profileOrigin;
                }
            }
        }

        /// <summary>Saves the skeleton's pose as it is before the write (the originals the deltas are taken
        /// against).</summary>
        internal void BeginFrame(IHostSkeleton skel, Span<BoneQs> pose)
        {
            // Save original positions/rotations for delta tracking (needed for head propagation).
            // Reuse a cached result across frames — StepAndApply runs every render frame, so a
            // fresh allocation here is steady GC pressure per active ragdoll.
            var result = cachedResult ??= new BoneModificationResult(skel.BoneCount);
            result.Reset(skel.BoneCount);
            for (int i = 0; i < skel.BoneCount; i++)
            {
                ref var m = ref pose[i];
                result.OriginalPositions[i] = new Vector3(m.Translation.X, m.Translation.Y, m.Translation.Z);
                result.OriginalRotations[i] = new Quaternion(m.Rotation.X, m.Rotation.Y, m.Rotation.Z, m.Rotation.W);
            }
        }

        /// <summary>Pass 1: reads every body's bone world pose into the buffers. Stops at the first body whose pose
        /// is not finite (logged), so the ragdoll can deactivate.</summary>
        internal BodyReadOutcome ReadBodies(IHostSkeleton skel, int boneCount, out bool anyAwake)
        {
            // --- Pass 1: Read physics bodies, compute bone world positions/rotations ---
            // We need all positions first to measure how far the ragdoll sank below
            // the real terrain (due to the lowered physics ground), then correct uniformly.
            var worldPositions = cachedWorldPositions;
            var worldRotations = cachedWorldRotations!;
            var boneValid = cachedBoneValid!;
            Array.Clear(boneValid, 0, boneCount);
            anyAwake = false;

            for (int i = 0; i < boneCount; i++)
            {
                var rb = ragdollBones[i];
                if (rb.BoneIndex < 0 || rb.BoneIndex >= skel.BoneCount) continue;

                var bodyRef = simulation!.Bodies.GetBodyReference(rb.BodyHandle);
                if (bodyRef.Awake) anyAwake = true;

                // Guard against NaN from physics explosion — deactivate instead of crashing.
                // Sum-check covers ALL pose components: a NaN in Orientation.X/Y/Z with a finite W
                // used to slip through for one frame and reach the mesh as garbage vertices.
                if (float.IsNaN(bodyRef.Pose.Position.X + bodyRef.Pose.Position.Y + bodyRef.Pose.Position.Z +
                                bodyRef.Pose.Orientation.X + bodyRef.Pose.Orientation.Y +
                                bodyRef.Pose.Orientation.Z + bodyRef.Pose.Orientation.W))
                {
                    log.Warning($"Ragdoll: NaN detected in body '{rb.Name}', deactivating");
                    return BodyReadOutcome.NotFinite;
                }

                // Reconstruct bone world rotation from physics capsule rotation + stored offset
                worldRotations[i] = Quaternion.Normalize(bodyRef.Pose.Orientation * rb.CapsuleToBoneOffset);

                // Convert capsule center (at segment midpoint) back to bone origin position.
                // The bone is at the parent-end of the capsule: center - halfLength * capsuleY.
                var profileOrigin = bodyRef.Pose.Position -
                                    Vector3.Transform(rb.ShapeCenterOffset, bodyRef.Pose.Orientation);
                if (rb.SegmentHalfLength > 0)
                {
                    var capsuleYAxis = Vector3.Transform(Vector3.UnitY, bodyRef.Pose.Orientation);
                    worldPositions![i] = profileOrigin - rb.SegmentHalfLength * capsuleYAxis;
                }
                else
                {
                    worldPositions![i] = profileOrigin;
                }

                boneValid[i] = true;
            }
            return BodyReadOutcome.Read;
        }

        /// <summary>Pass 2: writes every read body's bone (blended from the previous tick by
        /// <paramref name="renderAlpha"/>), then carries the motion to the bones without a body and to the partial
        /// skeletons on the head.</summary>
        internal void WritePose(IHostSkeleton skel, Span<BoneQs> pose, int boneCount, float renderAlpha,
            float yCorrection, bool logThisFrame, int frameCount)
        {
            var result = cachedResult!;
            var worldPositions = cachedWorldPositions!;
            var worldRotations = cachedWorldRotations!;
            var boneValid = cachedBoneValid!;

            // --- Pass 2: Apply correction and write to ModelPose ---
            for (int i = 0; i < boneCount; i++)
            {
                if (!boneValid[i]) continue;
                var rb = ragdollBones[i];

                // Blend from the previous physics tick toward the current one for smooth motion
                // between 60Hz ticks. At alpha = 1 these return the current pose exactly.
                var boneWorldPos = renderAlpha >= 1f
                    ? worldPositions[i]
                    : Vector3.Lerp(prevWorldPositions![i], worldPositions[i], renderAlpha);
                var boneWorldRot = renderAlpha >= 1f
                    ? worldRotations[i]
                    : Quaternion.Slerp(prevWorldRotations![i], worldRotations[i], renderAlpha);
                boneWorldPos.Y += yCorrection;

                var modelPos = ctx.WorldToModel(boneWorldPos);
                var modelRot = ctx.WorldRotToModel(boneWorldRot);

                if (logThisFrame)
                {
                    var bodyRef = simulation!.Bodies.GetBodyReference(rb.BodyHandle);
                    var wr = worldRotations[i];
                    var lv = bodyRef.Velocity.Linear;
                    var av = bodyRef.Velocity.Angular;
                    log.Info($"[Ragdoll F{frameCount}] '{rb.Name}' " +
                             $"wPos=({boneWorldPos.X:F3},{boneWorldPos.Y:F3},{boneWorldPos.Z:F3}) " +
                             $"wRot=({wr.X:F3},{wr.Y:F3},{wr.Z:F3},{wr.W:F3}) " +
                             $"linV=({lv.X:F3},{lv.Y:F3},{lv.Z:F3}) angV=({av.X:F3},{av.Y:F3},{av.Z:F3}) " +
                             $"awake={bodyRef.Awake}");
                }

                WriteBoneTransform(skel, rb.BoneIndex, modelPos, modelRot, result);
            }

            // Propagate ragdoll movement to non-ragdoll descendant bones (hands, fingers, toes).
            // Without this, these bones stay at their frozen animation positions while their
            // ragdoll parent bones move via physics, causing mesh tearing at boundaries.
            for (int i = 0; i < skel.BoneCount && i < skel.ParentCount; i++)
            {
                if (ragdollBoneIndices.Contains(i)) continue; // already handled by physics

                var parentIdx = skel.Parent(i);
                if (parentIdx < 0 || !result.HasAccumulated[parentIdx]) continue;

                var parentDelta = result.AccumulatedDeltas[parentIdx];
                var parentOrigPos = result.OriginalPositions[parentIdx];
                ref var parentModel = ref pose[parentIdx];
                var parentNewPos = new Vector3(parentModel.Translation.X, parentModel.Translation.Y, parentModel.Translation.Z);

                // Rotate this bone around its parent's original position, then translate by parent displacement
                var relPos = result.OriginalPositions[i] - parentOrigPos;
                relPos = Vector3.Transform(relPos, parentDelta);
                var newPos = parentOrigPos + relPos + (parentNewPos - parentOrigPos);
                var newRot = Quaternion.Normalize(parentDelta * result.OriginalRotations[i]);

                WriteBoneTransform(skel, i, newPos, newRot, result);
            }

            // Propagate head changes to face/hair partial skeletons
            if (kaoBodyBoneIndex >= 0 && result.HasAccumulated[kaoBodyBoneIndex])
            {
                PropagateToPartialSkeletons(skel, kaoBodyBoneIndex, skeletonProfile.Landmarks.Head, result);
            }
        }

        /// <summary>Dismemberment POC: collapses each severed limb's whole bone subtree (and the partial skeletons
        /// on it) to ~0 scale so it vanishes from the body. Done last so it overrides the physics/propagation writes
        /// for those bones.</summary>
        internal void HideSeveredLimbs(IHostSkeleton skel, List<string>? severedBones)
        {
            if (severedBones != null && severedBones.Count > 0)
                foreach (var boneName in severedBones)
                    HideLimbSubtree(skel, boneName);
        }

        // Collapse the given limb's entire bone subtree to ~0 scale so the limb vanishes from the body.
        private void HideLimbSubtree(IHostSkeleton skel, string rootName)
        {
            if (string.IsNullOrEmpty(rootName)) return;

            var rootIdx = ResolveBoneIndex(skel, rootName);
            if (rootIdx < 0)
            {
                foreach (var rb in ragdollBones)
                    if (rb.Name == rootName) { rootIdx = rb.BoneIndex; break; }
            }
            if (rootIdx < 0) return;

            var pose = skel.ModelPose;
            // Severance point = the root bone's model-space position (its joint). Collapse every subtree
            // bone to BOTH ~0 scale AND that single point, so the limb has no length and no leftover
            // sliver — it fully vanishes into the joint instead of compressing into a thin model.
            ref var rootM = ref pose[rootIdx];
            var rx = rootM.Translation.X;
            var ry = rootM.Translation.Y;
            var rz = rootM.Translation.Z;

            var n = Math.Min(skel.BoneCount, skel.ParentCount);
            for (int i = 0; i < n; i++)
            {
                if (!IsDescendantOrSelf(skel, i, rootIdx)) continue;
                ref var m = ref pose[i];
                m.Translation.X = rx;
                m.Translation.Y = ry;
                m.Translation.Z = rz;
                m.Scale.X = 0.0001f;
                m.Scale.Y = 0.0001f;
                m.Scale.Z = 0.0001f;
            }

            // Face/hair (and similar) live on SEPARATE partial skeletons attached to a body bone, so they
            // are not in the main parent chain above — collapse those too or the head stays visible.
            HideSubtreePartialSkeletons(skel, rootIdx, new Vector3(rx, ry, rz));
        }

        private void HideSubtreePartialSkeletons(IHostSkeleton skel, int rootIdx, Vector3 collapsePoint)
        {
            if (!skel.HasRoot) return;

            for (int ps = 1; ps < skel.PartCount; ps++)
            {
                if (!skel.PartAvailable(ps) || !skel.PartInSync(ps)) continue;

                var pose = skel.PartModelPose(ps);
                var cnt = pose.Length;
                if (cnt < 1) continue;

                // This partial attaches to its root bone (e.g. the head); collapse it only if that bone is
                // inside the hidden subtree.
                var rootName = skel.PartBoneName(ps, 0);
                if (string.IsNullOrEmpty(rootName)) continue;
                var mainIdx = ResolveBoneIndex(skel, rootName);
                if (mainIdx < 0 || !IsDescendantOrSelf(skel, mainIdx, rootIdx)) continue;

                for (int b = 0; b < cnt; b++)
                {
                    ref var m = ref pose[b];
                    m.Translation.X = collapsePoint.X;
                    m.Translation.Y = collapsePoint.Y;
                    m.Translation.Z = collapsePoint.Z;
                    m.Scale.X = 0.0001f;
                    m.Scale.Y = 0.0001f;
                    m.Scale.Z = 0.0001f;
                }
            }
        }

        /// <summary>
        /// Writes a bone's model-space transform and records the delta (what BoneTransformService.WriteBoneTransform
        /// does to the game's memory, here on any skeleton).
        /// </summary>
        private static void WriteBoneTransform(
            IHostSkeleton skel,
            int boneIndex,
            Vector3 newPos,
            Quaternion newRot,
            BoneModificationResult result)
        {
            if (boneIndex < 0 ||
                boneIndex >= skel.BoneCount ||
                boneIndex >= result.OriginalRotations.Length ||
                boneIndex >= result.AccumulatedDeltas.Length ||
                boneIndex >= result.HasAccumulated.Length)
                return;

            ref var model = ref skel.ModelPose[boneIndex];
            model.Translation.X = newPos.X;
            model.Translation.Y = newPos.Y;
            model.Translation.Z = newPos.Z;
            model.Rotation.X = newRot.X;
            model.Rotation.Y = newRot.Y;
            model.Rotation.Z = newRot.Z;
            model.Rotation.W = newRot.W;

            result.AccumulatedDeltas[boneIndex] = Quaternion.Normalize(
                newRot * Quaternion.Inverse(result.OriginalRotations[boneIndex]));
            result.HasAccumulated[boneIndex] = true;
        }

        /// <summary>
        /// Carries a connection bone's change over to every partial skeleton rooted at that bone (what
        /// BoneTransformService.PropagateToPartialSkeletons does to the game's memory, here on any skeleton).
        /// </summary>
        private static void PropagateToPartialSkeletons(
            IHostSkeleton skel,
            int boneIndex,
            string boneName,
            BoneModificationResult result)
        {
            if (!result.HasAccumulated[boneIndex]) return;

            var delta = result.AccumulatedDeltas[boneIndex];
            ref var boneModel = ref skel.ModelPose[boneIndex];
            var displacement = new Vector3(boneModel.Translation.X, boneModel.Translation.Y, boneModel.Translation.Z)
                               - result.OriginalPositions[boneIndex];
            var origBonePos = result.OriginalPositions[boneIndex];

            for (int ps = 1; ps < skel.PartCount; ps++)
            {
                if (!skel.PartAvailable(ps)) continue;
                if (!skel.PartInSync(ps)) continue;

                var otherPose = skel.PartModelPose(ps);
                var otherBoneCount = otherPose.Length;
                if (otherBoneCount < 1) continue;

                var rootName = skel.PartBoneName(ps, 0);
                if (rootName != boneName) continue;

                var otherParentCount = skel.PartParentCount(ps);
                for (int b = 0; b < otherBoneCount && b < otherParentCount; b++)
                {
                    ref var bm = ref otherPose[b];
                    var bOldPos = new Vector3(bm.Translation.X, bm.Translation.Y, bm.Translation.Z);
                    var bOldRot = new Quaternion(bm.Rotation.X, bm.Rotation.Y, bm.Rotation.Z, bm.Rotation.W);

                    var relToRoot = bOldPos - origBonePos;
                    relToRoot = Vector3.Transform(relToRoot, delta);
                    var bNewPos = origBonePos + relToRoot + displacement;
                    var bNewRot = Quaternion.Normalize(delta * bOldRot);

                    bm.Translation.X = bNewPos.X;
                    bm.Translation.Y = bNewPos.Y;
                    bm.Translation.Z = bNewPos.Z;
                    bm.Rotation.X = bNewRot.X;
                    bm.Rotation.Y = bNewRot.Y;
                    bm.Rotation.Z = bNewRot.Z;
                    bm.Rotation.W = bNewRot.W;
                }
            }
        }

        // a6-12
        /// <summary>Deactivation: the rig's bone index set goes.</summary>
        internal void ClearBoneIndices() => ragdollBoneIndices.Clear();

        /// <summary>Deactivation: no head bone.</summary>
        internal void ResetHeadBoneIndex() => kaoBodyBoneIndex = -1;

        /// <summary>The rig build: the bone index set of the new rig (fast lookup during propagation).</summary>
        internal void CollectBoneIndices()
        {
            ragdollBoneIndices.Clear();
            foreach (var rb in ctx.Bodies)
                ragdollBoneIndices.Add(rb.BoneIndex);
        }

        /// <summary>The rig build: the head bone of this skeleton (-1 when it has none).</summary>
        internal void SetHeadBoneIndex(int index) => kaoBodyBoneIndex = index;
    }
}
