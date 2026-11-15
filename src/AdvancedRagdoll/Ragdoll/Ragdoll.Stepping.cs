// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System;
using System.Numerics;
using AdvancedRagdoll.Recording;
using BepuPhysics;
using AdvancedRagdoll.Hosting;

namespace AdvancedRagdoll;

public unsafe partial class Ragdoll
{
    private void OnRenderFrameCore()
    {
        try
        {
            // The settings for this frame (and for everything until the next frame starts).
            settingsSource.Refresh(settings);
        }
        catch (Exception ex)
        {
            // Nothing may escape this handler: the render hook calls every controller through one multicast
            // delegate, and an exception would skip the controllers after this one.
            log.Error(ex, "Ragdoll: the settings could not be read; this frame is skipped");
            return;
        }
        try { meshVolumes.UpdateFrozenMeshCache(); }
        catch (Exception ex) { log.Warning(ex, "Frozen collision cache capture failed; retrying when the model is ready."); }
        if (!isActive)
        {
            try { hairRig.OnAnimatedFrame(); }
            catch (Exception ex) { hairRig.Remove(); log.Error(ex, "Animated hair update failed"); }
            return;
        }

        try
        {
            if (!settings.RagdollHairPhysics && hairRig.Active) hairRig.Remove();
            var dt = ComputeFrameDelta();
            elapsed += dt;
            if (rec != null) RecRenderFrame(dt);

            strike.CountDownWindow(dt);
            UpdateWeaponStrike(dt);

            // Wait for activation delay (death animation plays first)
            if (!physicsStarted && !TryStartPhysics(dt)) return;

            var stepStart = System.Diagnostics.Stopwatch.GetTimestamp();
            StepAndApply(dt);
            costTotalTicks += System.Diagnostics.Stopwatch.GetTimestamp() - stepStart;
            costFrames++;
        }
        catch (Exception ex)
        {
            log.Error(ex, "Ragdoll: Error in render frame");
            Deactivate();
        }
    }

    /// <summary>
    /// The handoff from the death animation to physics: waits for the start (the animated pose keeps being sampled),
    /// builds the rig and starts what was armed for it. False when this frame ends here: still waiting, or the rig
    /// could not be built (the ragdoll is deactivated then).
    /// </summary>
    private bool TryStartPhysics(float dt)
    {
        if (rec != null) RecPreFrame();
        if (!RecDecision("ShouldStartPhysics", ShouldStartPhysics(dt)))
        {
            hairRig.OnAnimatedFrame();
            // Keep sampling the animated pose so the last frame before handoff carries
            // its velocity into the physics bodies (no freeze on activation).
            SampleHandoffPose(dt);
            return false;
        }
        hairRig.ResetForPhysicsStart();
        if (!InitializePhysics()) { Deactivate(); return false; }
        // Physics has started: the frame counter, the settle window and a clean accumulator (don't carry the
        // activation-delay wait).
        stepper.StartPhysics();

        if (pendingDirectedCollapseSpike)
        {
            collapse.DisarmDirected();
            if (!string.IsNullOrWhiteSpace(pendingDirectedProfileId))
                BeginProfileDirectedCollapseSpike(pendingDirectedProfileId);
            else
                BeginDirectedKneelPitchSpike(
                    pendingDirectedDuration,
                    pendingDirectedFootForce,
                    pendingDirectedPelvisForce,
                    pendingDirectedDrop,
                    pendingDirectedForward,
                    pendingDirectedPitchDegrees);
        }

        if (pendingEntryConditionedKneePowerLoss)
        {
            collapse.DisarmEntryConditioning();
            if (!BeginCollapseEntryConditioning(BeginKneePowerLossForwardPattern))
                BeginKneePowerLossForwardPattern();
        }

        if (pendingKneePowerLossPattern)
        {
            knee.Disarm();
            BeginKneePowerLossForwardPattern();
        }
        return true;
    }

    /// <summary>
    /// Real wall-clock seconds since the previous render frame, clamped to a sane range.
    /// Returns one fixed timestep on the very first call (no prior timestamp yet).
    /// </summary>
    private float ComputeFrameDelta() => stepper.ComputeFrameDelta();

    private void BeginBiomechanicalSettle(float duration = BiomechanicalSettleDuration)
        => ctx.BeginBiomechanicalSettle(duration);

    private void WakeRagdollBodiesForBiomechanicalSettle() => ctx.WakeBodies();

    // The fall gravity (Stepper.ApplyFallGravity): the water, a grab or the standing hold and the ground height come
    // from their components.
    private void ApplyFallGravity(float dt)
        => stepper.ApplyFallGravity(dt, water.InWater, grabSlots.Count > 0 || standing.Active, groundY);

    private static float BodyBottom(in RagdollBone bone, BodyReference body)
    {
        if (bone.MeshGeometry != null)
            return bone.MeshGeometry.Bottom(body.Pose.Position, body.Pose.Orientation);
        if (bone.HullCorners != null)
        {
            var lowest = float.MaxValue;
            foreach (var corner in bone.HullCorners)
                lowest = MathF.Min(lowest, Vector3.Transform(corner, body.Pose.Orientation).Y);
            return body.Pose.Position.Y + lowest;
        }
        if (bone.ColliderShape == RagdollColliderShape.Box)
        {
            var x = Vector3.Transform(Vector3.UnitX, body.Pose.Orientation);
            var y = Vector3.Transform(Vector3.UnitY, body.Pose.Orientation);
            var z = Vector3.Transform(Vector3.UnitZ, body.Pose.Orientation);
            var verticalExtent = MathF.Abs(x.Y) * bone.BoxHalfExtents.X +
                                 MathF.Abs(y.Y) * bone.BoxHalfExtents.Y +
                                 MathF.Abs(z.Y) * bone.BoxHalfExtents.Z;
            return body.Pose.Position.Y - verticalExtent;
        }

        var capsuleY = Vector3.Transform(Vector3.UnitY, body.Pose.Orientation);
        return body.Pose.Position.Y - MathF.Abs(capsuleY.Y) * bone.CapsuleHalfLength - bone.CapsuleRadius;
    }

    /// <summary>
    /// True when the ragdoll target is still a live object. Matches against the object table rather
    /// than dereferencing <see cref="targetCharacterAddress"/> — once the actor is freed, its memory
    /// often still reads back the stale EntityId, so a pointer-based check happily passes and we hand
    /// a dangling pointer to native game code (that is an access violation, not a catchable exception).
    /// </summary>
    private bool TargetObjectAlive() => ctx.TargetObjectAlive();

    private void StepAndApply(float dt)
    {
        if (simulation == null) return;

        var skel = BeginStepFrame(dt);
        if (skel == null) return;

        // The local player is always object-table index 0. Used below to choose a render-only
        // follow (the render position, never the packet-synced GameObject.Position) and to
        // suppress the self-induced biomechanical settle while follow drags the corpse.
        var isLocalPlayerTarget =
            actors.GetObjectIndex(targetCharacterAddress) == 0;

        var skeletonMoved = TrackSkeletonRoot(skel, isLocalPlayerTarget);

        if (skeletonMoved)
            BeginBiomechanicalSettle();

        // Consume Framework.Update requests and drive the finite-mass swarm carriers only from
        // the render/physics owner. This ordering also makes their target velocity visible to the
        // resting decision below before the simulation is allowed to skip a step.
        UpdateNpcTraversalProxies(dt);

        var resting = DecideResting(dt, skeletonMoved);
        var colliderStart = System.Diagnostics.Stopwatch.GetTimestamp();
        characterColliders.ClearMovedNearCorpse(); // re-armed by this frame's collider updates for the next decision

        // Collapse controllers restrength their servos before stepping so updated torque ceilings
        // take effect this tick; each advances its own phase and retires itself.
        UpdateDirectedCollapseSpike(dt);
        UpdateCollapseEntryConditioning(dt);
        UpdateKneePowerLossPattern(dt);

        TrackCharacterColliders(dt);

        costColliderTicks += System.Diagnostics.Stopwatch.GetTimestamp() - colliderStart;
        var physicsStart = System.Diagnostics.Stopwatch.GetTimestamp();

        // Ensure per-frame buffers exist (cur reconstruction + interpolation prev), sized to
        // the bone count. Allocated here (before stepping) so prev can be snapshotted in the loop.
        var boneCount = ragdollBones.Count;
        if (poseWriter.EnsureBuffers(boneCount))
        {
            stepper.InvalidatePrevState(); // buffers reallocated — old snapshot is stale
        }

        // Restore any recoil-relaxed joint motors whose window has elapsed.
        TickRecoilRelaxers(dt);

        var substeps = StepPhysics(dt, resting, boneCount);
        costPhysicsTicks += System.Diagnostics.Stopwatch.GetTimestamp() - physicsStart;

        CacheNpcTraversalProxyPoses();

        // Snap any soft-tissue body that diverged/NaN'd this frame back onto its anchor
        // BEFORE the write-back reads body poses — flung flesh must never reach the mesh.
        ContainSoftTissueBodies();
        surfaceRuntime.Update();

        stepper.CountDownSettle(dt);

        // Fraction of a tick elapsed past the last completed tick. The write-back blends the
        // current physics pose toward it from the previous tick's pose, so the rendered ragdoll
        // advances a little every frame instead of snapping once per 60Hz tick. Resting or no
        // prior tick → render the current pose directly (alpha = 1).
        var renderAlpha = stepper.RenderAlpha(resting);
        if (rec != null) RecFrameEnd(substeps, resting, renderAlpha);

        if (!WriteFramePose(skel, dt, boneCount, renderAlpha)) return;
        UpdateFollowTarget(isLocalPlayerTarget);
    }

    /// <summary>The start of a step: the target is still the same character, its skeleton is readable and unchanged,
    /// the frame is recorded and the animation stays frozen. Null when this frame ends here (the ragdoll may have been
    /// deactivated, or the physics torn down for a rebuild).</summary>
    private IHostSkeleton? BeginStepFrame(float dt)
    {
        // Liveness guard: if the corpse despawned and its object-table slot was reused, the
        // address now points at an unrelated character with a different EntityId. Without this,
        // we would freeze that newcomer's animation and overwrite its bones with our ragdoll
        // pose. Deactivate cleanly instead (restores nothing — the original target is gone).
        if (targetCharacterAddress == nint.Zero ||
            actors.GetEntityId(targetCharacterAddress) != targetEntityId)
        {
            log.Info("Ragdoll: target despawned or slot reused; deactivating.");
            animationFrozen = false; // do not write OverallSpeed onto whatever now occupies the slot
            Deactivate();
            return null;
        }

        var skel = skeletons.TryGetSkeleton(targetCharacterAddress, PosePurpose.FrameIn);
        if (skel == null) return null;
        if (rec != null) RecFrameBegin(dt, skel);

        if (activeRagdollMeshVolumes != settings.RagdollMeshVolumes ||
            (activeRagdollMeshVolumes && activeMeshCacheRevision != settings.RagdollMeshCacheRevision))
        {
            log.Info("Frozen mesh collision option/cache refreshed; rebuilding physical geometry.");
            hairRig.Remove();
            DestroySimulation();
            physicsStarted = false;
            return null;
        }

        // Manual refresh above can rebuild a changed skeleton. Without refresh, stop
        // before using old bone indices; never read a new model to replace the cache.
        if (skel.BoneCount != initialBoneCount)
        {
            log.Info($"Ragdoll: skeleton bone count changed ({initialBoneCount}->{skel.BoneCount}); deactivating.");
            Deactivate();
            return null;
        }

        // Keep animation frozen (game may recalculate OverallSpeed each frame)
        actors.SetAnimationSpeed(targetCharacterAddress, 0f);
        return skel;
    }

    /// <summary>Follows the skeleton root (the game may move it), re-raycasting the ground after a real move. True when
    /// the move was not ours (a follow of the local player) and the rig must settle again.</summary>
    private bool TrackSkeletonRoot(IHostSkeleton skel, bool isLocalPlayerTarget)
    {
        // Update skeleton transform for WorldToModel conversion.
        // The game may reposition the character root (e.g., dismount, death transition).
        // Physics bodies stay at correct world positions; we just need the current
        // transform to convert back to the model space the game expects for rendering.
        var skeletonMoved = false;
        if (skel.HasRoot)
        {
            var skeletonRoot = skel.Root;
            var newSkelPos = skeletonRoot.Position;

            // If skeleton moved significantly, re-raycast ground at new position
            var skelDist = (newSkelPos - skelWorldPos).Length();
            if (skelDist > 0.1f)
            {
                // A follow-driven move (we pushed the draw object to track the flung corpse)
                // must not restart the biomechanical settle — otherwise a corpse sliding under
                // follow wakes its bodies every frame and never rests. Only settle on EXTERNAL
                // repositions (dismount / death transition). followWasActive means we wrote a
                // follow position on the previous frame, so this frame's skeleton move is ours.
                // Ground is still re-raycast below in both cases so the floor tracks the corpse.
                if (!(followWasActive && isLocalPlayerTarget))
                    skeletonMoved = true;
                if (settings.RagdollVerboseLog)
                    log.Info($"[Ragdoll F{frameCount}] Skeleton moved {skelDist:F3}m: ({skelWorldPos.X:F3},{skelWorldPos.Y:F3},{skelWorldPos.Z:F3})→({newSkelPos.X:F3},{newSkelPos.Y:F3},{newSkelPos.Z:F3})");
                if (world.Raycast(RaycastSite.SkeletonMoved,
                        new Vector3(newSkelPos.X, newSkelPos.Y + TerrainRaycastStartYOffset, newSkelPos.Z),
                        new Vector3(0, -1, 0),
                        TerrainRaycastDistance,
                        out var hitInfo))
                {
                    ground.SetFromRaycast(hitInfo.Point.Y);
                    if (settings.RagdollVerboseLog)
                        log.Info($"[Ragdoll F{frameCount}] Ground re-raycast: realY={realGroundY:F3} physicsY={groundY:F3}");
                }
            }

            skelWorldPos = newSkelPos;
            skelWorldRot = skeletonRoot.Rotation;
            skelWorldRotInv = Quaternion.Inverse(skelWorldRot);
        }
        return skeletonMoved;
    }

    /// <summary>Whether the rig rests this frame (no step): BEPU has it asleep and nothing around it is active. Wakes
    /// the rig in the settle window, refreshes the corpse bounds and probes the water on the way.</summary>
    private bool DecideResting(float dt, bool skeletonMoved)
    {
        var biomechanicalSettleActive = biomechanicalSettleRemaining > 0f;
        if (biomechanicalSettleActive)
            WakeRagdollBodiesForBiomechanicalSettle();

        // Resting fast-path: once the rig is fully asleep there is nothing left to react to (an
        // asleep body has ~0 velocity, so there is no energy left to pump), so skip the physics
        // step, the per-frame NPC-collision static rebuild, and hair — we just re-assert the held
        // pose below. Applies to generic (monster) rigs too: if their auto-built network never
        // settles, prevAllAsleep simply stays false and this never triggers; if it does settle,
        // there is nothing to clamp. A grab or a moved skeleton forces a step.
        var externalBodyAwake = AnyExternalBodyAwake();

        // Rough corpse centre for the collider proximity gate. Any body serves — the wake
        // radius dwarfs the rig's own extent.
        if (ragdollBones.Count > 0)
            characterColliders.SetWakeCenter(simulation!.Bodies.GetBodyReference(ragdollBones[0].BodyHandle).Pose.Position);
        RefreshCorpseBounds();

        water.UpdateProbe(dt);
        var activityBlocksRest = water.InWater || externalBodyAwake || hairRig.NeedsStep || npcTraversalProxyActivityThisFrame ||
            npcColliderMovedNearCorpse || grabSlots.Count > 0 ||
            directedCollapseActive ||
            entryConditioningActive || kneePowerLossActive || skeletonMoved || biomechanicalSettleActive;
        // BEPU owns rest detection for the whole constraint island. No ground-height heuristic or
        // velocity zeroing may freeze a distal limb before contact has actually settled it.
        var resting = stepper.IsResting(activityBlocksRest);
        if (!resting)
            costStepped++;
        return resting;
    }

    /// <summary>
    /// Update NPC collision volumes to track their current animated bone positions.
    /// Must call UpdateBounds() after repositioning — BEPU2 doesn't auto-update
    /// broad phase AABBs for statics, so without it collisions are never detected.
    /// The grabbing NPC's collision is parked far away so the player body can pass through.
    /// Collider poses continue to track while the island sleeps. A nearby moving collider marks
    /// activity, wakes the island through contact, and prevents the resting fast path next frame.
    /// </summary>
    private void TrackCharacterColliders(float dt)
    {
        var npcCollisionParkPos = new Vector3(0, -9999, 0);
        for (int i = 0; i < npcCollisionStates.Count; i++)
        {
            var npcState = npcCollisionStates[i];
            try
            {
                if (suspendedNpcAddress != nint.Zero && npcState.NpcAddress == suspendedNpcAddress)
                {
                    if (npcState.IsMesh)
                    {
                        MoveNpcKinematic(ref npcState, npcState.MeshHandle, npcCollisionParkPos, Quaternion.Identity, dt);
                    }
                    else if (npcState.IsFallback)
                    {
                        MoveNpcKinematic(ref npcState, npcState.FallbackHandle, npcCollisionParkPos, Quaternion.Identity, dt);
                    }
                    else
                    {
                        for (int b = 0; b < npcState.BoneStatics.Count; b++)
                        {
                            var bs = npcState.BoneStatics[b];
                            MoveNpcBoneKinematic(ref bs, npcCollisionParkPos, Quaternion.Identity, dt);
                            npcState.BoneStatics[b] = bs;
                        }
                    }
                    characterColliders.SetState(i, npcState);
                    continue;
                }

                // Distance gate: a character that cannot reach the corpse cannot contact it, so
                // skip the expensive skeleton read + per-bone reposition.
                var npcRootPos = actors.GetPosition(npcState.NpcAddress, ActorSite.ColliderRoot);
                if (rec != null) RecActorPosition(ActorSite.ColliderRoot, npcState.NpcAddress, npcRootPos);
                if (!NpcMayTouchCorpse(npcState, npcRootPos))
                {
                    // Park its capsules far below ONCE on exit so a character that walks away
                    // from the corpse doesn't leave its body-shaped statics frozen on top of
                    // the ragdoll (invisible "ghost" collision). Skip cheaply thereafter.
                    if (!npcState.Parked)
                    {
                        ParkNpcStatics(ref npcState, npcCollisionParkPos, dt);
                        npcState.Parked = true;
                        characterColliders.SetState(i, npcState);
                    }
                    continue;
                }
                costTracked++;
                if (npcState.Parked)
                {
                    // Back in range — the reposition below snaps the statics onto the skeleton.
                    npcState.Parked = false;
                    characterColliders.SetState(i, npcState);
                }

                if (npcState.IsMesh)
                {
                    var npcSkelMesh = skeletons.TryGetSkeleton(npcState.NpcAddress, PosePurpose.ColliderMeshRoot);
                    Vector3 meshWorldPos;
                    Quaternion meshWorldRot;
                    if (npcSkelMesh != null && npcSkelMesh.HasRoot)
                    {
                        var meshRoot = npcSkelMesh.Root;
                        meshWorldPos = meshRoot.Position;
                        meshWorldRot = meshRoot.Rotation;
                    }
                    else
                    {
                        meshWorldPos = actors.GetPosition(npcState.NpcAddress, ActorSite.ColliderMesh);
                        meshWorldRot = Quaternion.Identity;
                    }

                    if (rec != null) RecActorTransform(ActorSite.ColliderMesh, npcState.NpcAddress, meshWorldPos, meshWorldRot, false, Vector3.One);
                    MoveNpcKinematic(ref npcState, npcState.MeshHandle, meshWorldPos, meshWorldRot, dt);
                    characterColliders.SetState(i, npcState);
                    continue;
                }

                if (npcState.IsFallback)
                {
                    // Simple single-capsule position update
                    var fallbackRoot = actors.GetPosition(npcState.NpcAddress, ActorSite.ColliderFallback);
                    var visualScale = TryGetVisualScale(npcState.NpcAddress, out var drawScale)
                        ? drawScale
                        : Vector3.One;
                    if (rec != null) RecActorTransform(ActorSite.ColliderFallback, npcState.NpcAddress, fallbackRoot, Quaternion.Identity, true, visualScale);
                    RescaleNpcFallbackCollision(ref npcState, CollisionScale(visualScale));
                    MoveNpcKinematic(ref npcState, npcState.FallbackHandle,
                        new Vector3(fallbackRoot.X, fallbackRoot.Y + 0.8f * visualScale.Y, fallbackRoot.Z),
                        Quaternion.Identity, dt);
                    npcState.TraversalSampleRoot = fallbackRoot;
                    npcState.HasTraversalSampleRoot = true;
                    characterColliders.SetState(i, npcState);
                    continue;
                }

                // Bone-based update: read NPC skeleton and reposition each bone static
                var ns = skeletons.TryGetSkeleton(npcState.NpcAddress, PosePurpose.ColliderFrame);
                if (rec != null) RecSkeleton(RecordType.SkeletonPose, PosePurpose.ColliderFrame, npcState.NpcAddress, ns, false);
                if (ns == null) continue; // skeleton temporarily unavailable, keep last positions

                if (!ns.HasRoot) continue;

                var npcRoot = ns.Root;
                var npcSkelPos = npcRoot.Position;
                var npcSkelRot = npcRoot.Rotation;
                var npcVisualScale = GetCharacterScale(npcState.NpcAddress, ns);
                RescaleNpcBoneCollision(ref npcState, CollisionScale(npcVisualScale));
                npcState.TraversalSampleRoot = npcSkelPos;
                npcState.HasTraversalSampleRoot = true;

                var doStrike = attackStrikeTimer > 0f &&
                               (npcState.NpcAddress == strikeColliderAddress ||
                                strikeColliderAddresses.Contains(npcState.NpcAddress));
                var lightweightAttackMode = npcState.Lightweight && doStrike &&
                                            npcState.LightweightAttackProxyIndex >= 0;
                if (npcState.LightweightAttackMode != lightweightAttackMode)
                {
                    npcState.LightweightAttackMode = lightweightAttackMode;
                    npcState.LightweightExtrasParked = false;
                }
                if (npcState.Lightweight && !npcState.LightweightExtrasParked)
                {
                    // Keep the bodies allocated: removing collidables while NarrowPhase is flushing
                    // is the exact BEPU beta path that raised SequentialFallbackBatch's NRE. Parking
                    // the inactive proxies is deterministic and lets full detail be restored safely.
                    for (var b = 0; b < npcState.BoneStatics.Count; b++)
                    {
                        if (IsNpcBoneProxyActive(npcState, b, lightweightAttackMode))
                            continue;
                        var parked = npcState.BoneStatics[b];
                        MoveNpcBoneKinematic(ref parked, npcCollisionParkPos, Quaternion.Identity, dt);
                        npcState.BoneStatics[b] = parked;
                    }
                    npcState.LightweightExtrasParked = true;
                }
                else if (!npcState.Lightweight)
                {
                    npcState.LightweightExtrasParked = false;
                }

                for (int b = 0; b < npcState.BoneStatics.Count; b++)
                {
                    if (!IsNpcBoneProxyActive(npcState, b, lightweightAttackMode))
                        continue;
                    var bs = npcState.BoneStatics[b];
                    if (bs.BoneIndex < 0 || bs.BoneIndex >= ns.BoneCount) continue;
                    if (bs.ParentBoneIndex < 0 || bs.ParentBoneIndex >= ns.BoneCount) continue;

                    // Read parent→child segment from current animation pose
                    ref var parentMt = ref ns.ModelPose[bs.ParentBoneIndex];
                    var parentWorldPos = NpcModelToWorld(
                        new Vector3(parentMt.Translation.X, parentMt.Translation.Y, parentMt.Translation.Z),
                        npcSkelPos, npcSkelRot, npcVisualScale);

                    ref var childMt = ref ns.ModelPose[bs.BoneIndex];
                    var childWorldPos = NpcModelToWorld(
                        new Vector3(childMt.Translation.X, childMt.Translation.Y, childMt.Translation.Z),
                        npcSkelPos, npcSkelRot, npcVisualScale);

                    var segment = childWorldPos - parentWorldPos;
                    var segLen = segment.Length();

                    Vector3 capsuleCenter;
                    Quaternion capsuleRot;
                    if (segLen > 0.01f)
                    {
                        var segDir = segment / segLen;
                        capsuleCenter = parentWorldPos + (segLen * bs.CenterFactor) * segDir;
                        capsuleRot = RotationFromYToDirection(segment);
                    }
                    else
                    {
                        capsuleCenter = parentWorldPos;
                        capsuleRot = Quaternion.Identity;
                    }

                    var bodyRef = simulation!.Bodies.GetBodyReference(bs.Handle);
                    var oldPos = bodyRef.Pose.Position -
                                 Vector3.Transform(bs.ShapeCenterOffset, bodyRef.Pose.Orientation);
                    var shapeCenter = capsuleCenter +
                                      Vector3.Transform(bs.ShapeCenterOffset, capsuleRot);
                    var strikeTravel = capsuleCenter - oldPos;
                    var strikeMotionValid = strikeTravel.LengthSquared() <=
                                            MathF.Pow(MathF.Max(2f, CollisionScale(npcVisualScale) * 2f), 2f);
                    MoveNpcBoneKinematic(ref bs, shapeCenter, capsuleRot, dt);
                    npcState.BoneStatics[b] = bs;
                    var collisionScale = bs.AppliedCollisionScale > 0f ? bs.AppliedCollisionScale : 1f;
                    var boneReach = Vector3.Distance(capsuleCenter, npcRootPos) +
                                    (bs.BaseHalfLength + bs.BaseRadius) * collisionScale;
                    if (boneReach > npcState.Reach && float.IsFinite(boneReach))
                        npcState.Reach = boneReach;

                    // Monster strike: this collider bone moved from oldPos→capsuleCenter this frame.
                    // During the attack window, impart that swing velocity to nearby ragdoll bodies,
                    // so a fast-swinging limb forcefully flings the body at the real contact point.
                    if (doStrike && dt > 0f && strikeMotionValid)
                    {
                        // Keep infinite-mass animated contact gentle. The separately measured
                        // strike impulse below is the user-controlled part of the reaction.
                        SoftenAttackContactVelocity(bodyRef);
                        var swingVel = strikeTravel / dt;
                        if (swingVel.LengthSquared() > StrikeMinSpeed * StrikeMinSpeed)
                            ApplyStrikeImpulse(capsuleCenter, swingVel, StrikeContactRadius);
                    }
                }
                characterColliders.SetState(i, npcState);
            }
            catch { }
        }
    }

    /// <summary>
    /// Step physics with a fixed timestep, advancing as many substeps as real
    /// wall-clock time has accumulated. This keeps ragdoll motion the same speed
    /// regardless of the game's framerate (a fixed 1/60 per render frame ran slow
    /// below 60fps and fast above it). Below 60fps we take multiple substeps; above
    /// 60fps some frames take zero — render interpolation (below) keeps motion smooth.
    /// When resting (fully settled) we skip stepping entirely.
    /// Returns the substeps taken.
    /// </summary>
    private int StepPhysics(float dt, bool resting, int boneCount)
    {
        int substeps = 0;
        if (impactFreezeRemaining > 0f)
        {
            // Impact hitstop. The corpse simply stops, mid-air, for a few dozen milliseconds — and this
            // is where most of the weight actually comes from. The same motion with a freeze frame in it
            // reads heavy; without one it reads light. It is why a fighting game's knockdown lands like
            // a sack of cement while a physically identical ragdoll reads like a balloon.
            //
            // The accumulator is dropped rather than banked, or the frozen time would all come back at
            // once as a burst of catch-up substeps and undo the pause.
            impacts.CountDownFreeze(dt);
            stepper.DropBacklog();
        }
        else if (!resting)
        {
            stepper.Accumulate(dt);
            var meshFrameStart = activeRagdollMeshVolumes && !activeRagdollIsGeneric
                ? clock.GetTimestamp(ClockSite.MeshFrameEarlyOut) : 0;
            meshVolumes.ResetStepTimings();
            var maxLinear = ActiveClassProfile.ClassConstants.MaxLinearVelocity;
            var maxAngular = ActiveClassProfile.ClassConstants.MaxAngularVelocity;
            // In a slow frame (a crowd of corpses, a hitch) one substep only: catching up with
            // several made each slow frame slower still, every corpse multiplying it. The corpse
            // moves in slow motion for those frames instead.
            var substepCap = Stepper.SubstepCap(dt);
            while (stepper.SubstepDue(substeps, substepCap))
            {
                // Snapshot the pre-tick bone pose for render interpolation. Overwritten each
                // iteration, so it ends as the state immediately before the final tick.
                poseWriter.CapturePrevBodyPoses(boneCount);
                stepper.MarkPrevCaptured();
                DriveGrabConstraints(FixedTimestep); // advance finite-force targets before solve
                meshVolumes.ApplyCachedMeshLod();
                ApplyFallGravity(FixedTimestep); // a descent that bites, on top of the integrator's honest g
                water.ApplyForces(FixedTimestep);
                var steppedExternally = false;
                if (extensionSet.Step.Length != 0) extensionSet.OnSubstep(FixedTimestep, ref steppedExternally);
                if (!steppedExternally) StepPhysicsSimulation(FixedTimestep);
                stepper.ClampVelocities(maxLinear, maxAngular);
                ApplyStandingAnchorCorrection();
                surfaceRuntime.Update();
                hairRig.Step(); // one-way guides follow the accepted head pose
                DetectImpact(FixedTimestep);
                if (rec != null) RecStep(substeps, steppedExternally);
                stepper.EndSubstep();
                substeps++;

                // The impact staged a hitstop: stop stepping NOW, or the rest of this frame's catch-up
                // substeps would step straight through the pause we just asked for.
                if (impactFreezeRemaining > 0f)
                {
                    stepper.DropBacklog();
                    break;
                }
                // A costly mesh tick must not trigger several more costly catch-up
                // ticks in the same render hook. Keep the fixed solver timestep;
                // discard excess wall time just as at the existing substep cap.
                if (meshFrameStart != 0 &&
                    clock.GetElapsedTime(ClockSite.MeshFrameEarlyOut, meshFrameStart).TotalSeconds >= FixedTimestep)
                {
                    if (rec != null) RecDecision("MeshFrameEarlyOut", true);
                    stepper.DropBacklog();
                    break;
                }
            }
            meshVolumes.ReportMeshPhysicsBudget(meshFrameStart, substeps);
            // If we hit the substep cap (severe hitch / very low fps), drop the backlog so
            // we don't fire a burst of catch-up steps on the next frame (spiral of death).
            stepper.EndSubsteps(substeps, substepCap);
        }
        else
        {
            stepper.DropBacklog();
        }
        return substeps;
    }

    /// <summary>The write-back of this frame: the bodies' pose into the skeleton (blended by
    /// <paramref name="renderAlpha"/>), then the squash, the severed limbs, the hair and the written-pose record. False
    /// when a body was not finite and the ragdoll deactivated.</summary>
    private bool WriteFramePose(IHostSkeleton skel, float dt, int boneCount, float renderAlpha)
    {
        var pose = skel.ModelPose;

        poseWriter.BeginFrame(skel, pose);

        stepper.CountFrame();
        var logThisFrame = settings.RagdollVerboseLog && (frameCount <= 3 || frameCount % 12 == 0);

        // Pass 1 (the pose writer): read every body's bone world pose. A body that is not finite deactivates the
        // ragdoll instead of reaching the mesh.
        if (poseWriter.ReadBodies(skel, boneCount, out var anyAwake) == PoseWriter.BodyReadOutcome.NotFinite)
        {
            Deactivate();
            return false;
        }
        var worldPositions = poseWriter.WorldPositions;
        var boneValid = poseWriter.BoneValid;
        if (AnyExternalBodyAwake() || hairRig.NeedsStep) anyAwake = true;

        EnsureTerrainPatchCoverage(worldPositions, boneValid, boneCount);

        // Remember whether the rig is now fully asleep so next frame can take the
        // resting fast-path. Only meaningful once at least one body exists.
        prevAllAsleep = !anyAwake && boneCount > 0;

        // --- Floor offset correction ---
        // Currently a no-op: bodies settle naturally on the terrain mesh, so no uniform
        // Y shift is applied. Kept as a zero so the write-back loop below reads uniformly;
        // restore a real value here if capsule-vs-terrain sinking ever needs correcting.
        float yCorrection = 0f;

        // --- Frame summary (once per log frame, before per-bone data) ---
        if (logThisFrame)
        {
            var awakeBodies = 0;
            var maxLinVel = 0f;
            var maxAngVel = 0f;
            var lowestCapsuleBottomY = float.MaxValue;
            for (int i = 0; i < boneCount; i++)
            {
                if (!boneValid[i]) continue;
                var rb = ragdollBones[i];
                var bodyRef = simulation!.Bodies.GetBodyReference(rb.BodyHandle);
                if (bodyRef.Awake) awakeBodies++;
                var linSpeed = bodyRef.Velocity.Linear.Length();
                var angSpeed = bodyRef.Velocity.Angular.Length();
                if (linSpeed > maxLinVel) maxLinVel = linSpeed;
                if (angSpeed > maxAngVel) maxAngVel = angSpeed;

                activeDefByName.TryGetValue(rb.Name, out var boneDef);
                var capsuleYDir = Vector3.Transform(Vector3.UnitY, bodyRef.Pose.Orientation);
                var capsuleBottomY = BodyBottom(rb, bodyRef);
                if (capsuleBottomY < lowestCapsuleBottomY)
                    lowestCapsuleBottomY = capsuleBottomY;
            }
            log.Info($"[Ragdoll F{frameCount}] t={frameCount / 60f:F2}s awake={awakeBodies}/{boneCount} " +
                     $"yCorr={yCorrection:F3} lowestY={lowestCapsuleBottomY:F3} realGnd={realGroundY:F3} " +
                     $"maxLinVel={maxLinVel:F3} maxAngVel={maxAngVel:F3}");
        }

        // Pass 2 (the pose writer): write the bones, blended from the previous tick, and carry the motion to the
        // bones without a body and to the partial skeletons on the head.
        poseWriter.WritePose(skel, pose, boneCount, renderAlpha, yCorrection, logThisFrame, frameCount);

        // Soft-tissue squash & stretch: annotate per-bone scale after every positional write
        // of the frame (position/rotation writes leave the scale channel untouched, so order
        // within the frame only matters for reading an unmodified base scale on first touch).
        UpdateSquashAndStretch(skel, dt);

        // Dismemberment POC: collapse each selected limb's whole bone subtree to ~0 scale so it
        // vanishes from the body (the "hide" half of hide-and-substitute). Done last so it overrides
        // the physics/propagation writes for those bones.
        var severedBones = useLocalDismemberBones ? localDismemberBones : settings.DismemberPocBones;
        poseWriter.HideSeveredLimbs(skel, severedBones);

        // Reassert flexible hair after head propagation, including settled frames.
        if (hairRig.Active) hairRig.Readback(skel);
        if (rec != null) RecSkeleton(RecordType.PoseOut, PosePurpose.FrameOut, targetCharacterAddress, skel, true);
        return true;
    }

    // (Dev) Keep the character visible when the ragdoll is flung far from the frozen death
    // position. Culling tests the draw object's bounding sphere, whose centre tracks the
    // draw object's own position — NOT the skinned bone positions — so a corpse that flies
    // out of the original frustum gets culled even though its bones are drawn far away.
    //
    // Local player: move ONLY the draw object (client-side render transform). We must never
    // touch GameObject.Position here — that logical position feeds the server position-sync
    // packets, and teleporting it is a bannable action. The render position is pure render
    // state (the knob SimpleHeels/Brio drive) and never leaves the client. The game
    // re-derives skeleton->Transform.Position from it, so next frame our world→model bone
    // conversion re-centres on the new root and the corpse renders in the exact same world
    // spot — only the culling sphere now follows it. GameObject.Position stays frozen at the
    // death spot (as MovementBlockHook already enforces), so the server sees no movement.
    //
    // NPC phantoms: the server has no knowledge of them, so we keep moving the real
    // GameObject.Position via SetApproachPosition, which also drags their nameplate / HP-bar
    // anchor along with the flying corpse.
    private void UpdateFollowTarget(bool isLocalPlayerTarget)
    {
        if (ragdollBones.Count > 0 && targetCharacterAddress != nint.Zero)
        {
            try
            {
                // Record only — the actual transform write is deferred to the framework tick
                // (ApplyDeferredFollowTransform). Writing it here, inside the render hook,
                // corrupts the scene graph the game is currently walking.
                var rootBody = simulation!.Bodies.GetBodyReference(ragdollBones[0].BodyHandle);
                pendingFollowPos = rootBody.Pose.Position;
                pendingFollowIsLocalPlayer = isLocalPlayerTarget;
            }
            catch { }
            followWasActive = true;
        }
        else if (followWasActive && targetCharacterAddress != nint.Zero)
        {
            // Toggled off — restore the frozen death position on whichever transform we moved
            // (same deferral).
            pendingFollowPos = savedCharacterPosition;
            pendingFollowIsLocalPlayer = isLocalPlayerTarget;
            followWasActive = false;
        }
    }

    // The stepping loop (a component, below), and the frame counter the logs read.
    private readonly Stepper stepper;
    private int frameCount => stepper.FrameCount;

    /// <summary>
    /// The stepping loop's own state and rules (docs/ARCHITECTURE.md §8): the frame clock, the fixed-step accumulator
    /// and the previous-tick flag of the render interpolation (both in the context: the hair reads the interpolation
    /// too), the substep budget, the settle countdown, the fall gravity and the velocity clamp. The ragdoll's step
    /// method runs the frame and makes every call into the other components and every recorded call between these
    /// steps. A component.
    /// </summary>
    private sealed class Stepper
    {
        private readonly SimulationContext ctx;

        internal Stepper(SimulationContext ctx) { this.ctx = ctx; }

        // Frame-rate-independent physics timing. The render hook fires once per rendered
        // frame at whatever framerate the game runs (30/60/120/144…). Stepping a fixed
        // 1/60 every call made the ragdoll run in slow-motion below 60fps and fast above it.
        // We measure real wall-clock dt and advance the simulation in fixed-size substeps.
        private const int MaxSubstepsPerFrame = 4;  // cap catch-up to avoid a spiral of death
        private const float SlowFrameSeconds = 1f / 30f; // slower than this, one substep a frame
        private const float MaxFrameDelta = 0.1f;   // ignore huge gaps (loading screens, hitches)

        // The wall-clock stamp of the previous render frame (0 = none yet).
        private long lastFrameTimestamp;
        // Diagnostic frame counter.
        internal int FrameCount { get; private set; }

        /// <summary>Activation: the next frame is the first (its delta is one fixed step).</summary>
        internal void ResetFrameClock() => lastFrameTimestamp = 0;

        /// <summary>A frame is written back: the diagnostic counter advances.</summary>
        internal void CountFrame() => FrameCount++;

        /// <summary>
        /// Real wall-clock seconds since the previous render frame, clamped to a sane range.
        /// Returns one fixed timestep on the very first call (no prior timestamp yet). The frame-delta stamp is not
        /// recorded (the frame's dt is, in the render-frame record).
        /// </summary>
        internal float ComputeFrameDelta()
        {
            var clock = ctx.Clock;
            var now = clock.GetTimestamp(ClockSite.FrameDelta);
            if (lastFrameTimestamp == 0)
            {
                lastFrameTimestamp = now;
                return FixedTimestep;
            }
            var dt = (float)((now - lastFrameTimestamp) / (double)clock.Frequency);
            lastFrameTimestamp = now;
            if (dt < 0f) dt = 0f;
            if (dt > MaxFrameDelta) dt = MaxFrameDelta;
            return dt;
        }

        /// <summary>Physics takes over: the frame counter restarts, the rig settles, and the accumulator starts clean
        /// (the activation-delay wait is not carried).</summary>
        internal void StartPhysics()
        {
            ctx.PhysicsStarted = true;
            FrameCount = 0;
            ctx.BeginBiomechanicalSettle();
            ctx.PhysicsAccumulator = 0f;
        }

        /// <summary>The previous-tick snapshot is stale (the pose buffers were reallocated).</summary>
        internal void InvalidatePrevState() => ctx.HasPrevPhysicsState = false;

        /// <summary>Drops the banked wall time (a hitstop, a cap or an early out, a resting frame).</summary>
        internal void DropBacklog() => ctx.PhysicsAccumulator = 0f;

        internal void Accumulate(float dt) => ctx.PhysicsAccumulator += dt;

        // In a slow frame (a crowd of corpses, a hitch) one substep only: catching up with
        // several made each slow frame slower still, every corpse multiplying it. The corpse
        // moves in slow motion for those frames instead.
        internal static int SubstepCap(float dt) => dt > SlowFrameSeconds ? 1 : MaxSubstepsPerFrame;

        internal bool SubstepDue(int substeps, int substepCap)
            => ctx.PhysicsAccumulator >= FixedTimestep && substeps < substepCap;

        /// <summary>The pre-tick pose was just snapshotted for the render interpolation.</summary>
        internal void MarkPrevCaptured() => ctx.HasPrevPhysicsState = true;

        internal void EndSubstep() => ctx.PhysicsAccumulator -= FixedTimestep;

        /// <summary>If the substep cap was hit (severe hitch / very low fps), the backlog is dropped.</summary>
        internal void EndSubsteps(int substeps, int substepCap)
        {
            if (substeps == substepCap)
                ctx.PhysicsAccumulator = 0f;
        }

        internal void CountDownSettle(float dt)
        {
            if (ctx.BiomechanicalSettleRemaining > 0f)
                ctx.BiomechanicalSettleRemaining = MathF.Max(0f, ctx.BiomechanicalSettleRemaining - dt);
        }

        /// <summary>Resting: BEPU has the whole island asleep and nothing blocks the rest.</summary>
        internal bool IsResting(bool activityBlocksRest) => ctx.PrevAllAsleep && !activityBlocksRest;

        /// <summary>How far to blend from the previous tick's pose: the fraction of a tick past the last one, or 1 when
        /// resting or before the first tick.</summary>
        internal float RenderAlpha(bool resting)
            => (!resting && ctx.HasPrevPhysicsState)
                ? Math.Clamp(ctx.PhysicsAccumulator / FixedTimestep, 0f, 1f)
                : 1f;

        // === Heavy body: fall gravity ===
        //
        // True gravity is not the gravity an action game runs on. A game camera's distance and field of
        // view flatten vertical travel, and an audience's sense of how fast things should fall comes from
        // films and from other games, not from a stopwatch — which is why nearly every action game falls at
        // one and a half to two and a half g, and why engines ship a "fall gravity multiplier" as standard.
        // A knockdown in a fighting game drops harder still. We were running at an honest 9.8, and honest
        // read as floaty.
        //
        // Applied only on the way DOWN, which is the usual shape of the trick: the arc a blow throws the
        // body into keeps its silhouette, and only the descent is made to bite.
        //
        // Falling is a property of the RIG, not of a body. Asked per body, against the ground height, the
        // answer is wrong in the most ordinary case there is: an arm draped over the chest, a shin crossed
        // over a thigh, a limb resting on a rock. Those sit well clear of the floor and are not falling at
        // all — something is holding them up. Push them anyway and they are driven into whatever holds them,
        // the contact shoves them back, and the two of them buzz. Worse, the answer differs from limb to
        // limb, so the joints see a relative violation on top.
        //
        // So the question is asked once, of the whole rig — is anything touching down, and is the thing as a
        // whole on its way down — and the answer is applied to every body identically. Uniform means no
        // joint sees anything at all, and there is nothing for the solver to argue with.
        private const float FallGravityMultiplier = 1.8f;
        private const float AirborneClearance = 0.06f;  // m of daylight before the rig counts as off the floor
        private const float FallGravityMinDescent = 1f; // m/s — below this it is not falling, it is sitting

        /// <summary>The fall gravity: <paramref name="inWater"/> (sinking is not falling), <paramref name="held"/> (by a
        /// grab or the standing hold) and the physics ground height come from their components.</summary>
        internal void ApplyFallGravity(float dt, bool inWater, bool held, float groundY)
        {
            var simulation = ctx.Simulation;
            var ragdollBones = ctx.Bodies;
            if (simulation == null || ragdollBones.Count == 0) return;
            if (inWater) return; // sinking is not falling

            // Being held is not falling.
            if (held) return;

            var extra = RagdollTuning.Gravity * (FallGravityMultiplier - 1f) * dt;
            if (extra <= 0f) return;

            var lowest = float.MaxValue;
            var descent = 0f;
            foreach (var rb in ragdollBones)
            {
                var body = simulation.Bodies.GetBodyReference(rb.BodyHandle);
                lowest = MathF.Min(lowest, BodyBottom(rb, body));
                descent -= body.Velocity.Linear.Y;
            }
            descent /= ragdollBones.Count;

            if (lowest <= groundY + AirborneClearance) return; // something is touching down: nobody is falling
            if (descent < FallGravityMinDescent) return;       // airborne, but not on its way down

            foreach (var rb in ragdollBones)
            {
                var body = simulation.Bodies.GetBodyReference(rb.BodyHandle);
                if (body.Kinematic) continue;

                var velocity = body.Velocity.Linear;
                velocity.Y -= extra;
                body.Velocity.Linear = velocity;
            }
        }

        // Clamp per-body velocity each substep as a hard ceiling against energy blow-up.
        // Generic rigs inject energy through their stiff auto-built constraint network (small
        // bodies + dense joints); human rigs are normally stable but can still have the solver
        // diverge into a jitter-then-explode failure. Real ragdoll fall/fling speeds stay well
        // under the ceilings, so settling looks normal, but runaway growth is capped. Callers
        // pass tighter ceilings for generic rigs and looser ones for humans.
        internal void ClampVelocities(float maxLinear, float maxAngular)
        {
            var simulation = ctx.Simulation;
            foreach (var rb in ctx.Bodies)
            {
                var body = simulation!.Bodies.GetBodyReference(rb.BodyHandle);
                if (!body.Awake) continue;
                var lin = body.Velocity.Linear;
                var linSpeed = lin.Length();
                if (linSpeed > maxLinear)
                    body.Velocity.Linear = lin * (maxLinear / linSpeed);
                var ang = body.Velocity.Angular;
                var angSpeed = ang.Length();
                if (angSpeed > maxAngular)
                    body.Velocity.Angular = ang * (maxAngular / angSpeed);
            }
        }
    }
}
