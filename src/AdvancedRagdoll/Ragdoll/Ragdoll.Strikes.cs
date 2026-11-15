// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System;
using System.Collections.Generic;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Constraints;
using AdvancedRagdoll.Hosting;

namespace AdvancedRagdoll;

public unsafe partial class Ragdoll
{
    // ─── Direct impulse ──────────────────────────────────────────────────────

    private void ApplyImpulseCore(string boneName, Vector3 velocityDelta)
    {
        if (simulation == null || !isActive) return;
        var handle = FindBodyHandle(boneName);
        if (!handle.HasValue) return;
        var body = simulation.Bodies.GetBodyReference(handle.Value);
        body.Velocity.Linear += velocityDelta;
        body.Awake = true;
        BeginBiomechanicalSettle();
    }

    private const float RecoilRelaxDuration = 0.45f;

    /// <summary>Add an angular velocity to a bone's body (a rotational kick) and briefly release its
    /// joint (drop the motor, widen the swing limit) so the stump above a severed cut actually swings
    /// up instead of being held in place. Used for the body-side recoil when a limb is kicked off.</summary>
    private void ApplyAngularVelocityCore(string boneName, Vector3 angularVelocityDelta)
    {
        if (simulation == null || !isActive) return;
        var handle = FindBodyHandle(boneName);
        if (!handle.HasValue) return;
        var body = simulation.Bodies.GetBodyReference(handle.Value);
        body.Velocity.Angular += angularVelocityDelta;
        body.Awake = true;
        RelaxJointForRecoil(boneName, RecoilRelaxDuration);
        BeginBiomechanicalSettle();
    }

    /// <summary>
    /// World-space position of a ragdoll rigid body by bone name; null while the
    /// ragdoll is inactive or the bone has no body. Unlike reading the skeleton's
    /// ModelPose at framework time (which still reflects the animation pose at the
    /// death spot), this is where the physics actually put the body — cameras that
    /// track a kicked-around corpse must use it.
    /// </summary>
    internal Vector3? GetBodyWorldPosition(string boneName)
    {
        if (simulation == null || !isActive) return null;
        var handle = FindBodyHandle(boneName);
        if (!handle.HasValue) return null;
        var body = simulation.Bodies.GetBodyReference(handle.Value);
        var p = body.Pose.Position;
        return new Vector3(p.X, p.Y, p.Z);
    }

    // Temporarily free a joint so its child can swing: zero the angular motor force and widen the
    // swing-limit cone. Both are restored by TickRecoilRelaxers after the window (the joint model).
    private void RelaxJointForRecoil(string boneName, float duration)
        => joints.RelaxJoint(boneName, duration, widenSwing: true);

    /// <summary>
    /// Release a joint's motor on impact so the limb hanging off it carries its own momentum through.
    /// The swing limits are left ALONE — unlike a recoil, which is a limb being kicked and wants room to
    /// fly, a follow-through is a limb continuing under its own steam and must stay inside its anatomy.
    /// </summary>
    private void ReleaseJointForFollowThrough(string boneName, float duration)
        => joints.RelaxJoint(boneName, duration, widenSwing: false);

    /// <summary>
    /// Punt the ragdoll body nearest to <paramref name="from"/> within <paramref name="maxDist"/>.
    /// Uses the full set of ragdoll bodies (the body "point cloud") as the hit volume rather than a
    /// single named bone, so contact anywhere on the body registers. The impulse points away from
    /// <paramref name="from"/> horizontally plus an upward kick. Returns true if a body was hit.
    /// </summary>
    private bool PuntNearestCore(Vector3 from, float maxDist, float strength, float upBias = 0.5f)
    {
        if (simulation == null || !isActive) return false;

        BodyHandle? best = null;
        var bestDistSq = maxDist * maxDist;
        var bestPos = Vector3.Zero;
        foreach (var rb in ragdollBones)
        {
            var pos = simulation.Bodies.GetBodyReference(rb.BodyHandle).Pose.Position;
            var dSq = Vector3.DistanceSquared(pos, from);
            if (dSq <= bestDistSq) { bestDistSq = dSq; best = rb.BodyHandle; bestPos = pos; }
        }
        if (!best.HasValue) return false;

        var horiz = new Vector3(bestPos.X - from.X, 0f, bestPos.Z - from.Z);
        var dir = horiz.LengthSquared() > 0.0001f ? Vector3.Normalize(horiz) : Vector3.UnitX;
        var impulse = dir * strength + Vector3.UnitY * (strength * upBias);

        // A fully settled corpse is asleep. Setting velocity on a sleeping body before it is
        // activated doesn't stick, and a single woken body anchored to a sleeping constraint
        // island won't move. Wake the whole ragdoll first, THEN apply the impulse.
        WakeRagdollBodiesForBiomechanicalSettle();
        var body = simulation.Bodies.GetBodyReference(best.Value);
        body.Awake = true;
        body.Velocity.Linear += impulse;
        BeginBiomechanicalSettle();
        return true;
    }

    // Monster strike tuning: a collider bone must move faster than this to register a hit, and only
    // ragdoll bodies within this radius of the bone are hit.
    private const float StrikeMinSpeed = 1.5f;      // m/s
    private const float StrikeContactRadius = 0.6f; // m
    // A strike knocks the body AWAY from the contact point (outward + a little up), with strength
    // from the swing SPEED — using the raw swing velocity would drag the body along the arc (toward
    // the attacker on the retract phase).
    private const float StrikeUpBias = 0.3f;

    private const float WeaponStrikeRadius = 0.55f; // generous around the blade line (weapons are thin)

    // The strike window (Ragdoll.Strikes.cs).
    private readonly WeaponStrike strike;

    // Monster strike: during the attack window the monster's collider bones impart their swing
    // velocity as an impulse to nearby ragdoll bodies (a fast swing = a heavy hit, at the limb's
    // real contact point).
    private float attackStrikeTimer { get => strike.AttackStrikeTimer; set => strike.AttackStrikeTimer = value; }
    private float strikePower { get => strike.StrikePower; set => strike.StrikePower = value; }
    private nint strikeColliderAddress { get => strike.ColliderAddress; set => strike.ColliderAddress = value; }
    private HashSet<nint> strikeColliderAddresses => strike.ColliderAddresses;
    private bool weaponPrevValid { get => strike.WeaponPrevValid; set => strike.WeaponPrevValid = value; }
    private HashSet<int> struckThisWindow => strike.StruckThisWindow;

    /// <summary>Impart a swing as an impulse to ragdoll bodies near <paramref name="point"/>.</summary>
    private void ApplyStrikeImpulse(Vector3 point, Vector3 swingVel, float radius)
        => strike.ApplyImpulse(point, swingVel, radius, standingActive);

    /// <summary>
    /// A humanoid's weapon is a separate draw object, so it isn't in the bone collision capsules.
    /// Treat the blade as a line segment (its world transform lives on the weapon's own skeleton root
    /// — the render position alone doesn't move the mesh) and, during the attack window, impart its
    /// swing velocity to ragdoll bodies near the blade. No-op for unarmed/creature models.
    /// </summary>
    private void UpdateWeaponStrike(float dt)
    {
        if (simulation == null || strikeColliderAddress == nint.Zero) { weaponPrevValid = false; return; }
        if (!colliders.TryGetWeaponTransform(strikeColliderAddress, out var center, out var rot)) { weaponPrevValid = false; return; }
        if (rec != null) RecWeaponSample(center, rot);
        strike.SweepWeapon(center, rot, dt, standingActive);
    }

    private void CancelAttackStrikeCore() => strike.Cancel();

    /// <summary>
    /// The strike window: while it is open, the swing of the striking collider's bones and of its weapon's blade is
    /// imparted to the ragdoll bodies it passes, each body at most once per swing. A component
    /// (docs/ARCHITECTURE.md §8).
    /// </summary>
    private sealed class WeaponStrike
    {
        private readonly SimulationContext ctx;
        internal WeaponStrike(SimulationContext ctx) { this.ctx = ctx; }

        internal float AttackStrikeTimer;
        internal float StrikePower;
        internal nint ColliderAddress;
        // Bone-driven swarm attacks may have several animated collider sources in one strike window.
        // The single address above remains the optional weapon source; this set drives body bones.
        internal readonly HashSet<nint> ColliderAddresses = new();
        // Weapon strike: a humanoid's weapon is a separate draw object (not in the bone capsules), so
        // track its blade endpoints to give a sword swing its own velocity-driven hit.
        private Vector3 prevBladeA, prevBladeB;
        internal bool WeaponPrevValid;
        // Each body is struck at most once per swing — the window spans many frames, so striking every
        // frame accumulated into a launch. One hit per swing makes strike power linear/predictable.
        internal readonly HashSet<int> StruckThisWindow = new();

        private Vector3 StrikeImpulse(Vector3 bodyPos, Vector3 contactPoint, float swingSpeed)
        {
            var away = new Vector3(bodyPos.X - contactPoint.X, 0f, bodyPos.Z - contactPoint.Z);
            var dir = away.LengthSquared() > 1e-6f ? Vector3.Normalize(away) : Vector3.UnitX;
            return (dir + Vector3.UnitY * StrikeUpBias) * (swingSpeed * StrikePower);
        }

        /// <summary>Impart a swing as an impulse to ragdoll bodies near <paramref name="point"/>.
        /// <paramref name="standingActive"/>: the standing support holds the rig.</summary>
        internal void ApplyImpulse(Vector3 point, Vector3 swingVel, float radius, bool standingActive)
        {
            var simulation = ctx.Simulation;
            if (simulation == null) return;
            if (standingActive) return;
            var speed = swingVel.Length();
            var radiusSq = radius * radius;
            foreach (var rb in ctx.Bodies)
            {
                var bodyRef = simulation.Bodies.GetBodyReference(rb.BodyHandle);
                var p = bodyRef.Pose.Position;
                if (Vector3.DistanceSquared(p, point) > radiusSq) continue;
                if (!StruckThisWindow.Add(rb.BodyHandle.Value)) continue; // one hit per swing
                bodyRef.Velocity.Linear += StrikeImpulse(p, point, speed);
                bodyRef.Awake = true;
            }
        }

        /// <summary>The blade at its sampled transform (<paramref name="center"/>, <paramref name="rot"/>): during
        /// the attack window, its swing since the last sample strikes the bodies near it.</summary>
        internal void SweepWeapon(Vector3 center, Quaternion rot, float dt, bool standingActive)
        {
            // Weapon-drop models the weapon as a Y-aligned capsule, so the blade runs along local Y.
            // Sample both ends so the blade direction's sign doesn't matter.
            var dir = Vector3.Transform(Vector3.UnitY, rot);
            var half = MathF.Max(0.4f, RagdollTuning.WeaponDropHalfLength);
            var a = center - dir * half;
            var b = center + dir * half;

            if (AttackStrikeTimer > 0f && WeaponPrevValid && dt > 0f)
            {
                var vel = ((a - prevBladeA) + (b - prevBladeB)) * (0.5f / dt);
                if (vel.LengthSquared() > StrikeMinSpeed * StrikeMinSpeed)
                    ApplyImpulseSegment(a, b, vel, WeaponStrikeRadius, standingActive);
            }
            prevBladeA = a;
            prevBladeB = b;
            WeaponPrevValid = true;
        }

        /// <summary>Strike ragdoll bodies near the blade line segment a→b.</summary>
        private void ApplyImpulseSegment(Vector3 a, Vector3 b, Vector3 swingVel, float radius, bool standingActive)
        {
            var simulation = ctx.Simulation;
            if (simulation == null) return;
            if (standingActive) return;
            var speed = swingVel.Length();
            var ab = b - a;
            var abLenSq = ab.LengthSquared();
            var radiusSq = radius * radius;
            foreach (var rb in ctx.Bodies)
            {
                var bodyRef = simulation.Bodies.GetBodyReference(rb.BodyHandle);
                var p = bodyRef.Pose.Position;
                var t = abLenSq > 1e-6f ? Math.Clamp(Vector3.Dot(p - a, ab) / abLenSq, 0f, 1f) : 0f;
                var closest = a + ab * t;
                if (Vector3.DistanceSquared(p, closest) > radiusSq) continue;
                if (!StruckThisWindow.Add(rb.BodyHandle.Value)) continue; // one hit per swing
                bodyRef.Velocity.Linear += StrikeImpulse(p, closest, speed);
                bodyRef.Awake = true;
            }
        }

        internal void Cancel()
        {
            AttackStrikeTimer = 0f;
            StrikePower = 0f;
            ColliderAddresses.Clear();
            StruckThisWindow.Clear();
            WeaponPrevValid = false;
        }
    }

    /// <summary>Distance from <paramref name="from"/> to the nearest ragdoll body, or null if none.</summary>
    internal float? NearestBodyDistance(Vector3 from)
    {
        if (simulation == null || !isActive) return null;
        float? best = null;
        foreach (var rb in ragdollBones)
        {
            var pos = simulation.Bodies.GetBodyReference(rb.BodyHandle).Pose.Position;
            var d = Vector3.Distance(pos, from);
            if (best == null || d < best.Value) best = d;
        }
        return best;
    }



    private void ApplyShakeCore(float intensity)
    {
        if (simulation == null || !isActive) return;
        var handle = FindBodyHandle(skeletonProfile.Landmarks.Pelvis);
        if (!handle.HasValue) return;
        var angle = (float)(random.NextDouble(RandomSite.ApplyShake) * MathF.PI * 2f);
        var impulse = new Vector3(MathF.Cos(angle) * intensity, 0f, MathF.Sin(angle) * intensity);
        var body = simulation.Bodies.GetBodyReference(handle.Value);
        body.Velocity.Linear += impulse;
        body.Awake = true;
        BeginBiomechanicalSettle();
    }
}
